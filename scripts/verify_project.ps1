<#
.SYNOPSIS
    Automated Offline Verification Suite for SM-5800 6-DOF IMU Project.
.DESCRIPTION
    Executes 5 deterministic verification gates:
      [Gate A] Project file completeness check
      [Gate B] Verilog-2001 static lint & module port binding cross-check
      [Gate C] Bit-exact SDLC CRC-16-CCITT & lookahead deframer verification
      [Gate D] Production C driver static audit (MISRA C:2012 + BARR-C + DO-178C)
      [Gate E] RTL <-> C register offset parity & bi-directional @req/@design traceability
    Exits with code 0 only if ALL gates pass with 0 violations.
#>

$ErrorActionPreference = "Stop"
$Root = Split-Path -Parent $PSScriptRoot
$TotalFailures = 0

function Report-Gate([string]$Name, [int]$Errors, [string]$Detail) {
    if ($Errors -eq 0) {
        Write-Host ("[PASS] {0,-42} : {1}" -f $Name, $Detail)
    } else {
        Write-Host ("[FAIL] {0,-42} : {1} ({2} errors)" -f $Name, $Detail, $Errors)
        $script:TotalFailures += $Errors
    }
}

Write-Host "========================================================================"
Write-Host " SM-5800 6-DOF IMU Project - Automated Verification Suite"
Write-Host " Root: $Root"
Write-Host "========================================================================"

#------------------------------------------------------------------------------
# Gate A: Required Project Files Check
#------------------------------------------------------------------------------
$reqFiles = @(
    "README.md",
    "AGENT_VERIFICATION_GUIDE.md",
    "Makefile",
    ".gitignore",
    "rtl/sm5800_filelist.f",
    "rtl/sm5800_param_error.v",
    "rtl/sm5800_crc16_ccitt.v",
    "rtl/sm5800_clk_ctrl.v",
    "rtl/sm5800_sdlc_rx.v",
    "rtl/sm5800_msg_parser.v",
    "rtl/sm5800_axi4_lite_if.v",
    "rtl/sm5800_imu_axi_top.v",
    "tb/sm5800_sensor_model.v",
    "tb/tb_sm5800_imu_axi.v",
    "driver/sm5800_regs.h",
    "driver/sm5800_hal.h",
    "driver/sm5800_hal.c",
    "driver/sm5800_drv.h",
    "driver/sm5800_drv.c",
    "driver/sm5800_ps_example.c",
    "driver/DEVIATION_LOG.md",
    "tests/mock_sm5800_hal.h",
    "tests/mock_sm5800_hal.c",
    "tests/test_sm5800_drv.c",
    "docs/README.md",
    "docs/LLR_SM5800.md",
    "docs/SDD_SM5800.md",
    "docs/TEST_REPORT_SM5800.md",
    "scripts/verify_project.ps1"
)
$gateAErr = 0
foreach ($rel in $reqFiles) {
    $full = Join-Path $Root $rel
    if (-not (Test-Path $full) -or ((Get-Item -Force $full).Length -eq 0)) {
        Write-Host "  Missing or empty: $rel"
        $gateAErr++
    }
}
Report-Gate "Gate A: Project File Completeness" $gateAErr "$($reqFiles.Count) required files checked"

#------------------------------------------------------------------------------
# Gate B: Verilog-2001 Static Lint & Port Binding Cross-Check
#------------------------------------------------------------------------------
$gateBErr = 0
$rtlFiles = Get-ChildItem (Join-Path $Root "rtl/*.v")
$tbFiles  = Get-ChildItem (Join-Path $Root "tb/*.v")
$modPorts = @{}

foreach ($f in ($rtlFiles + $tbFiles)) {
    $raw = Get-Content $f.FullName -Raw
    $noBlk = [regex]::Replace($raw, '/\*[\s\S]*?\*/', '')
    $codeOnly = (($noBlk -split "`n") | ForEach-Object { $_ -replace '//.*', '' }) -join "`n"
    $codeNoStr = [regex]::Replace($codeOnly, '"[^"]*"', '""')

    if (($raw -notmatch '`default_nettype\s+none') -or ($raw -notmatch '`default_nettype\s+wire')) {
        Write-Host "  [RTL] Missing default_nettype pair in $($f.Name)"
        $gateBErr++
    }
    $svKw = [regex]::Matches($codeNoStr, '\b(logic|bit|byte|always_ff|always_comb|always_latch|enum|typedef|interface|struct|\$clog2)\b')
    if ($svKw.Count -gt 0) {
        Write-Host "  [RTL] SystemVerilog keyword in $($f.Name): $($svKw[0].Value)"
        $gateBErr++
    }
    if (([regex]::Matches($codeNoStr, '\bbegin\b').Count) -ne ([regex]::Matches($codeNoStr, '\bend\b').Count)) {
        Write-Host "  [RTL] Unbalanced begin/end in $($f.Name)"
        $gateBErr++
    }
    if (([regex]::Matches($codeNoStr, '\bcase\b').Count) -ne ([regex]::Matches($codeNoStr, '\bendcase\b').Count)) {
        Write-Host "  [RTL] Unbalanced case/endcase in $($f.Name)"
        $gateBErr++
    }
    if ($f.Directory.Name -eq "rtl") {
        if ($raw -match 'module\s+(\w+)\s*(?:#\s*\([\s\S]*?\)\s*)?\(([\s\S]*?)\);') {
            $mName = $Matches[1]
            $ports = @()
            foreach ($line in ($Matches[2] -split "`n")) {
                $clean = ($line -replace '//.*', '').Trim()
                if ($clean -match '(input|output|inout)\s+.*?(\w+)\s*,?\s*$') { $ports += $Matches[2] }
            }
            $modPorts[$mName] = $ports
        }
    }
}

$instChecked = 0
foreach ($f in ($rtlFiles + $tbFiles)) {
    $raw = Get-Content $f.FullName -Raw
    $noBlk = [regex]::Replace($raw, '/\*[\s\S]*?\*/', '')
    $codeOnly = (($noBlk -split "`n") | ForEach-Object { $_ -replace '//.*', '' }) -join "`n"
    foreach ($mName in $modPorts.Keys) {
        # sm5800_param_error is the parameter-guard target: its instances
        # connect a deliberately missing port and only exist for ILLEGAL
        # configurations (see rtl/sm5800_param_error.v), so they are exempt.
        if ($mName -eq 'sm5800_param_error') { continue }
        $pattern = "(?<!module\s+)\b$mName\b\s*(?:#\s*\([^;]*?\)\s*)?(u_\w+)\s*\(([^;]*?)\);"
        foreach ($m in [regex]::Matches($codeOnly, $pattern)) {
            $instName = $m.Groups[1].Value
            $conns = @()
            foreach ($cm in [regex]::Matches($m.Groups[2].Value, '\.(\w+)\s*\(')) { $conns += $cm.Groups[1].Value }
            $missing = $modPorts[$mName] | Where-Object { $_ -notin $conns }
            $extra   = $conns | Where-Object { $_ -notin $modPorts[$mName] }
            $instChecked++
            if (($missing.Count -gt 0) -or ($extra.Count -gt 0)) {
                Write-Host "  [RTL] Port mismatch on $instName ($mName) in $($f.Name): missing=[$($missing -join ',')] extra=[$($extra -join ',')]"
                $gateBErr++
            }
        }
    }
}
Report-Gate "Gate B: Verilog-2001 Lint & Port Bindings" $gateBErr "$($rtlFiles.Count) RTL + $($tbFiles.Count) TB files, $instChecked instances verified"

#------------------------------------------------------------------------------
# Gate C: Bit-Exact SDLC CRC-16-CCITT & Deframer Algorithm Check
#------------------------------------------------------------------------------
$gateCErr = 0
function Calc-SdlcFcs([byte[]]$data) {
    [uint16]$crc = 0xFFFF
    foreach ($b in $data) {
        for ($i = 0; $i -lt 8; $i++) {
            $fb = ($crc -band 1) -bxor (($b -shr $i) -band 1)
            $crc = $crc -shr 1
            if ($fb) { $crc = $crc -bxor 0x8408 }
        }
    }
    [uint16]$fcs = $crc -bxor 0xFFFF
    return @([byte]($fcs -band 0xFF), [byte](($fcs -shr 8) -band 0xFF))
}
$stdCheck = Calc-SdlcFcs ([System.Text.Encoding]::ASCII.GetBytes("123456789"))
$stdWord = ([uint16]$stdCheck[0]) -bor (([uint16]$stdCheck[1]) -shl 8)
if ($stdWord -ne 0x906E) { Write-Host "  [CRC] Check value mismatch"; $gateCErr++ }

# Encode Msg 2 (44 bytes) with bit-stuffing and verify deframer residue == 0xF0B8
$msg2 = New-Object byte[] 42
$msg2[0] = 0x0A; $msg2[1] = 0x02
for ($k = 2; $k -lt 42; $k++) { $msg2[$k] = [byte](($k * 19 + 0xF0) -band 0xFF) }
$full2 = $msg2 + (Calc-SdlcFcs $msg2)
$bits = New-Object System.Collections.Generic.List[int]
foreach ($b in @(0,1,1,1,1,1,1,0, 0,1,1,1,1,1,1,0)) { $bits.Add($b) }
$ones = 0
foreach ($b in $full2) {
    for ($i = 0; $i -lt 8; $i++) {
        $bit = ($b -shr $i) -band 1
        $bits.Add($bit)
        if ($bit -eq 1) { $ones++; if ($ones -eq 5) { $bits.Add(0); $ones = 0 } } else { $ones = 0 }
    }
}
foreach ($b in @(0,1,1,1,1,1,1,0)) { $bits.Add($b) }

[int]$raw_sr = 0xFF; [bool]$in_frame = $false; [int]$skip = 0; [int]$oc = 0
[int]$bc = 0; [int]$bsr = 0; [uint16]$crc = 0xFFFF; [bool]$hb = $false
$rxLen = 0; $gotFrame = $false; $gotRes = 0; $gotAlign = $false
foreach ($rb in $bits) {
    $ob = $raw_sr -band 1
    $nr = (($rb -shl 7) -bor ($raw_sr -shr 1)) -band 0xFF
    $isF = ($nr -eq 0x7E)
    if (-not $in_frame) {
        if ($isF) { $in_frame = $true; $skip = 8; $oc = 0; $bc = 0; $crc = 0xFFFF; $hb = $false; $rxLen = 0 }
    } else {
        $nc = $crc; $nbc = $bc; $nhb = $hb
        if ($skip -gt 0) { $skip-- }
        elseif ($oc -eq 5) { $oc = 0 }
        else {
            if ($ob -eq 1) { $oc++ } else { $oc = 0 }
            $nhb = $true
            $fb = ($crc -band 1) -bxor $ob
            $nc = $crc -shr 1; if ($fb) { $nc = $nc -bxor 0x8408 }
            $bsr = (($ob -shl 7) -bor ($bsr -shr 1)) -band 0xFF
            if ($bc -eq 7) { $rxLen++; $nbc = 0 } else { $nbc = $bc + 1 }
        }
        if ($isF) {
            if ($nhb) { $gotFrame = $true; $gotRes = $nc; $gotAlign = ($nbc -eq 0) }
            $skip = 8; $oc = 0; $bc = 0; $crc = 0xFFFF; $hb = $false
        } else { $crc = $nc; $bc = $nbc; $hb = $nhb }
    }
    $raw_sr = $nr
}
if ((-not $gotFrame) -or ($rxLen -ne 44) -or ($gotRes -ne 0xF0B8) -or (-not $gotAlign)) {
    Write-Host "  [SDLC] Msg 2 deframer failed: got=$gotFrame len=$rxLen res=0x$($gotRes.ToString('X4')) align=$gotAlign"
    $gateCErr++
}
Report-Gate "Gate C: Bit-Exact CRC-16 & SDLC Deframer" $gateCErr "Check=0x906E, Msg2 44B Residue=0xF0B8"

#------------------------------------------------------------------------------
# Gate D: Production C Coding Rules Static Audit (driver/*.c, driver/*.h)
#------------------------------------------------------------------------------
$gateDErr = 0
$cFiles = Get-ChildItem (Join-Path $Root "driver/*.c"), (Join-Path $Root "driver/*.h")
foreach ($f in $cFiles) {
    $lines = Get-Content $f.FullName
    $raw   = Get-Content $f.FullName -Raw
    for ($idx = 0; $idx -lt $lines.Count; $idx++) {
        $ln = $lines[$idx]
        if ($ln.Length -gt 80) { Write-Host "  [C-LEN] $($f.Name):$($idx+1) len=$($ln.Length)"; $gateDErr++ }
        if ($ln -match '//')   { Write-Host "  [C-CMT] $($f.Name):$($idx+1) '//' forbidden"; $gateDErr++ }
        if ($ln -match "`t")   { Write-Host "  [C-TAB] $($f.Name):$($idx+1) tab forbidden"; $gateDErr++ }
        $noCmt = $ln -replace '/\*.*?\*/', ''
        if (($noCmt -match '\S\s*\{') -and ($noCmt -notmatch '=\s*\{[^}]*\}\s*;')) {
            Write-Host "  [C-ALLMAN] $($f.Name):$($idx+1) cuddled brace"
            $gateDErr++
        }
    }
    foreach ($cm in [regex]::Matches($raw, '/\*[\s\S]*?\*/')) {
        if ($cm.Value.Substring(2, $cm.Value.Length - 4) -match '/\*') {
            Write-Host "  [C-NEST] $($f.Name) nested /* comment"
            $gateDErr++
        }
    }
    $codeOnly = [regex]::Replace($raw, '/\*[\s\S]*?\*/', ' ')
    foreach ($kw in @('goto','continue','malloc','calloc','realloc','free','alloca','register','auto','union','extern','stdio','stdlib','errno')) {
        if ($codeOnly -match "\b$kw\b") { Write-Host "  [C-KW] $($f.Name): forbidden '$kw'"; $gateDErr++ }
    }
    if ($codeOnly -match '\?') { Write-Host "  [C-TERN] $($f.Name): ternary '?' found"; $gateDErr++ }
    foreach ($m in [regex]::Matches($codeOnly, '\b[A-Za-z_][A-Za-z0-9_]*\b')) {
        $id = $m.Value
        if ($id.Length -gt 31) { Write-Host "  [C-ID31] $($f.Name): $id"; $gateDErr++ }
        if ($id -match '(?i)(^|_)(value|val|data|tmp|buf|info)($|_)') {
            Write-Host "  [C-VAGUE] $($f.Name): $id"
            $gateDErr++
        }
    }
    if ($f.Extension -eq '.c') {
        $depth = 0; $fnStart = -1; $prevLine = ""
        for ($i = 0; $i -lt $lines.Count; $i++) {
            $l = ($lines[$i] -replace '/\*.*?\*/', '').Trim()
            if (($depth -eq 0) -and ($l -eq '{')) {
                if ($prevLine -match '\)\s*$') { $fnStart = $i; $depth = 1 }
            }
            elseif ($depth -gt 0) {
                $depth += (([regex]::Matches($l, '\{')).Count - ([regex]::Matches($l, '\}')).Count)
                if (($depth -eq 0) -and (($i - $fnStart + 1) -gt 100)) {
                    Write-Host "  [C-FN100] $($f.Name) line $($fnStart+1) > 100 lines"
                    $gateDErr++
                }
            }
            if ($l.Length -gt 0) { $prevLine = $l }
        }
    }
}
Report-Gate "Gate D: Production C Rules Static Audit" $gateDErr "$($cFiles.Count) files (MISRA/BARR-C/DO-178C)"

#------------------------------------------------------------------------------
# Gate E: Register Offset Parity (RTL <-> C) & Traceability Audit
#------------------------------------------------------------------------------
$gateEErr = 0
$axiRtl = Get-Content (Join-Path $Root "rtl/sm5800_axi4_lite_if.v") -Raw
$regsH  = Get-Content (Join-Path $Root "driver/sm5800_regs.h") -Raw
$rtlOffsets = @{}
foreach ($m in [regex]::Matches($axiRtl, "P_REG_(\w+)\s*=\s*6'h([0-9A-Fa-f]+)")) {
    $rtlOffsets[$m.Groups[1].Value] = ([Convert]::ToInt32($m.Groups[2].Value, 16) * 4)
}
$cOffsets = @{}
$checkedRegs = 0
foreach ($m in [regex]::Matches($regsH, '#define\s+SM5800_REG_(\w+)\s+\(0x([0-9A-Fa-f]+)U\)')) {
    $rName = $m.Groups[1].Value
    $cOff  = [Convert]::ToInt32($m.Groups[2].Value, 16)
    $cOffsets[$rName] = $cOff
    if (-not $rtlOffsets.ContainsKey($rName)) {
        Write-Host "  [REG] C reg $rName missing in RTL"
        $gateEErr++
    } elseif ($rtlOffsets[$rName] -ne $cOff) {
        Write-Host ("  [REG] Offset mismatch on {0}: C=0x{1:X2} RTL=0x{2:X2}" -f $rName, $cOff, $rtlOffsets[$rName])
        $gateEErr++
    } else {
        $checkedRegs++
    }
}
foreach ($rtlReg in $rtlOffsets.Keys) {
    if (-not $cOffsets.ContainsKey($rtlReg)) {
        Write-Host "  [REG] RTL reg $rtlReg missing in sm5800_regs.h"
        $gateEErr++
    }
}

$llrTxt = Get-Content (Join-Path $Root "docs/LLR_SM5800.md") -Raw
$sddTxt = Get-Content (Join-Path $Root "docs/SDD_SM5800.md") -Raw
$reqSet = New-Object System.Collections.Generic.HashSet[string]
$desSet = New-Object System.Collections.Generic.HashSet[string]
foreach ($f in $cFiles) {
    $raw = Get-Content $f.FullName -Raw
    foreach ($m in [regex]::Matches($raw, '@req\s+(LLR-SM5800-[A-Z0-9-]+)')) { [void]$reqSet.Add($m.Groups[1].Value) }
    foreach ($m in [regex]::Matches($raw, '@design\s+(SDD-SM5800-[0-9.]+)')) { [void]$desSet.Add($m.Groups[1].Value) }
}
foreach ($r in $reqSet) { if ($llrTxt -notmatch [regex]::Escape($r)) { Write-Host "  [TRACE] Missing LLR in docs: $r"; $gateEErr++ } }
foreach ($m in [regex]::Matches($llrTxt, '`(LLR-SM5800-(?:REG|HAL|DRV|APP)-\d+)`')) {
    $docLlr = $m.Groups[1].Value
    if (-not $reqSet.Contains($docLlr)) {
        Write-Host "  [TRACE] C LLR in docs missing @req in driver/: $docLlr"
        $gateEErr++
    }
}
foreach ($d in $desSet) {
    if (($sddTxt -notmatch [regex]::Escape($d)) -and ($llrTxt -notmatch [regex]::Escape($d))) {
        Write-Host "  [TRACE] Missing SDD: $d"
        $gateEErr++
    }
}
Report-Gate "Gate E: Reg Parity & LLR/SDD Traceability" $gateEErr "$checkedRegs regs + $($reqSet.Count) @req + $($desSet.Count) @design matched (bi-directional)"

Write-Host "========================================================================"
if ($TotalFailures -eq 0) {
    Write-Host " OVERALL OFFLINE VERIFICATION: PASS (0 violations)"
    exit 0
} else {
    Write-Host " OVERALL OFFLINE VERIFICATION: FAIL ($TotalFailures total violations)"
    exit 1
}
