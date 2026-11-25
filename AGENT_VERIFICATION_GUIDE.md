# Agent Verification & Audit Guide — SM-5800 6-DOF IMU Project

> **Mục đích:** Tài liệu hướng dẫn chuẩn cho **một AI Agent kiểm thử / QA Engineer độc lập** khi truy cập vào dự án này để kiểm tra, đối chiếu thông số kỹ thuật, chạy kiểm chứng tự động và lập báo cáo nghiệm thu.

---

## 1. Bối cảnh & Hợp đồng Yêu cầu (Ground-Truth Baseline)

Dự án triển khai hệ thống đọc cảm biến **SM-5800 6-DOF Motion Sensor Module (IMU)** trên nền tảng **FPGA PL (Verilog-2001, AMBA AXI4-Lite)** kết hợp **ARM PS Bare-Metal Driver (C99 — MISRA C:2012 + BARR-C:2018 + DO-178A/B/C)**:

1. **Giao thức vật lý & SDLC (SM-5800 Sec. 4.0–5.4):**
   - Host FPGA cấp xung `RCV_CLK` **1.0 MHz ± 2%** (duty 50%) qua RS-422 (`26C31`); thu `SER_DATA` NRZ đồng bộ qua RS-422 (`26C32` + điện trở cuối `180 Ω`).
   - Định dạng SDLC: Cờ `0x7E`, chèn bit `0` sau 5 bit `1`, truyền **LSB-first / Little-Endian**, hỗ trợ cờ rảnh liên tiếp (`7E 7E...`), **Shared Flag**, phát hiện Abort (`≥ 7` bit `1`), **CRC-16-CCITT** ($X^{16}+X^{12}+X^5+1$, preset `0xFFFF`, đa thức phản xạ `0x8408`, residue chuẩn Z8530 = `0xF0B8` hoặc raw `0x0000`).
2. **Phân biệt bản tin & Chế độ 100 Hz (SM-5800 Sec. 5.1–5.2):**
   - `Module Addr = 0x0A`; **Msg ID 01** (600 Hz, 20 byte); **Msg ID 02** (**100 Hz**, 44 byte: 16B Motion Control + 24B Spatial Data + 2B CRC).
   - Khối `sm5800_msg_parser` và `sm5800_imu_axi_top` xuất **2 chân tách biệt** `msg1_valid_o` và `msg2_valid_o`.
   - Khi `CTRL[11] MSG2_ONLY_MODE = 1` và `INT_ENABLE = 0x26 (MSG2_READY | IMU_FAULT | LINK_TIMEOUT)`: FPGA chỉ commit toàn bộ 14 trường dữ liệu cho bản tin `02` ở **100 Hz** (bản tin `01` không ghi đè dữ liệu `02` và không làm gián đoạn PS), đồng thời báo ngắt ngay khi có lỗi BIT hoặc mất luồng dữ liệu.
3. **Điều khiển Reset & BIT từ PS → FPGA → IMU (SM-5800 Sec. 4.3.2, 4.4, 6.3):**
   - `RESET_L` (Pin 18, open-collector): xung thấp phần cứng **> 1.0 ms** (mặc định `2.0 ms`), thả nổi `1` khi FPGA reset; có cả chế độ giữ/nhả thủ công (`sm5800_hold_reset` / `sm5800_release_reset`).
   - `BIT_L` (Pin 12, open-collector): điều khiển Commanded BIT từ PS (`sm5800_start_cmd_bit` / `sm5800_stop_cmd_bit`) khi phát hiện lỗi trong `Status Word 1/2`.
4. **Hàm 1-lệnh-gọi đọc/quy đổi SI trên PS:**
   - `sm5800_read_msg2_eng(p_drv, p_eng)` (khóa `SNAP_LOCK` → đọc 15 thanh ghi → nhả khóa → quy đổi SI) và `sm5800_raw_to_eng_msg2(p_raw, p_eng)` trả về giá trị vật lý đọc & kiểm chứng được ngay (`rad/s`, `m/s²`, `rad`, `m/s`, `°C`, `bit_fault_mask`).

---

## 2. Quy trình kiểm tra bắt buộc cho Agent (7 Cổng kiểm chứng — Verification Gates)

Agent kiểm tra phải thực hiện **tuần tự** các cổng sau, ghi lại bằng chứng (`command`, `exit code`, `stdout`, `file:line`). Nếu một công cụ (như `iverilog` hay `arm-none-eabi-gcc`) chưa được cài trên máy, ghi rõ trạng thái **`BLOCKED (missing tool)`** — **tuyệt đối không giả lập kết quả PASS**.

### Gate 1 — Chạy bộ kiểm tra tĩnh & thuật toán tự động (KHÔNG cần EDA/GCC)
```powershell
powershell -ExecutionPolicy Bypass -File scripts/verify_project.ps1
```
- **Tiêu chí đạt:** `Exit code = 0` và `OVERALL OFFLINE VERIFICATION: PASS (0 violations)` trên cả 5 cổng con:
  - `Gate A`: Đủ 29 tệp cấu thành dự án.
  - `Gate B`: Thuần Verilog-2001 (`default_nettype none/wire`, 0 từ khóa SV, cân bằng khối, 100% kết nối cổng khớp giữa `rtl/` và `tb/`).
  - `Gate C`: Khớp toán học CRC-16-CCITT (`0x906E`, `0xF0B8`) và tách khung SDLC Msg ID 02 (44 byte có bit-stuffing).
  - `Gate D`: 0 vi phạm C Rules trên toàn bộ `driver/*.h`, `driver/*.c` (dòng ≤ 80, ngoặc Allman, 0 `//`, 0 tab, 0 từ mơ hồ, ID ≤ 31, hàm ≤ 100 dòng, 0 `? :`, 0 từ khóa cấm).
  - `Gate E`: Khớp 100% địa chỉ thanh ghi `rtl/sm5800_axi4_lite_if.v ↔ driver/sm5800_regs.h` và khớp 100% truy vết `@req`/`@design` ↔ `docs/LLR_SM5800.md` / `docs/SDD_SM5800.md`.

### Gate 2 — Mô phỏng RTL & Lint phần cứng (`[T1]`–`[T18]`)
Chạy một trong các trình mô phỏng sau (ưu tiên tất cả các công cụ có sẵn trên máy):
```bash
# Icarus Verilog
make sim

# Hoặc Xilinx Vivado xsim
make xsim

# Lint tĩnh Verilator (yêu cầu 0 error / 0 warning)
make lint-rtl
```
- **Tiêu chí đạt:**
  - `ALL TESTS PASSED! (... assertions passed, 0 errors)` trên toàn bộ 18 kịch bản `[T1]`–`[T18]`.
  - Xác nhận `[T10]` đo xung `RESET_L` thấp **> 1,000,000 ns (> 1.0 ms)**.
  - Xác nhận `[T13]` ở chế độ `MSG2_ONLY_MODE = 1`: Msg 01 không gây `irq_o` và không ghi đè dữ liệu Msg 02; Msg 02 cập nhật đủ 14 trường và dựng `irq_o = 1`.

### Gate 3 — Biên dịch nghiêm ngặt & Chạy Unit Test C trên Host (`TC-DRV-01`–`17`)
```bash
make test-c
```
- **Tiêu chí đạt:**
  - `driver/sm5800_drv.c` biên dịch **0 warning / 0 error** với đầy đủ cờ §12 (`-std=c99 -pedantic-errors -Wall -Wextra -Werror -Wconversion -Wsign-conversion -Wshadow -Wstrict-prototypes -Wmissing-prototypes -Wundef -Wcast-qual -Wcast-align=strict -Wvla -Wstack-usage=256 -fstack-usage -fno-common`).
  - `./build/test_sm5800_drv` kết thúc với exit code `0` và `0 failed`.
  - Kiểm tra tệp `build/sm5800_drv.su`: mọi hàm có stack usage **≤ 256 byte**.

### Gate 4 — Biên dịch chéo ARM Cortex-A9 PS & Kiểm tra mã máy ISR
```bash
make build-arm
make build-arm64   # Zynq UltraScale+ Cortex-A53 (AArch64)
```
- **Tiêu chí đạt:**
  - Cả 3 tệp `sm5800_hal.c`, `sm5800_drv.c`, `sm5800_ps_example.c` biên dịch **0 warning / 0 error**.
  - Kiểm tra `build/sm5800_drv.dis` tại hàm `sm5800_irq_isr`: xác nhận **0 lệnh VFP/NEON** (không dấu phẩy động trong ISR), **0 vòng lặp**, có lệnh `dsb` sau ghi W1C và `dmb` trước khi thoát.

### Gate 5 — Rà soát kỹ thuật thủ công các điểm dễ lỗi (Deep Code Review)
Agent kiểm tra cần đọc trực tiếp mã nguồn và xác nhận các điểm cốt lõi sau:

| Hạng mục kiểm tra | Tệp cần đọc | Điểm phải xác nhận đúng |
|---|---|---|
| **Chống nhiễm CRC từ cờ đóng SDLC** | `rtl/sm5800_sdlc_rx.v` | `raw_sr_r[7:0]` tạo trễ 8 bit thô; khi `next_raw_w == 8'h7E`, bit cờ không đi qua `u_crc16`; `frame_end_o` trễ 1 chu kỳ sau `rx_byte_valid_o`. |
| **AXI4-Lite độc lập AW/W** | `rtl/sm5800_axi4_lite_if.v` | `s_axi_awready`, `s_axi_wready`, `s_axi_arready` đều registered; xử lý đúng cả 3 thứ tự `AW+W`, `AW trước W`, `W trước AW`; `INT_STATUS` W1C ưu tiên phần cứng. |
| **Tách bản tin 01 / 02 & Watchdog 100 Hz** | `rtl/sm5800_msg_parser.v` | `msg1_valid_o` & `msg2_valid_o` loại trừ lẫn nhau; khi `msg2_only_mode_i == 1`, Msg 01 không ghi đè `live_*`, watchdog chỉ reset khi `is_msg2_w`, `prev_seq_cnt_r` vẫn theo dõi cả Msg 1 & 2. |
| **Giải mã bù 2 không UB** | `driver/sm5800_drv.c` | `decode_s16_lsb()` xử lý `-32768` và `decode_s32_lsb()` xử lý `INT32_MIN` (`0x80000000U`) qua `int64_t` không dịch bit trên kiểu có dấu. |
| **Toán học quy đổi 1-lệnh-gọi** | `driver/sm5800_drv.c` | Kiểm tra lại hằng số: $2^{-20}\times 600 = 5.722045898\times 10^{-4}$ rad/s; $2^{-14}\times 600\times 0.3048 = 0.011162109375$ m/s²; $2^{-33} = 1.164153218\times 10^{-10}$ rad; $2^{-27}\times 0.3048 = 2.270936966\times 10^{-9}$ m/s. |
| **Tính toàn vẹn PDI & SEU** | `driver/sm5800_drv.h`, `.c` | `sm5800_state_t` có khoảng cách Hamming ≥ 15; `sm5800_config_t` kiểm tra `schema_version`, biên `timeout_us` (tách nhánh không coupled MC/DC) và `CRC-32` tuần tự byte (không ép kiểu struct). |
| **Ngoại lệ MISRA** | `driver/DEVIATION_LOG.md`, `driver/sm5800_hal.c` | Mọi chú thích `/* MISRA deviation: ... */` trong mã nguồn đều được ghi nhận đầy đủ trong `DEVIATION_LOG.md`, và không có ngoại lệ nào ngoài `sm5800_hal.c`. |

### Gate 6 — Đối chiếu tài liệu DO-178C
- Đối chiếu `docs/LLR_SM5800.md`, `docs/SDD_SM5800.md`, `docs/TEST_REPORT_SM5800.md` và `docs/README.md`:
  - Không có mâu thuẫn giữa bản đồ thanh ghi, chế độ 100 Hz (`MSG2_ONLY_MODE`), và tên API C.
  - Cập nhật cột trạng thái trong `docs/TEST_REPORT_SM5800.md` từ `PENDING_TARGET_EXEC` sang `PASS` / `FAIL` kèm nhật ký chạy thực tế nếu Agent chạy được Gate 2–4.

### Gate 7 — Mẫu báo cáo đầu ra chuẩn cho Agent kiểm tra
Khi hoàn tất rà soát, Agent kiểm tra xuất báo cáo theo cấu trúc ngắn gọn sau:

```markdown
## Independent Verification Summary — SM-5800 IMU Project
- Gate 1 (verify_project.ps1): [PASS / FAIL] — <chi tiết>
- Gate 2 (RTL Sim [T1]-[T18] & Lint): [PASS / FAIL / BLOCKED] — <công cụ & log>
- Gate 3 (Host C Unit Tests TC-DRV-01..17): [PASS / FAIL / BLOCKED] — <pass/fail count, max .su stack>
- Gate 4 (ARM GCC Build & ISR Disasm): [PASS / FAIL / BLOCKED] — <warning count, VFP check>
- Gate 5 (Manual Deep Review): [PASS / ISSUES FOUND] — <liệt kê file:line nếu có lỗi>
- Gate 6 (Docs & Traceability): [PASS / ISSUES FOUND] — <nhận xét LLR/SDD/SVR>
- Verdict: [APPROVED FOR HIL / REWORK REQUIRED]
```
