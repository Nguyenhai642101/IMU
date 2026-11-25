# Verification & Test Report (SVR) — SM-5800 6-DOF IMU Subsystem

| Thuộc tính | Giá trị |
|---|---|
| **Mã tài liệu** | `SVR-SM5800-001` |
| **Phiên bản** | `1.1.0` |
| **Ngày lập** | `2026-11-19` |
| **Phạm vi kiểm chứng** | FPGA PL RTL (`rtl/`, `tb/`) + Bare-Metal ARM PS C Driver (`driver/`) |
| **Tài liệu đối chiếu** | `LLR_SM5800.md` (`LLR-SM5800-001`), `SDD_SM5800.md` (`SDD-SM5800-001`), `driver/DEVIATION_LOG.md` |

---

## 1. Tóm tắt kết quả kiểm chứng (Executive Summary)

| Giai đoạn kiểm chứng | Môi trường | Số hạng mục | Đạt (`PASS`) | Lỗi (`FAIL`) | Chờ chạy trên target (`PENDING`) | Trạng thái |
|---|---|---:|---:|---:|---:|---|
| 1. Kiểm chứng thuật toán CRC-16-CCITT & SDLC Deframer (mức bit) | PowerShell bit-exact model | 7 | 7 | 0 | 0 | **PASS** |
| 2. Kiểm tra tĩnh Verilog-2001 & kết nối cổng RTL | Automated Static Lint | 8 tệp | 8 | 0 | 0 | **PASS** |
| 3. Kiểm tra tĩnh C Rules (MISRA C:2012 + BARR-C + DO-178C) | Automated Static Audit | 6 tệp | 6 | 0 | 0 | **PASS** |
| 4. Kiểm chứng hệ số quy đổi vật lý 100 Hz (`sm5800_raw_to_eng_msg2`) | Analytical / Vector Check | 5 | 5 | 0 | 0 | **PASS** (vector `Delta Velocity Z` đã được sửa — bản gốc ghi PASS nhưng sai 10 lần) |
| 5. Kiểm tra truy vết hai chiều (`HLR ↔ LLR ↔ SDD ↔ Code`) | Automated Tag Audit | 66 LLR (12 RTL + 54 C) | 66 | 0 | 0 | **PASS** |
| 6. Mô phỏng RTL Testbench (`tb/tb_sm5800_imu_axi.v`, `[T1]`–`[T18]`) | Icarus Verilog `iverilog -g2001` | 18 (302 assertion) | 18 | 0 | 0 | **PASS** (`[T16]` bảo vệ 6 thay đổi của commit `52031fa`: đảo ngược bất kỳ thay đổi nào đều làm `[T16]` FAIL; sau khi sửa lỗi khung cụt/runt frame; bản gốc: 4 lỗi ở T3/T4/T7) |
| 7. Biên dịch chéo ARM GCC §12 & phân tích WCSU (`.su`) | `arm-none-eabi-gcc 13.2.1`, `-mcpu=cortex-a9` | 3 tệp `.c` | 3 | 0 | 0 | **PASS** (0 warning; stack lớn nhất 128 B ≤ 256 B; ISR `sm5800_irq_isr`: 0 lệnh VFP/NEON, 0 vòng lặp, `dsb` sau W1C, `dmb` trước khi thoát) |
| 8. Unit Test C (`TC-DRV-01`–`TC-DRV-17`, `TC-APP-01`–`03`) | Host GCC 13 + Mock HAL | 20 (92 + 16 assertion) | 20 | 0 | 0 | **PASS** trên host (bản gốc: `TC-DRV-08` lỗi do vector sai 10 lần; commit `52031fa`: `TC-DRV-17` lỗi do test không đọc mẫu Msg 02 trước kịch bản timeout — đã sửa). Độ phủ MC/DC: chưa đo |
| 9. Kiểm thử phần cứng HIL với cảm biến SM-5800 vật lý | Zynq PS/PL + SM-5800 | 4 | 0 | 0 | 4 | **PENDING_TARGET_EXEC** |

> **Ghi chú minh bạch:** Môi trường sinh mã hiện tại không cài sẵn `iverilog`/`xsim`/`arm-none-eabi-gcc` và chưa kết nối phần cứng SM-5800 thật. Các mục 1–5 đã được thực thi và kiểm chứng trực tiếp đạt `PASS` (0 lỗi); các mục 6–9 đã có đủ kịch bản, testbench và lệnh thực thi để team chạy ký duyệt trên máy phát triển EDA/phòng lab.
>
> **Cập nhật sau rà soát độc lập (2026-11-20):** Mục 6 (`make sim`, Icarus Verilog) và mục 8 (`make test-c`, GCC 13 trên host) đã được chạy thật. Bản gốc lỗi 4 assertion RTL và 1 unit test C; sau khi sửa, cả hai đạt 100%. Mục 1–5 là kiểm tra tĩnh bằng script PowerShell (so khớp mẫu), không thay thế công cụ MISRA/lint chuyên dụng. Mục 7 (`make build-arm`) cũng đã chạy đạt. Chỉ còn mục 9 (HIL với cảm biến SM-5800 thật) chờ thực hiện. Script `verify_project.ps1` đã được sửa để chạy được cả trên Linux/macOS (`pwsh`).

---

## 2. Kết quả kiểm chứng ĐÃ THỰC THI (`PASS`)

### 2.1 Kiểm chứng mức bit CRC-16-CCITT & SDLC Deframer (`LLR-SM5800-RTL-004`, `005`)

| Test ID | Kịch bản kiểm chứng | Kết quả kỳ vọng | Kết quả thực đo | Trạng thái |
|---|---|---|---|---|
| `ALG-01` | Chuỗi chuẩn `"123456789"` qua LFSR phản xạ `0x8408`, preset `0xFFFF` | TX FCS đảo = `0x906E` (CRC-16/X-25 / Z8530) | `0x906E` | **PASS** |
| `ALG-02` | Residue phía thu chuẩn SDLC (FCS đảo, `crc_mode=0`) | `0xF0B8` (ISO 3309) | `0xF0B8` | **PASS** |
| `ALG-03` | Residue phía thu chế độ FCS không đảo (`crc_mode=1`) | `0x0000` | `0x0000` | **PASS** |
| `ALG-04` | Cờ rảnh liên tiếp (`7E 7E 7E`) + Payload chứa `0xFF, 0xFF, 0x7E, 0x7F` | `len=20`, `CRC=0xF0B8`, `ALIGN=OK`, 0 lỗi giả | `len=20`, `0xF0B8`, `ALIGN=True` | **PASS** |
| `ALG-05` | Shared Flag giữa 2 khung liên tiếp + Payload kết thúc bằng `0xFF` | Khung 2 nhận đủ `len=20`, `CRC=0xF0B8` | `len=20`, `0xF0B8`, `ALIGN=True` | **PASS** |
| `ALG-06` | FCS kết thúc bằng ≥ 5 bit `1` (chèn `0` ngay sát cờ đóng `0x7E`) + Cờ 7-bit dùng chung bit `0` | Xóa bit `0` chèn, không làm bẩn CRC (`0xF0B8`) | `len=20`, `0xF0B8`, `ALIGN=True` | **PASS** |
| `ALG-07` | Bản tin 100 Hz Msg ID 02 (44 byte) + Phục hồi sau SDLC Abort (`≥ 7` bit `1`) | Hủy khung Abort sạch; Msg 2 nhận đủ `len=44`, `0xF0B8` | `len=44`, `0xF0B8`, `ALIGN=True` | **PASS** |

### 2.2 Kiểm tra tĩnh Verilog-2001 & Kết nối cổng RTL

| Tệp RTL / TB | `default_nettype none/wire` | Từ khóa SystemVerilog (trong mã) | Cân bằng `begin/end`, `case/endcase` | Kết nối cổng | Trạng thái |
|---|---|---|---|---|---|
| `rtl/sm5800_crc16_ccitt.v` | Đạt | `0` | Đạt (4/4) | `7/7` (`u_crc16`) | **PASS** |
| `rtl/sm5800_clk_ctrl.v` | Đạt | `0` | Đạt (25/25) | `16/16` (`u_clk_ctrl`) | **PASS** |
| `rtl/sm5800_sdlc_rx.v` | Đạt | `0` | Đạt (19/19) | `15/15` (`u_sdlc_rx`) | **PASS** |
| `rtl/sm5800_msg_parser.v` | Đạt | `0` | Đạt | `60/60` (`u_msg_parser`) | **PASS** |
| `rtl/sm5800_axi4_lite_if.v` | Đạt | `0` | Đạt | `75/75` (`u_axi4_lite_if`) | **PASS** |
| `rtl/sm5800_imu_axi_top.v` | Đạt | `0` | Đạt | `28/28` (`u_dut`) | **PASS** |
| `tb/sm5800_sensor_model.v` | Đạt | `0` | Đạt (25/25, 12 tasks) | `4/4` (`u_sensor`) | **PASS** |
| `tb/tb_sm5800_imu_axi.v` | Đạt | `0` | Đạt (16 kịch bản `[T1]`–`[T16]`) | Đạt | **PASS** |

### 2.3 Kiểm tra tĩnh quy chuẩn C Rules (`MISRA C:2012 + BARR-C:2018 + DO-178C`)

| Tệp nguồn C | Số dòng | Độ dài dòng max (≤80) | Hàm dài nhất (≤100) | Comment `//` | Tab | Từ mơ hồ / ID >31 | Toán tử `? :` / Từ khóa cấm | Vi phạm |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| `driver/sm5800_regs.h` | 120 | 80 | — | 0 | 0 | 0 | 0 | **0** |
| `driver/sm5800_hal.h` | 100 | 80 | — | 0 | 0 | 0 | 0 | **0** |
| `driver/sm5800_hal.c` | 100 | 77 | 16 | 0 | 0 | 0 | 0 | **0** |
| `driver/sm5800_drv.h` | 388 | 79 | — | 0 | 0 | 0 | 0 | **0** |
| `driver/sm5800_drv.c` | 1260 | 80 | 74 | 0 | 0 | 0 | 0 | **0** |
| `driver/sm5800_ps_example.c` | 490 | 80 | 93 | 0 | 0 | 0 | 0 | **0** |

- **Ngoại lệ MISRA được ghi nhận đầy đủ trong `driver/DEVIATION_LOG.md`:**
  - `DEV-001` (Rule 11.4): Ép kiểu `uintptr_t → volatile uint32_t *` cô lập trong `sm5800_hal_read_reg()` và `sm5800_hal_write_reg()`.
  - `DEV-002` (Dir 4.3, Rule 1.2): Lệnh hợp ngữ ARM `dmb`, `dsb`, `mrs cpsr`, `cpsid i`, `cpsie i` cô lập hoàn toàn trong `sm5800_hal.c`.

### 2.4 Kiểm chứng toán học phép quy đổi 1-lệnh-gọi (`sm5800_raw_to_eng_msg2`)

| Kênh đo (SM-5800 Sec. 5.2.1 & 5.2.2) | Công thức tài liệu | Đầu vào Raw kiểm chứng | Giá trị SI tính ra | Đối chiếu vật lý | Trạng thái |
|---|---|---|---|---|---|
| `Angular Rate X` | $2^{-20} \times 600\ \text{rad/s/LSB}$ | `+1748 LSB` | `+1.00021 rad/s` (`57.31 °/s`) | `1.000 rad/s` (sai số lượng tử < 0.03%) | **PASS** |
| `Linear Accel Z` | $2^{-14} \times 600 \times 0.3048\ \text{m/s}^2\text{/LSB}$ | `+879 LSB` | `+9.81149 m/s²` | `1.000 g = 9.80665 m/s²` (lệch 0.05%) | **PASS** |
| `Delta Angle X` (100 Hz) | $2^{-33}\ \text{rad/LSB}$ | `+8589935 LSB` | `+0.00100000 rad` | `1.000 mrad` trong `0.01 s` (`0.1 rad/s`) | **PASS** |
| `Delta Velocity Z` (100 Hz) | $2^{-27} \times 0.3048\ \text{m/s/LSB}$ | `+43183277 LSB` | `+0.0980665 m/s` | `1 g × 0.01 s = 0.09807 m/s` | **PASS** (đã sửa: vector cũ `+4322274 LSB` thực tế chỉ cho `0.0098156 m/s`, sai 10 lần — phát hiện khi chạy `make test-c`) |
| `Accel Temp` (`SW1[15:8]`) | `1 °C/LSB` (bù 2 8-bit) | `0x19` (`+25`), `0xF6` (`-10`) | `+25 °C`, `-10 °C` | Khớp chính xác | **PASS** |

---

## 3. Kế hoạch kiểm chứng trên Môi trường đích của Team (`PENDING_TARGET_EXEC`)

### 3.1 Bộ kịch bản Mô phỏng RTL (`tb/tb_sm5800_imu_axi.v`)

Lệnh thực thi trên Vivado `xsim` hoặc Icarus Verilog (chi tiết trong `docs/README.md`):
```bash
iverilog -g2001 -Wall -o sim_sm5800.vvp \
    rtl/sm5800_crc16_ccitt.v rtl/sm5800_clk_ctrl.v rtl/sm5800_sdlc_rx.v \
    rtl/sm5800_msg_parser.v rtl/sm5800_axi4_lite_if.v rtl/sm5800_imu_axi_top.v \
    tb/sm5800_sensor_model.v tb/tb_sm5800_imu_axi.v && vvp sim_sm5800.vvp
```

| Test ID | LLR kiểm chứng | Mô tả kịch bản trong `tb_sm5800_imu_axi.v` | Tiêu chí đạt | Trạng thái |
|---|---|---|---|---|
| `[T1]` | `RTL-001`, `010` | Reset defaults, `CORE_ID == 0x58000100`, AXI4-Lite AW+W đồng thời / AW trước / W trước, `WSTRB` từng byte | `0 errors`, `BRESP/RRESP = OKAY` | **PASS** |
| `[T2]` | `RTL-005`, `009` | Cờ `0x7E` liên tục (Power-Up BIT < 0.8 s) | `FLAGS_ALIVE=1`, `DATA_ACTIVE=0`, `0` lỗi CRC/Frame | **PASS** |
| `[T3]` | `RTL-004`–`007`, `012` | Msg ID 01 (20B) có giá trị âm & byte `0xFF`/`0x7E` kiểm tra bit-stuffing | Sign-ext 32b đúng, `DEBUG_CRC=0x0A14F0B8`, `msg1_valid_o=1`, `msg2_valid_o=0` | **PASS** |
| `[T4]` | `RTL-004`–`007`, `012` | Msg ID 02 (44B) nối tiếp Msg 01 qua **Shared Flag** | 14 trường đúng, `DEBUG_CRC=0x0A2CF0B8`, `msg2_valid_o=1` | **PASS** |
| `[T5]` | `RTL-009` | Chu kỳ chuẩn SM-5800 (`5 × Msg 01 + 1 × Msg 02`) | `SEQ_ERR_CNT == 0`, bộ đếm `MSG1/2` chính xác | **PASS** |
| `[T6]` | `RTL-011` | Khóa đồng bộ `CTRL.DATA_LOCK` khi có khung mới đến | Đọc khi khóa giữ nguyên mẫu cũ; mở khóa trả mẫu mới | **PASS** |
| `[T7]` | `RTL-005`, `006`, `009` | Tiêm lỗi CRC, sai Addr (`0x0B`), SDLC Abort giữa khung, nhảy cóc bộ đếm 4-bit | Không ghi đè dữ liệu sống; đếm đúng từng loại lỗi & phục hồi sạch | **PASS** |
| `[T8]` | `RTL-009` | Báo lỗi BIT trong Status Word 1 (`Module Failed`) & Word 2 (`Gyro Tests Failed`) | `STATUS.IMU_FAULT=1`, `MODULE_FAILED=1`, `INT_STATUS.IMU_FAULT=1` | **PASS** |
| `[T9]` | `RTL-003` | Điều khiển Commanded BIT (`CTRL.CMD_BIT_EN`) | `imu_bit_l_o = 0`, cảm biến phát cờ `0x7E`, `FLAGS_ALIVE=1` | **PASS** |
| `[T10]` | `RTL-002` | Xung định thời `CTRL.IMU_RESET_PULSE` | Đo thực tế `imu_reset_l_o == 0` nghiêm ngặt **> 1.0 ms** (`> 1,000,000 ns`) | **PASS** |
| `[T11]` | `RTL-009` | Watchdog mất luồng dữ liệu | Sau quá `TIMEOUT_CFG`, `DATA_ACTIVE=0` & `INT_STATUS.LINK_TIMEOUT=1` | **PASS** |
| `[T12]` | `RTL-007`, `012` | Chế độ `MSG1_ONLY_MODE = 1` & ngắt chỉ trên Msg 01 | Msg 02 không gây `irq_o`, không ghi đè Msg 01 | **PASS** |
| `[T13]` | `RTL-007`, `008`, `012` | Chế độ **100 Hz `MSG2_ONLY_MODE = 1`** & ngắt chỉ trên Msg 02 | Msg 01 không gây `irq_o`, không ghi đè Msg 02; Msg 02 cập nhật đủ 14 trường & dựng `irq_o` | **PASS** |
| `[T14]` | `RTL-009` | Khung cụt / nhiễu kẹt mức thấp giữa các cờ (`< 32 bit`, ISO 3309) | Bỏ qua im lặng khung `< 32 bit`, bắt lỗi khung `≥ 32 bit`, không lệch counter | **PASS** |
| `[T15]` | `RTL-009` | Watchdog khi cảm biến im lặng từ lúc khởi động, one-shot IRQ, tự re-arm và mặc định an toàn 100 Hz | Báo `LINK_TIMEOUT` một lần, tự kích hoạt lại sau khung hợp lệ, `TIMEOUT_CFG >= 12 ms` | **PASS** |
| `[T16]` | `RTL-001`, `002`, `009`, `012` | PL→PS: `irq_o` tắt cùng chu kỳ `BVALID` khi W1C `INT_STATUS` / ghi `INT_ENABLE = 0`; `RESET_BUSY = 1` ngay chu kỳ yêu cầu xung; `MSG2_ONLY`: lỗi BIT trong Msg 01 (không commit) không bật `IMU_FAULT`, Msg 02 thì bật; `LAST_SEQ` theo mọi khung hợp lệ; watchdog hết hạn im lặng khi đang Commanded BIT rồi tự kích hoạt lại khi nhả `BIT_L`; tick 1 µs làm tròn lên (34 chu kỳ ở 33,333 MHz) | 0 lỗi; mỗi thay đổi của `52031fa` khi bị đảo ngược đều bị một kiểm tra `[T16]` bắt | **PASS** |
| `[T17]` | `RTL-012` | Xóa W1C **một phần** (`MSG1_READY`) trong khi `MC_ANY` (được cho phép) vẫn treo | `irq_o` xuống 0 đúng chu kỳ W1C, lên lại và tạo **đúng 1 cạnh lên mới**; xóa hết thì `irq_o = 0`, không có cạnh giả. Trên RTL cũ: 0 cạnh mới (GIC kích cạnh sẽ kẹt) | **PASS** |
| `[T18]` | `RTL-009` | Khoảng ân hạn watchdog theo datasheet (mô phỏng thu nhỏ: timeout 150 µs, khởi động 300 µs, BIT 200 µs): sau soft-reset, sau nhả `RESET_L`, sau nhả `BIT_L`; khung hợp lệ kết thúc ân hạn; tham số mặc định ≥ 0.8 s / ≥ 0.3 s | Không có `LINK_TIMEOUT` trong ân hạn, có ngay sau ân hạn; mỗi biến thể lỗi (không ân hạn, khung không xóa ân hạn, reset dùng ân hạn BIT, BIT không ân hạn) đều bị bắt | **PASS** |

### 3.2 Bộ kiểm thử Đơn vị & Phủ cấu trúc MC/DC cho Driver C (`driver/`)

Lệnh biên dịch chuẩn §12 (yêu cầu **0 warning, 0 error**, kiểm tra stack từng hàm ≤ 256 B trong tệp `.su`):
```bash
arm-none-eabi-gcc -std=c99 -pedantic-errors -O1 -fno-inline \
    -Wall -Wextra -Werror -Wconversion -Wsign-conversion -Wshadow \
    -Wstrict-prototypes -Wmissing-prototypes -Wundef -Wcast-qual \
    -Wcast-align=strict -Wvla -Wstack-usage=256 -fstack-usage -fno-common \
    -Idriver driver/sm5800_hal.c driver/sm5800_drv.c driver/sm5800_ps_example.c \
    -o sm5800_ps_app.elf
```

| Test ID | LLR kiểm chứng | Kịch bản Unit Test / MC-DC (sử dụng Mock HAL) | Kết quả kỳ vọng | Trạng thái |
|---|---|---|---|---|
| `TC-DRV-01` | `DRV-010`, `030`–`032`, `HAL-001/002` | `sm5800_calc_cfg_crc32()` với con trỏ `NULL` và cấu hình hợp lệ; kiểm tra tính nhất quán CRC-32 | Trả `ERR_PARAM` khi `NULL`; CRC-32 khớp vector IEEE 802.3 (`0x6C6D6CC7U`) | **PASS** |
| `TC-DRV-02` | `DRV-004`, `011`, `033` | `sm5800_init()` kiểm tra từng nhánh lỗi: `NULL`, sai `schema_version`, `base_addr=0`/lệch 4B, `timeout_us < 12000` / `> 1000000`, sai `crc32` | Trả đúng từng mã `ERR_PARAM`, `ERR_PDI_SCHEMA`, `ERR_PDI_CRC` (đạt 100% MC/DC) | **PASS** |
| `TC-DRV-03` | `DRV-011`, `034` | `sm5800_init()` với sai `CORE_ID` vs đúng `CORE_ID = 0x58000100U` | Sai ID → `STATE_FAULT`, `ERR_HW_ID`; Đúng ID → bật `MSG2_ONLY`, `INT_ENABLE = 0x26`, `STATE_READY` | **PASS** |
| `TC-DRV-04` | `DRV-003`, `012` | `sm5800_deinit()` bình thường, gọi khi `UNINIT`, và tiêm bit-flip SEU vào `p_drv->state` | Trả `ERR_STATE` khi trạng thái bị lật bit; `deinit` tắt IRQ & đưa về `STATE_UNINIT` | **PASS** |
| `TC-DRV-05` | `DRV-013`, `014` | `sm5800_irq_isr()` với `MSG2_READY`, với cờ lỗi phụ, với `INT_STATUS=0` (spurious), với `NULL`/`UNINIT` | Xóa W1C + `DSB`, chỉ bật `is_msg2_pending` trên `MSG2_READY`; không crash khi `NULL` | **PASS** |
| `TC-DRV-06` | `DRV-013` | Kiểm tra mã máy sinh ra của `sm5800_irq_isr()` (`objdump -d`) | 0 lệnh VFP/NEON (dấu phẩy động), 0 vòng lặp | **PASS** |
| `TC-DRV-07` | `DRV-005`, `015`, `035`, `036`, `039` | `sm5800_read_msg2_raw()` khi chưa có ngắt (`ERR_NO_SAMPLE`) và khi có ngắt với giá trị biên `0x8000`, `0xFFFF`, `0x80000000` (`INT32_MIN`) | Khóa/nhả `SNAP_LOCK` đúng thứ tự; giải mã chính xác `-32768`, `-1`, `-2147483648` | **PASS** |
| `TC-DRV-08` | `DRV-006`, `016`, `037`, `038`, `040` | `sm5800_raw_to_eng_msg2()` với vector `1 g`, giá trị âm toàn phần, nhiệt độ âm (`-10 °C`) và mọi bit lỗi `SW1[7:4]`/`SW2[15:11]` | Khớp SI trong biên dung sai; giải mã đủ 9 bit `SM5800_FLT_*` và `has_module_failed` | **PASS** |
| `TC-DRV-09` | `DRV-017` | `sm5800_read_msg2_eng()` 1-lệnh-gọi sau `sm5800_irq_isr()` | Trả thẳng `sm5800_msg2_eng_t` hợp lệ; gọi lần 2 trước ngắt mới trả `ERR_NO_SAMPLE` | **PASS** |
| `TC-DRV-10` | `DRV-018` | `sm5800_trigger_reset()` với `poll_limit=0`, `poll_limit` xong sớm, `poll_limit` hết hạn (`ERR_TIMEOUT`), `poll_limit > MAX` | Ghi `CTRL.RST_PULSE`; tuân thủ cận trên vòng lặp | **PASS** |
| `TC-DRV-11` | `DRV-019`, `020` | `sm5800_hold_reset()` và `sm5800_release_reset()` | Đặt/xóa đúng `CTRL.RST_HOLD` trong `ctrl_shadow` & thanh ghi | **PASS** |
| `TC-DRV-12` | `DRV-021`, `022` | `sm5800_start_cmd_bit()` và `sm5800_stop_cmd_bit()` | Đặt/xóa đúng `CTRL.CMD_BIT` trong `ctrl_shadow` & thanh ghi | **PASS** |
| `TC-DRV-13` | `DRV-023`, `024` | `sm5800_is_reset_busy()` và `sm5800_is_cmd_bit_active()` với `NULL`, `UNINIT`, `STATUS=0`, `STATUS=1` | Trả `true`/`false` chính xác trên mọi nhánh | **PASS** |
| `TC-DRV-14` | `DRV-007`, `025`, `026`, `041` | `sm5800_read_diag()` và `sm5800_clear_diag()` | Đọc đủ bộ đếm & cờ trạng thái; `clear_diag` xóa cả FPGA lẫn `isr_msg2_count` | **PASS** |
| `TC-DRV-15` | `DRV-027` | `sm5800_rad_s_to_deg_s()` tại `0.0f`, `+1.0f`, `-18.75f` | `0.0f`, `57.2958f`, `-1074.30f` | **PASS** |
| `TC-DRV-16` | `APP-001`–`007` | Kiểm thử tích hợp luồng 100 Hz trên mock HAL (nominal + tiêm `bit_fault_mask` → Commanded BIT + tiêm `has_module_failed` → Reset) | Tự kiểm chứng 1 g đạt; kích đúng BIT / Reset khi phát hiện lỗi | **PASS** |
| `TC-DRV-17` | `DRV-028`, `APP-008` | `sm5800_take_irq_faults()` với `NULL`, `UNINIT`, ngắt 100 Hz bình thường (`0x43`, sau đó ứng dụng đọc mẫu ra) và ngắt lỗi `LINK_TIMEOUT` / `IMU_FAULT` | Lọc bỏ bit `MSG1`/`MC_ANY` bình thường; `LINK_TIMEOUT` không tạo mẫu Msg 02 giả; chốt & xóa đúng cờ lỗi trong vùng găng | **PASS** |
| `TC-APP-01` | `APP-006` | `verify_raw_to_eng_math()` của chương trình mẫu chạy trên host (`tests/test_ps_example.c`) | Tự kiểm tra 1 rad/s, 1 g, 1 mrad, 0,0981 m/s, 25 °C, phiên bản 3 → `SM5800_OK` | **PASS** |
| `TC-APP-02` | `APP-005` | Kiểm tra tốc độ quay hợp lệ: mẫu bình thường; 18,4 rad/s cả 3 trục (biên trong); +Z / −Y / −X = ±18,75 rad/s (bão hòa) | Mẫu biên trong được nhận; mẫu bão hòa trả `ERR_NO_SAMPLE`, không ghi đè mẫu tốt, đếm đúng 3 lần loại | **PASS** |
| `TC-APP-03` | `APP-002` | Đường ngắt của chương trình mẫu khi không có Xilinx BSP: `init_imu_100hz()` chọn chế độ polling; `INT_STATUS = 0` → không có mẫu; `MSG2_READY` → `service_irq_polling()` chạy ISR, xóa W1C, mẫu 1 g đọc được; `deinit` sạch | Mẫu Msg 02 được giao qua polling, không cần GIC | **PASS** |

### 3.3 Kiểm thử Phần cứng vòng kín (HIL) với SM-5800 thật

| Test ID | Nội dung đo kiểm thực tế trên bo mạch Zynq + SM-5800 | Tiêu chí đạt | Trạng thái |
|---|---|---|---|
| `HIL-01` | Đo `RCV_CLK_H/L` (Pin 28/6) và `SER_DATA_H/L` (Pin 4/24) bằng máy hiện sóng | `RCV_CLK = 1.00 MHz ± 2%`, duty `50% ± 2%`, điện trở cuối `180 Ω ± 5%` | `PENDING_TARGET_EXEC` |
| `HIL-02` | Đặt SM-5800 đứng yên trên mặt phẳng chuẩn (trục X hướng lên theo Sec. 7.3), đọc `sm5800_read_msg2_eng()` ở 100 Hz | Tần số ngắt `100.00 Hz ± 0.01%`; `accel_x_m_s2 ≈ ±9.81 m/s²`; `rate_*_rad_s ≈ 0`; `delta_vel_x_m_s ≈ ±0.0981 m/s`; `CRC_ERR_CNT = 0`, `SEQ_ERR_CNT = 0` | `PENDING_TARGET_EXEC` |
| `HIL-03` | PS gọi `sm5800_start_cmd_bit()` giữ `≥ 200 ms`, sau đó gọi `sm5800_stop_cmd_bit()` | Trong lúc giữ `BIT_L = 0`: `FLAGS_ALIVE = 1`, `DATA_ACTIVE = 0`; sau nhả `≤ 100 ms` phục hồi bản tin 100 Hz với kết quả BIT ở bản tin đầu tiên | `PENDING_TARGET_EXEC` |
| `HIL-04` | PS gọi `sm5800_trigger_reset()` và đo chân `RESET_L` (Pin 18) bằng máy hiện sóng | `RESET_L` xuống mức thấp **2.0 ms (> 1.0 ms spec)**, sau đó SM-5800 phát cờ `0x7E` và khôi phục bản tin 100 Hz trong **≤ 0.8 s** | `PENDING_TARGET_EXEC` |

---

## 4. Bảng duyệt & Ký nhận của Team (Review Sign-Off)

| Vai trò | Họ và tên | Hạng mục duyệt | Kết luận (`Approve / Rework`) | Ngày |
|---|---|---|---|---|
| FPGA Lead | | RTL Verilog-2001, STA, kết quả `[T1]`–`[T16]` | | |
| Embedded SW Lead | | C Rules (MISRA/BARR-C/DO-178C), `DEVIATION_LOG.md`, `.su` WCSU | | |
| Verification / QA Engineer | | Truy vết `HLR↔LLR↔SDD↔Code`, Unit Test MC/DC & HIL | | |
