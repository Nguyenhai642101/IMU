# Low-Level Requirements Specification (LLR) — SM-5800 6-DOF IMU Subsystem

| Thuộc tính | Giá trị |
|---|---|
| **Mã tài liệu** | `LLR-SM5800-001` |
| **Phiên bản** | `1.1.0` |
| **Ngày lập** | `2026-11-19` |
| **Tiêu chuẩn áp dụng** | DO-178C (DAL A/B), MISRA C:2012, BARR-C:2018, AMBA AXI4-Lite (ARM IHI 0022E), IEEE 1364-2001 |
| **Tài liệu tham chiếu** | *SM-5800 6-DOF Motion Sensor Module Installation and Interface User's Manual* (Sec. 4.0–6.4) |
| **Phạm vi** | FPGA PL IP Core (`rtl/`) + Bare-Metal ARM PS Driver (`driver/`) — chế độ bản tin **100 Hz (Message ID 02)** |

---

## 1. Yêu cầu cấp cao (High-Level Requirements — HLR)

| HLR ID | Mô tả yêu cầu cấp cao | Nguồn tham chiếu |
|---|---|---|
| `HLR-SM5800-01` | FPGA PL phải cấp xung `RCV_CLK` 1.0 MHz ± 2% và thu giải mã khung dữ liệu SDLC RS-422 từ SM-5800 kèm kiểm tra CRC-16-CCITT ($X^{16}+X^{12}+X^5+1$, preset `0xFFFF`). | SM-5800 Sec. 5.0–5.4 |
| `HLR-SM5800-02` | FPGA PL phải phân biệt độc lập bản tin số `01` (600 Hz, 20 byte) và bản tin số `02` (100 Hz, 44 byte) trên 2 chân đầu ra riêng, hỗ trợ chế độ chỉ cập nhật & ngắt PS theo bản tin `02` (100 Hz) qua bus AMBA AXI4-Lite. | Yêu cầu hệ thống & Sec. 5.1–5.2 |
| `HLR-SM5800-03` | Hệ thống phải cung cấp đường điều khiển từ PS → FPGA → SM-5800 cho tín hiệu `RESET_L` (open-collector, mức thấp > 1 ms) và `BIT_L` (Commanded BIT) nhằm xử lý/phục hồi khi phát hiện lỗi trong thanh ghi trạng thái cảm biến. | Yêu cầu hệ thống & Sec. 4.3.2, 4.4, 6.3 |
| `HLR-SM5800-04` | Driver C lớp PS phải đọc dữ liệu bản tin 100 Hz theo ngắt `irq_o` không bị xé dữ liệu (coherent snapshot), cung cấp hàm 1-lệnh-gọi quy đổi giá trị raw sang đơn vị vật lý SI để người dùng đọc và kiểm chứng trực tiếp. | Yêu cầu hệ thống & Sec. 5.2.1–5.2.2 |
| `HLR-SM5800-05` | Toàn bộ mã nguồn C lớp PS phải tuân thủ nghiêm ngặt quy chuẩn MISRA C:2012 + BARR-C:2018 + DO-178C (PDI CRC-32, trạng thái chống SEU, vòng lặp có cận trên WCET, truy vết 2 chiều `@req`/`@design`). | C Coding Rules §0–§14 |

---

## 2. Yêu cầu mức thấp — Khối FPGA PL (RTL)

| LLR ID | HLR cha | SDD | Mô tả yêu cầu chi tiết | Phương pháp kiểm chứng |
|---|---|---|---|---|
| `LLR-SM5800-RTL-001` | `HLR-SM5800-01` | `SDD-SM5800-1.1` | Khối `sm5800_clk_ctrl` phải tạo xung `imu_rcv_clk_o` tần số `1.0 MHz ± 2%`, chu kỳ nhiệm vụ `50% ± 2%` từ `s_axi_aclk`, đồng bộ hóa `imu_ser_data_i` qua chuỗi 2-FF `ASYNC_REG` và lấy mẫu NRZ theo sườn lên/xuống cấu hình được (`sample_edge_i`). | Simulation (`[T1]`–`[T4]`, `[T16]`), Static Timing |
| `LLR-SM5800-RTL-002` | `HLR-SM5800-03` | `SDD-SM5800-1.1` | Khi nhận xung `reset_pulse_req_i`, `sm5800_clk_ctrl` phải kéo `imu_reset_l_o = 0` trong thời gian **nghiêm ngặt > 1.0 ms** (`C_RESET_PULSE_US + 1 µs`, mặc định `2001 µs`), đồng thời hỗ trợ giữ thủ công `reset_hold_i` và thả nổi `imu_reset_l_o = 1` khi FPGA reset để SM-5800 tự reset khi cấp nguồn. | Simulation (`[T10]`, `[T16]`), Oscilloscope HIL |
| `LLR-SM5800-RTL-003` | `HLR-SM5800-03` | `SDD-SM5800-1.1` | Khối `sm5800_clk_ctrl` phải xuất `imu_bit_l_o = ~bit_cmd_i` (mặc định `1` khi reset) để điều khiển chân `BIT_L` kích hoạt Commanded BIT từ PS. | Simulation (`[T9]`), HIL |
| `LLR-SM5800-RTL-004` | `HLR-SM5800-01` | `SDD-SM5800-1.2` | Khối `sm5800_crc16_ccitt` phải tính CRC-16-CCITT bit-serial theo đa thức $X^{16}+X^{12}+X^5+1$ (phản xạ `0x8408`), khởi tạo `0xFFFF`, dịch LSB-first và xuất cả `crc_o` (registered) lẫn `crc_next_o` (combinational). | Bit-exact PowerShell check, Simulation (`[T3]`, `[T4]`, `[T7]`) |
| `LLR-SM5800-RTL-005` | `HLR-SM5800-01` | `SDD-SM5800-1.3` | Khối `sm5800_sdlc_rx` phải dùng pipeline lookahead 8-bit tách cờ `0x7E` sao cho bit cờ đóng không làm bẩn dữ liệu/CRC; bỏ bit chèn `0` sau 5 bit `1`; hỗ trợ cờ rảnh liên tiếp (`7E 7E...`), Shared Flag giữa 2 khung; phát hiện Abort/Idle (`≥ 7` bit `1`); kiểm tra căn chỉnh 8-bit và residue `0xF0B8` (chuẩn Z8530) hoặc `0x0000`. | Bit-exact PowerShell check, Simulation (`[T2]`–`[T4]`, `[T7]`) |
| `LLR-SM5800-RTL-006` | `HLR-SM5800-02` | `SDD-SM5800-1.4` | Khối `sm5800_msg_parser` phải xác thực `Addr == 0x0A`, độ dài 20B (Msg ID `0x01`) hoặc 44B (Msg ID `0x02`), căn chỉnh octet và CRC hợp lệ; loại bỏ hoàn toàn khung lỗi mà không ghi đè thanh ghi dữ liệu sống. | Simulation (`[T3]`, `[T4]`, `[T7]`) |
| `LLR-SM5800-RTL-007` | `HLR-SM5800-02` | `SDD-SM5800-1.4` | Khối `sm5800_msg_parser` phải phân biệt rõ bản tin `01` và bản tin `02` bằng 2 chân đầu ra độc lập **`msg1_valid_o`** và **`msg2_valid_o`** (cũng được xuất ra top-level `sm5800_imu_axi_top`). | Simulation (`[T3]`, `[T4]`, `[T12]`, `[T13]`) |
| `LLR-SM5800-RTL-008` | `HLR-SM5800-02` | `SDD-SM5800-1.4` | Khi `msg2_only_mode_i == 1`, `sm5800_msg_parser` chỉ cập nhật đồng thời (atomic) toàn bộ 14 trường (Motion Control + Spatial + Status Word 1/2 + Timestamp) và reset watchdog khi nhận bản tin `02` (100 Hz); bản tin `01` không ghi đè dữ liệu `02` nhưng vẫn xuất `msg1_valid_o`, đếm `msg1_rx_cnt_o` và theo dõi bộ đếm 4-bit liên tục. | Simulation (`[T13]`) |
| `LLR-SM5800-RTL-009` | `HLR-SM5800-01` | `SDD-SM5800-1.4` | Khối `sm5800_msg_parser` phải phát hiện nhảy cóc bộ đếm 4-bit (`Status Word 1[3:0]`), giải mã cờ lỗi BIT (`SW1[7:4]`, `SW2[15:11]`), giám sát cờ `0x7E` sống (`2000 µs`) và watchdog mất dữ liệu cấu hình được (`timeout_us_i`, tạm dừng báo lỗi giả khi đang Reset/Commanded BIT; được kích hoạt từ lúc reset để phát hiện cảm biến không bao giờ gửi dữ liệu, báo một lần rồi tự kích hoạt lại khi có khung hợp lệ). Ngưỡng timeout được cộng thêm **khoảng ân hạn theo datasheet** cho tới khung hợp lệ đầu tiên: `C_ACTIVATION_GRACE_US` (mặc định 1 s ≥ 0.8 s kích hoạt, Sec. 5.1/6.4) sau reset FPGA, soft-reset và khi nhả `RESET_L`; `C_BIT_GRACE_US` (mặc định 0.3 s = 0.2 s BIT + 0.1 s phục hồi, Sec. 6.3.2) khi nhả `BIT_L`. Khung ngắn hơn 32 bit giữa hai cờ bị bỏ qua im lặng (ISO 3309). | Simulation (`[T5]`, `[T7]`, `[T8]`, `[T9]`, `[T11]`, `[T14]`, `[T15]`, `[T16]`, `[T18]`) |
| `LLR-SM5800-RTL-010` | `HLR-SM5800-02` | `SDD-SM5800-1.5` | Khối `sm5800_axi4_lite_if` phải tuân thủ đầy đủ AMBA AXI4-Lite: chấp nhận `AW`/`W` mọi thứ tự (đồng thời, AW trước, W trước), không đường tổ hợp `VALID → READY`, hỗ trợ `WSTRB[3:0]`, và giữ ổn định `RDATA`/`BRESP` tới khi bắt tay. | Simulation (`[T1]`) |
| `LLR-SM5800-RTL-011` | `HLR-SM5800-04` | `SDD-SM5800-1.5` | Khi `CTRL.DATA_LOCK` chuyển `0 → 1`, `sm5800_axi4_lite_if` phải chốt nguyên tử (atomic snapshot) toàn bộ thanh ghi Motion Control, Spatial, Status Word 1/2 và Timestamp vào thanh ghi bóng để PS đọc nhiều thanh ghi không bị xé dữ liệu. | Simulation (`[T6]`) |
| `LLR-SM5800-RTL-012` | `HLR-SM5800-02` | `SDD-SM5800-1.5` | `INT_STATUS[6:0]` phải hỗ trợ W1C ưu tiên phần cứng (không mất sự kiện đồng thời), tách riêng bit `[0] MSG1_READY` và bit `[1] MSG2_READY`, và xuất ngắt `irq_o` đã đăng ký (glitch-free) khi `|(INT_STATUS & INT_ENABLE)`; mỗi lần ghi W1C vào `INT_STATUS` ép `irq_o = 0` trong 1 chu kỳ để cờ còn treo tạo **cạnh lên mới** (an toàn cho GIC kích cạnh lẫn kích mức). | Simulation (`[T3]`, `[T4]`, `[T12]`, `[T13]`, `[T16]`, `[T17]`) |

---

## 3. Yêu cầu mức thấp — Thanh ghi & HAL (PS)

| LLR ID | HLR cha | SDD | File / Hàm | Mô tả yêu cầu chi tiết | Kiểm chứng |
|---|---|---|---|---|---|
| `LLR-SM5800-REG-001` | `HLR-SM5800-02` | `SDD-SM5800-2.1` | `sm5800_regs.h` | Định nghĩa đầy đủ địa chỉ byte `0x00`–`0x6C`, hằng số `SM5800_CORE_ID_EXPECTED = 0x58000100U` và toàn bộ mặt nạ `_MASK`/`_SHIFT` không dùng C bit-field. | Static Review, Unit Test |
| `LLR-SM5800-HAL-001` | `HLR-SM5800-05` | `SDD-SM5800-2.2.1` | `sm5800_hal_read_reg()` | Đọc thanh ghi AXI4-Lite 32-bit qua con trỏ `uint32_t const volatile *` duy nhất trong HAL (kèm ghi nhận `DEV-001`), 1 truy cập `volatile` duy nhất. | Unit Test (`TC-DRV-01`) |
| `LLR-SM5800-HAL-002` | `HLR-SM5800-05` | `SDD-SM5800-2.2.2` | `sm5800_hal_write_reg()` | Ghi thanh ghi AXI4-Lite 32-bit qua con trỏ `uint32_t volatile *` duy nhất trong HAL (`DEV-001`), 1 truy cập `volatile` duy nhất. | Unit Test (`TC-DRV-01`) |
| `LLR-SM5800-HAL-003` | `HLR-SM5800-05` | `SDD-SM5800-2.2.3` | `sm5800_hal_dmb()` | Thực thi lệnh rào cản bộ nhớ `DMB` đảm bảo thứ tự dữ liệu giữa ngữ cảnh ISR và tác vụ chính (`DEV-002`). | Code Inspection, HIL |
| `LLR-SM5800-HAL-004` | `HLR-SM5800-05` | `SDD-SM5800-2.2.4` | `sm5800_hal_dsb()` | Thực thi lệnh đồng bộ `DSB` đảm bảo lệnh ghi AXI4-Lite (xóa ngắt W1C, khóa `SNAP_LOCK`, lệnh `RESET_L`/`BIT_L`) hoàn tất trước khi tiếp tục (`DEV-002`). | Code Inspection, HIL |
| `LLR-SM5800-HAL-005` | `HLR-SM5800-05` | `SDD-SM5800-2.2.5` | `sm5800_hal_enter_crit()`, `sm5800_hal_exit_crit()` | Lưu trạng thái bit IRQ (`CPSR.I` trên ARMv7 / `DAIF.I` trên AArch64) và tắt ngắt cục bộ có hỗ trợ lồng nhau (reentrant), khôi phục đúng trạng thái cũ khi thoát (`DEV-002`). | Code Inspection, HIL |
| `LLR-SM5800-HAL-006` | `HLR-SM5800-05` | `SDD-SM5800-2.2.6` | `SM5800_ASSERT()`, `sm5800_hal_assert_fail()` | Kiểm tra hợp đồng nội bộ ở bản debug và biên dịch thành `((void)0)` ở bản release/DO-178C để không tạo nhánh chết phủ cấu trúc. | Static Audit, Build Check |

---

## 4. Yêu cầu mức thấp — Driver C lớp PS (`sm5800_drv.h` / `.c`)

### 4.1 Kiểu dữ liệu & Tính toàn vẹn cấu hình

| LLR ID | HLR cha | SDD | Đối tượng | Mô tả yêu cầu chi tiết | Kiểm chứng |
|---|---|---|---|---|---|
| `LLR-SM5800-DRV-001` | `HLR-SM5800-04` | `SDD-SM5800-3.0` | Module `sm5800_drv` | Cung cấp API điều khiển và thu nhận bản tin 100 Hz (Msg ID 02) cho PS không dùng biến toàn cục `extern`, không cấp phát động. | Static Audit |
| `LLR-SM5800-DRV-002` | `HLR-SM5800-05` | `SDD-SM5800-3.1` | `sm5800_status_t` | Định nghĩa mã trạng thái với `SM5800_OK = 0` và 7 mã lỗi dương tường minh (`PARAM`, `STATE`, `PDI_SCHEMA`, `PDI_CRC`, `HW_ID`, `NO_SAMPLE`, `TIMEOUT`). | Code Inspection |
| `LLR-SM5800-DRV-003` | `HLR-SM5800-05` | `SDD-SM5800-3.1` | `sm5800_state_t` | Trạng thái vòng đời driver mã hóa 31-bit dương có khoảng cách Hamming ≥ 15 (`UNINIT=0x25C3691E`, `READY=0x5A3C16E1`, `FAULT=0x3C5A916E`) chống lật bit SEU. | Unit Test (`TC-DRV-04`) |
| `LLR-SM5800-DRV-004` | `HLR-SM5800-05` | `SDD-SM5800-3.2` | `sm5800_config_t` | Khối cấu hình PDI chứa `schema_version = 0x00010100U`, `base_addr`, `timeout_us` (`12000..1000000`), cờ cấu hình và `crc32`. | Unit Test (`TC-DRV-02`) |
| `LLR-SM5800-DRV-005` | `HLR-SM5800-04` | `SDD-SM5800-3.2` | `sm5800_msg2_raw_t` | Cấu trúc lưu trọn vẹn 14 trường raw LSB + `timestamp_us` của bản tin 100 Hz (kích thước `56 B ≤ 64 B`). | Unit Test (`TC-DRV-07`) |
| `LLR-SM5800-DRV-006` | `HLR-SM5800-04` | `SDD-SM5800-3.2` | `sm5800_msg2_eng_t` | Cấu trúc giá trị vật lý SI đọc/kiểm chứng được (`rad/s`, `m/s²`, `rad`, `m/s`, `°C`, `bit_fault_mask`, `seq_counter`, `sw_version`, `has_module_failed`, `timestamp_us`; kích thước `60 B ≤ 64 B`). | Unit Test (`TC-DRV-08`) |
| `LLR-SM5800-DRV-007` | `HLR-SM5800-03` | `SDD-SM5800-3.2` | `sm5800_diag_t` | Cấu trúc chẩn đoán liên kết: bộ đếm gói Msg 1/2, lỗi CRC/Frame/Abort/Seq, `isr_msg2_count`, `last_crc_residue` và các cờ trạng thái sống/lỗi BIT. | Unit Test (`TC-DRV-14`) |
| `LLR-SM5800-DRV-008` | `HLR-SM5800-05` | `SDD-SM5800-3.2` | `sm5800_drv_t` | Handle phiên bản do caller cấp phát tĩnh, cho phép hỗ trợ nhiều cảm biến độc lập (reentrant) mà không dùng biến toàn cục chia sẻ. | Code Inspection |

### 4.2 API Công khai (`sm5800_drv.h`)

| LLR ID | HLR cha | SDD | Hàm công khai | Mô tả yêu cầu chi tiết | Kiểm chứng |
|---|---|---|---|---|---|
| `LLR-SM5800-DRV-010` | `HLR-SM5800-05` | `SDD-SM5800-3.3.1` | `sm5800_calc_cfg_crc32()` | Tuần tự hóa tường minh từng byte Little-Endian (19 byte, không ép kiểu struct) các trường PDI và tính IEEE 802.3 CRC-32; trả `SM5800_ERR_PARAM` nếu con trỏ `NULL`. | `TC-DRV-01`, `TC-DRV-02` |
| `LLR-SM5800-DRV-011` | `HLR-SM5800-02` | `SDD-SM5800-3.3.2` | `sm5800_init()` | Kiểm tra `NULL`, xác thực PDI (`schema_version`, căn chỉnh 4B `base_addr`, `timeout_us` trong `12000..1000000`, `CRC-32`), đọc & xác thực `CORE_ID == 0x58000100U`, reset mềm parser, cấu hình `CTRL.MSG2_ONLY_MODE` và bật `INT_ENABLE = MSG2_READY | IMU_FAULT | LINK_TIMEOUT (0x26)` để PS được báo cả khi cảm biến im lặng (không có Msg 02). | `TC-DRV-02`, `TC-DRV-03` |
| `LLR-SM5800-DRV-012` | `HLR-SM5800-02` | `SDD-SM5800-3.3.3` | `sm5800_deinit()` | Trong vùng găng, chuyển trạng thái về `SM5800_STATE_UNINIT`, tắt `INT_ENABLE`, xóa W1C `INT_STATUS`, ghi `CTRL = 0`, gọi `DSB`. | `TC-DRV-04` |
| `LLR-SM5800-DRV-013` | `HLR-SM5800-04` | `SDD-SM5800-3.3.4` | `sm5800_irq_isr()` | Kiểm tra `NULL` và `STATE_READY`; đọc `INT_STATUS`, xóa W1C và gọi `DSB`; nếu có bit `MSG2_READY` thì tăng `isr_msg2_count` và đặt `is_msg2_pending = true`; chốt các cờ lỗi phụ; thực thi `DMB`. Thời gian O(1), không vòng lặp, không `float`. | `TC-DRV-05`, `TC-DRV-06` |
| `LLR-SM5800-DRV-014` | `HLR-SM5800-04` | `SDD-SM5800-3.3.5` | `sm5800_is_msg2_pending()` | Thực thi `DMB` và trả về trạng thái `is_msg2_pending` nếu handle hợp lệ và ở `STATE_READY`, ngược lại trả `false` an toàn. | `TC-DRV-05` |
| `LLR-SM5800-DRV-015` | `HLR-SM5800-04` | `SDD-SM5800-3.3.6` | `sm5800_read_msg2_raw()` | Xóa cờ `is_msg2_pending` trong vùng găng (trả `SM5800_ERR_NO_SAMPLE` nếu chưa có ngắt); khóa `CTRL.SNAP_LOCK`, đọc nguyên tử 15 thanh ghi raw của Msg 02, nhả khóa và giải mã bù 2 không UB. | `TC-DRV-07` |
| `LLR-SM5800-DRV-016` | `HLR-SM5800-04` | `SDD-SM5800-3.3.7` | `sm5800_raw_to_eng_msg2()` | Hàm 1-lệnh-gọi quy đổi toàn bộ 14 trường raw từ `sm5800_msg2_raw_t` sang đơn vị vật lý SI (`rad/s`, `m/s²`, `rad`, `m/s`, `°C`, `bit_fault_mask`, `seq_counter`, `sw_version`) để người dùng đọc & kiểm chứng. | `TC-DRV-08`, `TC-DRV-09` |
| `LLR-SM5800-DRV-017` | `HLR-SM5800-04` | `SDD-SM5800-3.3.8` | `sm5800_read_msg2_eng()` | Hàm 1-lệnh-gọi kết hợp đọc nguyên tử bản tin 100 Hz từ FPGA (`sm5800_read_msg2_raw`) và quy đổi thẳng sang `sm5800_msg2_eng_t`. | `TC-DRV-09` |
| `LLR-SM5800-DRV-018` | `HLR-SM5800-03` | `SDD-SM5800-3.3.9` | `sm5800_trigger_reset()` | PS ghi `CTRL.RST_PULSE` qua FPGA phát xung `RESET_L` thấp > 1 ms xuống IMU; xóa `is_msg2_pending`; nếu `poll_limit > 0` (≤ `SM5800_RESET_POLL_MAX`) chờ `STATUS.RST_BUSY == 0` với vòng lặp có cận trên, trả `SM5800_ERR_TIMEOUT` nếu quá hạn. | `TC-DRV-10` |
| `LLR-SM5800-DRV-019` | `HLR-SM5800-03` | `SDD-SM5800-3.3.10` | `sm5800_hold_reset()` | PS đặt bit `CTRL.RST_HOLD = 1` (và cập nhật `ctrl_shadow`) trong vùng găng để giữ thủ công chân `RESET_L` ở mức thấp. | `TC-DRV-11` |
| `LLR-SM5800-DRV-020` | `HLR-SM5800-03` | `SDD-SM5800-3.3.11` | `sm5800_release_reset()` | PS xóa bit `CTRL.RST_HOLD = 0` để nhả chân `RESET_L` về trạng thái Hi-Z (cảm biến khởi động lại trong ≤ 0.8 s). | `TC-DRV-11` |
| `LLR-SM5800-DRV-021` | `HLR-SM5800-03` | `SDD-SM5800-3.3.12` | `sm5800_start_cmd_bit()` | PS đặt `CTRL.CMD_BIT = 1` trong vùng găng kéo chân `BIT_L` thấp kích hoạt Commanded BIT (Sec. 6.3.2) và xóa `is_msg2_pending`. | `TC-DRV-12` |
| `LLR-SM5800-DRV-022` | `HLR-SM5800-03` | `SDD-SM5800-3.3.13` | `sm5800_stop_cmd_bit()` | PS xóa `CTRL.CMD_BIT = 0` trong vùng găng nhả chân `BIT_L` cao để SM-5800 quay lại chế độ bình thường trong ≤ 0.1 s. | `TC-DRV-12` |
| `LLR-SM5800-DRV-023` | `HLR-SM5800-03` | `SDD-SM5800-3.3.14` | `sm5800_is_reset_busy()` | Đọc `STATUS.RST_BUSY` kiểm tra xung `RESET_L` định thời đang chạy; trả `false` an toàn nếu handle chưa sẵn sàng. | `TC-DRV-13` |
| `LLR-SM5800-DRV-024` | `HLR-SM5800-03` | `SDD-SM5800-3.3.15` | `sm5800_is_cmd_bit_active()` | Đọc `STATUS.BIT_ACT` kiểm tra `BIT_L` đang được kéo thấp; trả `false` an toàn nếu handle chưa sẵn sàng. | `TC-DRV-13` |
| `LLR-SM5800-DRV-025` | `HLR-SM5800-03` | `SDD-SM5800-3.3.16` | `sm5800_read_diag()` | Đọc toàn bộ bộ đếm gói/lỗi (`MSG1/2`, `CRC`, `FRAME/ABORT`, `SEQ`), `DEBUG_CRC`, `STATUS` và `isr_msg2_count`. | `TC-DRV-14` |
| `LLR-SM5800-DRV-026` | `HLR-SM5800-03` | `SDD-SM5800-3.3.17` | `sm5800_clear_diag()` | Xóa bộ đếm thống kê FPGA qua xung `CTRL.CLR_STATS` và đặt lại `isr_msg2_count`, `isr_fault_flags` trong vùng găng. | `TC-DRV-14` |
| `LLR-SM5800-DRV-027` | `HLR-SM5800-04` | `SDD-SM5800-3.3.18` | `sm5800_rad_s_to_deg_s()` | Quy đổi vận tốc góc từ `rad/s` sang `deg/s` (`* 57.2957795f`) phục vụ đối chiếu dải `±1074 °/s` của tài liệu SM-5800. | `TC-DRV-15` |
| `LLR-SM5800-DRV-028` | `HLR-SM5800-03` | `SDD-SM5800-3.3.19` | `sm5800_take_irq_faults()` | Kiểm tra `NULL` và `STATE_READY`; trong vùng găng đọc rồi xóa `isr_fault_flags` (các bit `INT_*` ngoài `MSG2_READY` do ISR chốt, ví dụ `TIMEOUT`, `IMU_FAULT`) để không mất cờ do ngắt đồng thời; trả về qua `p_faults`. | `TC-DRV-17` |

### 4.3 Hàm nội bộ (`static` trong `sm5800_drv.c`)

| LLR ID | SDD | Hàm `static` | Yêu cầu chức năng |
|---|---|---|---|
| `LLR-SM5800-DRV-030` | `SDD-SM5800-3.4.1` | `compute_crc32_bytes()` | Tính IEEE 802.3 CRC-32 (đa thức phản xạ `0xEDB88320U`, init/xor `0xFFFFFFFFU`), vòng lặp cận trên `byte_count * 8`. |
| `LLR-SM5800-DRV-031` | `SDD-SM5800-3.4.2` | `pack_u32_le()` | Đóng gói 1 từ `uint32_t` theo thứ tự Little-Endian vào mảng byte đích. |
| `LLR-SM5800-DRV-032` | `SDD-SM5800-3.4.3` | `pack_u64_le()` | Đóng gói 1 từ `uint64_t` (dùng cho `uintptr_t` tương thích 32/64-bit) theo Little-Endian. |
| `LLR-SM5800-DRV-033` | `SDD-SM5800-3.4.4` | `validate_config()` | Xác thực `schema_version`, căn chỉnh `base_addr`, biên `timeout_us` (tách nhánh không ghép biến) và so khớp `CRC-32`. |
| `LLR-SM5800-DRV-034` | `SDD-SM5800-3.4.5` | `build_ctrl_word()` | Tổng hợp từ điều khiển `CTRL` từ cấu hình đã xác thực (luôn bật `SDLC_CLK_EN`). |
| `LLR-SM5800-DRV-035` | `SDD-SM5800-3.4.6` | `decode_s16_lsb()` | Giải mã số nguyên bù 2 16-bit từ từ 32-bit không dấu sang `int32_t` (`-32768..+32767`) không UB. |
| `LLR-SM5800-DRV-036` | `SDD-SM5800-3.4.7` | `decode_s32_lsb()` | Giải mã số nguyên bù 2 32-bit sang `int32_t` (`INT32_MIN..INT32_MAX`) dùng phép trừ `int64_t` chống tràn `INT32_MIN`. |
| `LLR-SM5800-DRV-037` | `SDD-SM5800-3.4.8` | `decode_s8_temp_degc()` | Tách và giải mã nhiệt độ gia tốc kế 8-bit có dấu (`1 °C/LSB`, `-128..+127 °C`) từ `Status Word 1[15:8]`. |
| `LLR-SM5800-DRV-038` | `SDD-SM5800-3.4.9` | `decode_bit_faults()` | Ghép 4 bit lỗi `SW1[7:4]` và 5 bit lỗi `SW2[15:11]` thành mặt nạ 9-bit `SM5800_FLT_*`. |
| `LLR-SM5800-DRV-039` | `SDD-SM5800-3.4.10` | `read_locked_msg2_raw()` | Khóa `CTRL.SNAP_LOCK`, gọi `DSB`, đọc đủ 15 thanh ghi Msg 02 (1 truy cập `volatile`/lệnh), nhả khóa ngay, gọi `DSB` và giải mã. |
| `LLR-SM5800-DRV-040` | `SDD-SM5800-3.4.11` | `convert_raw_to_eng()` | Quy đổi toàn bộ 14 trường Msg 02 sang SI (`rad/s`, `m/s²`, `rad`, `m/s`, `°C`, `bit_fault_mask`, `seq_counter`, `sw_version`, `has_module_failed`). |
| `LLR-SM5800-DRV-041` | `SDD-SM5800-3.4.12` | `populate_diag_flags()` | Tách các trường trạng thái từ thanh ghi `STATUS` vào `sm5800_diag_t` bằng `if/else` tường minh (không gán biểu thức logic phức). |

---

## 5. Yêu cầu mức thấp — Ứng dụng mẫu PS (`sm5800_ps_example.c`)

| LLR ID | HLR cha | SDD | Hàm / Đối tượng | Mô tả yêu cầu chi tiết | Kiểm chứng |
|---|---|---|---|---|---|
| `LLR-SM5800-APP-001` | `HLR-SM5800-05` | `SDD-SM5800-4.0` | `g_imu_drv` | Khởi tạo tĩnh toàn bộ handle driver cấp ứng dụng bằng designated initializer. | Static Audit |
| `LLR-SM5800-APP-002` | `HLR-SM5800-04` | `SDD-SM5800-4.1` | `sm5800_ps_gic_isr()`, `init_irq_path()`, `service_irq_polling()`, `g_gic`, `g_irq_polling` | Hàm ngắt ủy quyền trực tiếp cho `sm5800_irq_isr(&g_imu_drv)`. Khi có Xilinx BSP (`xscugic.h`), `init_irq_path()` (gọi **sau** `sm5800_init()`) khởi tạo `XScuGic` (luồng cũ: `XPAR_SCUGIC_0_DEVICE_ID`; SDT: `XPAR_XSCUGIC_0_BASEADDR`), đặt `IMU_IRQ_ID` kích **mức cao** (priority `0xA0`), đăng ký handler ngoại lệ IRQ, `XScuGic_Connect`/`Enable` rồi mở IRQ CPU. Không có BSP hoặc định nghĩa `APP_FORCE_IRQ_POLLING`: vòng lặp chính gọi `sm5800_irq_isr()` để polling `INT_STATUS`. | `TC-APP-03`, HIL / Integration |
| `LLR-SM5800-APP-003` | `HLR-SM5800-04` | `SDD-SM5800-4.2` | `main()` | Chạy tự kiểm chứng phép quy đổi 1 g, khởi tạo 100 Hz, vòng lặp có cận trên đọc Msg 02 bằng 1 lệnh gọi `sm5800_read_msg2_eng()` và dọn dẹp sạch qua `sm5800_deinit()`. | `TC-DRV-16`, HIL |
| `LLR-SM5800-APP-004` | `HLR-SM5800-02` | `SDD-SM5800-4.3` | `init_imu_100hz()` | Tạo khối PDI với `is_msg2_only = true`, tính `CRC-32` và gọi `sm5800_init()`. | `TC-DRV-16` |
| `LLR-SM5800-APP-005` | `HLR-SM5800-03` | `SDD-SM5800-4.4` | `handle_msg2_and_faults()`, `is_rate_plausible()`, `g_last_eng`, `g_last_rate_x_deg_s`, `g_rate_reject_count` | Loại bỏ mẫu có tốc độ quay ở **bất kỳ trục nào** đạt/vượt `APP_RATE_LIMIT_RAD_S = 18.5 rad/s` (gần full-scale 16-bit 18,75 rad/s ≈ 1074 °/s → bão hòa): tăng `g_rate_reject_count`, trả `SM5800_ERR_NO_SAMPLE`, **không** ghi đè `g_last_eng`, vòng lặp chính bỏ qua mẫu và tiếp tục thu. Mẫu hợp lệ: lưu mẫu SI mới nhất; nếu `has_module_failed` gọi `sm5800_trigger_reset()`, nếu có `bit_fault_mask` gọi `run_commanded_bit_cycle()`. | `TC-DRV-16`, `TC-APP-02`, HIL |
| `LLR-SM5800-APP-006` | `HLR-SM5800-04` | `SDD-SM5800-4.5` | `verify_raw_to_eng_math()` | Tự kiểm chứng `sm5800_raw_to_eng_msg2()` bằng vector kiểm thử 1 g (`accel_z_lsb = 879 → 9.75..9.86 m/s²`, không dùng `==` trên `float`). | `TC-DRV-08`, `TC-DRV-16` |
| `LLR-SM5800-APP-007` | `HLR-SM5800-03` | `SDD-SM5800-4.6` | `run_commanded_bit_cycle()` | Thực thi chu trình Commanded BIT có cận trên (`sm5800_start_cmd_bit` → chờ giữ `BIT_L` → `sm5800_stop_cmd_bit`). | `TC-DRV-16`, HIL |
| `LLR-SM5800-APP-008` | `HLR-SM5800-03` | `SDD-SM5800-4.7` | `handle_link_faults()`, `g_link_recovery_count` | Mỗi vòng lặp gọi `sm5800_take_irq_faults()`; khi có `LINK_TIMEOUT` thì phát xung `RESET_L` (tối đa `APP_MAX_LINK_RECOVERY = 3` lần liên tiếp, đếm lại từ 0 khi có mẫu tốt), quá giới hạn thì thoát với `SM5800_ERR_TIMEOUT`. | `TC-DRV-17`, HIL |
