# SM-5800 6-DOF IMU Subsystem (FPGA AXI4-Lite IP Core + ARM PS C Driver)

Dự án trọn gói giao tiếp cảm biến quán tính **SM-5800 6-DOF Motion Sensor Module** qua giao thức **SDLC RS-422 (1.0 MHz)** trên nền tảng **Zynq FPGA (PL) + ARM Cortex-A (PS)**, cấu hình mặc định cho chế độ **bản tin 100 Hz (Message ID 02: Motion Control + Spatial Data)** kèm điều khiển **Reset (`RESET_L`) & Commanded BIT (`BIT_L`)** từ PS xuống cảm biến và hàm quy đổi 1-lệnh-gọi ra đơn vị vật lý SI kiểm chứng được.

---

## Cấu trúc dự án

```text
IMU/
├── README.md                        # Tổng quan dự án và hướng dẫn chạy nhanh
├── AGENT_VERIFICATION_GUIDE.md      # Quy trình 7 cổng kiểm chứng cho AI Agent / QA độc lập
├── Makefile                         # Mục tiêu tự động: verify-static, sim, xsim, lint-rtl, check-guards, test-c, build-arm, build-arm64
├── .gitignore
├── rtl/                             # Mã nguồn FPGA Verilog-2001 (IEEE 1364-2001, AMBA AXI4-Lite)
│   ├── sm5800_filelist.f            # Danh sách thứ tự biên dịch RTL cho Vivado/Verilator/Icarus
│   ├── sm5800_param_error.v         # Module rỗng làm đích cho các chốt kiểm tra tham số (bắt buộc thêm vào Vivado)
│   ├── sm5800_crc16_ccitt.v         # Bộ tính CRC-16-CCITT (X^16+X^12+X^5+1, 0x8408, preset 0xFFFF)
│   ├── sm5800_clk_ctrl.v            # Xung 1.0 MHz RCV_CLK, 2-FF ASYNC_REG sync, >1ms RESET_L, BIT_L
│   ├── sm5800_sdlc_rx.v             # Bộ tách khung SDLC lookahead 8-bit, zero-destuff, Shared Flag, Abort
│   ├── sm5800_msg_parser.v          # Phân biệt msg1_valid_o/msg2_valid_o, chế độ 100Hz MSG2_ONLY, watchdog
│   ├── sm5800_axi4_lite_if.v        # Slave AMBA AXI4-Lite, thanh ghi 0x00-0x6C, SNAP_LOCK, ngắt W1C
│   └── sm5800_imu_axi_top.v         # Top-level IP Core (Vivado IP Packager ready)
├── tb/                              # Mô phỏng RTL (18 kịch bản tự kiểm tra [T1]-[T18])
│   ├── sm5800_sensor_model.v        # Mô hình cảm biến SM-5800 (Z8530 SCC bit-stuffing + CRC + tiêm lỗi)
│   └── tb_sm5800_imu_axi.v          # Testbench tự kiểm tra toàn diện (gồm [T13] chế độ 100 Hz, [T14]-[T18])
├── driver/                          # Driver C99 lớp PS (MISRA C:2012 + BARR-C:2018 + DO-178A/B/C)
│   ├── sm5800_regs.h                # Địa chỉ thanh ghi, _MASK, _SHIFT
│   ├── sm5800_hal.h / .c            # Lớp HAL ARM: Cortex-A9/R5 (CPSR) & Cortex-A53 AArch64 (DAIF), DMB/DSB
│   ├── sm5800_drv.h / .c            # Driver 100 Hz Msg 02 + quy đổi 1-lệnh-gọi SI + Reset/BIT PS->FPGA
│   ├── sm5800_ps_example.c          # Chương trình mẫu tích hợp GIC ISR + tự kiểm chứng 1g + xử lý lỗi BIT/Reset
│   └── DEVIATION_LOG.md             # Nhật ký ngoại lệ chuẩn MISRA Compliance:2020 & cờ biên dịch §12
├── tests/                           # Bộ Unit Test C99 trên Host với Mock HAL (TC-DRV-01..17)
│   ├── mock_sm5800_hal.h / .c       # Mô phỏng AXI4-Lite (W1C, W1P, SNAP_LOCK verify, RST_BUSY)
│   └── test_sm5800_drv.c            # Bộ tự kiểm tra 18 kịch bản LLR/MC-DC cho driver C
├── scripts/
│   └── verify_project.ps1           # Script kiểm tra tĩnh 5 cổng tự động (không cần cài EDA/GCC)
└── docs/                            # Bộ tài liệu kỹ thuật & DO-178C review cho Team
    ├── README.md                    # Chi tiết nối chân RS-422/OC, bản đồ thanh ghi & hướng dẫn sử dụng
    ├── LLR_SM5800.md                # Đặc tả yêu cầu mức thấp (5 HLR, 66 LLR khớp 1-1 với @req)
    ├── SDD_SM5800.md                # Tài liệu thiết kế hệ thống PL+PS, sơ đồ Mermaid, WCSU, SEU, truy vết
    ├── TEST_REPORT_SM5800.md        # Báo cáo kiểm chứng SVR & bảng ký duyệt cho Team
    └── reviews/                     # Báo cáo rà soát độc lập
        ├── REVIEW_REPORT_1.md       # Đợt 1: 5 lỗi gốc, lint, chốt tham số
        └── REVIEW_REPORT_2.md       # Đợt 2: đánh giá "Fix PL-PS IRQ", [T16], bão hòa gyro
```

---

## Chạy nhanh (Quick Start)

1. **Kiểm tra tĩnh & thuật toán ngay lập tức (Windows PowerShell — không cần cài công cụ ngoài):**
   ```powershell
   powershell -ExecutionPolicy Bypass -File scripts/verify_project.ps1
   ```
2. **Mô phỏng RTL (`[T1]`–`[T18]`) bằng Icarus Verilog hoặc Vivado `xsim`:**
   ```bash
   make sim        # Icarus Verilog
   make xsim       # Xilinx Vivado
   make lint-rtl   # Verilator lint (-Wall, 0 warning)
   make check-guards  # Cấu hình tham số sai (clock ±2%, RESET_L ≤ 1 ms, timeout 100 Hz, AXI) phải bị chặn lúc build
   ```
3. **Biên dịch nghiêm ngặt §12 & chạy Unit Test C (`TC-DRV-01`–`17`) trên máy host:**
   ```bash
   make test-c
   ```
4. **Biên dịch chéo cho ARM PS (Cortex-A9/R5 và Cortex-A53 AArch64) kèm báo cáo ngăn xếp `.su` & disassembly ISR:**
   ```bash
   make build-arm     # Cortex-A9 (Zynq-7000) + kiểm tra HAL Cortex-R5
   make build-arm64   # Cortex-A53 (Zynq UltraScale+, AArch64)
   ```

---

## Tài liệu dành cho Người duyệt / AI Agent kiểm tra

- **AI Agent hoặc QA vào kiểm tra độc lập:** Bắt đầu từ [AGENT_VERIFICATION_GUIDE.md](AGENT_VERIFICATION_GUIDE.md).
- **Đặc tả & Thiết kế (DO-178C):** Xem [docs/LLR_SM5800.md](docs/LLR_SM5800.md), [docs/SDD_SM5800.md](docs/SDD_SM5800.md), [docs/TEST_REPORT_SM5800.md](docs/TEST_REPORT_SM5800.md), và [driver/DEVIATION_LOG.md](driver/DEVIATION_LOG.md).
- **Hướng dẫn nối dây phần cứng & thanh ghi:** Xem [docs/README.md](docs/README.md).
- **Báo cáo rà soát độc lập:** [Đợt 1](docs/reviews/REVIEW_REPORT_1.md) (5 lỗi gốc, 7 cảnh báo lint) và [Đợt 2](docs/reviews/REVIEW_REPORT_2.md) (đánh giá "Fix PL-PS IRQ", `[T16]`, kiểm tra bão hòa gyro, việc còn lại).
