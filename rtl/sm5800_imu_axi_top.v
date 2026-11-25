//==============================================================================
// Module      : sm5800_imu_axi_top
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001) / AMBA AXI4-Lite (ARM IHI 0022E)
// Description : Top-level synthesizable IP core for reading the SM-5800 6-DOF
//               Motion Sensor Module over its 1.0 MHz RS-422 SDLC interface
//               and exposing Motion Control (600 Hz), Spatial Data (100 Hz),
//               BIT/Reset controls, timestamps, and diagnostics via AXI4-Lite.
//
// Hierarchy   : sm5800_imu_axi_top
//               ├── u_clk_ctrl      (sm5800_clk_ctrl)     : 1 MHz RCV_CLK, sync, RESET_L/BIT_L
//               ├── u_sdlc_rx       (sm5800_sdlc_rx)      : SDLC flag/destuff/abort + CRC-16
//               │   └── u_crc16     (sm5800_crc16_ccitt)  : CCITT X^16+X^12+X^5+1 (0xFFFF)
//               ├── u_msg_parser    (sm5800_msg_parser)   : Msg 1/2 parser, watchdog, BIT check
//               └── u_axi4_lite_if  (sm5800_axi4_lite_if) : AMBA AXI4-Lite slave, CSR & lock
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_imu_axi_top #(
    parameter integer C_CLK_FREQ_HZ        = 100_000_000, // s_axi_aclk frequency (Hz)
    parameter integer C_SDLC_CLK_HZ        = 1_000_000,   // SM-5800 RCV_CLK (1.0 MHz +/-2%)
    parameter integer C_RESET_PULSE_US     = 2000,        // RESET_L low pulse (us, >1000 us)
    parameter integer C_DEFAULT_CLK_EN     = 1,           // 1: enable 1 MHz clock out of reset
    parameter integer C_DEFAULT_MSG1_ONLY  = 0,           // 1: default to Msg ID 01-only mode
    parameter integer C_DEFAULT_MSG2_ONLY  = 0,           // 1: default to Msg ID 02-only mode (100 Hz)
    parameter integer C_DEFAULT_TIMEOUT_US = 30000,       // Data watchdog (us): 3x the 10 ms Msg 02 period
    parameter integer C_ACTIVATION_GRACE_US = 1_000_000,  // Extra watchdog time after start-up / RESET_L (us)
    parameter integer C_BIT_GRACE_US       = 300_000,     // Extra watchdog time after BIT_L release (us)
    parameter integer C_S_AXI_DATA_WIDTH   = 32,          // AXI4-Lite data width (32)
    parameter integer C_S_AXI_ADDR_WIDTH   = 8            // AXI4-Lite addr width (>=8)
) (
    //--------------------------------------------------------------------------
    // AMBA AXI4-Lite Slave Interface (Single Clock Domain)
    //--------------------------------------------------------------------------
    input  wire                              s_axi_aclk,
    input  wire                              s_axi_aresetn,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  wire [2:0]                        s_axi_awprot,
    input  wire                              s_axi_awvalid,
    output wire                              s_axi_awready,

    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                              s_axi_wvalid,
    output wire                              s_axi_wready,

    output wire [1:0]                        s_axi_bresp,
    output wire                              s_axi_bvalid,
    input  wire                              s_axi_bready,

    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  wire [2:0]                        s_axi_arprot,
    input  wire                              s_axi_arvalid,
    output wire                              s_axi_arready,

    output wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output wire [1:0]                        s_axi_rresp,
    output wire                              s_axi_rvalid,
    input  wire                              s_axi_rready,

    //--------------------------------------------------------------------------
    // Interrupt & Message-Type Strobe Outputs (Synchronous to s_axi_aclk)
    //--------------------------------------------------------------------------
    output wire                              irq_o,          // Active-high level IRQ to PS GIC
    output wire                              msg1_valid_o,   // 1-cycle pulse strictly on Msg ID 01
    output wire                              msg2_valid_o,   // 1-cycle pulse strictly on Msg ID 02

    //--------------------------------------------------------------------------
    // SM-5800 Physical Interface (Connect via 26C31/26C32 RS-422 & OC Drivers)
    //--------------------------------------------------------------------------
    output wire                              imu_rcv_clk_o,  // -> 26C31 -> RCV_CLK_H/L (Pins 28/6)
    input  wire                              imu_ser_data_i, // <- 26C32 <- SER_DATA_H/L (Pins 4/24)
    output wire                              imu_bit_l_o,    // -> OC driver -> BIT_L (Pin 12, active-low)
    output wire                              imu_reset_l_o   // -> OC driver -> RESET_L (Pin 18, active-low)
);

    //--------------------------------------------------------------------------
    // Elaboration-time parameter guard (Verilog-2001 has no $error):
    // in 100 Hz MSG2_ONLY mode a watchdog shorter than 12 ms (1.2x the 10 ms
    // Msg 02 period) would raise a false LINK_TIMEOUT on every frame. Such a
    // configuration connects a non-existent port of the empty module
    // sm5800_param_error, so synthesis / simulation stops with an error naming
    // the problem (see rtl/sm5800_param_error.v).
    //--------------------------------------------------------------------------
    generate
        if ((C_DEFAULT_MSG2_ONLY != 0) && (C_DEFAULT_TIMEOUT_US != 0) &&
            (C_DEFAULT_TIMEOUT_US < 12000)) begin : g_param_check
            /* verilator lint_off PINNOTFOUND */
            sm5800_param_error u_param_error (.SM5800_PARAM_ERROR_TIMEOUT_US_BELOW_12000_IN_MSG2_ONLY_MODE(1'b0));
            /* verilator lint_on PINNOTFOUND */
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Internal Interconnect Wires
    //--------------------------------------------------------------------------
    // CSR control signals
    wire        ctrl_clk_en_w;
    wire        ctrl_cmd_bit_en_w;
    wire        ctrl_reset_pulse_w;
    wire        ctrl_reset_hold_w;
    wire        ctrl_sample_edge_w;
    wire        ctrl_crc_mode_w;
    wire        ctrl_crc_bypass_w;
    wire        ctrl_clear_stats_w;
    wire        ctrl_soft_rst_w;
    wire        ctrl_msg1_only_w;
    wire        ctrl_msg2_only_w;
    wire [31:0] cfg_timeout_us_w;

    // Clock / timing / serial bit signals
    wire        rx_bit_valid_w;
    wire        rx_bit_w;
    wire        tick_1us_w;
    wire [31:0] timestamp_us_w;
    wire        imu_reset_busy_w;
    wire        imu_reset_active_w;

    // SDLC RX to Parser signals
    wire        rx_byte_valid_w;
    wire [7:0]  rx_byte_w;
    wire        frame_end_w;
    wire        frame_abort_w;
    wire        frame_crc_ok_w;
    wire        frame_align_err_w;
    wire [15:0] frame_crc_residue_w;
    wire        flag_detected_w;
    wire        sdlc_in_frame_w;

    // Live IMU data from Parser to AXI CSR
    wire [15:0] live_ang_rate_x_w;
    wire [15:0] live_ang_rate_y_w;
    wire [15:0] live_ang_rate_z_w;
    wire [15:0] live_lin_acc_x_w;
    wire [15:0] live_lin_acc_y_w;
    wire [15:0] live_lin_acc_z_w;
    wire [15:0] live_status_word1_w;
    wire [15:0] live_status_word2_w;
    wire [31:0] live_mc_timestamp_w;

    wire [31:0] live_delta_ang_x_w;
    wire [31:0] live_delta_ang_y_w;
    wire [31:0] live_delta_ang_z_w;
    wire [31:0] live_delta_vel_x_w;
    wire [31:0] live_delta_vel_y_w;
    wire [31:0] live_delta_vel_z_w;
    wire [31:0] live_sp_timestamp_w;

    // Events, status & diagnostics from Parser to AXI CSR
    wire        msg1_valid_w;
    wire        msg2_valid_w;
    wire        mc_data_valid_w;
    wire        imu_fault_pulse_w;
    wire        rx_err_pulse_w;
    wire        seq_err_pulse_w;
    wire        link_timeout_pulse_w;

    wire        flags_alive_w;
    wire        data_active_w;
    wire        imu_fault_active_w;
    wire        module_failed_w;
    wire [7:0]  last_msg_id_w;
    wire [3:0]  last_seq_cnt_w;
    wire [3:0]  last_err_code_w;

    wire [31:0] msg1_rx_cnt_w;
    wire [31:0] msg2_rx_cnt_w;
    wire [31:0] crc_err_cnt_w;
    wire [15:0] frame_err_cnt_w;
    wire [15:0] abort_err_cnt_w;
    wire [31:0] seq_err_cnt_w;
    wire [15:0] debug_crc_residue_w;
    wire [5:0]  debug_frame_len_w;
    wire [7:0]  debug_frame_addr_w;

    assign imu_reset_active_w = ~imu_reset_l_o;
    assign msg1_valid_o       = msg1_valid_w;
    assign msg2_valid_o       = msg2_valid_w;

    //--------------------------------------------------------------------------
    // 1. Physical Clock Generator, Synchronizer, RESET_L & BIT_L Controller
    //--------------------------------------------------------------------------
    sm5800_clk_ctrl #(
        .C_CLK_FREQ_HZ    (C_CLK_FREQ_HZ),
        .C_SDLC_CLK_HZ    (C_SDLC_CLK_HZ),
        .C_RESET_PULSE_US (C_RESET_PULSE_US)
    ) u_clk_ctrl (
        .clk_i             (s_axi_aclk),
        .rst_ni            (s_axi_aresetn),
        .clk_en_i          (ctrl_clk_en_w),
        .sample_edge_i     (ctrl_sample_edge_w),
        .bit_cmd_i         (ctrl_cmd_bit_en_w),
        .reset_pulse_req_i (ctrl_reset_pulse_w),
        .reset_hold_i      (ctrl_reset_hold_w),
        .rx_bit_valid_o    (rx_bit_valid_w),
        .rx_bit_o          (rx_bit_w),
        .tick_1us_o        (tick_1us_w),
        .timestamp_us_o    (timestamp_us_w),
        .reset_busy_o      (imu_reset_busy_w),
        .imu_rcv_clk_o     (imu_rcv_clk_o),
        .imu_ser_data_i    (imu_ser_data_i),
        .imu_bit_l_o       (imu_bit_l_o),
        .imu_reset_l_o     (imu_reset_l_o)
    );

    //--------------------------------------------------------------------------
    // 2. SDLC Bit-Oriented Receiver (Flag, Destuffing, Abort & CRC-16-CCITT)
    //--------------------------------------------------------------------------
    sm5800_sdlc_rx u_sdlc_rx (
        .clk_i               (s_axi_aclk),
        .rst_ni              (s_axi_aresetn),
        .soft_rst_i          (ctrl_soft_rst_w),
        .crc_mode_i          (ctrl_crc_mode_w),
        .rx_bit_valid_i      (rx_bit_valid_w),
        .rx_bit_i            (rx_bit_w),
        .rx_byte_valid_o     (rx_byte_valid_w),
        .rx_byte_o           (rx_byte_w),
        .frame_end_o         (frame_end_w),
        .frame_abort_o       (frame_abort_w),
        .frame_crc_ok_o      (frame_crc_ok_w),
        .frame_align_err_o   (frame_align_err_w),
        .frame_crc_residue_o (frame_crc_residue_w),
        .flag_detected_o     (flag_detected_w),
        .in_frame_o          (sdlc_in_frame_w)
    );

    //--------------------------------------------------------------------------
    // 3. SM-5800 Message Validator, Parser, Watchdog & Diagnostics
    //--------------------------------------------------------------------------
    sm5800_msg_parser #(
        .C_ACTIVATION_GRACE_US (C_ACTIVATION_GRACE_US),
        .C_BIT_GRACE_US        (C_BIT_GRACE_US)
    ) u_msg_parser (
        .clk_i                (s_axi_aclk),
        .rst_ni               (s_axi_aresetn),
        .soft_rst_i           (ctrl_soft_rst_w),
        .clear_stats_i        (ctrl_clear_stats_w),
        .crc_bypass_i         (ctrl_crc_bypass_w),
        .msg1_only_mode_i     (ctrl_msg1_only_w),
        .msg2_only_mode_i     (ctrl_msg2_only_w),
        .bit_cmd_active_i     (ctrl_cmd_bit_en_w),
        .imu_reset_active_i   (imu_reset_active_w),
        .timeout_us_i         (cfg_timeout_us_w),
        .tick_1us_i           (tick_1us_w),
        .timestamp_us_i       (timestamp_us_w),
        .rx_byte_valid_i      (rx_byte_valid_w),
        .rx_byte_i            (rx_byte_w),
        .frame_end_i          (frame_end_w),
        .frame_abort_i        (frame_abort_w),
        .frame_crc_ok_i       (frame_crc_ok_w),
        .frame_align_err_i    (frame_align_err_w),
        .frame_crc_residue_i  (frame_crc_residue_w),
        .flag_detected_i      (flag_detected_w),
        .ang_rate_x_o         (live_ang_rate_x_w),
        .ang_rate_y_o         (live_ang_rate_y_w),
        .ang_rate_z_o         (live_ang_rate_z_w),
        .lin_acc_x_o          (live_lin_acc_x_w),
        .lin_acc_y_o          (live_lin_acc_y_w),
        .lin_acc_z_o          (live_lin_acc_z_w),
        .status_word1_o       (live_status_word1_w),
        .status_word2_o       (live_status_word2_w),
        .mc_timestamp_o       (live_mc_timestamp_w),
        .delta_ang_x_o        (live_delta_ang_x_w),
        .delta_ang_y_o        (live_delta_ang_y_w),
        .delta_ang_z_o        (live_delta_ang_z_w),
        .delta_vel_x_o        (live_delta_vel_x_w),
        .delta_vel_y_o        (live_delta_vel_y_w),
        .delta_vel_z_o        (live_delta_vel_z_w),
        .sp_timestamp_o       (live_sp_timestamp_w),
        .msg1_valid_o         (msg1_valid_w),
        .msg2_valid_o         (msg2_valid_w),
        .mc_data_valid_o      (mc_data_valid_w),
        // Spatial-commit strobe is not needed at top level (msg2_valid_o and
        // INT_STATUS.MSG2_READY already mark every Spatial update).
        /* verilator lint_off PINCONNECTEMPTY */
        .sp_data_valid_o      (),
        /* verilator lint_on PINCONNECTEMPTY */
        .imu_fault_pulse_o    (imu_fault_pulse_w),
        .rx_err_pulse_o       (rx_err_pulse_w),
        .seq_err_pulse_o      (seq_err_pulse_w),
        .link_timeout_pulse_o (link_timeout_pulse_w),
        .flags_alive_o        (flags_alive_w),
        .data_active_o        (data_active_w),
        .imu_fault_active_o   (imu_fault_active_w),
        .module_failed_o      (module_failed_w),
        .last_msg_id_o        (last_msg_id_w),
        .last_seq_cnt_o       (last_seq_cnt_w),
        .last_err_code_o      (last_err_code_w),
        .msg1_rx_cnt_o        (msg1_rx_cnt_w),
        .msg2_rx_cnt_o        (msg2_rx_cnt_w),
        .crc_err_cnt_o        (crc_err_cnt_w),
        .frame_err_cnt_o      (frame_err_cnt_w),
        .abort_err_cnt_o      (abort_err_cnt_w),
        .seq_err_cnt_o        (seq_err_cnt_w),
        .debug_crc_residue_o  (debug_crc_residue_w),
        .debug_frame_len_o    (debug_frame_len_w),
        .debug_frame_addr_o   (debug_frame_addr_w)
    );

    //--------------------------------------------------------------------------
    // 4. AMBA AXI4-Lite Slave Interface & Register File
    //--------------------------------------------------------------------------
    sm5800_axi4_lite_if #(
        .C_S_AXI_DATA_WIDTH   (C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH   (C_S_AXI_ADDR_WIDTH),
        .C_DEFAULT_CLK_EN     (C_DEFAULT_CLK_EN),
        .C_DEFAULT_MSG1_ONLY  (C_DEFAULT_MSG1_ONLY),
        .C_DEFAULT_MSG2_ONLY  (C_DEFAULT_MSG2_ONLY),
        .C_DEFAULT_TIMEOUT_US (C_DEFAULT_TIMEOUT_US)
    ) u_axi4_lite_if (
        .s_axi_aclk           (s_axi_aclk),
        .s_axi_aresetn        (s_axi_aresetn),
        .s_axi_awaddr         (s_axi_awaddr),
        .s_axi_awprot         (s_axi_awprot),
        .s_axi_awvalid        (s_axi_awvalid),
        .s_axi_awready        (s_axi_awready),
        .s_axi_wdata          (s_axi_wdata),
        .s_axi_wstrb          (s_axi_wstrb),
        .s_axi_wvalid         (s_axi_wvalid),
        .s_axi_wready         (s_axi_wready),
        .s_axi_bresp          (s_axi_bresp),
        .s_axi_bvalid         (s_axi_bvalid),
        .s_axi_bready         (s_axi_bready),
        .s_axi_araddr         (s_axi_araddr),
        .s_axi_arprot         (s_axi_arprot),
        .s_axi_arvalid        (s_axi_arvalid),
        .s_axi_arready        (s_axi_arready),
        .s_axi_rdata          (s_axi_rdata),
        .s_axi_rresp          (s_axi_rresp),
        .s_axi_rvalid         (s_axi_rvalid),
        .s_axi_rready         (s_axi_rready),
        .irq_o                (irq_o),
        .ctrl_clk_en_o        (ctrl_clk_en_w),
        .ctrl_cmd_bit_en_o    (ctrl_cmd_bit_en_w),
        .ctrl_reset_pulse_o   (ctrl_reset_pulse_w),
        .ctrl_reset_hold_o    (ctrl_reset_hold_w),
        .ctrl_sample_edge_o   (ctrl_sample_edge_w),
        .ctrl_crc_mode_o      (ctrl_crc_mode_w),
        .ctrl_crc_bypass_o    (ctrl_crc_bypass_w),
        .ctrl_clear_stats_o   (ctrl_clear_stats_w),
        .ctrl_soft_rst_o      (ctrl_soft_rst_w),
        .ctrl_msg1_only_o     (ctrl_msg1_only_w),
        .ctrl_msg2_only_o     (ctrl_msg2_only_w),
        .cfg_timeout_us_o     (cfg_timeout_us_w),
        .live_ang_rate_x_i    (live_ang_rate_x_w),
        .live_ang_rate_y_i    (live_ang_rate_y_w),
        .live_ang_rate_z_i    (live_ang_rate_z_w),
        .live_lin_acc_x_i     (live_lin_acc_x_w),
        .live_lin_acc_y_i     (live_lin_acc_y_w),
        .live_lin_acc_z_i     (live_lin_acc_z_w),
        .live_status_word1_i  (live_status_word1_w),
        .live_status_word2_i  (live_status_word2_w),
        .live_mc_timestamp_i  (live_mc_timestamp_w),
        .live_delta_ang_x_i   (live_delta_ang_x_w),
        .live_delta_ang_y_i   (live_delta_ang_y_w),
        .live_delta_ang_z_i   (live_delta_ang_z_w),
        .live_delta_vel_x_i   (live_delta_vel_x_w),
        .live_delta_vel_y_i   (live_delta_vel_y_w),
        .live_delta_vel_z_i   (live_delta_vel_z_w),
        .live_sp_timestamp_i  (live_sp_timestamp_w),
        .msg1_valid_i         (msg1_valid_w),
        .msg2_valid_i         (msg2_valid_w),
        .mc_data_valid_i      (mc_data_valid_w),
        .imu_fault_pulse_i    (imu_fault_pulse_w),
        .rx_err_pulse_i       (rx_err_pulse_w),
        .seq_err_pulse_i      (seq_err_pulse_w),
        .link_timeout_pulse_i (link_timeout_pulse_w),
        .flags_alive_i        (flags_alive_w),
        .data_active_i        (data_active_w),
        .imu_fault_active_i   (imu_fault_active_w),
        .module_failed_i      (module_failed_w),
        .imu_reset_busy_i     (imu_reset_busy_w),
        .sdlc_in_frame_i      (sdlc_in_frame_w),
        .last_msg_id_i        (last_msg_id_w),
        .last_seq_cnt_i       (last_seq_cnt_w),
        .last_err_code_i      (last_err_code_w),
        .msg1_rx_cnt_i        (msg1_rx_cnt_w),
        .msg2_rx_cnt_i        (msg2_rx_cnt_w),
        .crc_err_cnt_i        (crc_err_cnt_w),
        .frame_err_cnt_i      (frame_err_cnt_w),
        .abort_err_cnt_i      (abort_err_cnt_w),
        .seq_err_cnt_i        (seq_err_cnt_w),
        .debug_crc_residue_i  (debug_crc_residue_w),
        .debug_frame_len_i    (debug_frame_len_w),
        .debug_frame_addr_i   (debug_frame_addr_w)
    );

endmodule

`default_nettype wire
