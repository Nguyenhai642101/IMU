//==============================================================================
// Module      : sm5800_axi4_lite_if
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001) / AMBA AXI4-Lite (ARM IHI 0022E)
// Description : Fully compliant AMBA AXI4-Lite Slave & Register File (CSR):
//               - Independent AW & W channel acceptance (handles AW-first,
//                 W-first, or simultaneous AW+W with zero comb VALID->READY).
//               - Full byte-enable (WSTRB) support on writable registers.
//               - Atomic Snapshot / Shadow Lock (CTRL.DATA_LOCK) preventing
//                 multi-register data tearing across 600 Hz / 100 Hz updates.
//               - Lossless W1C interrupt status (HW set overrides SW W1C)
//                 and glitch-free registered IRQ output, re-armed (1-cycle low)
//                 by every W1C write so edge- and level-triggered GICs both work.
//               - 32-bit sign extension on 16-bit signed Motion Control fields.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_axi4_lite_if #(
    parameter integer C_S_AXI_DATA_WIDTH   = 32,
    parameter integer C_S_AXI_ADDR_WIDTH   = 8,
    parameter integer C_DEFAULT_CLK_EN     = 1,
    parameter integer C_DEFAULT_MSG1_ONLY  = 0,
    parameter integer C_DEFAULT_MSG2_ONLY  = 0,
    parameter integer C_DEFAULT_TIMEOUT_US = 30000
) (
    //--------------------------------------------------------------------------
    // AMBA AXI4-Lite Slave Interface
    //--------------------------------------------------------------------------
    input  wire                              s_axi_aclk,
    input  wire                              s_axi_aresetn,

    // Write Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_awaddr,
    input  wire [2:0]                        s_axi_awprot,
    input  wire                              s_axi_awvalid,
    output wire                              s_axi_awready,

    // Write Data Channel
    input  wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_wdata,
    input  wire [(C_S_AXI_DATA_WIDTH/8)-1:0] s_axi_wstrb,
    input  wire                              s_axi_wvalid,
    output wire                              s_axi_wready,

    // Write Response Channel
    output wire [1:0]                        s_axi_bresp,
    output wire                              s_axi_bvalid,
    input  wire                              s_axi_bready,

    // Read Address Channel
    input  wire [C_S_AXI_ADDR_WIDTH-1:0]     s_axi_araddr,
    input  wire [2:0]                        s_axi_arprot,
    input  wire                              s_axi_arvalid,
    output wire                              s_axi_arready,

    // Read Data Channel
    output wire [C_S_AXI_DATA_WIDTH-1:0]     s_axi_rdata,
    output wire [1:0]                        s_axi_rresp,
    output wire                              s_axi_rvalid,
    input  wire                              s_axi_rready,

    // Interrupt Output (Active-High Level)
    output wire                              irq_o,

    //--------------------------------------------------------------------------
    // Control Outputs to Submodules
    //--------------------------------------------------------------------------
    output wire                              ctrl_clk_en_o,
    output wire                              ctrl_cmd_bit_en_o,
    output wire                              ctrl_reset_pulse_o,
    output wire                              ctrl_reset_hold_o,
    output wire                              ctrl_sample_edge_o,
    output wire                              ctrl_crc_mode_o,
    output wire                              ctrl_crc_bypass_o,
    output wire                              ctrl_clear_stats_o,
    output wire                              ctrl_soft_rst_o,
    output wire                              ctrl_msg1_only_o,
    output wire                              ctrl_msg2_only_o,
    output wire [31:0]                       cfg_timeout_us_o,

    //--------------------------------------------------------------------------
    // Live Sensor Data Inputs from sm5800_msg_parser
    //--------------------------------------------------------------------------
    input  wire [15:0]                       live_ang_rate_x_i,
    input  wire [15:0]                       live_ang_rate_y_i,
    input  wire [15:0]                       live_ang_rate_z_i,
    input  wire [15:0]                       live_lin_acc_x_i,
    input  wire [15:0]                       live_lin_acc_y_i,
    input  wire [15:0]                       live_lin_acc_z_i,
    input  wire [15:0]                       live_status_word1_i,
    input  wire [15:0]                       live_status_word2_i,
    input  wire [31:0]                       live_mc_timestamp_i,

    input  wire [31:0]                       live_delta_ang_x_i,
    input  wire [31:0]                       live_delta_ang_y_i,
    input  wire [31:0]                       live_delta_ang_z_i,
    input  wire [31:0]                       live_delta_vel_x_i,
    input  wire [31:0]                       live_delta_vel_y_i,
    input  wire [31:0]                       live_delta_vel_z_i,
    input  wire [31:0]                       live_sp_timestamp_i,

    //--------------------------------------------------------------------------
    // Event Pulses & Status Inputs from Submodules
    //--------------------------------------------------------------------------
    input  wire                              msg1_valid_i,
    input  wire                              msg2_valid_i,
    input  wire                              mc_data_valid_i,
    input  wire                              imu_fault_pulse_i,
    input  wire                              rx_err_pulse_i,
    input  wire                              seq_err_pulse_i,
    input  wire                              link_timeout_pulse_i,

    input  wire                              flags_alive_i,
    input  wire                              data_active_i,
    input  wire                              imu_fault_active_i,
    input  wire                              module_failed_i,
    input  wire                              imu_reset_busy_i,
    input  wire                              sdlc_in_frame_i,
    input  wire [7:0]                        last_msg_id_i,
    input  wire [3:0]                        last_seq_cnt_i,
    input  wire [3:0]                        last_err_code_i,

    input  wire [31:0]                       msg1_rx_cnt_i,
    input  wire [31:0]                       msg2_rx_cnt_i,
    input  wire [31:0]                       crc_err_cnt_i,
    input  wire [15:0]                       frame_err_cnt_i,
    input  wire [15:0]                       abort_err_cnt_i,
    input  wire [31:0]                       seq_err_cnt_i,
    input  wire [15:0]                       debug_crc_residue_i,
    input  wire [5:0]                        debug_frame_len_i,
    input  wire [7:0]                        debug_frame_addr_i
);

    //--------------------------------------------------------------------------
    // Core ID & Word-Aligned Register Offsets (Address bits [7:2])
    //--------------------------------------------------------------------------
    localparam [31:0] P_CORE_ID           = 32'h5800_0100; // SM-5800 IP v1.0.0
    localparam [1:0]  P_AXI_RESP_OKAY     = 2'b00;

    localparam [5:0]  P_REG_CORE_ID       = 6'h00; // 0x00
    localparam [5:0]  P_REG_CTRL          = 6'h01; // 0x04
    localparam [5:0]  P_REG_STATUS        = 6'h02; // 0x08
    localparam [5:0]  P_REG_INT_STATUS    = 6'h03; // 0x0C
    localparam [5:0]  P_REG_INT_ENABLE    = 6'h04; // 0x10
    localparam [5:0]  P_REG_TIMEOUT_CFG   = 6'h05; // 0x14
    localparam [5:0]  P_REG_MC_TIMESTAMP  = 6'h06; // 0x18
    localparam [5:0]  P_REG_SP_TIMESTAMP  = 6'h07; // 0x1C

    localparam [5:0]  P_REG_ANG_RATE_X    = 6'h08; // 0x20
    localparam [5:0]  P_REG_ANG_RATE_Y    = 6'h09; // 0x24
    localparam [5:0]  P_REG_ANG_RATE_Z    = 6'h0A; // 0x28
    localparam [5:0]  P_REG_LIN_ACC_X     = 6'h0B; // 0x2C
    localparam [5:0]  P_REG_LIN_ACC_Y     = 6'h0C; // 0x30
    localparam [5:0]  P_REG_LIN_ACC_Z     = 6'h0D; // 0x34
    localparam [5:0]  P_REG_STATUS_WORD1  = 6'h0E; // 0x38
    localparam [5:0]  P_REG_STATUS_WORD2  = 6'h0F; // 0x3C

    localparam [5:0]  P_REG_DELTA_ANG_X   = 6'h10; // 0x40
    localparam [5:0]  P_REG_DELTA_ANG_Y   = 6'h11; // 0x44
    localparam [5:0]  P_REG_DELTA_ANG_Z   = 6'h12; // 0x48
    localparam [5:0]  P_REG_DELTA_VEL_X   = 6'h13; // 0x4C
    localparam [5:0]  P_REG_DELTA_VEL_Y   = 6'h14; // 0x50
    localparam [5:0]  P_REG_DELTA_VEL_Z   = 6'h15; // 0x54

    localparam [5:0]  P_REG_MSG1_RX_CNT   = 6'h16; // 0x58
    localparam [5:0]  P_REG_MSG2_RX_CNT   = 6'h17; // 0x5C
    localparam [5:0]  P_REG_CRC_ERR_CNT   = 6'h18; // 0x60
    localparam [5:0]  P_REG_FRAME_ERR_CNT = 6'h19; // 0x64
    localparam [5:0]  P_REG_SEQ_ERR_CNT   = 6'h1A; // 0x68
    localparam [5:0]  P_REG_DEBUG_CRC     = 6'h1B; // 0x6C

    //--------------------------------------------------------------------------
    // Elaboration-time guard: the register decoder uses ADDR[7:2] and a fixed
    // 32-bit data path, so other widths would silently misbehave. An illegal
    // width connects a non-existent port of sm5800_param_error (see that file).
    //--------------------------------------------------------------------------
    generate
        if ((C_S_AXI_ADDR_WIDTH < 8) || (C_S_AXI_DATA_WIDTH != 32)) begin : g_err_axi
            /* verilator lint_off PINNOTFOUND */
            sm5800_param_error u_param_error (.SM5800_PARAM_ERROR_AXI_ADDR_LT_8_OR_DATA_NOT_32(1'b0));
            /* verilator lint_on PINNOTFOUND */
        end
    endgenerate

    //--------------------------------------------------------------------------
    // AXI4-Lite Handshake Registers
    //--------------------------------------------------------------------------
    reg                          aw_ready_r;
    reg                          w_ready_r;
    reg                          aw_captured_r;
    reg                          w_captured_r;
    reg [5:0]                    aw_word_addr_r; // AWADDR[7:2] (byte bits unused)
    reg [31:0]                   w_data_r;
    reg [3:0]                    w_strb_r;
    reg                          b_valid_r;
    reg [1:0]                    b_resp_r;

    reg                          ar_ready_r;
    reg                          r_valid_r;
    reg [31:0]                   r_data_r;
    reg [1:0]                    r_resp_r;

    //--------------------------------------------------------------------------
    // CSR & Interrupt Registers
    //--------------------------------------------------------------------------
    reg        ctrl_clk_en_r;
    reg        ctrl_data_lock_r;
    reg        ctrl_cmd_bit_en_r;
    reg        ctrl_reset_pulse_r;
    reg        ctrl_reset_hold_r;
    reg        ctrl_sample_edge_r;
    reg        ctrl_crc_mode_r;
    reg        ctrl_crc_bypass_r;
    reg        ctrl_clear_stats_r;
    reg        ctrl_soft_rst_r;
    reg        ctrl_msg1_only_r;
    reg        ctrl_msg2_only_r;

    reg [6:0]  int_status_r;
    reg [6:0]  int_enable_r;
    reg        irq_r;
    reg [31:0] timeout_cfg_r;

    //--------------------------------------------------------------------------
    // Coherent Snapshot / Shadow Registers (latched on DATA_LOCK 0 -> 1)
    //--------------------------------------------------------------------------
    reg [15:0] shd_ang_rate_x_r;
    reg [15:0] shd_ang_rate_y_r;
    reg [15:0] shd_ang_rate_z_r;
    reg [15:0] shd_lin_acc_x_r;
    reg [15:0] shd_lin_acc_y_r;
    reg [15:0] shd_lin_acc_z_r;
    reg [15:0] shd_status_word1_r;
    reg [15:0] shd_status_word2_r;
    reg [31:0] shd_mc_timestamp_r;

    reg [31:0] shd_delta_ang_x_r;
    reg [31:0] shd_delta_ang_y_r;
    reg [31:0] shd_delta_ang_z_r;
    reg [31:0] shd_delta_vel_x_r;
    reg [31:0] shd_delta_vel_y_r;
    reg [31:0] shd_delta_vel_z_r;
    reg [31:0] shd_sp_timestamp_r;

    //--------------------------------------------------------------------------
    // AXI4-Lite Handshake Wires
    //--------------------------------------------------------------------------
    wire        aw_hs_w;
    wire        w_hs_w;
    wire        b_hs_w;
    wire        wr_fire_w;
    wire [5:0]  wr_word_addr_w;
    wire [31:0] wr_data_w;
    wire [3:0]  wr_strb_w;

    wire        ar_hs_w;
    wire        r_hs_w;
    wire [5:0]  rd_word_addr_w;

    wire [6:0]  hw_int_events_w;
    wire [6:0]  sw_w1c_mask_w;
    wire [6:0]  int_status_next_w;
    wire [6:0]  int_enable_next_w;
    wire        unused_prot_w;

    // Explicitly consume unused AXI protection bits to avoid linter warnings
    assign unused_prot_w  = &{1'b0, s_axi_awprot, s_axi_arprot, s_axi_awaddr[1:0], s_axi_araddr[1:0]};

    assign aw_hs_w        = s_axi_awvalid && aw_ready_r;
    assign w_hs_w         = s_axi_wvalid  && w_ready_r;
    assign b_hs_w         = b_valid_r     && s_axi_bready;

    assign wr_fire_w      = !b_valid_r && (aw_captured_r || aw_hs_w) && (w_captured_r || w_hs_w);
    assign wr_word_addr_w = aw_captured_r ? aw_word_addr_r : s_axi_awaddr[7:2];
    assign wr_data_w      = w_captured_r  ? w_data_r       : s_axi_wdata;
    assign wr_strb_w      = w_captured_r  ? w_strb_r       : s_axi_wstrb;

    assign ar_hs_w        = s_axi_arvalid && ar_ready_r;
    assign r_hs_w         = r_valid_r     && s_axi_rready;
    assign rd_word_addr_w = s_axi_araddr[7:2];

    assign hw_int_events_w = {
        mc_data_valid_i,
        link_timeout_pulse_i,
        seq_err_pulse_i,
        rx_err_pulse_i,
        imu_fault_pulse_i,
        msg2_valid_i,
        msg1_valid_i
    };

    assign sw_w1c_mask_w = (wr_fire_w && (wr_word_addr_w == P_REG_INT_STATUS) && wr_strb_w[0])
                           ? wr_data_w[6:0] : 7'b000_0000;

    assign int_status_next_w = ctrl_soft_rst_r
                               ? 7'b000_0000
                               : ((int_status_r & ~sw_w1c_mask_w) | hw_int_events_w);
    assign int_enable_next_w = (wr_fire_w && (wr_word_addr_w == P_REG_INT_ENABLE) && wr_strb_w[0])
                               ? wr_data_w[6:0]
                               : int_enable_r;

    //--------------------------------------------------------------------------
    // Selected Read Data (Live when unlocked, Shadowed when DATA_LOCK=1)
    //--------------------------------------------------------------------------
    wire [15:0] sel_ang_rate_x_w;
    wire [15:0] sel_ang_rate_y_w;
    wire [15:0] sel_ang_rate_z_w;
    wire [15:0] sel_lin_acc_x_w;
    wire [15:0] sel_lin_acc_y_w;
    wire [15:0] sel_lin_acc_z_w;
    wire [15:0] sel_status_word1_w;
    wire [15:0] sel_status_word2_w;
    wire [31:0] sel_mc_timestamp_w;

    wire [31:0] sel_delta_ang_x_w;
    wire [31:0] sel_delta_ang_y_w;
    wire [31:0] sel_delta_ang_z_w;
    wire [31:0] sel_delta_vel_x_w;
    wire [31:0] sel_delta_vel_y_w;
    wire [31:0] sel_delta_vel_z_w;
    wire [31:0] sel_sp_timestamp_w;

    assign sel_ang_rate_x_w   = ctrl_data_lock_r ? shd_ang_rate_x_r   : live_ang_rate_x_i;
    assign sel_ang_rate_y_w   = ctrl_data_lock_r ? shd_ang_rate_y_r   : live_ang_rate_y_i;
    assign sel_ang_rate_z_w   = ctrl_data_lock_r ? shd_ang_rate_z_r   : live_ang_rate_z_i;
    assign sel_lin_acc_x_w    = ctrl_data_lock_r ? shd_lin_acc_x_r    : live_lin_acc_x_i;
    assign sel_lin_acc_y_w    = ctrl_data_lock_r ? shd_lin_acc_y_r    : live_lin_acc_y_i;
    assign sel_lin_acc_z_w    = ctrl_data_lock_r ? shd_lin_acc_z_r    : live_lin_acc_z_i;
    assign sel_status_word1_w = ctrl_data_lock_r ? shd_status_word1_r : live_status_word1_i;
    assign sel_status_word2_w = ctrl_data_lock_r ? shd_status_word2_r : live_status_word2_i;
    assign sel_mc_timestamp_w = ctrl_data_lock_r ? shd_mc_timestamp_r : live_mc_timestamp_i;

    assign sel_delta_ang_x_w  = ctrl_data_lock_r ? shd_delta_ang_x_r  : live_delta_ang_x_i;
    assign sel_delta_ang_y_w  = ctrl_data_lock_r ? shd_delta_ang_y_r  : live_delta_ang_y_i;
    assign sel_delta_ang_z_w  = ctrl_data_lock_r ? shd_delta_ang_z_r  : live_delta_ang_z_i;
    assign sel_delta_vel_x_w  = ctrl_data_lock_r ? shd_delta_vel_x_r  : live_delta_vel_x_i;
    assign sel_delta_vel_y_w  = ctrl_data_lock_r ? shd_delta_vel_y_r  : live_delta_vel_y_i;
    assign sel_delta_vel_z_w  = ctrl_data_lock_r ? shd_delta_vel_z_r  : live_delta_vel_z_i;
    assign sel_sp_timestamp_w = ctrl_data_lock_r ? shd_sp_timestamp_r : live_sp_timestamp_i;

    //--------------------------------------------------------------------------
    // AXI4-Lite Write Channel Control (AW, W, B)
    //--------------------------------------------------------------------------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            aw_ready_r    <= 1'b0;
            w_ready_r     <= 1'b0;
            aw_captured_r <= 1'b0;
            w_captured_r  <= 1'b0;
            aw_word_addr_r <= 6'd0;
            w_data_r      <= 32'd0;
            w_strb_r      <= 4'h0;
            b_valid_r     <= 1'b0;
            b_resp_r      <= P_AXI_RESP_OKAY;
        end else begin
            if (!b_valid_r) begin
                // Independent AW channel capture
                if (aw_hs_w) begin
                    aw_word_addr_r <= s_axi_awaddr[7:2];
                    aw_captured_r <= 1'b1;
                    aw_ready_r    <= 1'b0;
                end else if (!aw_captured_r) begin
                    aw_ready_r    <= 1'b1;
                end

                // Independent W channel capture
                if (w_hs_w) begin
                    w_data_r      <= s_axi_wdata;
                    w_strb_r      <= s_axi_wstrb;
                    w_captured_r  <= 1'b1;
                    w_ready_r     <= 1'b0;
                end else if (!w_captured_r) begin
                    w_ready_r     <= 1'b1;
                end

                // Commit write response once both AW and W have completed
                if (wr_fire_w) begin
                    b_valid_r     <= 1'b1;
                    b_resp_r      <= P_AXI_RESP_OKAY;
                    aw_ready_r    <= 1'b0;
                    w_ready_r     <= 1'b0;
                end
            end else if (b_hs_w) begin
                // B response accepted: immediately re-arm AW and W for next transaction
                b_valid_r     <= 1'b0;
                aw_captured_r <= 1'b0;
                w_captured_r  <= 1'b0;
                aw_ready_r    <= 1'b1;
                w_ready_r     <= 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // CSR Write Logic, Shadow Snapshot Capture & Interrupt Management
    //--------------------------------------------------------------------------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            ctrl_clk_en_r      <= (C_DEFAULT_CLK_EN != 0) ? 1'b1 : 1'b0;
            ctrl_data_lock_r   <= 1'b0;
            ctrl_cmd_bit_en_r  <= 1'b0;
            ctrl_reset_pulse_r <= 1'b0;
            ctrl_reset_hold_r  <= 1'b0;
            ctrl_sample_edge_r <= 1'b0;
            ctrl_crc_mode_r    <= 1'b0;
            ctrl_crc_bypass_r  <= 1'b0;
            ctrl_clear_stats_r <= 1'b0;
            ctrl_soft_rst_r    <= 1'b0;
            ctrl_msg1_only_r   <= (C_DEFAULT_MSG1_ONLY != 0) ? 1'b1 : 1'b0;
            ctrl_msg2_only_r   <= (C_DEFAULT_MSG2_ONLY != 0) ? 1'b1 : 1'b0;

            int_status_r       <= 7'b000_0000;
            int_enable_r       <= 7'b000_0000;
            irq_r              <= 1'b0;
            timeout_cfg_r      <= C_DEFAULT_TIMEOUT_US;

            shd_ang_rate_x_r   <= 16'd0;
            shd_ang_rate_y_r   <= 16'd0;
            shd_ang_rate_z_r   <= 16'd0;
            shd_lin_acc_x_r    <= 16'd0;
            shd_lin_acc_y_r    <= 16'd0;
            shd_lin_acc_z_r    <= 16'd0;
            shd_status_word1_r <= 16'd0;
            shd_status_word2_r <= 16'd0;
            shd_mc_timestamp_r <= 32'd0;
            shd_delta_ang_x_r  <= 32'd0;
            shd_delta_ang_y_r  <= 32'd0;
            shd_delta_ang_z_r  <= 32'd0;
            shd_delta_vel_x_r  <= 32'd0;
            shd_delta_vel_y_r  <= 32'd0;
            shd_delta_vel_z_r  <= 32'd0;
            shd_sp_timestamp_r <= 32'd0;
        end else begin
            // Self-clearing W1P control strobes
            ctrl_reset_pulse_r <= 1'b0;
            ctrl_clear_stats_r <= 1'b0;
            ctrl_soft_rst_r    <= 1'b0;

            // Lossless Interrupt Status: HW set takes priority over SW W1C clear
            int_status_r <= int_status_next_w;

            // Registered glitch-free interrupt output (deasserts same cycle as W1C BVALID).
            // Every W1C write forces irq_o low for one cycle: if an enabled flag is
            // still pending (set before or during the ISR), irq_o rises again and
            // an EDGE-triggered GIC sees a new rising edge instead of a line stuck
            // high forever. Harmless for a level-triggered GIC.
            irq_r <= (|(int_status_next_w & int_enable_next_w)) &&
                     (sw_w1c_mask_w == 7'b000_0000);

            // AXI Register Writes
            if (wr_fire_w) begin
                case (wr_word_addr_w)
                    P_REG_CTRL: begin
                        if (wr_strb_w[0]) begin
                            ctrl_clk_en_r      <= wr_data_w[0];
                            ctrl_data_lock_r   <= wr_data_w[1];
                            ctrl_cmd_bit_en_r  <= wr_data_w[2];
                            ctrl_reset_pulse_r <= wr_data_w[3];
                            ctrl_reset_hold_r  <= wr_data_w[4];
                            ctrl_sample_edge_r <= wr_data_w[5];
                            ctrl_crc_mode_r    <= wr_data_w[6];
                            ctrl_crc_bypass_r  <= wr_data_w[7];

                            // Atomically snapshot all live IMU registers on DATA_LOCK 0 -> 1
                            if (!ctrl_data_lock_r && wr_data_w[1]) begin
                                shd_ang_rate_x_r   <= live_ang_rate_x_i;
                                shd_ang_rate_y_r   <= live_ang_rate_y_i;
                                shd_ang_rate_z_r   <= live_ang_rate_z_i;
                                shd_lin_acc_x_r    <= live_lin_acc_x_i;
                                shd_lin_acc_y_r    <= live_lin_acc_y_i;
                                shd_lin_acc_z_r    <= live_lin_acc_z_i;
                                shd_status_word1_r <= live_status_word1_i;
                                shd_status_word2_r <= live_status_word2_i;
                                shd_mc_timestamp_r <= live_mc_timestamp_i;
                                shd_delta_ang_x_r  <= live_delta_ang_x_i;
                                shd_delta_ang_y_r  <= live_delta_ang_y_i;
                                shd_delta_ang_z_r  <= live_delta_ang_z_i;
                                shd_delta_vel_x_r  <= live_delta_vel_x_i;
                                shd_delta_vel_y_r  <= live_delta_vel_y_i;
                                shd_delta_vel_z_r  <= live_delta_vel_z_i;
                                shd_sp_timestamp_r <= live_sp_timestamp_i;
                            end
                        end
                        if (wr_strb_w[1]) begin
                            ctrl_clear_stats_r <= wr_data_w[8];
                            ctrl_soft_rst_r    <= wr_data_w[9];
                            ctrl_msg1_only_r   <= wr_data_w[10];
                            ctrl_msg2_only_r   <= wr_data_w[11];
                        end
                    end

                    P_REG_INT_ENABLE: begin
                        if (wr_strb_w[0]) begin
                            int_enable_r <= wr_data_w[6:0];
                        end
                    end

                    P_REG_TIMEOUT_CFG: begin
                        if (wr_strb_w[0]) timeout_cfg_r[7:0]   <= wr_data_w[7:0];
                        if (wr_strb_w[1]) timeout_cfg_r[15:8]  <= wr_data_w[15:8];
                        if (wr_strb_w[2]) timeout_cfg_r[23:16] <= wr_data_w[23:16];
                        if (wr_strb_w[3]) timeout_cfg_r[31:24] <= wr_data_w[31:24];
                    end

                    default: begin
                        // Read-only or reserved registers: ignore writes (RAZ/WI)
                    end
                endcase
            end
        end
    end

    //--------------------------------------------------------------------------
    // Combinational Read Multiplexer (default assignment prevents latches)
    //--------------------------------------------------------------------------
    reg [31:0] rd_mux_c;

    always @(*) begin
        rd_mux_c = 32'h0000_0000;
        case (rd_word_addr_w)
            P_REG_CORE_ID: begin
                rd_mux_c = P_CORE_ID;
            end

            P_REG_CTRL: begin
                rd_mux_c = {
                    20'd0,
                    ctrl_msg2_only_r,   // [11] MSG2_ONLY_MODE (100 Hz)
                    ctrl_msg1_only_r,   // [10] MSG1_ONLY_MODE (600 Hz)
                    1'b0,               // [9]  SOFT_RESET (W1P -> reads 0)
                    1'b0,               // [8]  CLEAR_STATS (W1P -> reads 0)
                    ctrl_crc_bypass_r,  // [7]
                    ctrl_crc_mode_r,    // [6]
                    ctrl_sample_edge_r, // [5]
                    ctrl_reset_hold_r,  // [4]
                    1'b0,               // [3]  IMU_RESET_PULSE (W1P -> reads 0)
                    ctrl_cmd_bit_en_r,  // [2]
                    ctrl_data_lock_r,   // [1]
                    ctrl_clk_en_r       // [0]
                };
            end

            P_REG_STATUS: begin
                rd_mux_c = {
                    6'd0,               // [31:26] Reserved
                    ctrl_msg2_only_r,   // [25]    MSG2_ONLY_MODE active (100 Hz)
                    ctrl_msg1_only_r,   // [24]    MSG1_ONLY_MODE active
                    last_err_code_i,    // [23:20] Last error mask [Len,Addr,Align,CRC]
                    last_seq_cnt_i,     // [19:16] Latest 4-bit counter
                    last_msg_id_i,      // [15:8]  Last valid Msg ID (0x01 or 0x02)
                    sdlc_in_frame_i,    // [7]     SDLC synced
                    ctrl_cmd_bit_en_r,  // [6]     Commanded BIT asserted
                    imu_reset_busy_i,   // [5]     Timed RESET_L active
                    ctrl_data_lock_r,   // [4]     Shadow registers locked
                    module_failed_i,    // [3]     Status Word 1[4] latched Module Fail
                    imu_fault_active_i, // [2]     Active BIT fault
                    data_active_i,      // [1]     Valid data stream active
                    flags_alive_i       // [0]     SDLC 0x7E flags detected
                };
            end

            P_REG_INT_STATUS:    rd_mux_c = {25'd0, int_status_r};
            P_REG_INT_ENABLE:    rd_mux_c = {25'd0, int_enable_r};
            P_REG_TIMEOUT_CFG:   rd_mux_c = timeout_cfg_r;
            P_REG_MC_TIMESTAMP:  rd_mux_c = sel_mc_timestamp_w;
            P_REG_SP_TIMESTAMP:  rd_mux_c = sel_sp_timestamp_w;

            // Motion Control 16-bit Signed Values (Sign-Extended to 32-bit)
            P_REG_ANG_RATE_X:    rd_mux_c = {{16{sel_ang_rate_x_w[15]}}, sel_ang_rate_x_w};
            P_REG_ANG_RATE_Y:    rd_mux_c = {{16{sel_ang_rate_y_w[15]}}, sel_ang_rate_y_w};
            P_REG_ANG_RATE_Z:    rd_mux_c = {{16{sel_ang_rate_z_w[15]}}, sel_ang_rate_z_w};
            P_REG_LIN_ACC_X:     rd_mux_c = {{16{sel_lin_acc_x_w[15]}},  sel_lin_acc_x_w};
            P_REG_LIN_ACC_Y:     rd_mux_c = {{16{sel_lin_acc_y_w[15]}},  sel_lin_acc_y_w};
            P_REG_LIN_ACC_Z:     rd_mux_c = {{16{sel_lin_acc_z_w[15]}},  sel_lin_acc_z_w};
            P_REG_STATUS_WORD1:  rd_mux_c = {16'h0000, sel_status_word1_w};
            P_REG_STATUS_WORD2:  rd_mux_c = {16'h0000, sel_status_word2_w};

            // Spatial Data 32-bit Signed Values
            P_REG_DELTA_ANG_X:   rd_mux_c = sel_delta_ang_x_w;
            P_REG_DELTA_ANG_Y:   rd_mux_c = sel_delta_ang_y_w;
            P_REG_DELTA_ANG_Z:   rd_mux_c = sel_delta_ang_z_w;
            P_REG_DELTA_VEL_X:   rd_mux_c = sel_delta_vel_x_w;
            P_REG_DELTA_VEL_Y:   rd_mux_c = sel_delta_vel_y_w;
            P_REG_DELTA_VEL_Z:   rd_mux_c = sel_delta_vel_z_w;

            // Diagnostics & Statistics
            P_REG_MSG1_RX_CNT:   rd_mux_c = msg1_rx_cnt_i;
            P_REG_MSG2_RX_CNT:   rd_mux_c = msg2_rx_cnt_i;
            P_REG_CRC_ERR_CNT:   rd_mux_c = crc_err_cnt_i;
            P_REG_FRAME_ERR_CNT: rd_mux_c = {abort_err_cnt_i, frame_err_cnt_i};
            P_REG_SEQ_ERR_CNT:   rd_mux_c = seq_err_cnt_i;
            P_REG_DEBUG_CRC:     rd_mux_c = {debug_frame_addr_i, 2'b00, debug_frame_len_i, debug_crc_residue_i};

            default:             rd_mux_c = 32'h0000_0000;
        endcase
    end

    //--------------------------------------------------------------------------
    // AXI4-Lite Read Channel Control (AR, R)
    //--------------------------------------------------------------------------
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            ar_ready_r <= 1'b0;
            r_valid_r  <= 1'b0;
            r_data_r   <= 32'd0;
            r_resp_r   <= P_AXI_RESP_OKAY;
        end else begin
            if (!r_valid_r) begin
                if (ar_hs_w) begin
                    ar_ready_r <= 1'b0;
                    r_valid_r  <= 1'b1;
                    r_data_r   <= rd_mux_c;
                    r_resp_r   <= P_AXI_RESP_OKAY;
                end else begin
                    ar_ready_r <= 1'b1;
                end
            end else if (r_hs_w) begin
                r_valid_r  <= 1'b0;
                ar_ready_r <= 1'b1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Output Assignments
    //--------------------------------------------------------------------------
    assign s_axi_awready      = aw_ready_r;
    assign s_axi_wready       = w_ready_r;
    assign s_axi_bresp        = b_resp_r;
    assign s_axi_bvalid       = b_valid_r;
    assign s_axi_arready      = ar_ready_r;
    assign s_axi_rdata        = r_data_r;
    assign s_axi_rresp        = r_resp_r;
    assign s_axi_rvalid       = r_valid_r;
    assign irq_o              = irq_r;

    assign ctrl_clk_en_o      = ctrl_clk_en_r;
    assign ctrl_cmd_bit_en_o  = ctrl_cmd_bit_en_r;
    assign ctrl_reset_pulse_o = ctrl_reset_pulse_r;
    assign ctrl_reset_hold_o  = ctrl_reset_hold_r;
    assign ctrl_sample_edge_o = ctrl_sample_edge_r;
    assign ctrl_crc_mode_o    = ctrl_crc_mode_r;
    assign ctrl_crc_bypass_o  = ctrl_crc_bypass_r;
    assign ctrl_clear_stats_o = ctrl_clear_stats_r;
    assign ctrl_soft_rst_o    = ctrl_soft_rst_r;
    assign ctrl_msg1_only_o   = ctrl_msg1_only_r;
    assign ctrl_msg2_only_o   = ctrl_msg2_only_r;
    assign cfg_timeout_us_o   = timeout_cfg_r;

endmodule

`default_nettype wire
