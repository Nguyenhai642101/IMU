//==============================================================================
// Module      : sm5800_msg_parser
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001)
// Description : SM-5800 Message Validator, Parser, Watchdog & Diagnostics:
//               - Validates SDLC frames against SM-5800 Table 3 & Sec. 5.2:
//                 * Module Addr == 8'h0A
//                 * Msg ID 1 (Motion Control)          : 20 bytes (Addr+ID+16B+2B CRC)
//                 * Msg ID 2 (Motion Control + Spatial): 44 bytes (Addr+ID+40B+2B CRC)
//                 * Octet-aligned & CRC-16-CCITT valid (unless crc_bypass_i=1)
//               - Silently discards runt frames (< 32 bits between flags,
//                 ISO 3309 / ISO 13239) so line noise or a stuck-low line
//                 between frames never raises RX_ERROR or error counters.
//               - Assembles Little-Endian 16-bit & 32-bit fields in staging
//                 registers and atomically commits to live outputs ONLY when
//                 a frame passes ALL checks (zero corruption on bad frames).
//               - Tracks 4-bit rolling counter continuity (Status Word 1[3:0]),
//                 BIT fault indicators (Status Word 1[7:4], Word 2[15:11]),
//                 SDLC flag activity, and configurable data-stream watchdog.
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_msg_parser #(
    // Extra watchdog time after start-up / soft reset / RESET_L release: the
    // SM-5800 sends only flags until power-up BIT ends (< 0.8 s, Sec. 5.1/6.4)
    parameter integer C_ACTIVATION_GRACE_US = 1_000_000,
    // Extra watchdog time after BIT_L release: results <= 0.2 s after BIT
    // start, normal output <= 0.1 s after release (Sec. 6.3.2)
    parameter integer C_BIT_GRACE_US        = 300_000
) (
    input  wire        clk_i,                // System clock (s_axi_aclk)
    input  wire        rst_ni,               // Synchronous active-low reset
    input  wire        soft_rst_i,           // Synchronous soft reset pulse
    input  wire        clear_stats_i,        // 1-cycle strobe: clear diagnostic counters

    // Configuration & system state inputs
    input  wire        crc_bypass_i,         // 1: ignore CRC error (debug only)
    input  wire        msg1_only_mode_i,     // 1: commit Motion Control only on Msg ID 01
    input  wire        msg2_only_mode_i,     // 1: commit MC+Spatial only on Msg ID 02 (100 Hz)
    input  wire        bit_cmd_active_i,     // 1: Commanded BIT asserted (suspends seq check)
    input  wire        imu_reset_active_i,   // 1: IMU RESET_L asserted (suspends seq check)
    input  wire [31:0] timeout_us_i,         // Data watchdog timeout in us (0 = disabled)
    input  wire        tick_1us_i,           // 1 us timing strobe
    input  wire [31:0] timestamp_us_i,       // Free-running 32-bit 1 us timestamp

    // Deframed byte stream & events from sm5800_sdlc_rx
    input  wire        rx_byte_valid_i,
    input  wire [7:0]  rx_byte_i,
    input  wire        frame_end_i,
    input  wire        frame_abort_i,
    input  wire        frame_crc_ok_i,
    input  wire        frame_align_err_i,
    input  wire [15:0] frame_crc_residue_i,
    input  wire        flag_detected_i,

    // Live atomically-updated Motion Control Data (600 Hz: Msg 1 & Msg 2)
    output wire [15:0] ang_rate_x_o,         // Signed 16-bit (LSB = 2^-20 * 600 rad/s)
    output wire [15:0] ang_rate_y_o,         // Signed 16-bit
    output wire [15:0] ang_rate_z_o,         // Signed 16-bit
    output wire [15:0] lin_acc_x_o,          // Signed 16-bit (LSB = 2^-14 * 600 ft/s^2)
    output wire [15:0] lin_acc_y_o,          // Signed 16-bit
    output wire [15:0] lin_acc_z_o,          // Signed 16-bit
    output wire [15:0] status_word1_o,       // Raw Status Word 1 (Sec. 5.2.1)
    output wire [15:0] status_word2_o,       // Raw Status Word 2 (Sec. 5.2.1)
    output wire [31:0] mc_timestamp_o,       // Timestamp (us) of last valid MC update

    // Live atomically-updated Spatial Data (100 Hz: Msg 2 only)
    output wire [31:0] delta_ang_x_o,        // Signed 32-bit (LSB = 2^-33 rad)
    output wire [31:0] delta_ang_y_o,        // Signed 32-bit
    output wire [31:0] delta_ang_z_o,        // Signed 32-bit
    output wire [31:0] delta_vel_x_o,        // Signed 32-bit (LSB = 2^-27 ft/s)
    output wire [31:0] delta_vel_y_o,        // Signed 32-bit
    output wire [31:0] delta_vel_z_o,        // Signed 32-bit
    output wire [31:0] sp_timestamp_o,       // Timestamp (us) of last valid Spatial update

    // Status & Interrupt Event Pulses (Distinct Msg 01 vs Msg 02 Outputs)
    output wire        msg1_valid_o,         // 1-cycle pulse strictly on valid Msg ID 01
    output wire        msg2_valid_o,         // 1-cycle pulse strictly on valid Msg ID 02
    output wire        mc_data_valid_o,      // 1-cycle pulse when Motion Control registers commit
    output wire        sp_data_valid_o,      // 1-cycle pulse when Spatial registers commit (Msg 02)
    output wire        imu_fault_pulse_o,    // 1-cycle pulse when committed frame reports fault
    output wire        rx_err_pulse_o,       // 1-cycle pulse on rejected/aborted frame
    output wire        seq_err_pulse_o,      // 1-cycle pulse on 4-bit counter gap
    output wire        link_timeout_pulse_o, // 1-cycle pulse when data watchdog expires

    output wire        flags_alive_o,        // 1: receiving 0x7E flags within 2 ms window
    output wire        data_active_o,        // 1: receiving valid data within watchdog window
    output wire        imu_fault_active_o,   // 1: latest valid Status Word 1/2 indicates fault
    output wire        module_failed_o,      // 1: Status Word 1 bit[4] Module Failed
    output wire [7:0]  last_msg_id_o,        // Last valid Module Msg ID (0x01 or 0x02)
    output wire [3:0]  last_seq_cnt_o,       // Latest Status Word 1[3:0] 4-bit counter
    output wire [3:0]  last_err_code_o,      // Last rejected frame error mask [Len,Addr,Align,CRC]

    // Diagnostics & Statistics
    output wire [31:0] msg1_rx_cnt_o,
    output wire [31:0] msg2_rx_cnt_o,
    output wire [31:0] crc_err_cnt_o,
    output wire [15:0] frame_err_cnt_o,
    output wire [15:0] abort_err_cnt_o,
    output wire [31:0] seq_err_cnt_o,
    output wire [15:0] debug_crc_residue_o,
    output wire [5:0]  debug_frame_len_o,
    output wire [7:0]  debug_frame_addr_o
);

    //--------------------------------------------------------------------------
    // SM-5800 Protocol Constants (Table 3, Sec. 5.2.1 & 5.2.2)
    //--------------------------------------------------------------------------
    localparam [7:0]  P_MODULE_ADDR   = 8'h0A;
    localparam [7:0]  P_MSG_ID_MC     = 8'h01; // Motion Control Data
    localparam [7:0]  P_MSG_ID_MC_SP  = 8'h02; // Motion Control + Spatial Data
    localparam [5:0]  P_LEN_MSG1      = 6'd20; // Addr(1) + ID(1) + Data(16) + CRC(2)
    localparam [5:0]  P_LEN_MSG2      = 6'd44; // Addr(1) + ID(1) + Data(40) + CRC(2)
    localparam [15:0] P_FLAG_ALIVE_US = 16'd2000; // 2 ms flag activity window
    // ISO 3309: fewer than 32 bits between flags is not a frame -> ignore silently
    localparam [5:0]  P_MIN_FRAME_BYTES = 6'd4;

    //--------------------------------------------------------------------------
    // Staging Registers (filled during frame reception, never exposed directly)
    //--------------------------------------------------------------------------
    reg [5:0]  byte_cnt_r;
    reg        stg_overflow_r;
    reg [7:0]  stg_addr_r;
    reg [7:0]  stg_msg_id_r;

    reg [15:0] stg_ang_rate_x_r;
    reg [15:0] stg_ang_rate_y_r;
    reg [15:0] stg_ang_rate_z_r;
    reg [15:0] stg_lin_acc_x_r;
    reg [15:0] stg_lin_acc_y_r;
    reg [15:0] stg_lin_acc_z_r;
    reg [15:0] stg_status_word1_r;
    reg [15:0] stg_status_word2_r;

    reg [31:0] stg_delta_ang_x_r;
    reg [31:0] stg_delta_ang_y_r;
    reg [31:0] stg_delta_ang_z_r;
    reg [31:0] stg_delta_vel_x_r;
    reg [31:0] stg_delta_vel_y_r;
    reg [31:0] stg_delta_vel_z_r;

    //--------------------------------------------------------------------------
    // Live Committed Output Registers
    //--------------------------------------------------------------------------
    reg [15:0] live_ang_rate_x_r;
    reg [15:0] live_ang_rate_y_r;
    reg [15:0] live_ang_rate_z_r;
    reg [15:0] live_lin_acc_x_r;
    reg [15:0] live_lin_acc_y_r;
    reg [15:0] live_lin_acc_z_r;
    reg [15:0] live_status_word1_r;
    reg [15:0] live_status_word2_r;
    reg [31:0] live_mc_timestamp_r;

    reg [31:0] live_delta_ang_x_r;
    reg [31:0] live_delta_ang_y_r;
    reg [31:0] live_delta_ang_z_r;
    reg [31:0] live_delta_vel_x_r;
    reg [31:0] live_delta_vel_y_r;
    reg [31:0] live_delta_vel_z_r;
    reg [31:0] live_sp_timestamp_r;

    //--------------------------------------------------------------------------
    // Status, Event, Watchdog & Diagnostic Registers
    //--------------------------------------------------------------------------
    reg        msg1_valid_r;
    reg        msg2_valid_r;
    reg        mc_data_valid_r;
    reg        sp_data_valid_r;
    reg        imu_fault_pulse_r;
    reg        rx_err_pulse_r;
    reg        seq_err_pulse_r;
    reg        link_timeout_pulse_r;

    reg        has_prev_seq_r;
    reg [3:0]  prev_seq_cnt_r;
    reg [7:0]  last_msg_id_r;
    reg [3:0]  last_err_code_r;

    reg [15:0] flag_idle_us_r;
    reg        flags_alive_r;
    reg [31:0] data_idle_us_r;
    reg        data_active_r;
    reg        wd_armed_r;      // 1: watchdog counting (armed at reset & on each valid frame)
    reg        inhibit_wd_d1_r; // Previous cycle state of (imu_reset_active_i || bit_cmd_active_i)
    reg        imu_reset_d1_r;  // Previous cycle imu_reset_active_i (release detect)
    reg        bit_cmd_d1_r;    // Previous cycle bit_cmd_active_i (release detect)
    reg [31:0] wd_grace_us_r;   // Extra watchdog time until the first valid frame

    reg [31:0] msg1_rx_cnt_r;
    reg [31:0] msg2_rx_cnt_r;
    reg [31:0] crc_err_cnt_r;
    reg [15:0] frame_err_cnt_r;
    reg [15:0] abort_err_cnt_r;
    reg [31:0] seq_err_cnt_r;

    reg [15:0] debug_crc_residue_r;
    reg [5:0]  debug_frame_len_r;
    reg [7:0]  debug_frame_addr_r;

    //--------------------------------------------------------------------------
    // Combinational Frame Validation Wires (evaluated on frame_end_i)
    //--------------------------------------------------------------------------
    wire       align_ok_w;
    wire       crc_pass_w;
    wire       addr_ok_w;
    wire       is_msg1_w;
    wire       is_msg2_w;
    wire       len_id_ok_w;
    wire       frame_valid_w;
    wire       frame_runt_w;
    wire       commit_frame_w;
    wire       stg_fault_w;
    wire [3:0] expected_seq_w;
    wire       inhibit_wd_w;
    wire       wd_frame_ok_w;
    wire [32:0] wd_limit_w;

    localparam [31:0] P_ACT_GRACE_US = C_ACTIVATION_GRACE_US;
    localparam [31:0] P_BIT_GRACE_US = C_BIT_GRACE_US;

    assign align_ok_w     = !frame_align_err_i;
    assign crc_pass_w     = frame_crc_ok_i || crc_bypass_i;
    assign addr_ok_w      = (stg_addr_r == P_MODULE_ADDR);
    assign is_msg1_w      = (stg_msg_id_r == P_MSG_ID_MC)    && (byte_cnt_r == P_LEN_MSG1) && !stg_overflow_r;
    assign is_msg2_w      = (stg_msg_id_r == P_MSG_ID_MC_SP) && (byte_cnt_r == P_LEN_MSG2) && !stg_overflow_r;
    assign len_id_ok_w    = is_msg1_w || is_msg2_w;
    assign frame_valid_w  = align_ok_w && crc_pass_w && addr_ok_w && len_id_ok_w;
    assign frame_runt_w   = (byte_cnt_r < P_MIN_FRAME_BYTES);
    assign commit_frame_w = is_msg1_w ? (!msg2_only_mode_i)
                                      : (msg2_only_mode_i || !msg1_only_mode_i);

    // Status Word 1[7:4] (Mod Fail, C/B/A Gyro PLC) or Status Word 2[15:11] (BIT Tests)
    assign stg_fault_w    = (|stg_status_word1_r[7:4]) || (|stg_status_word2_r[15:11]);
    assign expected_seq_w = prev_seq_cnt_r + 4'd1;
    assign inhibit_wd_w   = imu_reset_active_i || bit_cmd_active_i;
    // Valid frame of the selected stream (resets the data watchdog)
    assign wd_frame_ok_w  = frame_end_i && frame_valid_w &&
                            (msg2_only_mode_i ? is_msg2_w : (msg1_only_mode_i ? is_msg1_w : 1'b1));
    // 33-bit sum: no wrap-around for any TIMEOUT_CFG + grace combination
    assign wd_limit_w     = {1'b0, timeout_us_i} + {1'b0, wd_grace_us_r};

    //--------------------------------------------------------------------------
    // Staging Buffer Byte Collector
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            byte_cnt_r         <= 6'd0;
            stg_overflow_r     <= 1'b0;
            stg_addr_r         <= 8'h00;
            stg_msg_id_r       <= 8'h00;
            stg_ang_rate_x_r   <= 16'd0;
            stg_ang_rate_y_r   <= 16'd0;
            stg_ang_rate_z_r   <= 16'd0;
            stg_lin_acc_x_r    <= 16'd0;
            stg_lin_acc_y_r    <= 16'd0;
            stg_lin_acc_z_r    <= 16'd0;
            stg_status_word1_r <= 16'd0;
            stg_status_word2_r <= 16'd0;
            stg_delta_ang_x_r  <= 32'd0;
            stg_delta_ang_y_r  <= 32'd0;
            stg_delta_ang_z_r  <= 32'd0;
            stg_delta_vel_x_r  <= 32'd0;
            stg_delta_vel_y_r  <= 32'd0;
            stg_delta_vel_z_r  <= 32'd0;
        end else if (soft_rst_i || frame_end_i || frame_abort_i) begin
            byte_cnt_r     <= 6'd0;
            stg_overflow_r <= 1'b0;
        end else if (rx_byte_valid_i) begin
            // Store incoming byte into Little-Endian staging fields
            case (byte_cnt_r)
                6'd0:  stg_addr_r               <= rx_byte_i;
                6'd1:  stg_msg_id_r             <= rx_byte_i;
                // Motion Control Data (16 bytes, Little-Endian)
                6'd2:  stg_ang_rate_x_r[7:0]    <= rx_byte_i;
                6'd3:  stg_ang_rate_x_r[15:8]   <= rx_byte_i;
                6'd4:  stg_ang_rate_y_r[7:0]    <= rx_byte_i;
                6'd5:  stg_ang_rate_y_r[15:8]   <= rx_byte_i;
                6'd6:  stg_ang_rate_z_r[7:0]    <= rx_byte_i;
                6'd7:  stg_ang_rate_z_r[15:8]   <= rx_byte_i;
                6'd8:  stg_lin_acc_x_r[7:0]     <= rx_byte_i;
                6'd9:  stg_lin_acc_x_r[15:8]    <= rx_byte_i;
                6'd10: stg_lin_acc_y_r[7:0]     <= rx_byte_i;
                6'd11: stg_lin_acc_y_r[15:8]    <= rx_byte_i;
                6'd12: stg_lin_acc_z_r[7:0]     <= rx_byte_i;
                6'd13: stg_lin_acc_z_r[15:8]    <= rx_byte_i;
                6'd14: stg_status_word1_r[7:0]  <= rx_byte_i;
                6'd15: stg_status_word1_r[15:8] <= rx_byte_i;
                6'd16: stg_status_word2_r[7:0]  <= rx_byte_i;
                6'd17: stg_status_word2_r[15:8] <= rx_byte_i;
                // Spatial Data (24 bytes, Msg ID 2 only, Little-Endian)
                // (For Msg ID 1, bytes 18..19 are CRC and are never committed to Spatial)
                6'd18: stg_delta_ang_x_r[7:0]   <= rx_byte_i;
                6'd19: stg_delta_ang_x_r[15:8]  <= rx_byte_i;
                6'd20: stg_delta_ang_x_r[23:16] <= rx_byte_i;
                6'd21: stg_delta_ang_x_r[31:24] <= rx_byte_i;
                6'd22: stg_delta_ang_y_r[7:0]   <= rx_byte_i;
                6'd23: stg_delta_ang_y_r[15:8]  <= rx_byte_i;
                6'd24: stg_delta_ang_y_r[23:16] <= rx_byte_i;
                6'd25: stg_delta_ang_y_r[31:24] <= rx_byte_i;
                6'd26: stg_delta_ang_z_r[7:0]   <= rx_byte_i;
                6'd27: stg_delta_ang_z_r[15:8]  <= rx_byte_i;
                6'd28: stg_delta_ang_z_r[23:16] <= rx_byte_i;
                6'd29: stg_delta_ang_z_r[31:24] <= rx_byte_i;
                6'd30: stg_delta_vel_x_r[7:0]   <= rx_byte_i;
                6'd31: stg_delta_vel_x_r[15:8]  <= rx_byte_i;
                6'd32: stg_delta_vel_x_r[23:16] <= rx_byte_i;
                6'd33: stg_delta_vel_x_r[31:24] <= rx_byte_i;
                6'd34: stg_delta_vel_y_r[7:0]   <= rx_byte_i;
                6'd35: stg_delta_vel_y_r[15:8]  <= rx_byte_i;
                6'd36: stg_delta_vel_y_r[23:16] <= rx_byte_i;
                6'd37: stg_delta_vel_y_r[31:24] <= rx_byte_i;
                6'd38: stg_delta_vel_z_r[7:0]   <= rx_byte_i;
                6'd39: stg_delta_vel_z_r[15:8]  <= rx_byte_i;
                6'd40: stg_delta_vel_z_r[23:16] <= rx_byte_i;
                6'd41: stg_delta_vel_z_r[31:24] <= rx_byte_i;
                // Bytes 42..43 are Msg ID 2 CRC (verified in sm5800_sdlc_rx)
                default: begin
                    if (byte_cnt_r >= P_LEN_MSG2) begin
                        stg_overflow_r <= 1'b1;
                    end
                end
            endcase

            // Saturating byte counter (max 63 prevents wrap-around aliasing)
            if (byte_cnt_r != 6'd63) begin
                byte_cnt_r <= byte_cnt_r + 6'd1;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Atomic Commit, Diagnostics & Sequence Checker
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            live_ang_rate_x_r   <= 16'd0;
            live_ang_rate_y_r   <= 16'd0;
            live_ang_rate_z_r   <= 16'd0;
            live_lin_acc_x_r    <= 16'd0;
            live_lin_acc_y_r    <= 16'd0;
            live_lin_acc_z_r    <= 16'd0;
            live_status_word1_r <= 16'd0;
            live_status_word2_r <= 16'd0;
            live_mc_timestamp_r <= 32'd0;
            live_delta_ang_x_r  <= 32'd0;
            live_delta_ang_y_r  <= 32'd0;
            live_delta_ang_z_r  <= 32'd0;
            live_delta_vel_x_r  <= 32'd0;
            live_delta_vel_y_r  <= 32'd0;
            live_delta_vel_z_r  <= 32'd0;
            live_sp_timestamp_r <= 32'd0;

            msg1_valid_r        <= 1'b0;
            msg2_valid_r        <= 1'b0;
            mc_data_valid_r     <= 1'b0;
            sp_data_valid_r     <= 1'b0;
            imu_fault_pulse_r   <= 1'b0;
            rx_err_pulse_r      <= 1'b0;
            seq_err_pulse_r     <= 1'b0;

            has_prev_seq_r      <= 1'b0;
            prev_seq_cnt_r      <= 4'd0;
            last_msg_id_r       <= 8'h00;
            last_err_code_r     <= 4'h0;

            msg1_rx_cnt_r       <= 32'd0;
            msg2_rx_cnt_r       <= 32'd0;
            crc_err_cnt_r       <= 32'd0;
            frame_err_cnt_r     <= 16'd0;
            abort_err_cnt_r     <= 16'd0;
            seq_err_cnt_r       <= 32'd0;

            debug_crc_residue_r <= 16'h0000;
            debug_frame_len_r   <= 6'd0;
            debug_frame_addr_r  <= 8'h00;
        end else begin
            // Default single-cycle pulses
            msg1_valid_r      <= 1'b0;
            msg2_valid_r      <= 1'b0;
            mc_data_valid_r   <= 1'b0;
            sp_data_valid_r   <= 1'b0;
            imu_fault_pulse_r <= 1'b0;
            rx_err_pulse_r    <= 1'b0;
            seq_err_pulse_r   <= 1'b0;

            // Resynchronize 4-bit sequence tracker across Reset / Commanded BIT / Soft Reset
            if (soft_rst_i || imu_reset_active_i || bit_cmd_active_i) begin
                has_prev_seq_r <= 1'b0;
            end

            // Clear diagnostic counters if requested by software
            if (clear_stats_i || soft_rst_i) begin
                msg1_rx_cnt_r   <= 32'd0;
                msg2_rx_cnt_r   <= 32'd0;
                crc_err_cnt_r   <= 32'd0;
                frame_err_cnt_r <= 16'd0;
                abort_err_cnt_r <= 16'd0;
                seq_err_cnt_r   <= 32'd0;
                last_err_code_r <= 4'h0;
            end

            if (!soft_rst_i) begin
                if (frame_abort_i) begin
                    rx_err_pulse_r  <= 1'b1;
                    abort_err_cnt_r <= (clear_stats_i ? 16'd0 : abort_err_cnt_r) + 16'd1;
                end else if (frame_end_i && !frame_runt_w) begin
                    // (Runt frames < 32 bits fall through: no error, no debug update)
                    // Always latch last-frame debug snapshot for bring-up diagnostics
                    debug_crc_residue_r <= frame_crc_residue_i;
                    debug_frame_len_r   <= byte_cnt_r;
                    debug_frame_addr_r  <= (byte_cnt_r != 6'd0) ? stg_addr_r : 8'h00;

                    if (frame_valid_w) begin
                        //------------------------------------------------------
                        // Valid SM-5800 Frame: Distinguish Msg ID 01 vs Msg ID 02
                        //------------------------------------------------------
                        last_msg_id_r <= stg_msg_id_r;

                        if (is_msg1_w) begin
                            // Message ID 01: Always pulse msg1_valid_o & count;
                            // commit Motion Control unless MSG2_ONLY_MODE (100 Hz) is active
                            msg1_valid_r  <= 1'b1;
                            msg1_rx_cnt_r <= (clear_stats_i ? 32'd0 : msg1_rx_cnt_r) + 32'd1;

                            if (!msg2_only_mode_i) begin
                                live_ang_rate_x_r   <= stg_ang_rate_x_r;
                                live_ang_rate_y_r   <= stg_ang_rate_y_r;
                                live_ang_rate_z_r   <= stg_ang_rate_z_r;
                                live_lin_acc_x_r    <= stg_lin_acc_x_r;
                                live_lin_acc_y_r    <= stg_lin_acc_y_r;
                                live_lin_acc_z_r    <= stg_lin_acc_z_r;
                                live_status_word1_r <= stg_status_word1_r;
                                live_status_word2_r <= stg_status_word2_r;
                                live_mc_timestamp_r <= timestamp_us_i;
                                mc_data_valid_r     <= 1'b1;
                            end
                        end else begin
                            // Message ID 02 (100 Hz): Always pulse msg2_valid_o & count;
                            // commit all 14 fields (MC + Spatial) in normal or MSG2_ONLY_MODE
                            msg2_valid_r  <= 1'b1;
                            msg2_rx_cnt_r <= (clear_stats_i ? 32'd0 : msg2_rx_cnt_r) + 32'd1;

                            if (msg2_only_mode_i || !msg1_only_mode_i) begin
                                live_ang_rate_x_r   <= stg_ang_rate_x_r;
                                live_ang_rate_y_r   <= stg_ang_rate_y_r;
                                live_ang_rate_z_r   <= stg_ang_rate_z_r;
                                live_lin_acc_x_r    <= stg_lin_acc_x_r;
                                live_lin_acc_y_r    <= stg_lin_acc_y_r;
                                live_lin_acc_z_r    <= stg_lin_acc_z_r;
                                live_status_word1_r <= stg_status_word1_r;
                                live_status_word2_r <= stg_status_word2_r;
                                live_mc_timestamp_r <= timestamp_us_i;
                                live_delta_ang_x_r  <= stg_delta_ang_x_r;
                                live_delta_ang_y_r  <= stg_delta_ang_y_r;
                                live_delta_ang_z_r  <= stg_delta_ang_z_r;
                                live_delta_vel_x_r  <= stg_delta_vel_x_r;
                                live_delta_vel_y_r  <= stg_delta_vel_y_r;
                                live_delta_vel_z_r  <= stg_delta_vel_z_r;
                                live_sp_timestamp_r <= timestamp_us_i;
                                mc_data_valid_r     <= 1'b1;
                                sp_data_valid_r     <= 1'b1;
                            end
                        end

                        // Check 4-bit rolling counter continuity (Status Word 1[3:0])
                        if (has_prev_seq_r && (stg_status_word1_r[3:0] != expected_seq_w)) begin
                            seq_err_pulse_r <= 1'b1;
                            seq_err_cnt_r   <= (clear_stats_i ? 32'd0 : seq_err_cnt_r) + 32'd1;
                        end
                        prev_seq_cnt_r <= stg_status_word1_r[3:0];
                        has_prev_seq_r <= 1'b1;

                        // Notify host when a committed frame reports any BIT fault
                        if (commit_frame_w && stg_fault_w) begin
                            imu_fault_pulse_r <= 1'b1;
                        end
                    end else begin
                        //------------------------------------------------------
                        // Rejected Frame: Preserve Live Data & Record Error
                        //------------------------------------------------------
                        rx_err_pulse_r  <= 1'b1;
                        last_err_code_r <= {~len_id_ok_w, ~addr_ok_w, frame_align_err_i, ~crc_pass_w};

                        if (!crc_pass_w) begin
                            crc_err_cnt_r <= (clear_stats_i ? 32'd0 : crc_err_cnt_r) + 32'd1;
                        end
                        if (!align_ok_w || !addr_ok_w || !len_id_ok_w) begin
                            frame_err_cnt_r <= (clear_stats_i ? 16'd0 : frame_err_cnt_r) + 16'd1;
                        end
                    end
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Flag Activity Detector & Data Stream Watchdog (1 us resolution)
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            flag_idle_us_r       <= 16'd0;
            flags_alive_r        <= 1'b0;
            data_idle_us_r       <= 32'd0;
            data_active_r        <= 1'b0;
            wd_armed_r           <= 1'b1; // detect a sensor that never sends data
            inhibit_wd_d1_r      <= 1'b0;
            imu_reset_d1_r       <= 1'b0;
            bit_cmd_d1_r         <= 1'b0;
            wd_grace_us_r        <= P_ACT_GRACE_US; // sensor may still be starting
            link_timeout_pulse_r <= 1'b0;
        end else if (soft_rst_i) begin
            flag_idle_us_r       <= 16'd0;
            flags_alive_r        <= 1'b0;
            data_idle_us_r       <= 32'd0;
            data_active_r        <= 1'b0;
            wd_armed_r           <= 1'b1;
            inhibit_wd_d1_r      <= 1'b0;
            imu_reset_d1_r       <= 1'b0;
            bit_cmd_d1_r         <= 1'b0;
            wd_grace_us_r        <= P_ACT_GRACE_US;
            link_timeout_pulse_r <= 1'b0;
        end else begin
            link_timeout_pulse_r <= 1'b0;
            inhibit_wd_d1_r      <= inhibit_wd_w;
            imu_reset_d1_r       <= imu_reset_active_i;
            bit_cmd_d1_r         <= bit_cmd_active_i;

            // Datasheet grace: cleared by the first valid frame; reloaded when
            // RESET_L (sensor restarts) or BIT_L (BIT completes) is released
            if (wd_frame_ok_w) begin
                wd_grace_us_r <= 32'd0;
            end else if (imu_reset_d1_r && !imu_reset_active_i) begin
                wd_grace_us_r <= P_ACT_GRACE_US;
            end else if (bit_cmd_d1_r && !bit_cmd_active_i) begin
                // Keep the longer window if a start-up grace is still running
                wd_grace_us_r <= (wd_grace_us_r > P_BIT_GRACE_US) ? wd_grace_us_r
                                                                  : P_BIT_GRACE_US;
            end

            // SDLC Flag (0x7E) presence monitor (alive during normal op, Power-Up & Cmd BIT)
            if (flag_detected_i) begin
                flag_idle_us_r <= 16'd0;
                flags_alive_r  <= 1'b1;
            end else if (tick_1us_i && flags_alive_r) begin
                if (flag_idle_us_r >= P_FLAG_ALIVE_US) begin
                    flags_alive_r <= 1'b0;
                end else begin
                    flag_idle_us_r <= flag_idle_us_r + 16'd1;
                end
            end

            // Valid data stream watchdog (filters by MSG2_ONLY / MSG1_ONLY mode)
            if (wd_frame_ok_w) begin
                data_idle_us_r <= 32'd0;
                data_active_r  <= 1'b1;
                wd_armed_r     <= 1'b1;
            end else if (inhibit_wd_d1_r && !inhibit_wd_w) begin
                // Re-arm watchdog on release (1 -> 0) of RESET_L or BIT_L while
                // preserving DATA_ACTIVE drop during active BIT/Reset testing
                data_idle_us_r <= 32'd0;
                wd_armed_r     <= 1'b1;
            end else if (tick_1us_i && wd_armed_r && (timeout_us_i != 32'd0)) begin
                // Armed from reset (start-up silence) and after every valid frame;
                // one-shot: disarms on expiry until the next valid frame arrives.
                if ({1'b0, data_idle_us_r} >= wd_limit_w) begin
                    data_active_r <= 1'b0;
                    wd_armed_r    <= 1'b0;
                    // Suppress false timeout IRQ during intentional IMU reset or Commanded BIT
                    if (!inhibit_wd_w) begin
                        link_timeout_pulse_r <= 1'b1;
                    end
                end else begin
                    data_idle_us_r <= data_idle_us_r + 32'd1;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Output Assignments
    //--------------------------------------------------------------------------
    assign ang_rate_x_o         = live_ang_rate_x_r;
    assign ang_rate_y_o         = live_ang_rate_y_r;
    assign ang_rate_z_o         = live_ang_rate_z_r;
    assign lin_acc_x_o          = live_lin_acc_x_r;
    assign lin_acc_y_o          = live_lin_acc_y_r;
    assign lin_acc_z_o          = live_lin_acc_z_r;
    assign status_word1_o       = live_status_word1_r;
    assign status_word2_o       = live_status_word2_r;
    assign mc_timestamp_o       = live_mc_timestamp_r;

    assign delta_ang_x_o        = live_delta_ang_x_r;
    assign delta_ang_y_o        = live_delta_ang_y_r;
    assign delta_ang_z_o        = live_delta_ang_z_r;
    assign delta_vel_x_o        = live_delta_vel_x_r;
    assign delta_vel_y_o        = live_delta_vel_y_r;
    assign delta_vel_z_o        = live_delta_vel_z_r;
    assign sp_timestamp_o       = live_sp_timestamp_r;

    assign msg1_valid_o         = msg1_valid_r;
    assign msg2_valid_o         = msg2_valid_r;
    assign mc_data_valid_o      = mc_data_valid_r;
    assign sp_data_valid_o      = sp_data_valid_r;
    assign imu_fault_pulse_o    = imu_fault_pulse_r;
    assign rx_err_pulse_o       = rx_err_pulse_r;
    assign seq_err_pulse_o      = seq_err_pulse_r;
    assign link_timeout_pulse_o = link_timeout_pulse_r;

    assign flags_alive_o        = flags_alive_r;
    assign data_active_o        = data_active_r;
    assign imu_fault_active_o   = (|live_status_word1_r[7:4]) || (|live_status_word2_r[15:11]);
    assign module_failed_o      = live_status_word1_r[4];
    assign last_msg_id_o        = last_msg_id_r;
    assign last_seq_cnt_o       = prev_seq_cnt_r;
    assign last_err_code_o      = last_err_code_r;

    assign msg1_rx_cnt_o        = msg1_rx_cnt_r;
    assign msg2_rx_cnt_o        = msg2_rx_cnt_r;
    assign crc_err_cnt_o        = crc_err_cnt_r;
    assign frame_err_cnt_o      = frame_err_cnt_r;
    assign abort_err_cnt_o      = abort_err_cnt_r;
    assign seq_err_cnt_o        = seq_err_cnt_r;
    assign debug_crc_residue_o  = debug_crc_residue_r;
    assign debug_frame_len_o    = debug_frame_len_r;
    assign debug_frame_addr_o   = debug_frame_addr_r;

endmodule

`default_nettype wire
