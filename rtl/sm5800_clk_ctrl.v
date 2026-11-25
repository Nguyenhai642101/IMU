//==============================================================================
// Module      : sm5800_clk_ctrl
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001)
// Description : Physical timing & control interface for the SM-5800 IMU:
//               1. Generates synchronous 1.0 MHz (+/-2%, 50% duty cycle)
//                  RCV_CLK output for the SM-5800 (Sec. 5.3, Table 5).
//               2. Synchronizes SER_DATA input via a 2-FF ASYNC_REG chain and
//                  samples NRZ serial bits at a configurable RCV_CLK edge.
//               3. Generates free-running 1 us tick & 32-bit timestamp counter.
//               4. Drives active-low RESET_L (open-collector compatible) with
//                  hardware-enforced >1 ms pulse duration (Sec. 4.4) and
//                  active-low BIT_L for Commanded BIT (Sec. 4.3.2 & 6.3.2).
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module sm5800_clk_ctrl #(
    parameter integer C_CLK_FREQ_HZ    = 100_000_000, // AXI clock frequency (Hz)
    parameter integer C_SDLC_CLK_HZ    = 1_000_000,   // Target RCV_CLK frequency (1 MHz)
    parameter integer C_RESET_PULSE_US = 2000         // Timed RESET_L pulse width (us, >1000 us)
) (
    input  wire        clk_i,            // System clock (s_axi_aclk)
    input  wire        rst_ni,           // Synchronous active-low reset

    // Control inputs from CSR
    input  wire        clk_en_i,         // Enable 1.0 MHz RCV_CLK output
    input  wire        sample_edge_i,    // 0: sample on RCV_CLK rising edge, 1: falling edge
    input  wire        bit_cmd_i,        // 1: assert BIT_L low (Commanded BIT)
    input  wire        reset_pulse_req_i,// 1-cycle strobe: trigger timed RESET_L low pulse
    input  wire        reset_hold_i,     // 1: manually hold RESET_L low

    // Synchronized serial bit stream to SDLC RX
    output wire        rx_bit_valid_o,   // 1-cycle strobe per sampled serial bit
    output wire        rx_bit_o,         // Sampled NRZ serial data bit

    // Timing & status outputs
    output wire        tick_1us_o,       // Free-running 1 us strobe
    output wire [31:0] timestamp_us_o,   // Free-running 32-bit 1 us timestamp
    output wire        reset_busy_o,     // 1: timed RESET_L pulse currently active

    // External SM-5800 interface signals (connect via RS-422 26C31/26C32 & OC drivers)
    output wire        imu_rcv_clk_o,    // 1.0 MHz clock to SM-5800 RCV_CLK (via 26C31)
    input  wire        imu_ser_data_i,   // Serial data from SM-5800 SER_DATA (via 26C32)
    output wire        imu_bit_l_o,      // Active-low Commanded BIT output (1=release, 0=assert)
    output wire        imu_reset_l_o     // Active-low Reset output (1=release/Hi-Z, 0=assert)
);

    //--------------------------------------------------------------------------
    // Derived timing parameters (computed as 32-bit integers, then sliced to
    // the 16-bit counter width; the guards below make the slice lossless)
    //--------------------------------------------------------------------------
    localparam integer P_CLK_PERIOD_INT  = (C_CLK_FREQ_HZ / C_SDLC_CLK_HZ);
    localparam integer P_HALF_LOW_INT    = (P_CLK_PERIOD_INT / 2);
    // Ceiling division ensures 1 us tick is >= 1.000 us on fractional-MHz clocks
    localparam integer P_US_PERIOD_INT   = ((C_CLK_FREQ_HZ + 999_999) / 1_000_000);
    localparam integer P_CLK_LAST_INT    = P_CLK_PERIOD_INT - 1;
    localparam integer P_US_LAST_INT     = P_US_PERIOD_INT - 1;
    // Add +1 us so pulse duration is strictly > C_RESET_PULSE_US regardless of phase
    localparam integer P_RST_LOAD_INT    = C_RESET_PULSE_US + 1;

    localparam [15:0] P_CLK_PERIOD_LAST = P_CLK_LAST_INT[15:0];
    localparam [15:0] P_HALF_LOW_CYC    = P_HALF_LOW_INT[15:0];
    localparam [15:0] P_US_PERIOD_LAST  = P_US_LAST_INT[15:0];
    localparam [15:0] P_RST_PULSE_LOAD  = P_RST_LOAD_INT[15:0];

    //--------------------------------------------------------------------------
    // Elaboration-time parameter guards (Verilog-2001 has no $error): an
    // illegal configuration connects a non-existent port of the empty module
    // sm5800_param_error; the port name states the problem, so synthesis /
    // simulation stops immediately (see rtl/sm5800_param_error.v).
    //   1. RCV_CLK must be within +/-2 % of C_SDLC_CLK_HZ and 50 % +/-2 % duty
    //      (SM-5800 Table 5): error = r / (P * f_target) <= 1/50, and P even
    //      or P >= 25.
    //   2. Counters are 16 bits: divider >= 2 and <= 65535 cycles
    //   3. RESET_L pulse must be > 1000 us (Sec. 4.4) and fit 16 bits
    //--------------------------------------------------------------------------
    generate
        if ((P_CLK_PERIOD_INT < 2) || (P_CLK_PERIOD_INT > 65535) ||
            (P_US_PERIOD_INT < 1) || (P_US_PERIOD_INT > 65535)) begin : g_err_div
            /* verilator lint_off PINNOTFOUND */
            sm5800_param_error u_param_error (.SM5800_PARAM_ERROR_CLK_DIVIDER_OUT_OF_16BIT_RANGE(1'b0));
            /* verilator lint_on PINNOTFOUND */
        end
        if (((50 * (C_CLK_FREQ_HZ % C_SDLC_CLK_HZ)) >
             (P_CLK_PERIOD_INT * C_SDLC_CLK_HZ)) ||
            (((P_CLK_PERIOD_INT % 2) != 0) && (P_CLK_PERIOD_INT < 25))) begin : g_err_freq
            /* verilator lint_off PINNOTFOUND */
            sm5800_param_error u_param_error (.SM5800_PARAM_ERROR_RCV_CLK_OUTSIDE_2PCT_TOLERANCE(1'b0));
            /* verilator lint_on PINNOTFOUND */
        end
        if ((C_RESET_PULSE_US <= 1000) || (C_RESET_PULSE_US >= 65535)) begin : g_err_rst
            /* verilator lint_off PINNOTFOUND */
            sm5800_param_error u_param_error (.SM5800_PARAM_ERROR_RESET_PULSE_US_NOT_IN_1001_TO_65534(1'b0));
            /* verilator lint_on PINNOTFOUND */
        end
    endgenerate

    //--------------------------------------------------------------------------
    // Internal registers
    //--------------------------------------------------------------------------
    reg [15:0] div_cnt_r;
    reg        rcv_clk_r;
    reg        rx_bit_valid_r;
    reg        rx_bit_r;

    (* ASYNC_REG = "TRUE" *) reg ser_data_meta_r;
    (* ASYNC_REG = "TRUE" *) reg ser_data_sync_r;

    reg [15:0] us_div_cnt_r;
    reg        tick_1us_r;
    reg [31:0] timestamp_us_r;

    reg [15:0] rst_pulse_cnt_r;
    reg        imu_reset_l_r;
    reg        imu_bit_l_r;

    //--------------------------------------------------------------------------
    // 2-Stage Input Synchronizer for asynchronous SER_DATA input
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            ser_data_meta_r <= 1'b1;
            ser_data_sync_r <= 1'b1;
        end else begin
            ser_data_meta_r <= imu_ser_data_i;
            ser_data_sync_r <= ser_data_meta_r;
        end
    end

    //--------------------------------------------------------------------------
    // 1.0 MHz RCV_CLK Generator and Synchronous NRZ Bit Sampler
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            div_cnt_r      <= 16'd0;
            rcv_clk_r      <= 1'b0;
            rx_bit_valid_r <= 1'b0;
            rx_bit_r       <= 1'b1;
        end else if (!clk_en_i) begin
            div_cnt_r      <= 16'd0;
            rcv_clk_r      <= 1'b0;
            rx_bit_valid_r <= 1'b0;
        end else begin
            rx_bit_valid_r <= 1'b0;

            // Clock period counter
            if (div_cnt_r == P_CLK_PERIOD_LAST) begin
                div_cnt_r <= 16'd0;
            end else begin
                div_cnt_r <= div_cnt_r + 16'd1;
            end

            // Generate 50% duty-cycle 1.0 MHz RCV_CLK
            if (div_cnt_r == 16'd0) begin
                rcv_clk_r <= 1'b0;
            end else if (div_cnt_r == P_HALF_LOW_CYC) begin
                rcv_clk_r <= 1'b1;
            end

            // Sample synchronized SER_DATA at configured RCV_CLK edge
            if (!sample_edge_i) begin
                // Default SDLC: sensor shifts on falling edge, sample on rising edge
                if (div_cnt_r == P_HALF_LOW_CYC) begin
                    rx_bit_valid_r <= 1'b1;
                    rx_bit_r       <= ser_data_sync_r;
                end
            end else begin
                // Alternate mode: sample on falling edge (only after clock has gone high)
                if ((div_cnt_r == 16'd0) && rcv_clk_r) begin
                    rx_bit_valid_r <= 1'b1;
                    rx_bit_r       <= ser_data_sync_r;
                end
            end
        end
    end

    //--------------------------------------------------------------------------
    // Free-Running 1 us Tick and 32-bit Timestamp Counter
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            us_div_cnt_r   <= 16'd0;
            tick_1us_r     <= 1'b0;
            timestamp_us_r <= 32'd0;
        end else begin
            if (us_div_cnt_r == P_US_PERIOD_LAST) begin
                us_div_cnt_r   <= 16'd0;
                tick_1us_r     <= 1'b1;
                timestamp_us_r <= timestamp_us_r + 32'd1;
            end else begin
                us_div_cnt_r <= us_div_cnt_r + 16'd1;
                tick_1us_r   <= 1'b0;
            end
        end
    end

    //--------------------------------------------------------------------------
    // Hardware-Timed RESET_L (>1 ms per Sec. 4.4) and BIT_L (Sec. 4.3.2) Drivers
    // Note: Outputs release to logic '1' (open-collector Hi-Z) during FPGA reset
    //       to permit SM-5800 self-reset during power application.
    //--------------------------------------------------------------------------
    always @(posedge clk_i) begin
        if (!rst_ni) begin
            rst_pulse_cnt_r <= 16'd0;
            imu_reset_l_r   <= 1'b1;
            imu_bit_l_r     <= 1'b1;
        end else begin
            // Timed RESET_L countdown (retriggerable)
            if (reset_pulse_req_i) begin
                rst_pulse_cnt_r <= P_RST_PULSE_LOAD;
            end else if ((rst_pulse_cnt_r != 16'd0) && tick_1us_r) begin
                rst_pulse_cnt_r <= rst_pulse_cnt_r - 16'd1;
            end

            // Drive active-low RESET_L if timed pulse active or manual hold asserted
            imu_reset_l_r <= ~(reset_hold_i || (rst_pulse_cnt_r != 16'd0) || reset_pulse_req_i);

            // Drive active-low BIT_L when Commanded BIT is enabled
            imu_bit_l_r   <= ~bit_cmd_i;
        end
    end

    //--------------------------------------------------------------------------
    // Output Assignments
    //--------------------------------------------------------------------------
    assign rx_bit_valid_o = rx_bit_valid_r;
    assign rx_bit_o       = rx_bit_r;
    assign tick_1us_o     = tick_1us_r;
    assign timestamp_us_o = timestamp_us_r;
    assign reset_busy_o   = (rst_pulse_cnt_r != 16'd0) || reset_pulse_req_i;
    assign imu_rcv_clk_o  = rcv_clk_r;
    assign imu_bit_l_o    = imu_bit_l_r;
    assign imu_reset_l_o  = imu_reset_l_r;

endmodule

`default_nettype wire
