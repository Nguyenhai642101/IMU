//==============================================================================
// Module      : tb_sm5800_imu_axi
// Project     : SM-5800 6-DOF Motion Sensor Module (IMU) AXI4-Lite Interface
// Standard    : Verilog-2001 (IEEE 1364-2001) — Self-Checking Testbench
// Description : Comprehensive verification suite covering:
//               [T1]  Reset defaults, CORE_ID, AXI4-Lite write orderings
//                     (Simultaneous, AW-first, W-first) & WSTRB byte masking
//               [T2]  Power-Up BIT idle 0x7E flags (FLAGS_ALIVE=1, DATA_ACTIVE=0)
//               [T3]  Msg ID 1 (Motion Control, 20B) + signed values + bit-stuffing
//               [T4]  Msg ID 2 (Motion Control + Spatial, 44B) via Shared Flag
//               [T5]  6-frame SM-5800 cadence (Msg 2 + 5x Msg 1) & 4-bit counter
//               [T6]  Atomic Snapshot Lock (CTRL.DATA_LOCK) coherency test
//               [T7]  Error rejection (CRC error, wrong Addr, SDLC Abort, Seq Gap)
//               [T8]  SM-5800 BIT Fault detection (Status Word 1 & Status Word 2)
//               [T9]  Commanded BIT control (BIT_L) & flag-only operation
//               [T10] Hardware-timed RESET_L pulse (>1.0 ms spec verification)
//               [T11] Data watchdog timeout & LINK_TIMEOUT interrupt
//               [T12] Msg ID 01-only mode (MSG1_ONLY_MODE) & distinct Msg1 IRQ
//               [T13] 100 Hz Msg ID 02-only mode (MSG2_ONLY_MODE) & Msg2 IRQ
//               [T14] Runt frames / stuck-low line between frames (ISO 3309)
//               [T15] Watchdog start-up silence, one-shot, re-arm, safe default
//               [T16] PL-PS IRQ timing, RESET_BUSY, fault gating, LAST_SEQ,
//                     watchdog re-arm after BIT, 1 us tick rounding
//               [T17] irq_o re-arm: new rising edge after a partial W1C
//               [T18] Watchdog grace after start-up / RESET_L / Commanded BIT
//==============================================================================

`timescale 1ns / 1ps
`default_nettype none

module tb_sm5800_imu_axi;

    //--------------------------------------------------------------------------
    // Testbench Parameters (100 MHz AXI Clock, 1.0 MHz SM-5800 RCV_CLK)
    //--------------------------------------------------------------------------
    localparam integer C_CLK_PERIOD_NS      = 10;          // 100 MHz
    localparam integer C_CLK_FREQ_HZ        = 100_000_000;
    localparam integer C_SDLC_CLK_HZ        = 1_000_000;
    localparam integer C_RESET_PULSE_US     = 1100;        // >1000 us (Sec. 4.4)
    localparam integer C_S_AXI_DATA_WIDTH   = 32;
    localparam integer C_S_AXI_ADDR_WIDTH   = 8;

    // Register Offsets
    localparam [7:0] ADDR_CORE_ID       = 8'h00;
    localparam [7:0] ADDR_CTRL          = 8'h04;
    localparam [7:0] ADDR_STATUS        = 8'h08;
    localparam [7:0] ADDR_INT_STATUS    = 8'h0C;
    localparam [7:0] ADDR_INT_ENABLE    = 8'h10;
    localparam [7:0] ADDR_TIMEOUT_CFG   = 8'h14;
    localparam [7:0] ADDR_MC_TIMESTAMP  = 8'h18;
    localparam [7:0] ADDR_SP_TIMESTAMP  = 8'h1C;
    localparam [7:0] ADDR_ANG_RATE_X    = 8'h20;
    localparam [7:0] ADDR_ANG_RATE_Y    = 8'h24;
    localparam [7:0] ADDR_ANG_RATE_Z    = 8'h28;
    localparam [7:0] ADDR_LIN_ACC_X     = 8'h2C;
    localparam [7:0] ADDR_LIN_ACC_Y     = 8'h30;
    localparam [7:0] ADDR_LIN_ACC_Z     = 8'h34;
    localparam [7:0] ADDR_STATUS_WORD1  = 8'h38;
    localparam [7:0] ADDR_STATUS_WORD2  = 8'h3C;
    localparam [7:0] ADDR_DELTA_ANG_X   = 8'h40;
    localparam [7:0] ADDR_DELTA_ANG_Y   = 8'h44;
    localparam [7:0] ADDR_DELTA_ANG_Z   = 8'h48;
    localparam [7:0] ADDR_DELTA_VEL_X   = 8'h4C;
    localparam [7:0] ADDR_DELTA_VEL_Y   = 8'h50;
    localparam [7:0] ADDR_DELTA_VEL_Z   = 8'h54;
    localparam [7:0] ADDR_MSG1_RX_CNT   = 8'h58;
    localparam [7:0] ADDR_MSG2_RX_CNT   = 8'h5C;
    localparam [7:0] ADDR_CRC_ERR_CNT   = 8'h60;
    localparam [7:0] ADDR_FRAME_ERR_CNT = 8'h64;
    localparam [7:0] ADDR_SEQ_ERR_CNT   = 8'h68;
    localparam [7:0] ADDR_DEBUG_CRC     = 8'h6C;

    //--------------------------------------------------------------------------
    // Signals
    //--------------------------------------------------------------------------
    reg         s_axi_aclk;
    reg         s_axi_aresetn;

    reg  [7:0]  s_axi_awaddr;
    reg  [2:0]  s_axi_awprot;
    reg         s_axi_awvalid;
    wire        s_axi_awready;

    reg  [31:0] s_axi_wdata;
    reg  [3:0]  s_axi_wstrb;
    reg         s_axi_wvalid;
    wire        s_axi_wready;

    wire [1:0]  s_axi_bresp;
    wire        s_axi_bvalid;
    reg         s_axi_bready;

    reg  [7:0]  s_axi_araddr;
    reg  [2:0]  s_axi_arprot;
    reg         s_axi_arvalid;
    wire        s_axi_arready;

    wire [31:0] s_axi_rdata;
    wire [1:0]  s_axi_rresp;
    wire        s_axi_rvalid;
    reg         s_axi_rready;

    wire        irq_o;
    wire        msg1_valid_o;
    wire        msg2_valid_o;
    wire        imu_rcv_clk_o;
    wire        imu_ser_data_w;
    wire        imu_bit_l_o;
    wire        imu_reset_l_o;

    integer     err_cnt;
    integer     test_pass_cnt;
    integer     msg1_pin_cnt;
    integer     t14_pin_base;
    integer     t17_rise_base;
    integer     msg2_pin_cnt;
    reg  [31:0] rd_val;
    integer     i;
    reg  [15:0] seq_idx_r;
    time        t_rst_start;
    time        t_rst_dur_ns;

    //--------------------------------------------------------------------------
    // DUT Instantiation
    //--------------------------------------------------------------------------
    sm5800_imu_axi_top #(
        .C_CLK_FREQ_HZ        (C_CLK_FREQ_HZ),
        .C_SDLC_CLK_HZ        (C_SDLC_CLK_HZ),
        .C_RESET_PULSE_US     (C_RESET_PULSE_US),
        .C_ACTIVATION_GRACE_US(300),   // scaled from 1 s for simulation
        .C_BIT_GRACE_US       (200),   // scaled from 0.3 s for simulation
        .C_DEFAULT_CLK_EN     (1),
        .C_DEFAULT_MSG1_ONLY  (0),
        .C_DEFAULT_MSG2_ONLY  (0),
        .C_DEFAULT_TIMEOUT_US (5000),
        .C_S_AXI_DATA_WIDTH   (C_S_AXI_DATA_WIDTH),
        .C_S_AXI_ADDR_WIDTH   (C_S_AXI_ADDR_WIDTH)
    ) u_dut (
        .s_axi_aclk     (s_axi_aclk),
        .s_axi_aresetn  (s_axi_aresetn),
        .s_axi_awaddr   (s_axi_awaddr),
        .s_axi_awprot   (s_axi_awprot),
        .s_axi_awvalid  (s_axi_awvalid),
        .s_axi_awready  (s_axi_awready),
        .s_axi_wdata    (s_axi_wdata),
        .s_axi_wstrb    (s_axi_wstrb),
        .s_axi_wvalid   (s_axi_wvalid),
        .s_axi_wready   (s_axi_wready),
        .s_axi_bresp    (s_axi_bresp),
        .s_axi_bvalid   (s_axi_bvalid),
        .s_axi_bready   (s_axi_bready),
        .s_axi_araddr   (s_axi_araddr),
        .s_axi_arprot   (s_axi_arprot),
        .s_axi_arvalid  (s_axi_arvalid),
        .s_axi_arready  (s_axi_arready),
        .s_axi_rdata    (s_axi_rdata),
        .s_axi_rresp    (s_axi_rresp),
        .s_axi_rvalid   (s_axi_rvalid),
        .s_axi_rready   (s_axi_rready),
        .irq_o          (irq_o),
        .msg1_valid_o   (msg1_valid_o),
        .msg2_valid_o   (msg2_valid_o),
        .imu_rcv_clk_o  (imu_rcv_clk_o),
        .imu_ser_data_i (imu_ser_data_w),
        .imu_bit_l_o    (imu_bit_l_o),
        .imu_reset_l_o  (imu_reset_l_o)
    );

    //--------------------------------------------------------------------------
    // Second DUT: IP packaged for 100 Hz use (MSG2_ONLY default) with every
    // other parameter left at its default. Only its reset values are checked
    // ([T15d]); its AXI port is idle and its SER_DATA input is held at mark.
    //--------------------------------------------------------------------------
    sm5800_imu_axi_top #(
        .C_DEFAULT_MSG2_ONLY  (1)
    ) u_dut_dflt (
        .s_axi_aclk     (s_axi_aclk),
        .s_axi_aresetn  (s_axi_aresetn),
        .s_axi_awaddr   (8'h00),
        .s_axi_awprot   (3'b000),
        .s_axi_awvalid  (1'b0),
        .s_axi_awready  (),
        .s_axi_wdata    (32'd0),
        .s_axi_wstrb    (4'h0),
        .s_axi_wvalid   (1'b0),
        .s_axi_wready   (),
        .s_axi_bresp    (),
        .s_axi_bvalid   (),
        .s_axi_bready   (1'b1),
        .s_axi_araddr   (8'h00),
        .s_axi_arprot   (3'b000),
        .s_axi_arvalid  (1'b0),
        .s_axi_arready  (),
        .s_axi_rdata    (),
        .s_axi_rresp    (),
        .s_axi_rvalid   (),
        .s_axi_rready   (1'b1),
        .irq_o          (),
        .msg1_valid_o   (),
        .msg2_valid_o   (),
        .imu_rcv_clk_o  (),
        .imu_ser_data_i (1'b1),
        .imu_bit_l_o    (),
        .imu_reset_l_o  ()
    );

    //--------------------------------------------------------------------------
    // [T16] Monitors for timing details that AXI reads cannot observe
    //--------------------------------------------------------------------------
    // (a) irq_o value in the first cycle BVALID is high (1 ns after the edge)
    reg         bvalid_d1_r;
    reg         irq_at_bvalid_r;
    initial begin
        bvalid_d1_r     = 1'b0;
        irq_at_bvalid_r = 1'b1;
    end
    always @(posedge s_axi_aclk) begin
        #1;
        if (s_axi_bvalid && !bvalid_d1_r) begin
            irq_at_bvalid_r = irq_o;
        end
        bvalid_d1_r = s_axi_bvalid;
    end

    // (a2) Count rising edges of irq_o (what an edge-triggered GIC sees)
    integer     irq_rise_cnt;
    reg         irq_d1_r;
    initial begin
        irq_rise_cnt = 0;
        irq_d1_r     = 1'b0;
    end
    always @(posedge s_axi_aclk) begin
        if (irq_o && !irq_d1_r) begin
            irq_rise_cnt = irq_rise_cnt + 1;
        end
        irq_d1_r = irq_o;
    end

    // (b) RESET_BUSY must already be 1 in the cycle the pulse is requested
    integer     rst_req_seen_cnt;
    integer     rst_busy_gap_cnt;
    initial begin
        rst_req_seen_cnt = 0;
        rst_busy_gap_cnt = 0;
    end
    always @(posedge s_axi_aclk) begin
        if (u_dut.u_clk_ctrl.reset_pulse_req_i) begin
            rst_req_seen_cnt = rst_req_seen_cnt + 1;
            if (!u_dut.u_clk_ctrl.reset_busy_o) begin
                rst_busy_gap_cnt = rst_busy_gap_cnt + 1;
            end
        end
    end

    // (f) Clock controller built for a fractional-MHz AXI clock (33.333 MHz):
    //     its 1 us tick must be >= 1.000 us, i.e. 34 cycles (33 would be 0.99 us)
    wire        frac_tick_w;
    integer     frac_cyc_cnt;
    integer     frac_tick_period;
    sm5800_clk_ctrl #(
        .C_CLK_FREQ_HZ    (33_333_333),
        .C_SDLC_CLK_HZ    (1_000_000),
        .C_RESET_PULSE_US (2000)
    ) u_clk_frac (
        .clk_i             (s_axi_aclk),
        .rst_ni            (s_axi_aresetn),
        .clk_en_i          (1'b1),
        .sample_edge_i     (1'b0),
        .bit_cmd_i         (1'b0),
        .reset_pulse_req_i (1'b0),
        .reset_hold_i      (1'b0),
        .rx_bit_valid_o    (),
        .rx_bit_o          (),
        .tick_1us_o        (frac_tick_w),
        .timestamp_us_o    (),
        .reset_busy_o      (),
        .imu_rcv_clk_o     (),
        .imu_ser_data_i    (1'b1),
        .imu_bit_l_o       (),
        .imu_reset_l_o     ()
    );
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            frac_cyc_cnt     = 0;
            frac_tick_period = 0;
        end else begin
            frac_cyc_cnt = frac_cyc_cnt + 1;
            if (frac_tick_w) begin
                frac_tick_period = frac_cyc_cnt;
                frac_cyc_cnt     = 0;
            end
        end
    end

    // Track dedicated Msg 01 / Msg 02 strobe output pins
    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            msg1_pin_cnt <= 0;
            msg2_pin_cnt <= 0;
        end else begin
            if (msg1_valid_o) msg1_pin_cnt <= msg1_pin_cnt + 1;
            if (msg2_valid_o) msg2_pin_cnt <= msg2_pin_cnt + 1;
        end
    end

    //--------------------------------------------------------------------------
    // SM-5800 Behavioral Sensor Model Instantiation
    //--------------------------------------------------------------------------
    sm5800_sensor_model u_sensor (
        .rcv_clk_i  (imu_rcv_clk_o),
        .bit_l_i    (imu_bit_l_o),
        .reset_l_i  (imu_reset_l_o),
        .ser_data_o (imu_ser_data_w)
    );

    //--------------------------------------------------------------------------
    // 100 MHz System Clock Generator
    //--------------------------------------------------------------------------
    initial begin
        s_axi_aclk = 1'b0;
        forever #(C_CLK_PERIOD_NS / 2) s_axi_aclk = ~s_axi_aclk;
    end

    //--------------------------------------------------------------------------
    // Verification Helper Tasks
    //--------------------------------------------------------------------------
    task check_eq;
        input [31:0]  actual;
        input [31:0]  expected;
        input [511:0] label;
        begin
            if (actual !== expected) begin
                $display("[FAIL] %0s: got 0x%08X, expected 0x%08X (t=%0t)", label, actual, expected, $time);
                err_cnt = err_cnt + 1;
            end else begin
                test_pass_cnt = test_pass_cnt + 1;
            end
        end
    endtask

    // AXI4-Lite Write (mode: 0=simultaneous, 1=AW first, 2=W first)
    task axi_write;
        input [7:0]  addr;
        input [31:0] data;
        input [3:0]  strb;
        input [1:0]  order_mode;
        begin
            @(posedge s_axi_aclk);
            if (order_mode == 2'd0) begin
                // Simultaneous AW + W
                s_axi_awaddr  <= addr;
                s_axi_awvalid <= 1'b1;
                s_axi_wdata   <= data;
                s_axi_wstrb   <= strb;
                s_axi_wvalid  <= 1'b1;
                s_axi_bready  <= 1'b1;

                // Wait for both handshakes
                fork
                    begin : wait_aw
                        @(posedge s_axi_aclk);
                        while (!s_axi_awready) @(posedge s_axi_aclk);
                        s_axi_awvalid <= 1'b0;
                    end
                    begin : wait_w
                        @(posedge s_axi_aclk);
                        while (!s_axi_wready) @(posedge s_axi_aclk);
                        s_axi_wvalid <= 1'b0;
                    end
                join
            end else if (order_mode == 2'd1) begin
                // AW arrives 2 cycles before W
                s_axi_awaddr  <= addr;
                s_axi_awvalid <= 1'b1;
                s_axi_bready  <= 1'b1;
                @(posedge s_axi_aclk);
                while (!s_axi_awready) @(posedge s_axi_aclk);
                s_axi_awvalid <= 1'b0;
                repeat (2) @(posedge s_axi_aclk);
                s_axi_wdata   <= data;
                s_axi_wstrb   <= strb;
                s_axi_wvalid  <= 1'b1;
                @(posedge s_axi_aclk);
                while (!s_axi_wready) @(posedge s_axi_aclk);
                s_axi_wvalid  <= 1'b0;
            end else begin
                // W arrives 2 cycles before AW
                s_axi_wdata   <= data;
                s_axi_wstrb   <= strb;
                s_axi_wvalid  <= 1'b1;
                s_axi_bready  <= 1'b1;
                @(posedge s_axi_aclk);
                while (!s_axi_wready) @(posedge s_axi_aclk);
                s_axi_wvalid  <= 1'b0;
                repeat (2) @(posedge s_axi_aclk);
                s_axi_awaddr  <= addr;
                s_axi_awvalid <= 1'b1;
                @(posedge s_axi_aclk);
                while (!s_axi_awready) @(posedge s_axi_aclk);
                s_axi_awvalid <= 1'b0;
            end

            // Wait for Write Response (BVALID)
            while (!s_axi_bvalid) @(posedge s_axi_aclk);
            check_eq({30'd0, s_axi_bresp}, 32'd0, "AXI BRESP OKAY");
            @(posedge s_axi_aclk);
            s_axi_bready <= 1'b0;
        end
    endtask

    // AXI4-Lite Read (with optional RREADY backpressure cycles)
    task axi_read;
        input  [7:0]  addr;
        output [31:0] data;
        begin
            @(posedge s_axi_aclk);
            s_axi_araddr  <= addr;
            s_axi_arvalid <= 1'b1;
            s_axi_rready  <= 1'b0;

            @(posedge s_axi_aclk);
            while (!s_axi_arready) @(posedge s_axi_aclk);
            s_axi_arvalid <= 1'b0;

            // Wait for RVALID, then hold 1 cycle to test RREADY backpressure
            while (!s_axi_rvalid) @(posedge s_axi_aclk);
            s_axi_rready  <= 1'b1;
            @(posedge s_axi_aclk);
            data = s_axi_rdata;
            check_eq({30'd0, s_axi_rresp}, 32'd0, "AXI RRESP OKAY");
            s_axi_rready  <= 1'b0;
        end
    endtask

    // Wait a few AXI clock cycles after sensor finishes shifting closing flag
    task wait_rx_settle;
        begin
            repeat (150) @(posedge s_axi_aclk); // 1.5 bit periods for 2-FF sync + pipeline
        end
    endtask

    //--------------------------------------------------------------------------
    // Main Test Sequence
    //--------------------------------------------------------------------------
    initial begin
        err_cnt       = 0;
        test_pass_cnt = 0;

        s_axi_aresetn = 1'b0;
        s_axi_awaddr  = 8'h00;
        s_axi_awprot  = 3'b000;
        s_axi_awvalid = 1'b0;
        s_axi_wdata   = 32'd0;
        s_axi_wstrb   = 4'h0;
        s_axi_wvalid  = 1'b0;
        s_axi_bready  = 1'b0;
        s_axi_araddr  = 8'h00;
        s_axi_arprot  = 3'b000;
        s_axi_arvalid = 1'b0;
        s_axi_rready  = 1'b0;

        $display("==============================================================");
        $display(" SM-5800 6-DOF IMU AXI4-Lite IP Verification Suite Starting");
        $display("==============================================================");

        // Hold reset for 10 cycles; verify open-collector outputs released HIGH
        repeat (10) @(posedge s_axi_aclk);
        check_eq({31'd0, imu_reset_l_o}, 32'd1, "RESET_L released during FPGA rst");
        check_eq({31'd0, imu_bit_l_o},   32'd1, "BIT_L released during FPGA rst");
        s_axi_aresetn = 1'b1;
        repeat (5) @(posedge s_axi_aclk);

        //----------------------------------------------------------------------
        // [T1] CORE_ID, Reset Defaults, AXI4-Lite Orderings & WSTRB
        //----------------------------------------------------------------------
        $display("[T1] Testing CORE_ID, AXI4-Lite AW/W orderings, and WSTRB...");
        axi_read(ADDR_CORE_ID, rd_val);
        check_eq(rd_val, 32'h5800_0100, "CORE_ID == 0x58000100");

        axi_read(ADDR_CTRL, rd_val);
        check_eq(rd_val, 32'h0000_0001, "CTRL reset default (CLK_EN=1)");

        // Test AW-first write on TIMEOUT_CFG
        axi_write(ADDR_TIMEOUT_CFG, 32'h1122_3344, 4'hF, 2'd1);
        axi_read(ADDR_TIMEOUT_CFG, rd_val);
        check_eq(rd_val, 32'h1122_3344, "AXI AW-first write TIMEOUT_CFG");

        // Test W-first write with partial WSTRB (update byte 1 and byte 3 only)
        axi_write(ADDR_TIMEOUT_CFG, 32'hAA00_BB00, 4'b1010, 2'd2);
        axi_read(ADDR_TIMEOUT_CFG, rd_val);
        check_eq(rd_val, 32'hAA22_BB44, "AXI W-first partial WSTRB");

        // Restore TIMEOUT_CFG to 5000 us and enable all 7 interrupts
        axi_write(ADDR_TIMEOUT_CFG, 32'd5000, 4'hF, 2'd0);
        axi_write(ADDR_INT_ENABLE,  32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T2] Power-Up BIT Idle 0x7E Flags Only
        //----------------------------------------------------------------------
        $display("[T2] Testing continuous 0x7E flags (Power-Up BIT phase)...");
        u_sensor.send_idle_flags(6);
        wait_rx_settle();

        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0003, 32'h0000_0001, "FLAGS_ALIVE=1, DATA_ACTIVE=0");
        axi_read(ADDR_CRC_ERR_CNT, rd_val);
        check_eq(rd_val, 32'd0, "No CRC errors on idle flags");
        axi_read(ADDR_FRAME_ERR_CNT, rd_val);
        check_eq(rd_val, 32'd0, "No Frame/Abort errors on idle flags");

        //----------------------------------------------------------------------
        // [T3] Message ID 1 (Motion Control, 20B) + Bit-Stuffing Stress + Signed
        //----------------------------------------------------------------------
        $display("[T3] Testing Msg ID 1 with negative values & 0xFF/0x7E bit-stuffing...");
        // RateX = 0x1234 (+4660), RateY = 0xFFFE (-2, tests sign-ext & bit-stuffing!),
        // RateZ = 0x7E7F (flag-like bytes inside payload!), AccX = 0x8001 (-32767),
        // AccY = 0x00FF, AccZ = 0xFFFF (-1), SW1 = 0x1900 (Temp=25C, Cnt=0), SW2 = 0x0003 (SW Ver 3)
        u_sensor.send_msg1(
            8'h0A,
            16'h1234, 16'hFFFE, 16'h7E7F,
            16'h8001, 16'h00FF, 16'hFFFF,
            16'h1900, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();

        axi_read(ADDR_ANG_RATE_X,   rd_val); check_eq(rd_val, 32'h0000_1234, "Msg1 ANG_RATE_X");
        axi_read(ADDR_ANG_RATE_Y,   rd_val); check_eq(rd_val, 32'hFFFF_FFFE, "Msg1 ANG_RATE_Y sign-ext");
        axi_read(ADDR_ANG_RATE_Z,   rd_val); check_eq(rd_val, 32'h0000_7E7F, "Msg1 ANG_RATE_Z (7E7F)");
        axi_read(ADDR_LIN_ACC_X,    rd_val); check_eq(rd_val, 32'hFFFF_8001, "Msg1 LIN_ACC_X sign-ext");
        axi_read(ADDR_LIN_ACC_Y,    rd_val); check_eq(rd_val, 32'h0000_00FF, "Msg1 LIN_ACC_Y");
        axi_read(ADDR_LIN_ACC_Z,    rd_val); check_eq(rd_val, 32'hFFFF_FFFF, "Msg1 LIN_ACC_Z (-1)");
        axi_read(ADDR_STATUS_WORD1, rd_val); check_eq(rd_val, 32'h0000_1900, "Msg1 STATUS_WORD1");
        axi_read(ADDR_STATUS_WORD2, rd_val); check_eq(rd_val, 32'h0000_0003, "Msg1 STATUS_WORD2");
        axi_read(ADDR_MSG1_RX_CNT,  rd_val); check_eq(rd_val, 32'd1,         "MSG1_RX_CNT == 1");
        axi_read(ADDR_DEBUG_CRC,    rd_val); check_eq(rd_val, 32'h0A14_F0B8, "DEBUG_CRC (0A, 20B, F0B8)");
        check_eq({31'd0, irq_o}, 32'd1, "IRQ asserted on MSG1_READY");
        axi_read(ADDR_INT_STATUS,   rd_val); check_eq(rd_val, 32'h0000_0041, "INT_STATUS == MSG1|MC_ANY");
        check_eq(msg1_pin_cnt, 32'd1, "msg1_valid_o pulsed once");
        check_eq(msg2_pin_cnt, 32'd0, "msg2_valid_o stayed 0 on Msg1");

        // W1C clear INT_STATUS[6:0] -> verify IRQ deasserts
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        repeat (3) @(posedge s_axi_aclk);
        check_eq({31'd0, irq_o}, 32'd0, "IRQ deasserted after W1C");

        //----------------------------------------------------------------------
        // [T4] Message ID 2 (Motion Control + Spatial, 44B) via Shared Flag
        //----------------------------------------------------------------------
        $display("[T4] Testing Msg ID 2 (44B) back-to-back via Shared Flag...");
        // Send Msg 1 (cnt=1) immediately followed by Msg 2 (cnt=2) sharing the flag!
        u_sensor.send_msg1(
            8'h0A,
            16'h0101, 16'h0202, 16'h0303,
            16'h0404, 16'h0505, 16'h0606,
            16'h1A01, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        // skip_open_flag = 1'b1 -> shares Msg 1's closing 0x7E flag!
        u_sensor.send_msg2(
            8'h0A,
            16'h1111, 16'h2222, 16'h3333,
            16'h4444, 16'h5555, 16'h6666,
            16'h1B02, 16'h0003,
            32'h1234_5678, 32'h9ABC_DEF0, 32'hFEDC_BA98,
            32'h0102_0304, 32'hFFFF_FF00, 32'h7FFF_FFFF,
            1'b1, 1'b0, 1'b1
        );
        wait_rx_settle();

        axi_read(ADDR_ANG_RATE_X,  rd_val); check_eq(rd_val, 32'h0000_1111, "Msg2 ANG_RATE_X");
        axi_read(ADDR_DELTA_ANG_X, rd_val); check_eq(rd_val, 32'h1234_5678, "Msg2 DELTA_ANG_X");
        axi_read(ADDR_DELTA_ANG_Y, rd_val); check_eq(rd_val, 32'h9ABC_DEF0, "Msg2 DELTA_ANG_Y");
        axi_read(ADDR_DELTA_ANG_Z, rd_val); check_eq(rd_val, 32'hFEDC_BA98, "Msg2 DELTA_ANG_Z");
        axi_read(ADDR_DELTA_VEL_X, rd_val); check_eq(rd_val, 32'h0102_0304, "Msg2 DELTA_VEL_X");
        axi_read(ADDR_DELTA_VEL_Y, rd_val); check_eq(rd_val, 32'hFFFF_FF00, "Msg2 DELTA_VEL_Y");
        axi_read(ADDR_DELTA_VEL_Z, rd_val); check_eq(rd_val, 32'h7FFF_FFFF, "Msg2 DELTA_VEL_Z");
        axi_read(ADDR_MSG1_RX_CNT, rd_val); check_eq(rd_val, 32'd2,         "MSG1_RX_CNT == 2");
        axi_read(ADDR_MSG2_RX_CNT, rd_val); check_eq(rd_val, 32'd1,         "MSG2_RX_CNT == 1");
        axi_read(ADDR_SEQ_ERR_CNT, rd_val); check_eq(rd_val, 32'd0,         "SEQ_ERR_CNT == 0");
        axi_read(ADDR_DEBUG_CRC,   rd_val); check_eq(rd_val, 32'h0A2C_F0B8, "DEBUG_CRC (0A, 44B, F0B8)");
        axi_read(ADDR_INT_STATUS,  rd_val); check_eq(rd_val, 32'h0000_0043, "INT_STATUS == MSG1|MSG2|MC");
        check_eq(msg1_pin_cnt, 32'd2, "msg1_valid_o pulsed twice");
        check_eq(msg2_pin_cnt, 32'd1, "msg2_valid_o pulsed once");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T5] Full 6-Frame Cadence (5x Msg 1 + 1x Msg 2) & Rolling Counter
        //----------------------------------------------------------------------
        $display("[T5] Testing 6-frame cadence (5x Msg1 + 1x Msg2, cnt 3..8)...");
        for (i = 3; i <= 7; i = i + 1) begin
            seq_idx_r = i;
            u_sensor.send_msg1(
                8'h0A,
                16'h0010 + seq_idx_r, 16'h0020, 16'h0030,
                16'h0040, 16'h0050, 16'h0060,
                {12'h1C0, seq_idx_r[3:0]}, 16'h0003,
                1'b1, 1'b0, 1'b0
            );
        end
        u_sensor.send_msg2(
            8'h0A,
            16'h0018, 16'h0028, 16'h0038,
            16'h0048, 16'h0058, 16'h0068,
            16'h1C08, 16'h0003,
            32'h0001_0001, 32'h0002_0002, 32'h0003_0003,
            32'h0004_0004, 32'h0005_0005, 32'h0006_0006,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();

        axi_read(ADDR_MSG1_RX_CNT, rd_val); check_eq(rd_val, 32'd7, "MSG1_RX_CNT == 7");
        axi_read(ADDR_MSG2_RX_CNT, rd_val); check_eq(rd_val, 32'd2, "MSG2_RX_CNT == 2");
        axi_read(ADDR_SEQ_ERR_CNT, rd_val); check_eq(rd_val, 32'd0, "SEQ_ERR_CNT == 0");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T6] Atomic Snapshot Lock (CTRL.DATA_LOCK) Coherency Test
        //----------------------------------------------------------------------
        $display("[T6] Testing Atomic Snapshot Lock (DATA_LOCK)...");
        // Lock snapshot (CTRL.CLK_EN=1, DATA_LOCK=1 -> 0x03)
        axi_write(ADDR_CTRL, 32'h0000_0003, 4'hF, 2'd0);

        // Send a brand-new Msg 2 (cnt=9) while registers are locked!
        u_sensor.send_msg2(
            8'h0A,
            16'hAAAA, 16'hBBBB, 16'hCCCC,
            16'hDDDD, 16'hEEEE, 16'h0777,
            16'h1D09, 16'h0003,
            32'hCAFE_BABE, 32'hDEAD_BEEF, 32'h55AA_55AA,
            32'h1122_3344, 32'h5566_7788, 32'h99AA_BBCC,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();

        // While locked, reads MUST still show the previous coherent snapshot (0x0018 / 0x0001_0001)!
        axi_read(ADDR_ANG_RATE_X,  rd_val); check_eq(rd_val, 32'h0000_0018, "Locked ANG_RATE_X frozen");
        axi_read(ADDR_DELTA_ANG_X, rd_val); check_eq(rd_val, 32'h0001_0001, "Locked DELTA_ANG_X frozen");

        // Unlock (DATA_LOCK=0 -> 0x01) -> reads immediately reflect newly arrived live frame!
        axi_write(ADDR_CTRL, 32'h0000_0001, 4'hF, 2'd0);
        axi_read(ADDR_ANG_RATE_X,  rd_val); check_eq(rd_val, 32'hFFFF_AAAA, "Unlocked ANG_RATE_X live");
        axi_read(ADDR_DELTA_ANG_X, rd_val); check_eq(rd_val, 32'hCAFE_BABE, "Unlocked DELTA_ANG_X live");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T7] Error Rejection (CRC, Wrong Addr, Abort, Sequence Gap)
        //----------------------------------------------------------------------
        $display("[T7] Testing CRC error, bad Addr, SDLC Abort, and Sequence Gap...");
        // 7a. Inject CRC error -> MUST NOT overwrite live ANG_RATE_X (0xAAAA)
        u_sensor.send_msg1(
            8'h0A,
            16'h0BAD, 16'h0BAD, 16'h0BAD,
            16'h0BAD, 16'h0BAD, 16'h0BAD,
            16'h1D0A, 16'h0003,
            1'b1, 1'b1, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_ANG_RATE_X,  rd_val); check_eq(rd_val, 32'hFFFF_AAAA, "Bad CRC did NOT corrupt data");
        axi_read(ADDR_CRC_ERR_CNT, rd_val); check_eq(rd_val, 32'd1,         "CRC_ERR_CNT == 1");
        axi_read(ADDR_INT_STATUS,  rd_val); check_eq(rd_val & 32'h08, 32'h08, "INT_STATUS.RX_ERROR set");

        // 7b. Inject Wrong Module Address (0x0B != 0x0A)
        u_sensor.send_msg1(
            8'h0B,
            16'h0BAD, 16'h0BAD, 16'h0BAD,
            16'h0BAD, 16'h0BAD, 16'h0BAD,
            16'h1D0A, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_FRAME_ERR_CNT, rd_val); check_eq(rd_val & 32'hFFFF, 32'd1, "FRAME_ERR_CNT == 1");

        // 7c. Inject Mid-frame SDLC Abort (>=7 ones)
        u_sensor.send_abort_frame();
        wait_rx_settle();
        axi_read(ADDR_FRAME_ERR_CNT, rd_val); check_eq(rd_val >> 16, 32'd1, "ABORT_ERR_CNT == 1");

        // 7d. Recover with valid frame whose 4-bit counter jumps (9 -> 12 = 0xC) -> SEQ_GAP!
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_msg1(
            8'h0A,
            16'h0123, 16'h0456, 16'h0789,
            16'h0111, 16'h0222, 16'h0333,
            16'h1E0C, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_ANG_RATE_X,  rd_val); check_eq(rd_val, 32'h0000_0123, "Recovered after Abort");
        axi_read(ADDR_SEQ_ERR_CNT, rd_val); check_eq(rd_val, 32'd1,         "SEQ_ERR_CNT == 1");
        axi_read(ADDR_INT_STATUS,  rd_val); check_eq(rd_val & 32'h10, 32'h10, "INT_STATUS.SEQ_GAP set");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T8] SM-5800 BIT Fault Reporting (Status Word 1 & Status Word 2)
        //----------------------------------------------------------------------
        $display("[T8] Testing SM-5800 BIT fault reporting...");
        // Set SW1[4] Module Failed & SW2[11] Gyro Tests Failed (cnt=0xD)
        u_sensor.send_msg1(
            8'h0A,
            16'h0001, 16'h0002, 16'h0003,
            16'h0004, 16'h0005, 16'h0006,
            16'h1E1D, 16'h0803,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_000C, 32'h0000_000C, "STATUS: IMU_FAULT & MODULE_FAILED");
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0004, 32'h0000_0004, "INT_STATUS.IMU_FAULT set");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T9] Commanded BIT Control (BIT_L)
        //----------------------------------------------------------------------
        $display("[T9] Testing Commanded BIT (BIT_L active-low)...");
        axi_write(ADDR_CTRL, 32'h0000_0005, 4'hF, 2'd0); // CLK_EN=1, CMD_BIT_EN=1
        repeat (5) @(posedge s_axi_aclk);
        check_eq({31'd0, imu_bit_l_o}, 32'd0, "BIT_L asserted LOW");
        // During Commanded BIT, sensor transmits 0x7E flags only (Sec. 6.3.2.1)
        u_sensor.send_idle_flags(4);
        wait_rx_settle();
        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0041, 32'h0000_0041, "STATUS: CMD_BIT_ACTIVE & FLAGS_ALIVE");
        // Release Commanded BIT
        axi_write(ADDR_CTRL, 32'h0000_0001, 4'hF, 2'd0);
        repeat (5) @(posedge s_axi_aclk);
        check_eq({31'd0, imu_bit_l_o}, 32'd1, "BIT_L released HIGH");

        //----------------------------------------------------------------------
        // [T10] Hardware-Timed RESET_L Pulse (> 1.0 ms per Sec. 4.4)
        //----------------------------------------------------------------------
        $display("[T10] Testing hardware-timed RESET_L pulse (>1.0 ms spec)...");
        fork
            begin
                axi_write(ADDR_CTRL, 32'h0000_0009, 4'hF, 2'd0); // CLK_EN=1, IMU_RESET_PULSE=1
            end
            begin
                @(negedge imu_reset_l_o);
                t_rst_start = $time;
            end
        join
        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "STATUS.IMU_RESET_BUSY == 1");
        @(posedge imu_reset_l_o);
        t_rst_dur_ns = $time - t_rst_start;
        $display("      Measured RESET_L low pulse duration = %0t ns (%0d us)", t_rst_dur_ns, t_rst_dur_ns / 1000);
        if (t_rst_dur_ns <= 1_000_000) begin
            $display("[FAIL] RESET_L pulse duration <= 1.0 ms spec!");
            err_cnt = err_cnt + 1;
        end else begin
            test_pass_cnt = test_pass_cnt + 1;
        end

        //----------------------------------------------------------------------
        // [T11] Data Stream Watchdog Timeout (LINK_TIMEOUT)
        //----------------------------------------------------------------------
        $display("[T11] Testing Data Stream Watchdog Timeout...");
        axi_write(ADDR_TIMEOUT_CFG, 32'd150, 4'hF, 2'd0); // 150 us watchdog for fast sim
        axi_write(ADDR_INT_STATUS,  32'h0000_003F, 4'hF, 2'd0);

        // Send 1 valid frame to activate data stream, then stay idle > 150 us
        u_sensor.send_msg1(
            8'h0A,
            16'h0001, 16'h0002, 16'h0003,
            16'h0004, 16'h0005, 16'h0006,
            16'h1900, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        u_sensor.send_idle_flags(25); // 25 flags * 8 us = 200 us > 150 us timeout
        wait_rx_settle();

        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0003, 32'h0000_0001, "Watchdog: FLAGS_ALIVE=1, DATA_ACTIVE=0");
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "INT_STATUS.LINK_TIMEOUT set");

        //----------------------------------------------------------------------
        // [T12] Msg ID 01-Only Mode & Distinct Msg 01 / Msg 02 Pin & IRQ Check
        //----------------------------------------------------------------------
        $display("[T12] Testing MSG1_ONLY_MODE & PS IRQ strictly on Msg ID 01...");
        axi_write(ADDR_TIMEOUT_CFG, 32'd5000,      4'hF, 2'd0);
        axi_write(ADDR_CTRL,        32'h0000_0401, 4'hF, 2'd0); // CLK_EN=1, MSG1_ONLY_MODE=1
        axi_write(ADDR_INT_ENABLE,  32'h0000_0001, 4'hF, 2'd0); // Enable MSG1_READY ONLY
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0); // Clear all pending IRQs
        repeat (3) @(posedge s_axi_aclk);
        check_eq({31'd0, irq_o}, 32'd0, "IRQ low before Msg2 in MSG1_ONLY");

        // Send Msg ID 02 -> msg2_valid_o pulses, but irq_o stays 0 & RateX stays 0x0001!
        u_sensor.send_msg2(
            8'h0A,
            16'h5555, 16'h5555, 16'h5555,
            16'h5555, 16'h5555, 16'h5555,
            16'h1901, 16'h0003,
            32'h1111_1111, 32'h2222_2222, 32'h3333_3333,
            32'h4444_4444, 32'h5555_5555, 32'h6666_6666,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        check_eq({31'd0, irq_o}, 32'd0, "IRQ remains 0 on Msg2 in MSG1_ONLY");
        axi_read(ADDR_ANG_RATE_X, rd_val);
        check_eq(rd_val, 32'h0000_0001, "Msg2 did NOT overwrite Msg1 RateX");
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0003, 32'h0000_0002, "Only MSG2_READY flag set");

        // Now send Msg ID 01 -> msg1_valid_o pulses, RateX updates to 0x2468, irq_o asserts!
        u_sensor.send_msg1(
            8'h0A,
            16'h2468, 16'h1357, 16'hF001,
            16'h0100, 16'h0200, 16'hFF00,
            16'h1902, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        check_eq({31'd0, irq_o}, 32'd1, "IRQ asserted strictly on Msg1");
        axi_read(ADDR_ANG_RATE_X, rd_val);
        check_eq(rd_val, 32'h0000_2468, "Msg1 updated ANG_RATE_X");
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0001, 32'h0000_0001, "INT_STATUS.MSG1_READY set");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T13] 100 Hz Msg ID 02-Only Mode & PS IRQ strictly on Msg ID 02
        //----------------------------------------------------------------------
        $display("[T13] Testing 100 Hz MSG2_ONLY_MODE & PS IRQ strictly on Msg ID 02...");
        axi_write(ADDR_TIMEOUT_CFG, 32'd30000,     4'hF, 2'd0); // 30 ms watchdog (100 Hz)
        axi_write(ADDR_CTRL,        32'h0000_0801, 4'hF, 2'd0); // CLK_EN=1, MSG2_ONLY_MODE=1
        axi_write(ADDR_INT_ENABLE,  32'h0000_0002, 4'hF, 2'd0); // Enable MSG2_READY ONLY
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);
        repeat (3) @(posedge s_axi_aclk);
        check_eq({31'd0, irq_o}, 32'd0, "IRQ low before Msg1 in MSG2_ONLY");

        // Send Msg ID 01 -> msg1_valid_o pulses, but irq_o stays 0 & RateX stays 0x2468!
        u_sensor.send_msg1(
            8'h0A,
            16'h6666, 16'h6666, 16'h6666,
            16'h6666, 16'h6666, 16'h6666,
            16'h1903, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        check_eq({31'd0, irq_o}, 32'd0, "IRQ remains 0 on Msg1 in MSG2_ONLY");
        axi_read(ADDR_ANG_RATE_X, rd_val);
        check_eq(rd_val, 32'h0000_2468, "Msg1 did NOT overwrite Msg2 RateX");
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0003, 32'h0000_0001, "Only MSG1_READY flag set");

        // Send Msg ID 02 (100 Hz) -> updates all 14 fields & asserts irq_o to PS!
        u_sensor.send_msg2(
            8'h0A,
            16'h06D4, 16'hFF90, 16'h0010,
            16'h036F, 16'h0000, 16'hFC91,
            16'h1A04, 16'h0005,
            32'h0083_126F, 32'hFF7C_ED91, 32'h0000_0100,
            32'h0041_F372, 32'h0000_0000, 32'hFFBE_0C8E,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        check_eq({31'd0, irq_o}, 32'd1, "IRQ asserted strictly on 100Hz Msg2");
        axi_read(ADDR_ANG_RATE_X,  rd_val); check_eq(rd_val, 32'h0000_06D4, "Msg2 updated ANG_RATE_X");
        axi_read(ADDR_DELTA_ANG_X, rd_val); check_eq(rd_val, 32'h0083_126F, "Msg2 updated DELTA_ANG_X");
        axi_read(ADDR_DELTA_VEL_Z, rd_val); check_eq(rd_val, 32'hFFBE_0C8E, "Msg2 updated DELTA_VEL_Z");
        axi_read(ADDR_INT_STATUS,  rd_val); check_eq(rd_val & 32'h0000_0002, 32'h0000_0002, "INT_STATUS.MSG2_READY set");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        //----------------------------------------------------------------------
        // [T14] Runt-frame robustness (ISO 3309: < 32 bits between flags)
        //----------------------------------------------------------------------
        $display("[T14] Testing runt frames / stuck-low line between frames...");
        axi_write(ADDR_CTRL,        32'h0000_0101, 4'hF, 2'd0); // CLK_EN=1, CLEAR_STATS, normal mode
        axi_write(ADDR_INT_ENABLE,  32'h0000_007F, 4'hF, 2'd0);
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);
        t14_pin_base = msg1_pin_cnt;

        // 14a. Flag, line stuck low for 5 bit periods, then a valid Msg 1
        //      (this is what produced "ghost frames" before the fix)
        u_sensor.send_flag();
        u_sensor.send_raw_zeros(5);
        u_sensor.send_msg1(
            8'h0A,
            16'h0A0A, 16'h0B0B, 16'h0C0C,
            16'h0D0D, 16'h0E0E, 16'h0F0F,
            16'h1905, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_ANG_RATE_X,    rd_val); check_eq(rd_val, 32'h0000_0A0A, "T14a valid Msg1 after stuck-low bits");
        axi_read(ADDR_MSG1_RX_CNT,   rd_val); check_eq(rd_val, 32'd1, "T14a MSG1_RX_CNT == 1");
        axi_read(ADDR_CRC_ERR_CNT,   rd_val); check_eq(rd_val, 32'd0, "T14a runt not counted as CRC error");
        axi_read(ADDR_FRAME_ERR_CNT, rd_val); check_eq(rd_val, 32'd0, "T14a runt not counted as frame error");
        axi_read(ADDR_INT_STATUS,    rd_val); check_eq(rd_val & 32'h08, 32'h00, "T14a no RX_ERROR on runt");
        axi_read(ADDR_DEBUG_CRC,     rd_val); check_eq(rd_val, 32'h0A14_F0B8, "T14a DEBUG_CRC not overwritten by runt");

        // 14b. 3-byte runt (24 bits) between flags -> silently ignored
        u_sensor.send_runt_frame(3);
        wait_rx_settle();
        axi_read(ADDR_CRC_ERR_CNT,   rd_val); check_eq(rd_val, 32'd0, "T14b 24-bit runt ignored (CRC)");
        axi_read(ADDR_FRAME_ERR_CNT, rd_val); check_eq(rd_val, 32'd0, "T14b 24-bit runt ignored (FRAME)");
        axi_read(ADDR_INT_STATUS,    rd_val); check_eq(rd_val & 32'h08, 32'h00, "T14b no RX_ERROR on 24-bit runt");

        // 14c. Boundary: 4 bytes (32 bits) is a real (bad) frame -> must be reported
        u_sensor.send_runt_frame(4);
        wait_rx_settle();
        axi_read(ADDR_CRC_ERR_CNT,   rd_val); check_eq(rd_val, 32'd1, "T14c 32-bit frame counted (CRC)");
        axi_read(ADDR_FRAME_ERR_CNT, rd_val); check_eq(rd_val & 32'hFFFF, 32'd1, "T14c 32-bit frame counted (FRAME)");
        axi_read(ADDR_INT_STATUS,    rd_val); check_eq(rd_val & 32'h08, 32'h08, "T14c RX_ERROR on 32-bit bad frame");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        // 14d. Link still healthy afterwards: next Msg 1 (cnt 6) accepted, no seq gap
        u_sensor.send_msg1(
            8'h0A,
            16'h0606, 16'h0606, 16'h0606,
            16'h0606, 16'h0606, 16'h0606,
            16'h1906, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_ANG_RATE_X,  rd_val); check_eq(rd_val, 32'h0000_0606, "T14d Msg1 accepted after runts");
        axi_read(ADDR_SEQ_ERR_CNT, rd_val); check_eq(rd_val, 32'd0, "T14d no false SEQ_GAP from runts");
        check_eq(msg1_pin_cnt - t14_pin_base, 32'd2, "T14 msg1_valid_o pulsed exactly twice");

        //----------------------------------------------------------------------
        // [T15] Watchdog: start-up silence, one-shot IRQ, re-arm, safe defaults
        //----------------------------------------------------------------------
        $display("[T15] Testing watchdog on start-up silence & 100 Hz-safe defaults...");
        axi_write(ADDR_TIMEOUT_CFG, 32'd150,       4'hF, 2'd0); // 150 us for fast sim
        axi_write(ADDR_CTRL,        32'h0000_0201, 4'hF, 2'd0); // CLK_EN + SOFT_RESET
        axi_write(ADDR_INT_ENABLE,  32'h0000_007F, 4'hF, 2'd0);
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);

        // 15a. Sensor alive (flags) but NEVER sends data after reset -> must time out
        u_sensor.send_idle_flags(70); // 560 us > 150 us timeout + 300 us start-up grace
        wait_rx_settle();
        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0003, 32'h0000_0001, "T15a FLAGS_ALIVE=1, DATA_ACTIVE=0");
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "T15a LINK_TIMEOUT when no data ever arrives");

        // 15b. Still silent: timeout is one-shot (no interrupt storm)
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_idle_flags(25);
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0000, "T15b LINK_TIMEOUT not repeated while silent");

        // 15c. Data returns -> DATA_ACTIVE, then silence again -> re-armed timeout
        u_sensor.send_msg1(
            8'h0A,
            16'h1515, 16'h1515, 16'h1515,
            16'h1515, 16'h1515, 16'h1515,
            16'h1901, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0002, 32'h0000_0002, "T15c DATA_ACTIVE=1 after valid frame");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_idle_flags(25);
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "T15c LINK_TIMEOUT re-armed after data");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        // 15d. IP instantiated for 100 Hz (MSG2_ONLY) with default parameters:
        //      reset TIMEOUT_CFG must exceed the 10 ms Msg 02 period with margin
        check_eq({31'd0, (u_dut_dflt.u_axi4_lite_if.timeout_cfg_r >= 32'd12000)}, 32'd1,
                 "T15d default TIMEOUT_CFG >= 12 ms in 100 Hz mode");

        //----------------------------------------------------------------------
        // [T16] PL->PS IRQ timing, RESET_BUSY, fault gating, LAST_SEQ,
        //       watchdog re-arm after Commanded BIT, 1 us tick rounding
        //----------------------------------------------------------------------
        $display("[T16] Testing PL-PS IRQ timing, fault gating & watchdog re-arm...");
        axi_write(ADDR_TIMEOUT_CFG, 32'd30000,     4'hF, 2'd0);
        axi_write(ADDR_CTRL,        32'h0000_0201, 4'hF, 2'd0); // CLK_EN + SOFT_RESET
        axi_write(ADDR_INT_ENABLE,  32'h0000_007F, 4'hF, 2'd0);
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);

        // 16a. IRQ must already be low in the cycle BVALID rises
        //      (i) after a W1C clear of INT_STATUS, (ii) after INT_ENABLE = 0
        u_sensor.send_msg1(
            8'h0A,
            16'h1616, 16'h1616, 16'h1616,
            16'h1616, 16'h1616, 16'h1616,
            16'h1900, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        check_eq({31'd0, irq_o}, 32'd1, "T16a IRQ high before W1C");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        check_eq({31'd0, irq_at_bvalid_r}, 32'd0, "T16a IRQ low in same cycle as W1C BVALID");
        u_sensor.send_msg1(
            8'h0A,
            16'h1616, 16'h1616, 16'h1616,
            16'h1616, 16'h1616, 16'h1616,
            16'h1901, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        check_eq({31'd0, irq_o}, 32'd1, "T16a IRQ high before INT_ENABLE=0");
        axi_write(ADDR_INT_ENABLE, 32'h0000_0000, 4'hF, 2'd0);
        check_eq({31'd0, irq_at_bvalid_r}, 32'd0, "T16a IRQ low in same cycle as INT_ENABLE BVALID");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        axi_write(ADDR_INT_ENABLE, 32'h0000_007F, 4'hF, 2'd0);

        // 16b. RESET_BUSY covered the request cycle of every RESET_L pulse (T10)
        check_eq({31'd0, (rst_req_seen_cnt > 0)}, 32'd1, "T16b at least one RESET_L pulse observed");
        check_eq(rst_busy_gap_cnt, 32'd0, "T16b RESET_BUSY=1 in the pulse-request cycle");

        // 16c. MSG2_ONLY: a fault in an uncommitted Msg 01 must NOT raise IMU_FAULT
        axi_write(ADDR_CTRL,       32'h0000_0801, 4'hF, 2'd0); // CLK_EN + MSG2_ONLY
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_msg1(
            8'h0A,
            16'h0C0C, 16'h0C0C, 16'h0C0C,
            16'h0C0C, 16'h0C0C, 16'h0C0C,
            16'h1912, 16'h0003,            // SW1[4] Module Failed, cnt 2
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0005, 32'h0000_0001, "T16c Msg1 fault in MSG2_ONLY: MSG1_READY only, no IMU_FAULT");
        u_sensor.send_msg2(
            8'h0A,
            16'h0C0C, 16'h0C0C, 16'h0C0C,
            16'h0C0C, 16'h0C0C, 16'h0C0C,
            16'h1913, 16'h0003,            // SW1[4] Module Failed, cnt 3
            32'd1, 32'd2, 32'd3, 32'd4, 32'd5, 32'd6,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0004, 32'h0000_0004, "T16c Msg2 fault in MSG2_ONLY raises IMU_FAULT");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        // 16d. MSG2_ONLY: LAST_SEQ follows every valid frame, not only Msg 02
        u_sensor.send_msg1(
            8'h0A,
            16'h0D0D, 16'h0D0D, 16'h0D0D,
            16'h0D0D, 16'h0D0D, 16'h0D0D,
            16'h1904, 16'h0003,            // cnt 4, no fault
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_STATUS, rd_val);
        check_eq((rd_val >> 16) & 32'h0000_000F, 32'd4, "T16d STATUS.LAST_SEQ == 4 after Msg1 in MSG2_ONLY");
        axi_read(ADDR_SEQ_ERR_CNT, rd_val);
        check_eq(rd_val, 32'd0, "T16d no SEQ_GAP across Msg1/Msg2 in MSG2_ONLY");

        // 16e. Watchdog expires silently during Commanded BIT, then MUST re-arm
        //      on BIT_L release and report a sensor that stays silent
        axi_write(ADDR_CTRL,        32'h0000_0001, 4'hF, 2'd0); // normal mode
        axi_write(ADDR_TIMEOUT_CFG, 32'd150,       4'hF, 2'd0); // 150 us
        u_sensor.send_msg1(
            8'h0A,
            16'h0E0E, 16'h0E0E, 16'h0E0E,
            16'h0E0E, 16'h0E0E, 16'h0E0E,
            16'h1905, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        axi_write(ADDR_CTRL,       32'h0000_0005, 4'hF, 2'd0); // CMD_BIT_EN = 1
        u_sensor.send_idle_flags(25);                           // 200 us > 150 us
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0000, "T16e no LINK_TIMEOUT while Commanded BIT active");
        axi_write(ADDR_CTRL,       32'h0000_0001, 4'hF, 2'd0); // release BIT_L
        u_sensor.send_idle_flags(50);                           // 400 us > 150 + 200 us BIT grace
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "T16e LINK_TIMEOUT re-armed after BIT_L release");
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);
        axi_write(ADDR_TIMEOUT_CFG, 32'd30000,     4'hF, 2'd0);

        // 16f. 33.333 MHz clock controller: 1 us tick = 34 cycles (>= 1.000 us)
        check_eq(frac_tick_period, 32'd34, "T16f 1 us tick rounds up on 33.333 MHz clock");

        //----------------------------------------------------------------------
        // [T17] Edge-triggered GIC safety: every W1C write re-arms irq_o, so an
        //       enabled flag left pending produces a NEW rising edge
        //----------------------------------------------------------------------
        $display("[T17] Testing irq_o re-arm (new rising edge) after partial W1C...");
        axi_write(ADDR_CTRL,        32'h0000_0001, 4'hF, 2'd0); // normal mode
        axi_write(ADDR_INT_ENABLE,  32'h0000_007F, 4'hF, 2'd0); // all sources
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_msg1(
            8'h0A,
            16'h1717, 16'h1717, 16'h1717,
            16'h1717, 16'h1717, 16'h1717,
            16'h1906, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0041, 32'h0000_0041, "T17 MSG1_READY and MC_ANY pending");
        check_eq({31'd0, irq_o}, 32'd1, "T17 irq_o high before partial W1C");
        t17_rise_base = irq_rise_cnt;

        // Clear MSG1_READY only: MC_ANY (enabled) stays pending
        axi_write(ADDR_INT_STATUS, 32'h0000_0001, 4'hF, 2'd0);
        repeat (3) @(posedge s_axi_aclk);
        check_eq({31'd0, irq_at_bvalid_r}, 32'd0, "T17 irq_o dropped in the W1C cycle");
        check_eq({31'd0, irq_o}, 32'd1, "T17 irq_o high again (MC_ANY still pending)");
        check_eq(irq_rise_cnt - t17_rise_base, 32'd1, "T17 exactly one NEW rising edge after partial W1C");

        // Full clear: irq_o low, no further edge
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        repeat (3) @(posedge s_axi_aclk);
        check_eq({31'd0, irq_o}, 32'd0, "T17 irq_o low after full W1C");
        check_eq(irq_rise_cnt - t17_rise_base, 32'd1, "T17 no spurious edge after full W1C");

        //----------------------------------------------------------------------
        // [T18] Datasheet timing: watchdog grace after start-up / RESET_L (0.8 s
        //       activation, Sec. 5.1 & 6.4) and after Commanded BIT (Sec. 6.3.2).
        //       Simulation scale: timeout 150 us, start-up grace 300 us, BIT 200 us
        //----------------------------------------------------------------------
        $display("[T18] Testing watchdog grace after start-up, RESET_L and BIT...");
        axi_write(ADDR_TIMEOUT_CFG, 32'd150,       4'hF, 2'd0);
        axi_write(ADDR_INT_ENABLE,  32'h0000_007F, 4'hF, 2'd0);

        // 18a. Soft reset: sensor still in power-up BIT (flags only)
        axi_write(ADDR_CTRL,        32'h0000_0201, 4'hF, 2'd0); // CLK_EN + SOFT_RESET
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_idle_flags(50);                            // 400 us < 450 us
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0000, "T18a no LINK_TIMEOUT inside start-up grace");
        u_sensor.send_idle_flags(25);                            // now > 450 us
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "T18a LINK_TIMEOUT after start-up grace expires");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        // 18b. RESET_L pulse: after release the sensor restarts -> start-up grace
        u_sensor.send_msg1(
            8'h0A,
            16'h1818, 16'h1818, 16'h1818,
            16'h1818, 16'h1818, 16'h1818,
            16'h1901, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_write(ADDR_CTRL, 32'h0000_0009, 4'hF, 2'd0);        // CLK_EN + RESET_PULSE
        @(posedge imu_reset_l_o);                                // RESET_L released
        axi_read(ADDR_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0000, "T18b RESET_L pulse finished");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_idle_flags(50);                            // 400 us: < 450, > 350
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0000, "T18b no LINK_TIMEOUT inside post-RESET_L grace");
        u_sensor.send_idle_flags(25);
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "T18b LINK_TIMEOUT after post-RESET_L grace");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        // 18c. Commanded BIT: after BIT_L release the shorter BIT grace applies
        u_sensor.send_msg1(
            8'h0A,
            16'h1818, 16'h1818, 16'h1818,
            16'h1818, 16'h1818, 16'h1818,
            16'h1902, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_write(ADDR_CTRL, 32'h0000_0005, 4'hF, 2'd0);        // CMD_BIT_EN
        u_sensor.send_idle_flags(10);
        axi_write(ADDR_CTRL, 32'h0000_0001, 4'hF, 2'd0);        // release BIT_L
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_idle_flags(37);                            // ~300 us < 350 us
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0000, "T18c no LINK_TIMEOUT inside post-BIT grace");
        u_sensor.send_idle_flags(25);
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "T18c LINK_TIMEOUT after post-BIT grace");
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);

        // 18d. The first valid frame ends the grace: normal 150 us timeout again
        axi_write(ADDR_CTRL,       32'h0000_0201, 4'hF, 2'd0);  // soft reset -> grace
        u_sensor.send_msg1(
            8'h0A,
            16'h1818, 16'h1818, 16'h1818,
            16'h1818, 16'h1818, 16'h1818,
            16'h1903, 16'h0003,
            1'b1, 1'b0, 1'b0
        );
        wait_rx_settle();
        axi_write(ADDR_INT_STATUS, 32'h0000_007F, 4'hF, 2'd0);
        u_sensor.send_idle_flags(25);                            // 200 us > 150 us
        wait_rx_settle();
        axi_read(ADDR_INT_STATUS, rd_val);
        check_eq(rd_val & 32'h0000_0020, 32'h0000_0020, "T18d valid frame ends grace (150 us timeout)");
        axi_write(ADDR_INT_STATUS,  32'h0000_007F, 4'hF, 2'd0);
        axi_write(ADDR_TIMEOUT_CFG, 32'd30000,     4'hF, 2'd0);

        // 18e. Default parameters cover the datasheet times (0.8 s, 0.1 + 0.2 s)
        check_eq({31'd0, (u_dut_dflt.u_msg_parser.C_ACTIVATION_GRACE_US >= 800000)}, 32'd1,
                 "T18e default start-up grace >= 0.8 s");
        check_eq({31'd0, (u_dut_dflt.u_msg_parser.C_BIT_GRACE_US >= 300000)}, 32'd1,
                 "T18e default BIT grace >= 0.3 s");

        //----------------------------------------------------------------------
        // Final Summary
        //----------------------------------------------------------------------
        $display("==============================================================");
        if (err_cnt == 0) begin
            $display(" ALL TESTS PASSED! (%0d assertions passed, 0 errors)", test_pass_cnt);
        end else begin
            $display(" VERIFICATION FAILED! (%0d errors, %0d passed)", err_cnt, test_pass_cnt);
        end
        $display("==============================================================");
        $finish;
    end

endmodule

`default_nettype wire
