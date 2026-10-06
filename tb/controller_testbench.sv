`timescale 1ns / 1ps

module controller_testbench;

    import design_1_axi_vip_0_0_pkg::*;

    design_1_axi_vip_0_0_slv_mem_t slv_agent;

    // ------------------------------------------------------------
    // Clock / reset / calibration
    // ------------------------------------------------------------
    logic clk_100MHz = 0;
    logic reset = 1;
    logic init_calib_complete_0 = 0;

    always #5 clk_100MHz = ~clk_100MHz;

    // ------------------------------------------------------------
    // Generic command / data interface
    // ------------------------------------------------------------
    logic         cmd_valid_0;
    logic         cmd_ready_0;
    logic         cmd_write_0;
    logic         cmd_done_0;
    logic         error_0;

    logic [28:0]  cmd_addr_0;
    logic [8:0]   cmd_len_0;

    logic [127:0] wr_data_0;
    logic         wr_valid_0;
    logic         wr_ready_0;

    logic [127:0] rd_data_0;
    logic         rd_valid_0;
    logic         rd_last_0;
    logic         rd_ready_0;

    // ------------------------------------------------------------
    // DUT
    // ------------------------------------------------------------
    design_1_wrapper dut (
        .clk_100MHz            (clk_100MHz),
        .reset                 (reset),
        .init_calib_complete_0 (init_calib_complete_0),

        .cmd_valid_0 (cmd_valid_0),
        .cmd_write_0 (cmd_write_0),
        .cmd_addr_0  (cmd_addr_0),
        .cmd_len_0   (cmd_len_0),
        .cmd_ready_0 (cmd_ready_0),
        .cmd_done_0  (cmd_done_0),
        .error_0     (error_0),

        .wr_data_0   (wr_data_0),
        .wr_valid_0  (wr_valid_0),
        .wr_ready_0  (wr_ready_0),

        .rd_data_0   (rd_data_0),
        .rd_valid_0  (rd_valid_0),
        .rd_last_0   (rd_last_0),
        .rd_ready_0  (rd_ready_0)
    );

    // ------------------------------------------------------------
    // AXI VIP slave
    // ------------------------------------------------------------
    initial begin
        slv_agent = new(
            "AXI slave VIP",
            dut.design_1_i.axi_vip_0.inst.IF
        );

        slv_agent.start_slave();
    end

    // ------------------------------------------------------------
    // Test data
    // ------------------------------------------------------------
    bit [127:0] wdata_q [0:3];

    // ------------------------------------------------------------
    // Test sequence
    // ------------------------------------------------------------
    initial begin

        // Initial values
        wdata_q[0] = 128'h1;
        wdata_q[1] = 128'h2;
        wdata_q[2] = 128'h3;
        wdata_q[3] = 128'h4;

        cmd_valid_0 = 0;
        cmd_write_0 = 0;
        cmd_addr_0  = 0;
        cmd_len_0   = 0;

        wr_data_0   = 0;
        wr_valid_0  = 0;

        rd_ready_0  = 0;

        init_calib_complete_0 = 0;

        // ========================================================
        // RESET
        // ========================================================

        reset = 1;

        repeat (20)
            @(posedge clk_100MHz);

        @(negedge clk_100MHz);
        reset = 0;

        repeat (20)
            @(posedge clk_100MHz);

        @(negedge clk_100MHz);
        init_calib_complete_0 = 1;

        // Give controller a cycle to enter IDLE
        repeat (2)
            @(posedge clk_100MHz);

        // ========================================================
        // WRITE COMMAND
        // ========================================================

        @(negedge clk_100MHz);

        cmd_write_0 = 1;
        cmd_addr_0  = 29'h00000000;
        cmd_len_0   = 9'd4;
        cmd_valid_0 = 1;

        $display("%0t : asserting cmd_valid", $time);

        wait (cmd_ready_0 === 1'b1);
        @(posedge clk_100MHz);

        $display("%0t : COMMAND HANDSHAKE", $time);

        @(negedge clk_100MHz);
        cmd_valid_0 = 0;


        // ========================================================
        // WRITE DATA
        // ========================================================

       for (int i = 0; i < 4; i++) begin
            // Make sure VALID is low before changing DATA
            @(negedge clk_100MHz);
            wr_valid_0 = 0;
            wr_data_0  = wdata_q[i];

            // Present the new beat
            @(negedge clk_100MHz);
            wr_valid_0 = 1;

            $display(
                "%0t : presenting WRITE beat %0d, data=%h",
                $time,
                i,
                wr_data_0
            );

            // Wait for a REAL rising-edge handshake
            @(posedge clk_100MHz iff
                (wr_valid_0 === 1'b1 && wr_ready_0 === 1'b1));

            $display(
                "%0t : WRITE HANDSHAKE beat %0d",
                $time,
                i
            );

            // Remove VALID after the accepted beat
            @(negedge clk_100MHz);
            wr_valid_0 = 0;
        end

        wr_data_0 = 0;


        // ========================================================
        // RESPONSE
        // ========================================================

        wait (cmd_done_0 === 1'b1);

        $display("=================================");
        $display("WRITE COMPLETE");
        $display("ERROR = %b", error_0);
        $display("=================================");

        $finish;
    end

endmodule