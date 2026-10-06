`timescale 1ns / 1ps

/*
 * Generic DDR4 AXI4 controller
 *
 * Purpose:
 *   Accept one generic memory command and perform one AXI4 burst.
 */

module ddr_axi_controller #(
    parameter integer C_M_AXI_ADDR_WIDTH = 29,
    parameter integer C_M_AXI_DATA_WIDTH = 128,
    parameter integer C_M_AXI_ID_WIDTH   = 4
)(
    input  wire                             clk,
    input  wire                             ui_rst,
    input  wire                             init_calib_complete,

    // ============================================================
    // Generic command interface
    // ============================================================

    input  wire                             cmd_valid,
    output wire                             cmd_ready,

    // 1 = write, 0 = read
    input  wire                             cmd_write,

    // AXI byte address
    input  wire [C_M_AXI_ADDR_WIDTH-1:0]    cmd_addr,

    // Number of AXI beats in the burst: 1 to 256
    input  wire [8:0]                       cmd_len,

    // Pulses for one clock when command completes
    output reg                              cmd_done,

    // Sticky error flag
    output reg                              error,

    // ============================================================
    // Write data input
    // ============================================================

    input  wire [C_M_AXI_DATA_WIDTH-1:0]    wr_data,
    input  wire                             wr_valid,
    output wire                             wr_ready,

    // ============================================================
    // AXI4 write-address channel
    // ============================================================

    output wire [C_M_AXI_ID_WIDTH-1:0]      M_AXI_AWID,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]     M_AXI_AWADDR,
    output wire [7:0]                       M_AXI_AWLEN,
    output wire [2:0]                       M_AXI_AWSIZE,
    output wire [1:0]                       M_AXI_AWBURST,
    output wire                             M_AXI_AWLOCK,
    output wire [3:0]                       M_AXI_AWCACHE,
    output wire [2:0]                       M_AXI_AWPROT,
    output wire [3:0]                       M_AXI_AWQOS,
    output wire                             M_AXI_AWVALID,
    input  wire                             M_AXI_AWREADY,

    // ============================================================
    // AXI4 write-data channel
    // ============================================================

    output wire [C_M_AXI_DATA_WIDTH-1:0]    M_AXI_WDATA,
    output wire [(C_M_AXI_DATA_WIDTH/8)-1:0] M_AXI_WSTRB,
    output wire                             M_AXI_WLAST,
    output wire                             M_AXI_WVALID,
    input  wire                             M_AXI_WREADY,

    // ============================================================
    // AXI4 write-response channel
    // ============================================================

    input  wire [C_M_AXI_ID_WIDTH-1:0]      M_AXI_BID,
    input  wire [1:0]                       M_AXI_BRESP,
    input  wire                             M_AXI_BVALID,
    output wire                             M_AXI_BREADY,

    // ============================================================
    // AXI4 read-address channel
    // ============================================================

    output wire [C_M_AXI_ID_WIDTH-1:0]      M_AXI_ARID,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]     M_AXI_ARADDR,
    output wire [7:0]                       M_AXI_ARLEN,
    output wire [2:0]                       M_AXI_ARSIZE,
    output wire [1:0]                       M_AXI_ARBURST,
    output wire                             M_AXI_ARLOCK,
    output wire [3:0]                       M_AXI_ARCACHE,
    output wire [2:0]                       M_AXI_ARPROT,
    output wire [3:0]                       M_AXI_ARQOS,
    output wire                             M_AXI_ARVALID,
    input  wire                             M_AXI_ARREADY,

    // ============================================================
    // AXI4 read-data channel
    // ============================================================

    input  wire [C_M_AXI_ID_WIDTH-1:0]      M_AXI_RID,
    input  wire [C_M_AXI_DATA_WIDTH-1:0]    M_AXI_RDATA,
    input  wire [1:0]                       M_AXI_RRESP,
    input  wire                             M_AXI_RLAST,
    input  wire                             M_AXI_RVALID,
    output wire                             M_AXI_RREADY,

    // ============================================================
    // Generic read-data output
    // ============================================================

    output wire [C_M_AXI_DATA_WIDTH-1:0]    rd_data,
    output wire                             rd_valid,
    output wire                             rd_last,
    input  wire                             rd_ready
);


    // ============================================================
    // AXI constants
    // ============================================================

    localparam integer BYTES_PER_BEAT =
        C_M_AXI_DATA_WIDTH / 8;

    localparam [2:0] AXI_SIZE =
        $clog2(BYTES_PER_BEAT);

    localparam [1:0] AXI_BURST_INCR = 2'b01;
    localparam [1:0] AXI_RESP_OKAY  = 2'b00;

    localparam [C_M_AXI_ID_WIDTH-1:0] AXI_ID_ZERO =
        {C_M_AXI_ID_WIDTH{1'b0}};


    // ============================================================
    // Controller states
    // ============================================================

    localparam [3:0]
        S_RESET       = 4'd0,
        S_WAIT_CALIB  = 4'd1,
        S_IDLE        = 4'd2,
        S_WRITE       = 4'd3,
        S_WRITE_RESP  = 4'd4,
        S_READ_ADDR   = 4'd5,
        S_READ_DATA   = 4'd6,
        S_ERROR       = 4'd7;

    reg [3:0] state;


    // ============================================================
    // Stored command
    // ============================================================

    reg [C_M_AXI_ADDR_WIDTH-1:0] active_addr;
    reg [8:0]                    active_len;
    reg                          active_write;


    // ============================================================
    // Transaction progress
    // ============================================================

    // 0 -> first beat
    // 1 -> second beat
    // ...
    // active_len-1 -> final beat
    reg [8:0] beat_count;


    // Write address and write data can complete independently.
    reg aw_done;
    reg w_done;

    // Registered AXI write-data channel
    reg [C_M_AXI_DATA_WIDTH-1:0] axi_wdata;
    reg                          axi_wvalid;
    reg                          axi_wlast;

    // Number of beats accepted from the upstream wr_* interface
    reg [8:0] wr_load_count;


    // ============================================================
    // Command validation
    // ============================================================

    wire cmd_len_valid =
        (cmd_len != 9'd0) &&
        (cmd_len <= 9'd256);

    wire cmd_addr_aligned =
        (cmd_addr[AXI_SIZE-1:0] == 0);

    /*
     * AXI bursts cannot cross a 4 KiB boundary.
     *
     * cmd_total_bytes is extended so that a 256-beat burst
     * is represented correctly.
     */
    wire [C_M_AXI_ADDR_WIDTH:0] cmd_total_bytes =
        {{(C_M_AXI_ADDR_WIDTH+1-9){1'b0}}, cmd_len}
        << AXI_SIZE;

    wire [C_M_AXI_ADDR_WIDTH:0] cmd_page_offset =
        {{(C_M_AXI_ADDR_WIDTH+1-12){1'b0}},
         cmd_addr[11:0]};

    wire cmd_4kb_valid =
        (cmd_page_offset + cmd_total_bytes) <= 32'd4096;

    wire cmd_valid_parameters =
        cmd_len_valid &&
        cmd_addr_aligned &&
        cmd_4kb_valid;


    // ============================================================
    // Command interface
    // ============================================================

    /*
     * A new command can only be accepted when the controller
     * is idle and DDR calibration has completed.
     */
    assign cmd_ready =
        (state == S_IDLE) &&
        init_calib_complete &&
        !error;


    // ============================================================
    // AXI handshakes
    // ============================================================

    wire aw_handshake =
        M_AXI_AWVALID && M_AXI_AWREADY;

    wire w_handshake =
        M_AXI_WVALID && M_AXI_WREADY;

    wire wr_handshake =
        wr_valid && wr_ready;

    wire b_handshake =
        M_AXI_BVALID && M_AXI_BREADY;

    wire ar_handshake =
        M_AXI_ARVALID && M_AXI_ARREADY;

    wire r_handshake =
        M_AXI_RVALID && M_AXI_RREADY;


    // ============================================================
    // Expected final write/read beat
    // ============================================================

    wire expected_last_beat =
        (beat_count == (active_len - 9'd1));


    // ============================================================
    // AXI write-address channel
    // ============================================================

    assign M_AXI_AWID =
        AXI_ID_ZERO;

    assign M_AXI_AWADDR =
        active_addr;

    assign M_AXI_AWLEN = 
        active_len[7:0] - 8'd1;
    
    assign M_AXI_AWSIZE =
        AXI_SIZE;

    assign M_AXI_AWBURST =
        AXI_BURST_INCR;

    assign M_AXI_AWLOCK =
        1'b0;

    assign M_AXI_AWCACHE =
        4'b0011;

    assign M_AXI_AWPROT =
        3'b000;

    assign M_AXI_AWQOS =
        4'b0000;


    /*
     * IMPORTANT:
     *
     * AWVALID does not wait for AWREADY.
     *
     * It remains asserted until the AW handshake occurs.
     */
    assign M_AXI_AWVALID =
        (state == S_WRITE) &&
        !aw_done;


    // ============================================================
    // AXI write-data channel
    // ============================================================

    assign M_AXI_WDATA  = axi_wdata;

    assign M_AXI_WSTRB =
        {(C_M_AXI_DATA_WIDTH/8){1'b1}};

    assign M_AXI_WVALID = axi_wvalid;

    assign M_AXI_WLAST  = axi_wlast;


    // Upstream may provide a new beat when:
    //
    // 1. We are writing
    // 2. We still need more beats
    // 3. The AXI buffer is empty OR the current AXI beat
    //    is being accepted this cycle
    assign wr_ready =
        (state == S_WRITE) &&
        !w_done &&
        (wr_load_count < active_len) &&
        (!axi_wvalid || M_AXI_WREADY);


    // ============================================================
    // AXI write-response channel
    // ============================================================

    assign M_AXI_BREADY =
        (state == S_WRITE_RESP);


    // ============================================================
    // AXI read-address channel
    // ============================================================

    assign M_AXI_ARID =
        AXI_ID_ZERO;

    assign M_AXI_ARADDR =
        active_addr;

    assign M_AXI_ARLEN =
        active_len[7:0] - 8'd1;

    assign M_AXI_ARSIZE =
        AXI_SIZE;

    assign M_AXI_ARBURST =
        AXI_BURST_INCR;

    assign M_AXI_ARLOCK =
        1'b0;

    assign M_AXI_ARCACHE =
        4'b0011;

    assign M_AXI_ARPROT =
        3'b000;

    assign M_AXI_ARQOS =
        4'b0000;


    assign M_AXI_ARVALID =
        (state == S_READ_ADDR);


    // ============================================================
    // AXI read-data channel
    // ============================================================

    assign rd_data =
        M_AXI_RDATA;

    assign rd_valid =
        (state == S_READ_DATA) &&
        M_AXI_RVALID;

    assign rd_last =
        (state == S_READ_DATA) &&
        M_AXI_RVALID &&
        M_AXI_RLAST;


    /*
     * The downstream read consumer controls whether we
     * accept read data.
     */
    assign M_AXI_RREADY =
        (state == S_READ_DATA) &&
        rd_ready;


    // ============================================================
    // Store command
    // ============================================================

    always @(posedge clk) begin

        if (ui_rst) begin

            active_addr  <= {C_M_AXI_ADDR_WIDTH{1'b0}};
            active_len   <= 9'd0;
            active_write <= 1'b0;

        end
        else if (cmd_valid && cmd_ready) begin

            active_addr  <= cmd_addr;
            active_len   <= cmd_len;
            active_write <= cmd_write;

        end

    end


    // ============================================================
    // Main controller FSM
    // ============================================================

    always @(posedge clk) begin

        if (ui_rst) begin

            state      <= S_RESET;

            beat_count <= 9'd0;

            aw_done    <= 1'b0;
            w_done     <= 1'b0;

            cmd_done   <= 1'b0;
            error      <= 1'b0;

            wr_load_count <= 9'd0;

            axi_wdata  <= {C_M_AXI_DATA_WIDTH{1'b0}};
            axi_wvalid <= 1'b0;
            axi_wlast  <= 1'b0;

        end
        else begin

            // cmd_done is a one-cycle pulse
            cmd_done <= 1'b0;


            // ----------------------------------------------------
            // Calibration lost
            // ----------------------------------------------------

            if (!init_calib_complete) begin

                state      <= S_WAIT_CALIB;
                beat_count <= 9'd0;

                aw_done    <= 1'b0;
                w_done     <= 1'b0;

                wr_load_count <= 9'd0;

                axi_wvalid <= 1'b0;
                axi_wlast  <= 1'b0;

            end
            else begin

                case (state)

                    // ------------------------------------------------
                    // Reset
                    // ------------------------------------------------

                    S_RESET: begin

                        state <= S_WAIT_CALIB;

                    end


                    // ------------------------------------------------
                    // Wait for DDR calibration
                    // ------------------------------------------------

                    S_WAIT_CALIB: begin

                        if (init_calib_complete)
                            state <= S_IDLE;

                    end


                    // ------------------------------------------------
                    // Idle
                    // ------------------------------------------------

                    S_IDLE: begin

                        beat_count <= 9'd0;
                        wr_load_count <= 9'd0;

                        axi_wvalid <= 1'b0;
                        axi_wlast  <= 1'b0;

                        if (cmd_valid && cmd_ready) begin

                            /*
                             * Reject invalid commands.
                             */
                            if (!cmd_valid_parameters) begin

                                error    <= 1'b1;
                                cmd_done <= 1'b1;
                                state    <= S_ERROR;

                            end
                            else begin

                                /*
                                 * Start a new transaction.
                                 */
                                aw_done <= 1'b0;
                                w_done  <= 1'b0;

                                if (cmd_write) begin
                                    state <= S_WRITE;
                                end
                                else begin
                                    state <= S_READ_ADDR;
                                end

                            end

                        end

                    end


                    // ------------------------------------------------
                    // Write
                    // ------------------------------------------------

                    S_WRITE: begin

                        /*
                         * AW and W operate independently.
                         */

                        if (aw_handshake)
                            aw_done <= 1'b1;


                        // ------------------------------------------------
                        // AXI accepted the currently buffered beat
                        // ------------------------------------------------
                        if (w_handshake) begin

                            if (axi_wlast) begin
                                w_done <= 1'b1;
                            end
                            else begin
                                beat_count <= beat_count + 9'd1;
                            end

                            // If we don't simultaneously receive a new
                            // upstream beat, the AXI buffer becomes empty.
                            if (!wr_handshake) begin
                                axi_wvalid <= 1'b0;
                                axi_wlast  <= 1'b0;
                            end

                        end


                        // ------------------------------------------------
                        // Accept a new upstream write beat
                        // ------------------------------------------------
                        if (wr_handshake) begin

                            axi_wdata  <= wr_data;
                            axi_wvalid <= 1'b1;

                            // Mark the final beat when loading it
                            axi_wlast <=
                                (wr_load_count == (active_len - 9'd1));

                            wr_load_count <= wr_load_count + 9'd1;

                        end


                        /*
                         * The write transaction is complete only
                         * after BOTH:
                         *
                         *   1. AW has completed
                         *   2. final W beat has completed
                         */
                        if (
                            (aw_done || aw_handshake) &&
                            (w_done ||
                            (w_handshake && axi_wlast))
                        ) begin

                            state <= S_WRITE_RESP;
                        end

                    end


                    // ------------------------------------------------
                    // Write response
                    // ------------------------------------------------

                    S_WRITE_RESP: begin

                        if (b_handshake) begin

                            cmd_done <= 1'b1;

                            if (
                                (M_AXI_BRESP != AXI_RESP_OKAY) ||
                                (M_AXI_BID   != AXI_ID_ZERO)
                            ) begin

                                error <= 1'b1;
                                state <= S_ERROR;

                            end
                            else begin

                                state <= S_IDLE;

                            end

                        end

                    end


                    // ------------------------------------------------
                    // Read address
                    // ------------------------------------------------

                    S_READ_ADDR: begin

                        if (ar_handshake) begin

                            beat_count <= 9'd0;
                            state      <= S_READ_DATA;

                        end

                    end


                    // ------------------------------------------------
                    // Read data
                    // ------------------------------------------------

                    S_READ_DATA: begin

                        if (r_handshake) begin

                            /*
                             * Check the returned transaction.
                             */
                            if (
                                (M_AXI_RRESP != AXI_RESP_OKAY) ||
                                (M_AXI_RID   != AXI_ID_ZERO)
                            ) begin

                                error <= 1'b1;

                            end


                            /*
                             * RLAST must occur on the final
                             * expected beat.
                             */
                            if (M_AXI_RLAST && !expected_last_beat) begin

                                // RLAST arrived too early.
                                error    <= 1'b1;
                                cmd_done <= 1'b1;
                                state    <= S_ERROR;

                            end
                            else if (expected_last_beat) begin

                                /*
                                 * Expected final beat.
                                 */
                                cmd_done <= 1'b1;

                                if (
                                    !M_AXI_RLAST ||
                                    (M_AXI_RRESP != AXI_RESP_OKAY) ||
                                    (M_AXI_RID   != AXI_ID_ZERO) ||
                                    error
                                ) begin

                                    error <= 1'b1;
                                    state <= S_ERROR;

                                end
                                else begin

                                    state <= S_IDLE;

                                end

                            end
                            else begin

                                beat_count <= beat_count + 9'd1;

                            end

                        end

                    end


                    // ------------------------------------------------
                    // Error
                    // ------------------------------------------------

                    S_ERROR: begin

                        /*
                         * Sticky error state.
                         *
                         * Reset the controller to recover.
                         */

                        state <= S_ERROR;

                    end


                    default: begin

                        state <= S_ERROR;
                        error <= 1'b1;

                    end

                endcase

            end

        end

    end

endmodule