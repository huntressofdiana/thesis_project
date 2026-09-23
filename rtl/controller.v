`timescale 1ns / 1ps

/*
 * DDR4 AXI packet-stream test controller
 */
module controller #(
    parameter integer C_M_AXI_ADDR_WIDTH = 29,
    parameter integer C_M_AXI_DATA_WIDTH = 128,
    parameter integer C_M_AXI_ID_WIDTH   = 4
)(
    input  wire                              clk,
    input  wire                              ui_rst,
    
    // User Control Interface
    // Generic memory command
    input  wire                              cmd_valid,
    output wire                              cmd_ready,
    input  wire                              cmd_write,
    input  wire [C_M_AXI_ADDR_WIDTH-1:0]     cmd_addr,
    input  wire [8:0]                        cmd_len,
    
    output reg                               cmd_done,
    output reg                               error,
    
    // Input data
    input  wire [C_M_AXI_DATA_WIDTH-1:0]    wr_data,
    input  wire                             wr_valid,
    output wire                         wr_ready,


    // AXI4 write-address channel (AW)
    output wire [C_M_AXI_ID_WIDTH-1:0]        M_AXI_AWID,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]      M_AXI_AWADDR,
    output wire [7:0]                         M_AXI_AWLEN,
    output wire [2:0]                         M_AXI_AWSIZE,
    output wire [1:0]                         M_AXI_AWBURST,
    output wire                               M_AXI_AWLOCK,
    output wire [3:0]                         M_AXI_AWCACHE,
    output wire [2:0]                         M_AXI_AWPROT,
    output wire [3:0]                         M_AXI_AWQOS,
    output wire                               M_AXI_AWVALID,
    input  wire                               M_AXI_AWREADY,

    // AXI4 write-data channel (W)
    output wire [C_M_AXI_DATA_WIDTH-1:0]      M_AXI_WDATA,
    output wire [(C_M_AXI_DATA_WIDTH/8)-1:0]  M_AXI_WSTRB,
    output wire                               M_AXI_WLAST,
    output wire                               M_AXI_WVALID,
    input  wire                               M_AXI_WREADY,

    // AXI4 write-response channel (B)
    input  wire [C_M_AXI_ID_WIDTH-1:0]        M_AXI_BID,
    input  wire [1:0]                         M_AXI_BRESP,
    input  wire                               M_AXI_BVALID,
    output wire                               M_AXI_BREADY,

    // AXI4 read-address channel (AR)
    output wire [C_M_AXI_ID_WIDTH-1:0]        M_AXI_ARID,
    output wire [C_M_AXI_ADDR_WIDTH-1:0]      M_AXI_ARADDR,
    output wire [7:0]                         M_AXI_ARLEN,
    output wire [2:0]                         M_AXI_ARSIZE,
    output wire [1:0]                         M_AXI_ARBURST,
    output wire                               M_AXI_ARLOCK,
    output wire [3:0]                         M_AXI_ARCACHE,
    output wire [2:0]                         M_AXI_ARPROT,
    output wire [3:0]                         M_AXI_ARQOS,
    output wire                               M_AXI_ARVALID,
    input  wire                               M_AXI_ARREADY,

    // AXI4 read-data channel (R)
    input  wire [C_M_AXI_ID_WIDTH-1:0]        M_AXI_RID,
    input  wire [C_M_AXI_DATA_WIDTH-1:0]      M_AXI_RDATA,
    input  wire [1:0]                         M_AXI_RRESP,
    input  wire                               M_AXI_RLAST,
    input  wire                               M_AXI_RVALID,
    output wire                               M_AXI_RREADY
);

    localparam [2:0] AXI_SIZE_16_BYTES = 3'd4;
    localparam [1:0] AXI_BURST_INCR    = 2'b01;
    localparam [1:0] AXI_RESP_OKAY     = 2'b00;

    localparam [3:0]
        S_RESET       = 4'd0,
        S_WAIT_CALIB  = 4'd1,
        S_WAIT_START  = 4'd2,
        S_WRITE_ADDR  = 4'd3,
        S_WRITE_DATA  = 4'd4,
        S_WRITE_RESP  = 4'd5,
        S_READ_ADDR   = 4'd6,
        S_READ_DATA   = 4'd7;

    reg [3:0]  state;
   

    // AXI channel constants.
    assign M_AXI_AWID    = {C_M_AXI_ID_WIDTH{1'b0}};
    assign M_AXI_AWLEN   = WORDS_PER_PACKET - 1;
    assign M_AXI_AWSIZE  = AXI_SIZE_16_BYTES;
    assign M_AXI_AWBURST = AXI_BURST_INCR;
    assign M_AXI_AWLOCK  = 1'b0;
    assign M_AXI_AWCACHE = 4'b0011;
    assign M_AXI_AWPROT  = 3'b000;
    assign M_AXI_AWQOS   = 4'b0000;

    assign M_AXI_ARID    = {C_M_AXI_ID_WIDTH{1'b0}};
    assign M_AXI_ARLEN   = WORDS_PER_PACKET - 1;
    assign M_AXI_ARSIZE  = AXI_SIZE_16_BYTES;
    assign M_AXI_ARBURST = AXI_BURST_INCR;
    assign M_AXI_ARLOCK  = 1'b0;
    assign M_AXI_ARCACHE = 4'b0011;
    assign M_AXI_ARPROT  = 3'b000;
    assign M_AXI_ARQOS   = 4'b0000;

    assign M_AXI_AWVALID =
        (state == S_AW_A) || (state == S_AW_B);

    assign M_AXI_AWADDR =
        (state == S_AW_B) ? ROW_B_ADDR : ROW_A_ADDR;

    assign M_AXI_ARVALID =
        (state == S_AR_A) || (state == S_AR_B);

    assign M_AXI_ARADDR =
        (state == S_AR_B) ? ROW_B_ADDR : ROW_A_ADDR;

    /*
     * Packet stream drives AXI W directly. Because the packet generator
     * advances only when packet_ready is high, WDATA/WLAST remain stable
     * whenever the MIG stalls WREADY.
     */
    assign M_AXI_WDATA  = packet_data;
    assign M_AXI_WSTRB  = {(C_M_AXI_DATA_WIDTH/8){1'b1}};
    assign M_AXI_WLAST  = write_state && packet_valid && expected_last_beat;
    assign M_AXI_WVALID = write_state && packet_valid;

    assign M_AXI_BREADY =
        (state == S_B_A) || (state == S_B_B);

    assign M_AXI_RREADY =
        reading_packet_a || reading_packet_b;

    always @(posedge clk) begin
        if (ui_rst) begin
            state                <= S_RESET;
            beat_count           <= 8'd0;
            done                 <= 1'b0;
            error                <= 1'b0;
            completed_tests      <= 32'd0;
            active_cycle_count   <= 32'd0;
            last_test_cycles     <= 32'd0;
            first_error_beat     <= 32'd0;
            first_error_expected <= {C_M_AXI_DATA_WIDTH{1'b0}};
            first_error_received <= {C_M_AXI_DATA_WIDTH{1'b0}};
        end
        else begin
            done <= 1'b0;

            if (!init_calib_complete) begin
                state              <= S_WAIT_CALIB;
                beat_count         <= 8'd0;
                active_cycle_count <= 32'd0;
            end
            else begin
                if ((state >= S_AW_A) && (state <= S_R_B))
                    active_cycle_count <= active_cycle_count + 32'd1;

                case (state)
                    S_RESET: begin
                        state <= S_WAIT_CALIB;
                    end

                    S_WAIT_CALIB: begin
                        beat_count <= 8'd0;
                        state      <= S_WAIT_START;
                    end

                    S_WAIT_START: begin
                        beat_count <= 8'd0;
                        if (start) begin
                            error                <= 1'b0;
                            active_cycle_count   <= 32'd0;
                            first_error_beat     <= 32'd0;
                            first_error_expected <= {C_M_AXI_DATA_WIDTH{1'b0}};
                            first_error_received <= {C_M_AXI_DATA_WIDTH{1'b0}};
                            state                <= S_AW_A;
                        end
                    end

                    // Write Packet A.
                    S_AW_A: begin
                        if (M_AXI_AWVALID && M_AXI_AWREADY) begin
                            beat_count <= 8'd0;
                            state      <= S_W_A;
                        end
                    end

                    S_W_A: begin
                        if (write_handshake) begin
                            if (packet_last != expected_last_beat) begin
                                error            <= 1'b1;
                                first_error_beat <= beat_count;
                                state            <= S_ERROR;
                            end
                            else if (expected_last_beat && packet_slast) begin
                                // Packet A is not the final packet of the frame.
                                error            <= 1'b1;
                                first_error_beat <= beat_count;
                                state            <= S_ERROR;
                            end
                            else if (expected_last_beat) begin
                                beat_count <= 8'd0;
                                state      <= S_B_A;
                            end
                            else begin
                                beat_count <= beat_count + 8'd1;
                            end
                        end
                    end

                    S_B_A: begin
                        if (M_AXI_BVALID && M_AXI_BREADY) begin
                            if ((M_AXI_BRESP != AXI_RESP_OKAY) ||
                                (M_AXI_BID != {C_M_AXI_ID_WIDTH{1'b0}})) begin
                                error <= 1'b1;
                                state <= S_ERROR;
                            end
                            else begin
                                state <= S_AW_B;
                            end
                        end
                    end

                    // Write Packet B.
                    S_AW_B: begin
                        if (M_AXI_AWVALID && M_AXI_AWREADY) begin
                            beat_count <= 8'd0;
                            state      <= S_W_B;
                        end
                    end

                    S_W_B: begin
                        if (write_handshake) begin
                            if (packet_last != expected_last_beat) begin
                                error            <= 1'b1;
                                first_error_beat <= beat_count;
                                state            <= S_ERROR;
                            end
                            else if (expected_last_beat && !packet_slast) begin
                                // Packet B should finish the two-packet frame.
                                error            <= 1'b1;
                                first_error_beat <= beat_count;
                                state            <= S_ERROR;
                            end
                            else if (expected_last_beat) begin
                                beat_count <= 8'd0;
                                state      <= S_B_B;
                            end
                            else begin
                                beat_count <= beat_count + 8'd1;
                            end
                        end
                    end

                    S_B_B: begin
                        if (M_AXI_BVALID && M_AXI_BREADY) begin
                            if ((M_AXI_BRESP != AXI_RESP_OKAY) ||
                                (M_AXI_BID != {C_M_AXI_ID_WIDTH{1'b0}})) begin
                                error <= 1'b1;
                                state <= S_ERROR;
                            end
                            else begin
                                state <= S_AR_A;
                            end
                        end
                    end

                    // Read and verify Packet A.
                    S_AR_A: begin
                        if (M_AXI_ARVALID && M_AXI_ARREADY) begin
                            beat_count <= 8'd0;
                            state      <= S_R_A;
                        end
                    end

                    S_R_A: begin
                        if (read_handshake) begin
                            if ((M_AXI_RRESP != AXI_RESP_OKAY) ||
                                (M_AXI_RID != {C_M_AXI_ID_WIDTH{1'b0}}) ||
                                (M_AXI_RDATA != expected_read_word) ||
                                (M_AXI_RLAST != expected_last_beat)) begin
                                error                <= 1'b1;
                                first_error_beat     <= beat_count;
                                first_error_expected <= expected_read_word;
                                first_error_received <= M_AXI_RDATA;
                                state                <= S_ERROR;
                            end
                            else if (expected_last_beat) begin
                                beat_count <= 8'd0;
                                state      <= S_AR_B;
                            end
                            else begin
                                beat_count <= beat_count + 8'd1;
                            end
                        end
                    end

                    // Read and verify Packet B.
                    S_AR_B: begin
                        if (M_AXI_ARVALID && M_AXI_ARREADY) begin
                            beat_count <= 8'd0;
                            state      <= S_R_B;
                        end
                    end

                    S_R_B: begin
                        if (read_handshake) begin
                            if ((M_AXI_RRESP != AXI_RESP_OKAY) ||
                                (M_AXI_RID != {C_M_AXI_ID_WIDTH{1'b0}}) ||
                                (M_AXI_RDATA != expected_read_word) ||
                                (M_AXI_RLAST != expected_last_beat)) begin
                                error                <= 1'b1;
                                first_error_beat     <= beat_count;
                                first_error_expected <= expected_read_word;
                                first_error_received <= M_AXI_RDATA;
                                state                <= S_ERROR;
                            end
                            else if (expected_last_beat) begin
                                beat_count <= 8'd0;
                                state      <= S_PASS;
                            end
                            else begin
                                beat_count <= beat_count + 8'd1;
                            end
                        end
                    end

                    S_PASS: begin
                        done             <= 1'b1;
                        completed_tests  <= completed_tests + 32'd1;
                        last_test_cycles <= active_cycle_count;
                        beat_count       <= 8'd0;

                        if (CONTINUOUS_TEST != 0) begin
                            active_cycle_count <= 32'd0;
                            state              <= S_AW_A;
                        end
                        else begin
                            state <= S_WAIT_START;
                        end
                    end

                    S_ERROR: begin
                        error <= 1'b1;
                        state <= S_ERROR;
                    end

                    default: begin
                        error <= 1'b1;
                        state <= S_ERROR;
                    end
                endcase
            end
        end
    end

endmodule

