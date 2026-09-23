/*
    CAN 2.0B bus controller - bit-stream FSM
    
    Unincluded features:
    - bit stuffing
    - CRC generation
    - error handling
    
*/
`include "../src/Params.v"

module CAN_BUS (
    /* Inputs */
    input clk,
    input reset,
    input start,                   // pulse to request frame transmission
    input [ID_WIDTH-1:0] id,       // 11-bit Unique Identifier
  	input [DLC_WIDTH-1:0] dlc,	   // Data Length Code, number of data bytes (0-8)
    input [DATA_WIDTH-1:0] data,   // Payload data, MSB of byte 0 sent first
    input ack_in,                  // bus level sampled during the ACK slot

    /* Outputs */
    output reg tx,              // serial bus output (0 = dominant,1 = recessive)
    output reg busy,            // high while a frame is in flight
    output reg done,            // one-cycle pulse at end of IFS
    output reg ack_err         // set if no receiver acknowledged
);

    /* FSM States */
    localparam IDLE     = 4'd0;  // IDLE state, waiting for a frame to transmit
    localparam SOF      = 4'd1;  // Start of Frame, transmit a dominant bit
    localparam ARB      = 4'd2;  // Arbitration, resolve bus contention
    localparam CTRL     = 4'd3;  // Control, transmit control bits (IDE, r0, DLC)
    localparam DATA     = 4'd4;  // Data, transmit the payload data
    localparam CRC      = 4'd5;  // Cyclic Redundancy Check, verify frame integrity
    localparam ACK      = 4'd6;  // Acknowledge, check for receiver acknowledgment
    localparam EOF      = 4'd7;  // End of Frame, transmit the end of frame sequence
    localparam IFS      = 4'd8;  // Interframe Space, wait before next frame

    reg [NUM_STATES_WIDTH-1:0] current_state;
    reg [BIT_CNT_WIDTH-1:0] bit_cnt; // Count bits transmitted in the current field
  	reg [CRC_WIDTH-1:0] crc = 15'h7234; // Arbitrary CRC (Not currently calculated)


	task reset_all_outputs;
        begin// Reset all outputs and internal signals
            tx <= 1'b1;         // Recessive state
            busy <= 1'b0;       // Not busy
            done <= 1'b0;       // Not done
            ack_err <= 1'b0;    // No acknowledgment error
            bit_cnt <= {BIT_CNT_WIDTH{1'b0}};    // Reset bit counter
        end 
    endtask

    always @(posedge clk) begin
        if (reset) begin
            // Reset all outputs and internal signals
            reset_all_outputs();
            current_state <= IDLE; // Go to IDLE state
        end 
        else begin
            done <= 1'b0;
            case (current_state)
                IDLE: begin
                    // Stay in IDLE state waiting for a frame to transmit
                    tx <= 1'b1;
                    busy <= 1'b0;
                    // Move to the Start of Frame state once new frame is requested
                    if (start) begin
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        busy <= 1'b1; // Set busy when a new frame is requested
                        ack_err <= 1'b0; // Clear the previous frame's result
                        current_state <= SOF; // Move to Start of Frame state
                    end
                end
                SOF: begin
                    // Transmit a dominant bit (0) for the Start of Frame
                    tx <= 1'b0;
                    bit_cnt <= {BIT_CNT_WIDTH{1'b0}}; // Reset bit counter
                    current_state <= ARB; // Move to the Arbitration state
                end
                ARB: begin
                    // Transmit the arbitration field (ID 11 + RTR 1).
                    // RTR is dominant (0) for data frames.
                    tx <= (bit_cnt < ID_WIDTH) ? id[ID_WIDTH-1-bit_cnt] : 1'b0;
                    // Leave on the same clock that drives the last bit, so no
                    // bit time is spent on the state change itself.
                    if (bit_cnt == ID_WIDTH) begin
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        current_state <= CTRL; // Move to the Control state
                    end
                    else bit_cnt <= bit_cnt + 1;
                end
                CTRL: begin
                    // Transmit the control bits (IDE, r0, DLC)
                    tx <= (bit_cnt < 2) ? 1'b0 // IDE = 0, r0 = 0
                                        : dlc[DLC_WIDTH-1+2-bit_cnt]; // DLC bits
                    if (bit_cnt == DLC_WIDTH+1) begin
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        current_state <= DATA; // Move to the Data state
                    end
                    else bit_cnt <= bit_cnt + 1;
                end
                DATA: begin
                    // Transmit the payload data, MSB of byte 0 first.
                  	tx <= data[(dlc*8)-1 - bit_cnt];
                  if (bit_cnt == (dlc*8)-1) begin
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        current_state <= CRC; // Move to the CRC state
                    end
                    else bit_cnt <= bit_cnt + 1;
                end
                CRC: begin
                    // Transmit the CRC sequence
                    tx <= crc[CRC_WIDTH-1 - bit_cnt];
                    if (bit_cnt == CRC_WIDTH-1) begin
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        current_state <= ACK; // Move to the Acknowledge state
                    end
                    else bit_cnt <= bit_cnt + 1;
                end
                ACK: begin
                    // The transmitter leaves both the ACK slot and its
                    // delimiter recessive; a receiver pulls the slot dominant.
                    tx <= 1'b1;
                    if (bit_cnt == 0) bit_cnt <= bit_cnt + 1;
                    else begin
                        // tx is registered, so the slot assigned last clock is
                        // only on the bus now - this is when ack_in is valid.
                        // Still recessive means nobody acknowledged.
                        ack_err <= ack_in;
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        current_state <= EOF; // Move to the End of Frame state
                    end
                end
                EOF: begin
                    // Transmit the end of frame sequence (7 recessive bits)
                    tx <= 1'b1;
                    if (bit_cnt == EOF_WIDTH-1) begin
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        current_state <= IFS; // Move to the Interframe Space state
                    end
                    else bit_cnt <= bit_cnt + 1;
                end
                IFS: begin
                    // Hold the bus recessive for the interframe space before
                    // allowing a new frame, then go back to IDLE.
                    tx <= 1'b1;
                    if (bit_cnt == IFS_WIDTH-1) begin
                        bit_cnt <= {BIT_CNT_WIDTH{1'b0}};
                        current_state <= IDLE;
                        done <= 1'b1; // Signal that the frame transmission is done
                    end
                    else bit_cnt <= bit_cnt + 1;
                end
                default: begin
                    // Default case, go back to IDLE
                    current_state <= IDLE;
                end
            endcase
        end
    end

endmodule