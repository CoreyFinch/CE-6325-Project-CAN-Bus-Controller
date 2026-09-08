/* CAN 2.0B bus controller - bit-stream FSM (skeleton) */

module can_bus_controller (
    input  wire clk,        // driven by the testbench
    input  wire reset       // synchronous, active high
);

    /* FSM States */
    localparam STATE_W = 4;             // widen as states are added
    localparam IDLE    = 4'd0;

    reg [STATE_W-1:0] current_state;
    reg [STATE_W-1:0] next_state;

    // State register: advances on the clock, clears to IDLE on reset
    always @(posedge clk) begin
        if (reset)
            current_state <= IDLE;
        else
            current_state <= next_state;
    end

    // Next-state logic: combinational, so every branch must assign next_state
    always @(*) begin
        next_state = current_state;     // default: hold
        case (current_state)
            IDLE: next_state = IDLE;
            // More states to add here
            default: next_state = IDLE;
        endcase
    end

endmodule