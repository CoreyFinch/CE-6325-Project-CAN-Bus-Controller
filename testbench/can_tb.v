/*
  CAN bus controller bit-stream FSM testbench.

  This testbench controls the inputs and monitors the outputs of the CAN_BUS module and simulate a single CAN frame.
  Initially, the FSM is at the "IDLE" state. Then, the "start" input is primed to request on a negative clk edge.
  The bit-stream begins and walks through each state, outputting to "tx", until the bit-stream has completed.
  Output "done" is flagged for one clock cycle upon bit-stream completion. 
  The clock cycle is 10ms (5ms high + 5ms low) and was arbitrarilly selected.

  - Arbitrary Testbench Input Fields:
        id      = 0x007B
        dlc     = 0x2 for 2 bytes sent in the data field
        data    = 0x000055EE
        ack_in  = 1'b1  // Resesive (1) ==> Transmitter
        reset   = 1'b1 to reset the FSM
*/

`include "../src/Params.v"

module CAN_BUS_TB ();
    /* Inputs */
    reg clk;                     
    reg reset;                   
    reg start;                   // pulse to request frame transmission
    reg [ID_WIDTH-1:0] id;       // 11-bit Unique Identifier
    reg [DLC_WIDTH-1:0] dlc;     // Byte count of payload data
    reg [DATA_WIDTH-1:0] data;   // Payload data, MSB of byte 0 sent first
    reg ack_in;                  // bus level sampled during the ACK slot
    
    /* Outputs */
    wire tx;              // serial bus output (0 = dominant,1 = recessive)
    wire busy;            // high while a frame is in flight
    wire done;            // one-cycle pulse at end of IFS
    wire ack_err;         // set if no receiver acknowledged
    wire [NUM_STATES_WIDTH-1:0] state_dbg; // 

    // Can bus is device under test
    CAN_BUS dut(
        .clk(clk),
        .reset(reset),
        .start(start),
        .id(id),
        .dlc(dlc),
        .data(data),
        .ack_in(ack_in),
        .tx(tx),
        .busy(busy),
        .done(done),
        .ack_err(ack_err)
    );

    assign state_dbg = dut.current_state;

    initial begin
      clk = 1'b0;
      reset = 1'b1;	// Reset FSM for initialization
      id = 11'h7B;
      dlc = 4'd2;
      data = 64'h000055EE;
      ack_in = 1'b1;  // Resesive (1) ==> Transmitter
    end
    always #5 clk <= ~clk;
    
    // Testbench signals
    initial begin
        // Dump waveform to VCD file for GTKWave. iverilog_compile.sh passes
        // the path in as -DVCD_FILE so the dump lands where gtkwave looks;
        // the fallback keeps a bare `iverilog` invocation working.
      `ifdef VCD_FILE
      	$dumpfile(`VCD_FILE);
      `else
      	$dumpfile("dump.vcd");
      `endif
       $dumpvars(0, CAN_BUS_TB);
		
        #12 reset = 0;
        
      	@(posedge clk) start = 1;
      	@(posedge clk) start = 0;
        
        #750 $finish; // Finish simulation
    end
endmodule