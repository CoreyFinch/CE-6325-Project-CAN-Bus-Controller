
module can_bus_controller_tb ();
    /* Inputs */
    reg clk;                     
    reg reset;                   
    reg start;                   // pulse to request frame transmission
    reg [ID_WIDTH-1:0] id;       // 11-bit Unique Identifier
    reg [DATA_WIDTH-1:0] data;   // Payload data, MSB of byte 0 sent first
    reg ack_in;                  // bus level sampled during the ACK slot
    
    /* Outputs */
    wire tx;              // serial bus output (0 = dominant,1 = recessive)
    wire busy;            // high while a frame is in flight
    wire done;            // one-cycle pulse at end of IFS
    wire ack_err;         // set if no receiver acknowledged
    wire [NUM_STATES_WIDTH:0] state_dbg; // exposed for waveform inspection

    // Can bus is device under test
    CAN_BUS dut(
        .clk(clk),
        .reset(reset),
        .start(start),
        .id(id),
        .data(data),
        .ack_in(ack_in),
        .tx(tx),
        .busy(busy),
        .done(done),
        .ack_err(ack_err),
        .state_dbg(state_dbg)
    );

    initial begin
      clk = 1'b0;
      reset = 1'b1;	// Reset FSM for initialization
      id = 11'h7B;
      data = 64'h000055EE;
      ack_in = 1'b1;  // Resesive (1) ==> Transmitter
    end
    always #5 clk <= ~clk;
    
    // Testbench signals
    initial begin
        // Dump waveform to VCD file for GTKWave
      	$dumpfile("dump.vcd");
        $dumpvars(0, can_bus_controller_tb);
		
        #12 reset = 0;
      	//while (done == 1'b0) // Wait until bit-stream is done
      	@(posedge clk) start = 1;
      	@(posedge clk) start = 0;
	
        
    	#1000 $finish; // Finish simulation
    end
endmodule