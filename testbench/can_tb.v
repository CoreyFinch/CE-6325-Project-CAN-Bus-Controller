`ifndef VCD_FILE
    `define VCD_FILE "output/can_waveform.vcd"
`endif
    
module can_bus_controller_tb;
    // Testbench signals

    initial begin
        // Dump waveform to VCD file for GTKWave
        $dumpfile(`VCD_FILE);
        $dumpvars(0, can_bus_controller_tb);

        // Monitor signals for debugging
        // $monitor();

        #10 $finish; // Finish simulation
    end
endmodule