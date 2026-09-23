// Include guard: this file is both `include`d by can_bus_fsm.v / can_tb.v and
// picked up by the src/*.v glob in iverilog_compile.sh. Without the guard the
// parameters get declared twice in the same compilation unit.
`ifndef PARAMS_V
`define PARAMS_V

    parameter ID_WIDTH = 11;
    parameter DLC_WIDTH = 4;
    parameter DATA_WIDTH = 8*8; // 8 bytes * 8 bits/byte
    parameter CRC_WIDTH = 15;
    parameter EOF_WIDTH = 7;
    parameter NUM_STATES_WIDTH = 4;
    parameter BIT_CNT_WIDTH = 7;
    parameter IFS_WIDTH = 3;

`endif
