# CAN Bus Controller — Bit-Stream FSM

Verilog implementation of a CAN 2.0B transmitter. A 9-state FSM serializes a
frame onto `tx` one bit per clock: IDLE → SOF → ARB → CTRL → DATA → CRC → ACK →
EOF → IFS → IDLE.

Not implemented: bit stuffing, CRC generation (a fixed constant is sent), error
handling, and arbitration loss/retransmission.

## Members
- Corey Finch (cxf210015)
- Adil Bhojwani ()
- Paresh Grover (pxg210048)

## Layout
| Path | |
|---|---|
| `src/can_bus_fsm.v` | `CAN_BUS` — the FSM |
| `src/Params.v` | field widths, `` `include ``d by both modules |
| `testbench/can_tb.v` | `CAN_BUS_TB` — drives one frame |
| `iverilog_compile.sh` | compile + simulate + open waveform |
| `.env` | output and VCD paths |

## Build and run

Requires `iverilog` and `gtkwave` on `PATH`.

```bash
./iverilog_compile.sh
```

Compiles every `.v` under `src/` and `testbench/`, runs the simulation, then
opens the waveform in gtkwave (reusing `wave.gtkw` if present).

To compile and run without the viewer:

```bash
iverilog -g2012 -I src -o output/can_compiled.v src/*.v testbench/*.v
vvp output/can_compiled.v
```

Both flags are required. `-g2012` allows the compilation-unit scope parameters
in `Params.v`; `-I src` resolves the `` `include ``. Run from the repo root.

## Test stimulus

| | |
|---|---|
| `id` | `0x7B` |
| `dlc` | `2` (2 payload bytes) |
| `data` | `0x55EE` |
| `ack_in` | `1` (recessive — no receiver acking) |
| clock | 10 ns period |

## Results

Compiles clean and transmits one complete frame:

- The FSM walks all nine states in order and returns to IDLE.
- `busy` is held high for the duration of the frame.
- `done` pulses high for one cycle at the end of IFS.
- `ack_err` asserts during the ACK slot. This is correct for the stimulus —
  the testbench holds `ack_in` recessive, meaning no node acknowledged.

Simulation ends at `$finish` at t=775. Waveform is written to the path set in
`.env` (`output/dump_wav.vcd`).
