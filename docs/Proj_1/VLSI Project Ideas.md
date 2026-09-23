Below are designs that exist in shipping silicon — each maps to a real standard or a real product subsystem, which also gives you a citable spec to verify against and a credible framing for the final report. Same evaluation frame as before: FSM quality, cell count, scale knob, and how it survives Projects 3–6.

## Automotive & motor control

**CAN 2.0B bus controller (ISO 11898)** — in every car built since ~1995.
FSM is the real thing: `IDLE → SOF → ARBITRATION → CONTROL → DATA → CRC → ACK → EOF → INTERMISSION`, plus a parallel error-management FSM (`ERROR_ACTIVE → ERROR_PASSIVE → BUS_OFF`) driven by TX/RX error counters. Bit-stuffing and arbitration-loss detection are genuinely interesting logic. ~2,500–4,000 cells with TX/RX FIFOs and acceptance filters. Scale knob: FIFO depth, number of acceptance filters. The dual-FSM structure makes an unusually strong state diagram for the report.

**BLDC motor commutation + dead-time PWM generator** — every drone ESC, every EV traction inverter.
Hall-sensor decode drives a 6-step commutation FSM; a dead-time insertion FSM prevents shoot-through on each half-bridge (a real safety mechanism, not a toy). Add a current-loop PI and a ramp generator. ~3,000–5,000 cells. Scale knob: PWM resolution (counter width), number of phases, adding Clarke/Park transforms for true FOC. Datapath is counters and comparators — a tiny cell library.

## Communications & interfaces

**8b/10b encoder/decoder (IEEE 802.3 / PCIe Gen1-2 / SATA / DisplayPort)** — the most widely deployed line code in existence.
Running-disparity tracking is inherently stateful; the receiver needs a comma-detect and lane-alignment FSM. ~800 cells per lane, so a 4-lane block with alignment and elastic buffers lands ~3,500–5,000. Scale knob: lane count — and lanes are *identical tiles*, which is close to ideal for Project 6.

**I²C multi-master controller with clock stretching and arbitration loss** — in essentially every SoC.
Deceptively hard and therefore respectable: `IDLE → START → ADDR → ACK → DATA → STOP`, with arbitration-loss detection and stretch handling layered on. Small on its own (~800 cells) — needs a FIFO and multi-channel replication to reach target. Good if you want a design you fully understand.

**USB 1.1 full-speed device SIE** — NRZI decode, bit unstuffing, CRC5/CRC16, PID decode, endpoint state machines, and a device-state FSM (`POWERED → DEFAULT → ADDRESS → CONFIGURED`).
~3,000–5,000 cells naturally, no padding needed. Highest realism of anything on this list, but also the highest verification burden — you're implementing a protocol with real timing rules.

## Storage & memory

**SDRAM/DDR-style memory controller** — the single most performance-critical block in most SoCs.
Per-bank FSM (`IDLE → ACTIVATE → ACTIVE → READ/WRITE → PRECHARGE`) replicated across 8 banks, plus a refresh counter, timing-constraint checkers (tRCD, tRP, tRAS), and a command scheduler/arbiter. ~3,000–5,000 cells. Scale knob: bank count and burst length. Eight identical bank FSMs give you real structural regularity, and the state diagram practically draws itself.

**BCH or Hamming SEC-DED ECC codec for NAND flash** — in every SSD and every server DIMM.
Syndrome computation, Chien search, and error correction, sequenced by an FSM. GF(2^m) arithmetic is pure XOR trees — an extremely small cell library, which is a real advantage in Project 4. ~2,500–4,000 cells. Scale knob: correction strength `t` and codeword length.

## Signal processing

**CIC decimation filter + FIR compensator (sigma-delta ADC front end)** — inside every audio codec, every ΔΣ ADC, every SDR downconverter.
N integrator stages at the fast clock, decimator, N comb stages at the slow clock; FSM handles rate switching and pipeline flush. A 5th-order 32-bit CIC is ~3,000 cells before the compensating FIR. Scale knobs: order, accumulator width, decimation ratio — all linear and independent. **Structurally the best fit on this list for Projects 5–6**: it's almost entirely adder+register slices, so you draw one bit-slice layout and tile it.

**DDS / numerically-controlled oscillator with CORDIC phase-to-amplitude** — every function generator, every SDR transmitter, every radar chirp source.
32-bit phase accumulator, phase dithering, quadrature output. ~3,000–4,500 cells. Scale knob: accumulator width and CORDIC stages. Waveform results are self-evidently correct in simulation, which makes the report easy.

**Goertzel DTMF decoder (ITU-T Q.23)** — the actual algorithm in telephone tone detection.
One shared multiplier, FSM sequences 8 tone bins per sample, then a validation FSM enforces tone-duration and twist requirements. ~2,000–3,000 cells. Scale knob: number of bins, sample width. Nice because the FSM does something non-trivial rather than just sequencing a datapath.

## Security

**SHA-256 core (FIPS 180-4)** — TLS, Bitcoin, secure boot, everywhere.
FSM: `IDLE → LOAD → SCHEDULE → ROUND(×64) → FINALIZE`. The 16×32-bit message schedule and 8×32-bit working registers give you ~768 flip-flops honestly, plus 6 32-bit adders in the round function. ~4,000–6,000 cells with the K-constant table. Scale knob: unroll 2 or 4 rounds per cycle. Rotations are free wiring, so the cell library stays at DFF/XOR/MAJ/adder.

**AES-GCM authentication tag path (GF(2^128) multiplier)** — if you want something less common than plain AES. Carry-less multiplication is a large, perfectly regular XOR array.

## Display & imaging

**8×8 DCT + quantizer + zigzag (JPEG / MPEG baseline)** — the compute core of every image and video encoder.
Row-column decomposition with a transpose buffer (64×16 bits = 1,024 flip-flops), Loeffler shift-add butterflies, FSM sequencing `LOAD → ROW_DCT → TRANSPOSE → COL_DCT → QUANT → ZIGZAG`. ~4,000–6,000 cells. Scale knob: coefficient precision, parallel 1-D DCT units. Highly regular butterfly structure.

**HDMI/DVI TMDS encoder + video timing generator (CEA-861)** — real display standard.
Transition-minimization and DC-balancing per channel, three channels plus H/V sync generation and a test-pattern source. ~1,500 cells for the encoders alone, so pair it with a line buffer or pattern generator to reach target.

## Test & debug

**IEEE 1149.1 JTAG TAP controller + boundary scan chain** — mandated on nearly every packaged chip, including the ones you're taping out in this class.
The TAP controller is a *standardized, published 16-state FSM* — you can reproduce the official state diagram in your report and cite the spec. Alone it's ~150 cells, but a boundary-scan chain of 200 cells at ~15 cells each puts you at ~3,000, and boundary-scan cells are the most regular structure imaginable for placement. Thematically appropriate: it's the design a VLSI course would actually put on a chip.

---

## Shortlist

If "real world" is the priority and you want the sequence to stay tractable:

1. **CIC + FIR decimator** — best structural match to Projects 4–6, cleanest scaling, real ADC/SDR pedigree.
2. **CAN controller** — best state diagram, unambiguously real-world, moderate size.
3. **SHA-256** — hits cell count honestly with no padding, small library, universally recognized.
4. **DDR bank controller** — strongest "this is what real chips do" story, eight-way regularity.

One caution that applies across all of these: there's a difference between reaching 3,000 cells *structurally* and reaching it by instantiating a giant flip-flop array. An SD-card controller with a 512-byte buffer in flops "hits" 4,096 cells, but that block is a routing nightmare in Project 6 and teaches you nothing. Prefer designs where the cell count comes from replicated datapath slices — CIC stages, bank FSMs, SHA rounds, DCT butterflies, scan cells — because those are the ones you can tile.
