#!/usr/bin/env bash
# Bash version of script.cmd, for the Linux/NoMachine environment.
set -euo pipefail

# Load variables from .env (set -a exports everything defined in between)
set -a
. "$(dirname "$0")/.env"
set +a

mkdir -p "$(dirname "$OUTPUT")" "$(dirname "$VCD_FILE")"

# Collect every .v file under SRC_DIR and TB_DIR into an array.
SOURCES=("$SRC_DIR"/*.v "$TB_DIR"/*.v)
if (( ${#SOURCES[@]} == 0 )); then
    echo "error: no .v files found in $SRC_DIR/ or $TB_DIR/" >&2
    exit 1
fi

echo "Compiling ${#SOURCES[@]} file(s): ${SOURCES[*]}"

# Compile Verilog code. The inner \" \" are part of the macro value, so
# VCD_FILE expands to a Verilog string literal: "output/comp.vcd"
iverilog -o "$OUTPUT" -DVCD_FILE="\"$VCD_FILE\"" "${SOURCES[@]}"

# Run Verilog simulation
vvp "$OUTPUT"

# Open waveform viewer, reusing the saved signal layout if there is one
if [[ -f wave.gtkw ]]; then
    gtkwave "$VCD_FILE" -a wave.gtkw
else
    gtkwave "$VCD_FILE"
fi
