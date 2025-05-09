#!/bin/bash
for difficulty in "SS0" "SS2" "SS3" "SS5" "SS7"; do
    uv run python run_simulation.py --difficulty "$difficulty"
done