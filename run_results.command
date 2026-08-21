#!/bin/bash
set -e
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

if [ ! -x ".venv/bin/python3" ]; then
  echo "ERROR: The project environment is missing. Run run_localization.command first."
  read -r -p "Press Enter to close..."
  exit 1
fi

echo "Calculating metrics and generating the report figure..."
.venv/bin/python3 scripts/analyze_results.py
.venv/bin/python3 scripts/plot_results.py
open results/report_results.png
