#!/bin/bash
set -e
PROJECT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$PROJECT_DIR"

if [ ! -x ".venv/bin/python3" ]; then
  echo "Creating the project Python environment..."
  python3 -m venv .venv
fi

echo "Installing/checking Python requirements..."
.venv/bin/python3 -m pip install --quiet --upgrade pip
.venv/bin/python3 -m pip install --quiet -r requirements.txt

echo "Running the automated tests..."
.venv/bin/python3 -m unittest discover -s tests -q

echo "Preparing TurtleBot3 mapping and opening Webots..."
exec .venv/bin/python3 scripts/prepare_and_run_webots.py mapping

