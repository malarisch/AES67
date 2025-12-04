#!/bin/bash

# Check if .venv exists, if not create a Python virtual environment
if [ ! -d ".venv" ]; then
    echo "Creating Python virtual environment in .venv..."
    python3 -m venv .venv
    echo "Virtual environment created successfully."
else
    echo "Virtual environment already exists in .venv"
fi

source .venv/bin/activate
pip3 install --upgrade west
.venv/bin/west init -l --mf app/west-manifest/west.yml
.venv/bin/west update
.venv/bin/west zephyr-export
.venv/bin/west packages pip --install
.venv/bin/west sdk install