#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Go three levels up to get to the workspace root
WS_DIR="$(realpath "$SCRIPT_DIR/../../../")"

# Source the ROS 2 workspace setup script
if [ -f "$WS_DIR/install/setup.bash" ]; then
    source "$WS_DIR/install/setup.bash"
else
    echo "Error: Could not find setup.bash in $WS_DIR/install/"
    exit 1
fi

# Run your Python script
PY_SCRIPT="$WS_DIR/src/remote_interface/remote_interface/alexa_interface.py"
if [ -f "$PY_SCRIPT" ]; then
    python3 "$PY_SCRIPT"
else
    echo "Error: Could not find Python script at $PY_SCRIPT"
    exit 1
fi
