#!/bin/bash
set -e

# source
source /home/htxgrrt/.htxgrrt/app/goaway_ws/custom.env

# Launch
echo "Launching system..."
exec ros2 launch goaway_master operation.launch.py