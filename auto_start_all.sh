#!/bin/bash

# Script to automate the setup of Pilot and OSSDC Sim Terminals
if command -v gnome-terminal >/dev/null; then
    TERMINAL_CMD="gnome-terminal --tab --"
elif command -v xfce4-terminal >/dev/null; then
    TERMINAL_CMD="xfce4-terminal --tab -x"
elif command -v konsole >/dev/null; then
    TERMINAL_CMD="konsole -e"
else
    echo "No supported terminal emulator found (gnome-terminal, xfce4-terminal, konsole)"
    exit 1
fi

# Define directories
HITCH_DIR="$(pwd)"
OSSDC_DIR="../OSSDC-SIM-ART-Linux"
WISE_DIR="${OSSDC_DIR}/wise"

# Check if directories exist
if [ ! -d "$OSSDC_DIR" ] || [ ! -d "$WISE_DIR" ]; then
    echo "Error: Required OSSDC directories not found, please change the setting in the shell script to your native directory"
    exit 1
fi

# Function to source ROS and workspace in Pilot Terminal
source_pilot() {
    echo "Sourcing Pilot Terminal..."
    if [ -f "./source_all.sh" ]; then
        source ./source_all.sh
    else
        # Fallback to manual ROS sourcing
        if [ -f "/opt/ros/humble/setup.bash" ]; then
            source /opt/ros/humble/setup.bash
        fi
        # Source workspace if exists
        if [ -f "${HITCH_DIR}/install/setup.bash" ]; then
            source "${HITCH_DIR}/install/setup.bash"
        fi
    fi
}

# 1. Start wise server in OSSDC Sim Terminal
echo "Starting wise server..."
$TERMINAL_CMD bash -c "cd ${WISE_DIR} && python3 -m http.server 9090"


# Wait briefly to ensure wise server starts
sleep 3

# 2. Start simulator in OSSDC Sim Terminal
echo "Starting simulator..."
$TERMINAL_CMD bash -c "cd ${OSSDC_DIR} && ./start_sim_local.sh"


# Wait briefly to ensure simulator starts
sleep 5

# 3. Start lgsvl_bridge in Pilot Terminal
echo "Starting lgsvl_bridge..."
$TERMINAL_CMD bash -c "cd ${HITCH_DIR} && { if [ -f \"./source_all.sh\" ]; then source ./source_all.sh; else if [ -f \"/opt/ros/humble/setup.bash\" ]; then source /opt/ros/humble/setup.bash; fi; if [ -f \"${HITCH_DIR}/install/setup.bash\" ]; then source \"${HITCH_DIR}/install/setup.bash\"; fi; fi; } && lgsvl_bridge --port 9091"


# # Wait briefly to ensure bridge starts
# sleep 3

# # 4. Start Python API in Pilot Terminal
# echo "Starting Python API..."
# $TERMINAL_CMD bash -c "cd ${HITCH_DIR} && { if [ -f \"./source_all.sh\" ]; then source ./source_all.sh; else if [ -f \"/opt/ros/humble/setup.bash\" ]; then source /opt/ros/humble/setup.bash; fi; if [ -f \"${HITCH_DIR}/install/setup.bash\" ]; then source \"${HITCH_DIR}/install/setup.bash\"; fi; fi; }  && python3 src/launch/svl_launch/scripts/launch_ossdc.py --env svl.env"


echo "All processes started in separate terminals."