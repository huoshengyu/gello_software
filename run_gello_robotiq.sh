#!/bin/bash -e

# Please place GELLO in the start position before running this script

# Swap to directory of script
cd "$(dirname "$0" )"

# Exit command to be called when script exits
function kill {
    # Kill GELLO node
    echo "Killing GELLO node"
    pkill -9 -f launch_nodes.py
}
trap kill EXIT

# Allow the node to communicate with the Robotiq gripper
python3 ./scripts/tool_communication.py &

sleep 1 # Wait 1 second

# Launch all of the node
python3 ./experiments/launch_nodes.py --robot=ur &

sleep 1 # Wait 1 second

# Run the enviroment loop
python3 ./experiments/run_env.py --agent=gello --robot_type=ur