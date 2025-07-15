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

# Start the Trossen robot so GELLO can connect to it
# roslaunch morpheus_teleop trossen.launch robot_mode:=real control_mode:=none &

sleep 1 & # Wait 1 seconds

# Launch all of the node
python3 ./scripts/launch_nodes.py --robot=trossen &

sleep 1 # Wait 1 second

# Run the enviroment loop
python3 ./scripts/run_env.py --agent=gello --robot_type=trossen