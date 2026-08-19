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

# Start the onrobot gripper action server
roslaunch onrobot_rg2ft_action_server onrobot_rg2ft_action_server.launch &

sleep 1 # Wait 1 seconds

# Launch all of the node
python3 ./experiments/launch_nodes.py --robot=ur_onrobot &

sleep 1 # Wait 1 second

# Run the enviroment loop
python3 ./experiments/run_env.py --agent=gello --robot_type=ur