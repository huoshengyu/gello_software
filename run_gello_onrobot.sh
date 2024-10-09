#!/bin/bash

# Please place GELLO in the start position before running this script

# Launch all of the node
gnome-terminal -- bash -c "python experiments/launch_nodes.py --robot=ur_onrobot"

sleep 1 # Wait 1 second

# Run the enviroment loop
gnome-terminal -- bash -c "python experiments/run_env.py --agent=gello"