#!/bin/bash

# Allow the node to communicate with the Robotiq gripper
gnome-terminal -- bash -c "python scripts/tool_communication.py"

sleep 1 # Wait 1 second

# Launch all of the node
gnome-terminal -- bash -c "python experiments/launch_nodes.py --robot=ur"

sleep 1 # Wait 1 second

# Run the enviroment loop
gnome-terminal -- bash -c "python experiments/run_env.py --agent=gello"