#!/bin/bash
python3 /opt/ros_ws/src/franka_teleop/scripts/oculus_tranforms_vrpolicy.py | tee /dev/tty &

# Run the second command in the background
python3 /opt/ros_ws/src/franka_teleop/scripts/teleop_quest.py &

# Wait for all background processes to finish
wait
