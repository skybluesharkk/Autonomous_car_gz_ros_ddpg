#!/bin/bash
export IGN_PARTITION=david_sim
export IGN_IP=127.0.0.1
sleep 5
echo "Attempting to unpause Gazebo simulation..."
/home/david/miniconda3/envs/ros_humble/bin/ign topic -t /world/my_car_world/control -m ignition.msgs.WorldControl -p 'pause: false' 2>&1
EXIT_CODE=$?
if [ $EXIT_CODE -eq 0 ]; then
    echo "Successfully sent unpause command"
else
    echo "Failed to send unpause command, exit code: $EXIT_CODE"
fi
echo "Done"
