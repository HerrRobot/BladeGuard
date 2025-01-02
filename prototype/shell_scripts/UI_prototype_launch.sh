#!/bin/bash

source /opt/ros/humble/setup.bash
source install/setup.bash

echo "Starting ROS nodes..."

ros2 launch motors UI_prototype_launch.py &

echo "ROS nodes started!"

echo "Starting flask server..."

cd /home/urosgluscevic/BladeGuard/prototype/web/flask_server
source flask_venv/bin/activate
python server.py

echo "started flask server!"