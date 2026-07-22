#!/usr/bin/env bash

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source /usr/share/gazebo/setup.sh

export TURTLEBOT3_MODEL=waffle_pi
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=1

echo "Cleaning old Gazebo processes..."
killall -9 gzserver gzclient gazebo 2>/dev/null || true
pkill -f spawn_entity.py 2>/dev/null || true
pkill -f robot_state_publisher 2>/dev/null || true
pkill -f camera_node 2>/dev/null || true
fuser -k 11345/tcp 2>/dev/null || true

ros2 daemon stop 2>/dev/null || true
ros2 daemon start

sleep 3

TB3_GAZEBO_PREFIX=$(ros2 pkg prefix turtlebot3_gazebo)
WORLD=$HOME/ros2_ws/src/human_following_robot/worlds/big_square_human_world.world
MODEL=$TB3_GAZEBO_PREFIX/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf

export GAZEBO_MODEL_PATH=$TB3_GAZEBO_PREFIX/share/turtlebot3_gazebo/models:$HOME/.gazebo/models:/usr/share/gazebo-11/models:$GAZEBO_MODEL_PATH

echo "World: $WORLD"
echo "Robot model: $MODEL"
echo "GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH"

echo "Starting gzserver..."
gzserver --verbose "$WORLD" \
  -s libgazebo_ros_init.so \
  -s libgazebo_ros_factory.so &

GZSERVER_PID=$!

sleep 6

echo "Starting gzclient..."
gzclient &

sleep 4

echo "Waiting for /spawn_entity service..."
until ros2 service list | grep -q "/spawn_entity"; do
  sleep 1
done

echo "Spawning TurtleBot3 Waffle Pi..."
ros2 run gazebo_ros spawn_entity.py \
  -entity waffle_pi \
  -file "$MODEL" \
  -x 0.0 \
  -y -8.0 \
  -z 0.05 \
  -Y 1.5708

echo "Simulation ready."
echo "Keep this terminal open."

wait $GZSERVER_PID
