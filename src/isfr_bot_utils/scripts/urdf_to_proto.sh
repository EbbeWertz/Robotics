#!/bin/bash
# Converts URDFs to Webots PROTO files

echo "PROTO files generaten..."
DESC_DIR="../../isfr_bot_description"
PROTO_DIR="$DESC_DIR/protos"
mkdir -p $PROTO_DIR

# TurtleBot3 Waffle
ros2 run webots_ros2_importer urdf2proto \
  --input=$DESC_DIR/urdf/turtlebot3_waffle.urdf \
  --disable-mesh-optimization \
  --output=$PROTO_DIR/turtlebot3_waffle.proto

# OpenManipulator X
ros2 run webots_ros2_importer urdf2proto \
  --input=$DESC_DIR/urdf/openmanipulator_x.urdf \
  --disable-mesh-optimization \
  --output=$PROTO_DIR/openmanipulator_x.proto
 
echo "klaar"