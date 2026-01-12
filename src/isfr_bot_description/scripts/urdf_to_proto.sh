#!/bin/bash
# Converts URDFs to Webots PROTO files

echo "PROTO files generaten..."
URDF_DIR=../urdf
DEST_DIR=../protos
mkdir -p $DEST_DIR

# TurtleBot3 Waffle
ros2 run webots_ros2_importer urdf2proto \
  --input=$URDF_DIR/openmanipulator_x_RAW.urdf \
  --optimize-mesh \
  --output=$DEST_DIR/turtlebot3_waffle_RAW.proto

# OpenManipulator X
ros2 run webots_ros2_importer urdf2proto \
  --input=$URDF_DIR/openmanipulator_x_RAW.urdf \
  --optimize-mesh \
  --output=$DEST_DIR/openmanipulator_x_RAW.proto
 
echo "klaar"