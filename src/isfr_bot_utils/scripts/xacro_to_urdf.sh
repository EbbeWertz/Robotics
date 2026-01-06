#!/bin/bash
# Converts all XACROs to URDF for Webots

echo "URDF files generaten..."

DESC_DIR=../../isfr_bot_description
mkdir -p $DESC_DIR/urdf

# TurtleBot3 Waffle
xacro $(ros2 pkg prefix turtlebot3_description)/share/turtlebot3_description/urdf/turtlebot3_waffle.urdf \
  -o $DESC_DIR/urdf/turtlebot3_waffle.urdf

# OpenManipulator X
xacro $(ros2 pkg prefix open_manipulator_description)/share/open_manipulator_description/urdf/open_manipulator_x/open_manipulator_x.urdf.xacro \
  -o $DESC_DIR/urdf/openmanipulator_x.urdf

echo "normaal zitten er nu 2 urdf files in: $DESC_DIR/urdf"

echo "URDF files validaten..."
check_urdf $DESC_DIR/urdf/turtlebot3_waffle.urdf | grep "Successfully Parsed XML"
check_urdf $DESC_DIR/urdf/openmanipulator_x.urdf | grep "Successfully Parsed XML"
echo "klaar"