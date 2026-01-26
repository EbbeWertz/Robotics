echo "global ros environment laden..."
source /opt/ros/jazzy/setup.bash
echo "(re)builden..."
colcon build
echo "local environment laden..."
source install/setup.bash