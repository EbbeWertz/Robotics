# Bringup 
ros2 launch isfr_bot_bringup security.launch.py

# ACTIVATE PATROLLING BEHAVOUR
ros2 service call /toggle_patrol std_srvs/srv/SetBool "{data: true}"