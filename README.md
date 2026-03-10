ros2 service call /set_control_mode agv_interfaces/srv/ControlMode "{mode: 1}"

ros2 service call /set_control_mode agv_interfaces/srv/ControlMode "{mode: 2}"

ros2 launch agv_bringup bringup.launch.py
