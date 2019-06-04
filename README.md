# Utility robot_upstart

## Usage example

For the automatic roscore starting:

```
rosrun robot_upstart install --job roscore --setup $XBOT_ROOT/robotology-setup.bash xbot_ros/launch/upstart.launch --master $ROS_MASTER_URI

sudo systemctl daemon-reload && sudo systemctl start roscore
```



```
rosrun robot_upstart install --job robot_state_logger --setup $ROBOTOLOGY_ROOT/robotology-setup.bash xbot_ros/launch/robot_state_logger.launch --master $ROS_MASTER_URI
```
