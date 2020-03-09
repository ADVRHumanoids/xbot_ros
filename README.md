# AuxRosPublisher
This RT plugin for XBotCore allows to manage the publication to ROS of the PDO aux field.
By default, all defined aux fields are published in a multiplexed fashion. The user can customize
this behavior inside the XBotCore configuration file.

## Notes on compilation
 - Make sure to check out the branch `aux_publisher` of the `xbot_msgs` repo.
 - On the RT system, make sure to have the Xenomai flags enabled (`ENABLE_XENO=ON`).

## How to run
Add the plugin to the `XBotRTPlugins` field of the XBotCore config file. Optionally,
published fields can be customized as in the example. Currently, a single list of
aux fields can be configured for all of the joints.

```yaml

XBotRTPlugins:
  plugins: ["HomingExample", "AuxRosPublisher"]
  io_plugins: []

AuxRosPublisher:
  aux_field_names: [iq_out, iq_ref, board_temperature, motor_temperature]
```

Remember to enable the plugin with the usual `rosservice call /xbotcore/AuxRosPublisher_switch true`.
You'll see all aux field published to ROS topics with name `/xbotcore/aux/aux_field_name`.

**Note:** the robot state logger will automatically log all aux values to the usual MAT file (`rosrun xbot_ros robot_state_logger`).

## Available names
Available aux field names are directly taken from the board firmware, except that spaces
have been removed, and all CamelCase names have been turned into snake_case. Details are
given inside the `AuxRosPublisher.h` header file.


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
