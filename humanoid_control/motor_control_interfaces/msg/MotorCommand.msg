# Source observation time for policy commands; forwarding must preserve it.
std_msgs/Header header
# Absolute ROS expiry. Zero keeps the legacy source-stamp watchdog contract.
# Every hop may shorten this deadline, never renew it. DISABLE is always allowed.
builtin_interfaces/Time valid_until
string[] joint_name
uint8[] mode
float64[] position
float64[] velocity
float64[] acceleration
float64[] torque
float64[] kp
float64[] kd

uint8 MODE_VELOCITY=0
uint8 MODE_POSITION=1
uint8 MODE_MOTION=2
uint8 MODE_ENABLE=3
uint8 MODE_DISABLE=4
