# One-shot control intent. Numbers are independent of CAN communication types.
uint8 SET_MECHANICAL_ZERO=1
uint8 CANCEL=2
uint8 RESET=3
std_msgs/Header header
builtin_interfaces/Time valid_until
string request_id
string joint_name
uint8 operation
