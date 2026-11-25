# One-shot control intent. Numbers are independent of CAN communication types.
uint8 SET_MECHANICAL_ZERO=1
uint8 CANCEL=2
uint8 RESET=3
# Move to the current joint-coordinate zero; never writes encoder zero.
uint8 HOME=4
uint8 ENABLE=5
uint8 DISABLE=6
std_msgs/Header header
builtin_interfaces/Time valid_until
string request_id
string joint_name
uint8 operation
