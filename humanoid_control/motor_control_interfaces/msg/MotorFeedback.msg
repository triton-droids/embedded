# One type-2 operation-status sample, before polarity/offset/linkage conversion.
# These are raw motor coordinates, distinct from /joint_states coordinates.
string joint_name
string can_interface
uint8 motor_id
string model
bool available
bool fresh
# Host-derived source ROS time; not a synchronized motor hardware timestamp.
builtin_interfaces/Time stamp
float64 age_s
float64 position_rad
float64 velocity_radps
float64 torque_nm
float64 temperature_c
# Status mode: 0 Reset, 1 Cali, 2 Motor/run; NOT the run_mode parameter.
uint8 status_mode
uint8 fault_bits
