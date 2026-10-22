# Candidate only: never forward directly to motors.
std_msgs/Header header
string task_id
uint64 generation
uint64 input_id
string model_version
string[] joint_names
float64[] delta_positions
builtin_interfaces/Time expires_at
