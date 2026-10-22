# Times are ROS clock timestamps; joint ordering is fixed by the model manifest.
std_msgs/Header header
string task_id
uint64 generation
uint64 input_id
sensor_msgs/JointState joints
geometry_msgs/PoseStamped object_pose
geometry_msgs/PoseStamped target_pose
string phase
