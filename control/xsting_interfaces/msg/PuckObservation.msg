string object_id                             # 인식 추적 id (RunTask PICK_STORE에 넘긴다)
int32 tag_id                                 # 퍽 AprilTag 번호 = 퍽 id, 못 읽으면 -1
string color                                 # "R" | "G" | "" (모름)
string shape                                 # "cylinder" | "box" | "" (모름)
float32 confidence                           # 0~1
geometry_msgs/PoseStamped pose
