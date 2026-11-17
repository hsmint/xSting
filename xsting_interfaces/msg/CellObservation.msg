uint8 cell                                   # 0~8 (위쪽 줄 0 1 2, 지도 +Y가 위)
string state                                 # "empty" | "occupied" | "unknown"
int32 tag_id                                 # 그 칸 퍽의 AprilTag 번호, 모르면 -1
geometry_msgs/Point offset                   # cell 중심에서 퍽 중심까지 (lN_grid, m)
