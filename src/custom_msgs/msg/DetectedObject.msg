# 탐지된 하나의 객체에 대한 정보

# ROS Header (탐지 시점의 타임스탬프 및 프레임 ID)
std_msgs/Header header

# 클래스 이름 (예: car, red, green, dummy 등)
string class_name

# 이미지 상의 중심 좌표 (픽셀 단위)
int32 x_center
int32 y_center

# 깊이 카메라 기반 거리 정보 (단위: meter)
float32 distance

# 카메라 좌표계 기준 3D 좌표
geometry_msgs/Point point

# 변환된 map 좌표계 기준 3D 좌표 (TF 변환 성공 시)
float32 map_x
float32 map_y
float32 map_z

# 변환된 좌표계의 frame_id 및 timestamp
string frame_id
builtin_interfaces/Time stamp
