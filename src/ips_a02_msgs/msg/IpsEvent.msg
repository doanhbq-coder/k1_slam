# Sự kiện KHÔNG phải Pos từ tag UWB — đây là chỗ nút bấm trên tag xuất hiện.

std_msgs/Header header

string KIND_CALL="Call"
string KIND_CANCEL="CancelCall"
string KIND_ACK="Ack"

string kind             # Call | CancelCall | Ack
string uid              # định danh thật của tag — dùng cái này làm key
string device_name      # nhãn hiển thị, đổi được trong IPS Tool

string req_id           # chỉ có ở Ack: CallAck | CancelCallAck

# Vị trí tag ngay lúc phát sự kiện, đóng gói tại nguồn để node điều hướng khỏi phải tự đi đọc
# topic pose (dễ vớ phải mẫu cũ hoặc mẫu vừa nhảy).
geometry_msgs/PoseStamped tag_pose      # frame_id = tham số frame_id của bridge (ips_map)
bool  pose_valid                        # false nếu chưa từng nhận Pos của tag này
float32 pose_age                        # giây, tuổi của mẫu Pos dùng ở trên
float32 pose_noise                      # noiseRadius của mẫu đó (m)
