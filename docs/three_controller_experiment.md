# Hướng dẫn thực nghiệm Backstepping — BSMC — SMC

Tài liệu này dùng workspace đầy đủ:

```text
/home/hoang/Paper1/Paper1-main
```

Không build package trong `/home/hoang/Paper1/src`: đó là bản code cũ, không
có Square paper runner, embedded logger, SMC và pipeline so sánh ba controller.

## 1. Output được tạo

Mỗi controller run tự ghi:

- CSV gốc;
- trajectory PDF/PNG;
- `e_x`, `e_y`, `e_theta` PDF/PNG;
- `v_cmd`, `w_cmd` PDF/PNG;
- summary JSON chứa gain và metric.

Khi cùng một thư mục đã có đủ Backstepping, BSMC và SMC của cùng trajectory,
pipeline tự tạo trong `publication_three/`:

```text
circle_three_controller_trajectory.pdf/png
circle_three_controller_errors.pdf/png
circle_three_controller_ex.pdf/png
circle_three_controller_ey.pdf/png
circle_three_controller_etheta.pdf/png
circle_three_controller_commands.pdf/png
circle_three_controller_vcmd.pdf/png
circle_three_controller_wcmd.pdf/png
```

Square và Eight có tên tương tự, bắt đầu bằng `square_` hoặc `eight_`.

Với thư mục Circle có disturbance và đủ ba controller, pipeline tạo thêm:

```text
circle_three_controller_disturbance_recovery.pdf/png
circle_three_controller_disturbance_trajectory.pdf/png
circle_three_controller_disturbance_alignment.json
```

PDF vector là file nên đưa vào LaTeX. PNG 600 dpi dùng để xem nhanh. Quỹ đạo
thực và sai số trong comparison lấy từ AprilTag còn mới không quá `0.30 s`.
Pipeline chỉ căn hệ tọa độ cứng và không smoothing dữ liệu.

Hai file kiểm tra nguồn dữ liệu và metric được tạo cùng figure:

```text
three_controller_provenance.json
three_controller_metrics.csv
```

## 2. Định nghĩa ba controller trong code

- **Backstepping:** đặt `Ks1=Ks2=0`; giữ feedback Backstepping.
- **BSMC:** feedback Backstepping cộng bounded sliding injection.
- **SMC:** giữ feedforward/reference, boundary-layer sliding injection, giới
  hạn actuator và logic bảo vệ phần cứng; đặt các feedback tỷ lệ Backstepping
  về zero.

SMC mới là baseline thực nghiệm, chưa phải gain đã được xác thực trên robot.
Phải chạy pilot với nút dừng khẩn cấp trước khi thu dữ liệu paper. Không tăng
`Ks1`, `Ks2` khi chưa xem `v_cmd`, `w_cmd` và độ rung bánh xe.

## 3. Build

Mở terminal:

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_control --symlink-install
source install/setup.bash
```

Kiểm tra sáu executable mới/cũ:

```bash
ros2 pkg executables amr_control | grep -E \
  'backstepping_(circle|square|eight)|bsmc_(circle|square|eight)|smc_(circle|square|eight)|three_controller_report'
```

Mỗi terminal mới phải chạy:

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
source install/setup.bash
```

## 4. Node nền

Mở bốn terminal riêng.

Terminal 1 — giao tiếp robot:

```bash
ros2 run amr_control robot_serial_bridge
```

Terminal 2 — trạng thái:

```bash
ros2 run amr_control state_bridge
```

Terminal 3 — EKF:

```bash
ros2 run amr_control custom_ekf_node
```

Terminal 4 — camera:

Circle hoặc Square:

```bash
ros2 run amr_control camera_circle_square
```

Figure-eight:

```bash
ros2 run amr_control camera_eight
```

Không chạy hai camera profile cùng lúc. Trước mỗi run:

```bash
ros2 topic hz /odom_camera
ros2 topic hz /odometry/filtered
ros2 topic hz /espnow_link
ros2 topic echo /odom_camera --once
```

Chỉ tiếp tục khi pose AprilTag hợp lý và hai topic odometry cập nhật liên tục.

## 5. Quy tắc thu dữ liệu

1. Chỉ chạy một controller vì cả ba đều publish `/cmd_vel`.
2. Giữ cùng pose đầu, pin, ánh sáng, camera và vật cản.
3. Mỗi bộ ba so sánh phải dùng cùng số vòng và cùng output directory.
4. Đặt robot đứng yên trước khi start controller.
5. Không thay gain giữa ba run của một set.
6. Nên đổi thứ tự controller giữa các repeat, ví dụ:

```text
Set 1: Backstepping -> BSMC -> SMC
Set 2: SMC -> Backstepping -> BSMC
Set 3: BSMC -> SMC -> Backstepping
```

Một run 3 vòng vẫn là `n=1`, không phải `n=3`. Paper tối thiểu nên có ba
khởi động độc lập/controller/trajectory.

## 6. Circle nominal

Tạo thư mục cho một set:

```bash
mkdir -p /home/hoang/Paper1/Paper1-main/paper_runs/three_controller/circle/set01
```

Backstepping:

```bash
ros2 run amr_control backstepping_circle --ros-args \
  -p angular_speed:=0.108 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/circle/set01 \
  -p paper_run_id:=circle_bs_set01
```

BSMC:

```bash
ros2 run amr_control bsmc_circle --ros-args \
  -p angular_speed:=0.108 \
  -p ks1:=0.024 \
  -p ks2:=0.050 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/circle/set01 \
  -p paper_run_id:=circle_bsmc_set01
```

SMC, dùng cùng sliding gains để phép so sánh đầu tiên chỉ thay controller
family:

```bash
ros2 run amr_control smc_circle --ros-args \
  -p angular_speed:=0.108 \
  -p ks1:=0.024 \
  -p ks2:=0.050 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/circle/set01 \
  -p paper_run_id:=circle_smc_set01
```

Sau run thứ ba, xem:

```text
paper_runs/three_controller/circle/set01/publication_three/
```

## 7. Square nominal

Ví dụ Square cạnh `2 m`. Nếu vùng camera không chứa được toàn bộ quỹ đạo, đổi
cả ba lệnh thành `square_profile:=1m`.

```bash
mkdir -p /home/hoang/Paper1/Paper1-main/paper_runs/three_controller/square/set01
```

Backstepping:

```bash
ros2 run amr_control backstepping_square --ros-args \
  -p square_profile:=2m \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/square/set01 \
  -p paper_run_id:=square_bs_set01
```

BSMC:

```bash
ros2 run amr_control bsmc_square --ros-args \
  -p square_profile:=2m \
  -p ks1:=0.08 \
  -p ks2:=0.10 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/square/set01 \
  -p paper_run_id:=square_bsmc_set01
```

SMC:

```bash
ros2 run amr_control smc_square --ros-args \
  -p square_profile:=2m \
  -p ks1:=0.08 \
  -p ks2:=0.10 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/square/set01 \
  -p paper_run_id:=square_smc_set01
```

Theo log cũ, bánh trong có thể stall ở góc. Trước bộ validation cuối phải kiểm
tra RPM trái/phải; không dùng run có lỗi phần cứng làm bằng chứng controller.

## 8. Figure-eight nominal

Tắt camera Circle/Square rồi chạy `camera_eight`.

```bash
mkdir -p /home/hoang/Paper1/Paper1-main/paper_runs/three_controller/eight/set01
```

Backstepping:

```bash
ros2 run amr_control backstepping_eight --ros-args \
  -p center_k3:=0.0 \
  -p ki_y:=0.0 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/eight/set01 \
  -p paper_run_id:=eight_bs_set01
```

BSMC:

```bash
ros2 run amr_control bsmc_eight --ros-args \
  -p center_k3:=0.0 \
  -p ki_y:=0.0 \
  -p ks1:=0.03683289 \
  -p ks2:=0.1159106176 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/eight/set01 \
  -p paper_run_id:=eight_bsmc_set01
```

SMC:

```bash
ros2 run amr_control smc_eight --ros-args \
  -p center_k3:=0.0 \
  -p ki_y:=0.0 \
  -p ks1:=0.03683289 \
  -p ks2:=0.1159106176 \
  -p paper_laps:=3 \
  -p paper_output_dir:=/home/hoang/Paper1/Paper1-main/paper_runs/three_controller/eight/set01 \
  -p paper_run_id:=eight_smc_set01
```

Đặt robot gần `(0,0,0)` trong hệ camera. Profile xoay đường `-45 deg`, vì vậy
tiếp tuyến tại giao điểm hướng theo `+X`.

## 9. Circle controlled inward displacement

Nếu không có load cell/force gauge, không đẩy bằng tay trong khi controller
vẫn chạy rồi gọi đó là cùng lực. Dùng paused displacement-matched protocol:

1. Chạy Circle ổn định đến cùng vị trí đã đánh dấu trên sàn.
2. Trong terminal controller, nhập `p` rồi Enter. Robot gửi zero velocity và
   reference time được đóng băng.
3. Dịch robot theo phương bán kính vào phía trong `0.15 ± 0.03 m`. Không đổi
   yaw nếu mục tiêu là lateral-position perturbation.
4. Kiểm tra displacement bằng AprilTag.
5. Giữ robot đứng yên và kiểm tra `/odometry/filtered` đã hội tụ lại với
   `/odom_camera`. Với displacement vượt gate `0.30 m`, chờ EKF báo
   `CAMERA RELOCALIZATION` (mặc định cần 40 frame nhất quán, khoảng `2 s`).
   Không resume khi hai topic vẫn còn lệch.
6. Nhập `d` rồi Enter để đánh dấu release/resume event.
7. Thả robot hoàn toàn và ngay lập tức nhập `p` rồi Enter để resume.
8. Không chạm robot thêm trong ít nhất `15 s`.

Tạo thư mục riêng:

```bash
mkdir -p /home/hoang/Paper1/Paper1-main/paper_runs/three_controller/circle_disturbance/set01
```

Chạy ba lệnh Circle giống Mục 6 nhưng:

- đổi `paper_output_dir` sang thư mục `circle_disturbance/set01`;
- đổi run ID thành `circle_dist_bs_set01`, `circle_dist_bsmc_set01`,
  `circle_dist_smc_set01`;
- dùng `paper_laps:=1` cho pilot hoặc `paper_laps:=3` cho validation.

Pipeline đo:

- baseline position error trong `3 s` trước event;
- peak incremental error trong `10 s`;
- `IAE10`;
- recovery về baseline `+3 cm`, giữ liên tục `1 s`;
- unrecovered/censored nếu không recovery trong `10 s`.

Trong paper, gọi đây là:

```text
recovery from a controlled inward pose perturbation
```

Không gọi là identical-force disturbance rejection.

## 10. Render lại mà không chạy robot

Pipeline tự render khi run thứ ba kết thúc. Có thể render lại:

```bash
ros2 run amr_control three_controller_report --input-dir \
  /home/hoang/Paper1/Paper1-main/paper_runs/three_controller/circle/set01 \
  --trajectory circle
```

Hoặc không qua ROS entry point:

```bash
cd /home/hoang/Paper1/Paper1-main
MPLCONFIGDIR=/tmp/paper1-mpl \
PYTHONPATH=src/amr_control \
python3 -m amr_control.three_controller_report \
  --input-dir paper_runs/three_controller/circle/set01 \
  --trajectory circle
```

Nếu thiếu controller, chương trình in:

```text
INCOMPLETE circle: missing SMC
```

Mỗi thư mục set nên chỉ chứa đúng các run muốn ghép. Nếu có nhiều run cùng
controller, pipeline chọn summary hợp lệ mới nhất và ghi lựa chọn vào:

```text
publication_three/three_controller_provenance.json
```

Luôn kiểm tra file provenance trước khi đưa figure vào bài.

## 11. Checklist trước khi chấp nhận run

- summary có `"valid": true`;
- đúng controller label và run ID;
- đúng `requested_laps`;
- `camera_age_s <= 0.30` trong vùng phân tích;
- không emergency stop, collision hoặc che AprilTag;
- gain trong summary đúng protocol;
- không bão hòa/chattering kéo dài trong command plot;
- Square không có bánh trong đứng do lỗi motor/firmware;
- disturbance có event và đủ ít nhất `10 s` dữ liệu sau event.

Không loại run chỉ vì RMSE cao. Nếu lỗi kỹ thuật, ghi lý do và chạy lại đúng
controller/set.
