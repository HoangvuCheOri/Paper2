# Prompt khôi phục phiên làm việc thực nghiệm

Sao chép toàn bộ phần nằm trong khối `text` bên dưới và gửi cho Codex ở phiên
mới.

```text
Tôi đang làm bài báo nghiên cứu về điều khiển bám quỹ đạo robot WMR. Workspace
đầy đủ cần dùng là:

/home/hoang/Paper1/Paper1-main

Không dùng /home/hoang/Paper1/src vì đó là package cũ, thiếu Square paper
runner, SMC, embedded logger và pipeline so sánh ba controller.

MỤC TIÊU

Tôi muốn so sánh ba controller:

1. Backstepping
2. BSMC
3. pure SMC

trên ba quỹ đạo:

1. Circle
2. Square
3. Figure-eight

Ngoài nominal tracking, tôi muốn thí nghiệm Circle khi robot được dịch/đẩy vào
phía trong và so sánh khả năng recovery.

Output cuối cho mỗi quỹ đạo phải có:

- một hình Reference + ba đường thực nghiệm Backstepping/BSMC/SMC;
- hình tổng hợp và hình riêng của e_x, e_y, e_theta;
- hình tổng hợp và hình riêng của v_cmd, w_cmd;
- PDF vector, PNG 600 dpi;
- CSV metric và JSON provenance.

Circle disturbance cần thêm:

- disturbance recovery theo thời gian từ release/resume;
- trajectory quanh disturbance event;
- peak incremental error, IAE10, recovery time và unrecovered/censored.

DỮ LIỆU CŨ ĐÃ CÓ

- Circle nominal: đã có 5 Backstepping + 5 BSMC, mỗi run 3 vòng, audit hợp lệ.
  Hai controller cho kết quả nominal gần tương đương, khoảng 3 cm position
  RMSE. Không có SMC.
- Square: có một số cặp Backstepping/BSMC và nhiều run tuning BSMC nhưng chưa
  phải bộ validation ba controller. Có cảnh báo bánh trong stall ở góc do
  PI/deadzone STM32.
- Figure-eight: có BSMC r08 chạy 3 vòng đẹp, position RMSE khoảng 2.75 cm,
  heading RMSE khoảng 3.56 deg. Không có Backstepping và SMC tương ứng.
- Circle disturbance cũ: chỉ là pilot Backstepping/BSMC; thời điểm tác động,
  gain và protocol không đồng nhất nên không dùng làm kết quả chính.
- Hoàn toàn chưa có log SMC thật. Không được tạo hoặc giả lập đường SMC để đưa
  vào paper.

CODE ĐÃ ĐƯỢC BỔ SUNG

- amr_control/controller_modes.py:
  cấu hình pure SMC bằng cách giữ reference/feedforward, bounded sliding
  injection, giới hạn actuator và bảo vệ phần cứng; loại feedback tỷ lệ
  Backstepping.
- smc_circle.py, smc_square.py, smc_eight.py.
- ROS executable:
  smc_circle, smc_square, smc_eight, three_controller_report.
- three_controller_report.py:
  ghép ba controller; cũng có --allow-partial để render trung thực log cũ khi
  thiếu SMC.
- embedded_paper_capture.py:
  ghi control_paused, event disturbance và metric recovery dựa trên nominal
  baseline trước khi pause.
- Circle direct defaults được đổi về angular_speed=0.108, Ks1=Ks2=0.
  bsmc_circle paper runner tự dùng bộ Circle BSMC đã validation:
  Ks1=0.024, Ks2=0.050.
- Figure-eight profile đã đồng bộ center_k3=0 với run r08.
- setup.py đã đăng ký executable mới.

Package đã colcon build thành công và ROS đã liệt kê đủ executable. Pipeline
figure đã render smoke-test thành công. Môi trường không có pytest nên chưa
chạy pytest đầy đủ. Chưa có hardware validation cho SMC.

Hướng dẫn chi tiết đã có tại:

/home/hoang/Paper1/Paper1-main/docs/three_controller_experiment.md

THƯ MỤC FIGURE LOG CŨ

/home/hoang/Paper1/Paper1-main/paper_exports/existing_logs_paper_ready

Thư mục này chỉ là figure ghép từ controller đang có. Nó không có SMC và không
phải bộ kết quả ba-controller cuối.

QUYẾT ĐỊNH KHOA HỌC HIỆN TẠI

Không chạy ngay bộ validation chính thức. Backstepping và SMC chưa được tune
độc lập để đạt hiệu suất tốt nhất:

- Backstepping hiện dùng gain nền của BSMC rồi đặt Ks1=Ks2=0.
- SMC hiện loại k1/k2/k3 Backstepping và dùng sliding gains lấy từ BSMC.
- BSMC đã được tuning nhiều hơn hai baseline.

Nếu muốn gọi đây là best-tuned three-controller comparison, phải tune riêng
Backstepping và SMC cho từng Circle/Square/Eight với cùng tuning budget và
cùng objective. Nếu chỉ giữ tham số chung và bật/tắt thành phần, phải gọi đúng
là component ablation.

KẾ HOẠCH TIẾP THEO

1. Đọc git diff/status và các file nêu trên; không xóa hoặc đảo các thay đổi
   hiện có.
2. Kiểm tra code/build trước khi robot chuyển động.
3. Bắt đầu tuning Backstepping Circle.
4. Sau đó tuning SMC Circle.
5. Khi Circle đã an toàn và khóa gain, tiếp tục Square rồi Figure-eight.
6. Mỗi cấu hình mới phải chạy pilot 1 vòng trước; kiểm tra RMSE, max error,
   v_cmd/w_cmd, saturation, chattering, camera freshness và RPM bánh.
7. Không dùng run tuning trong bảng validation cuối.
8. Sau khi khóa gain mới chạy repeat chính thức theo randomized blocks.

Objective tuning cần được khóa trước, ví dụ kết hợp:

- camera-derived position RMSE;
- heading RMSE;
- max position error;
- command variation/chattering;
- v/w saturation penalty;
- invalid/incomplete-run penalty.

Không tối ưu chỉ theo một đường nhìn đẹp.

CÁCH PHỐI HỢP CHẠY ROBOT

Bạn (Codex) có thể chạy và quản lý các terminal ROS giúp tôi. Tôi chỉ đặt lại
robot. Quy trình:

1. Khởi động/kiểm tra serial, state, EKF và camera.
2. Kiểm tra /odom_camera, /odometry/filtered và /espnow_link.
3. Báo controller, trajectory, gain và run ID sắp chạy.
4. Chờ tôi xác nhận chính xác "robot ready" trước khi phát lệnh chuyển động.
5. Chỉ chạy một controller vì tất cả publish /cmd_vel.
6. Theo dõi log; dừng khi có stale feedback, mất tag, chattering mạnh, stall,
   bão hòa kéo dài hoặc tôi yêu cầu dừng.
7. Sau run, kiểm tra CSV/summary/figure rồi báo tôi đặt lại robot.
8. Chờ xác nhận mới chạy run tiếp theo.

Tôi phải luôn giữ nút dừng khẩn cấp và vùng chạy phải trống. Không tự động chạy
liên tiếp qua bước đặt lại robot.

Với Circle displacement protocol khi không có load cell:

1. Chạy đến phase/vị trí đã khóa.
2. Pause và gửi zero velocity.
3. Tôi dịch robot vào trong 0.15 +/- 0.03 m, cố gắng không đổi yaw.
4. Chờ tôi xác nhận đã đặt xong.
5. Đánh dấu release/resume event.
6. Resume controller và không chạm robot ít nhất 15 s.

Gọi đúng thí nghiệm này là:
"recovery from a controlled inward pose perturbation"
không gọi là identical-force disturbance rejection.

BUILD/SOURCE

cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_control --symlink-install
source install/setup.bash

Các node nền:

ros2 run amr_control robot_serial_bridge
ros2 run amr_control state_bridge
ros2 run amr_control custom_ekf_node

Circle/Square camera:

ros2 run amr_control camera_circle_square

Figure-eight camera:

ros2 run amr_control camera_eight

TRƯỚC KHI BẮT ĐẦU

Hãy trả lời bằng:

1. git status và workspace/package thực sự sẽ dùng;
2. xác nhận executable Backstepping/BSMC/SMC;
3. bảng gain hiện tại của Backstepping Circle và SMC Circle;
4. kế hoạch tuning Circle ngắn gọn, số pilot dự kiến và tiêu chí dừng;
5. các node/topic cần kiểm tra.

Không khởi động robot cho đến khi tôi nhắn "robot ready".
```

