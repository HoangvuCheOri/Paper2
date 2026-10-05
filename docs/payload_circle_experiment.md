# Thí nghiệm Circle với 7 điều kiện tải

Runner `payload_circle_experiment` chạy 7 điều kiện cho Backstepping, BSMC và
SMC. Mỗi run tự tạo CSV, summary JSON và figure như các thí nghiệm paper hiện
có. CSV và summary đều chứa điều kiện khối lượng/mô-men quán tính; manifest của
cả phiên nằm trong thư mục session.

## 1. Khối lượng và mô-men quán tính

Với khối lượng robot không tải `m0 = 1.55 kg`, tải `dm`, khoảng lệch giữa tâm
tải và trục đứng qua tâm robot `d`:

```text
m_total = m0 + dm
Jz_total = Jz_robot + Jz_payload,COM + dm*d^2
delta_Jz_payload = Jz_payload,COM + dm*d^2
```

`dm*d^2` là định lý trục song song. Nếu chỉ xét phần tăng do chuyển cùng một
cục tải từ tâm ra `d = 0.05 m`, ba giá trị lần lượt là:

| dm (kg) | tăng thêm do lệch tâm dm*d^2 (kg.m^2) |
|---:|---:|
| 0.5 | 0.00125 |
| 1.0 | 0.00250 |
| 1.5 | 0.00375 |

Không suy ra được `Jz_robot` chỉ từ khối lượng 1.55 kg. Lấy giá trị này từ CAD
hoặc đo bằng con lắc xoắn/bifilar. Nếu tải gần hộp chữ nhật đồng nhất, có thể
ước lượng `Jz_payload,COM = dm*(a^2+b^2)/12`, với `a`, `b` là chiều dài/rộng
của đáy tải. Không có kích thước thì runner ghi mô hình
`point_mass_approximation`; lúc đó giá trị ở case đặt tâm bằng zero chỉ là xấp
xỉ, không phải mô-men thực của tải.

## 2. Build và kiểm tra kế hoạch

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
colcon build --packages-select amr_control --symlink-install
source install/setup.bash

ros2 run amr_control payload_circle_experiment \
  --payload-length-m 0.20 --payload-width-m 0.10
```

Lệnh trên chỉ in kế hoạch 21 run, chưa làm robot chạy. Nếu đã có `Jz_robot`,
thêm `--base-jz VALUE`. Nếu có mô-men riêng của tải chính xác, dùng
`--payload-jz-com VALUE` thay cho kích thước (chỉ phù hợp khi các tải có cùng
mô-men riêng, hoặc khi chạy từng case).

## 3. Chạy lấy dữ liệu

Khởi động trước bốn node nền `robot_serial_bridge`, `state_bridge`,
`custom_ekf_node`, `camera_circle_square` như SOP Circle. Sau đó:

```bash
ros2 run amr_control payload_circle_experiment \
  --execute \
  --session-id payload_circle_set01 \
  --repeat 1 \
  --laps 3 \
  --payload-length-m 0.20 \
  --payload-width-m 0.10 \
  --base-jz VALUE
```

Runner dừng trước từng run để xác nhận đúng tải, đúng vị trí và pose đầu. Không
dùng `--yes` khi thay tải thủ công. Có thể chạy riêng một tổ hợp để tiếp tục sau
khi gián đoạn:

```bash
ros2 run amr_control payload_circle_experiment \
  --execute --session-id payload_circle_set01 \
  --case m1p0_offset_5cm --controller bsmc \
  --payload-length-m 0.20 --payload-width-m 0.10 --base-jz VALUE
```

Mặc định điểm lệch là 5 cm về phía trước robot (`x=+0.05 m, y=0`). Phải dùng
cùng một hướng cho toàn bộ thí nghiệm. Có thể đổi bằng
`--offset-direction rear|left|right`; quy ước hệ thân robot là `x` hướng trước,
`y` hướng trái. Hướng lệch không đổi công thức `Jz`, nhưng có thể đổi đáp ứng
động lực học nên được ghi rõ.

Các case hợp lệ: `nominal`, `m0p5_center`, `m1p0_center`, `m1p5_center`,
`m0p5_offset_5cm`, `m1p0_offset_5cm`, `m1p5_offset_5cm`.

Output mặc định:

```text
paper_runs/payload_circle/<session-id>/
  *_circle_<controller>_*.csv
  *_summary.json
  payload_experiment_manifest.json
```

Các cột vật lý trong CSV gồm `base_mass_kg`, `payload_mass_kg`,
`total_mass_kg`, `payload_offset_m`, `base_jz_kg_m2`,
`payload_offset_x_m`, `payload_offset_y_m`, `payload_jz_com_kg_m2`,
`payload_delta_jz_kg_m2`, `total_jz_kg_m2`.

## 4. Thiết kế số lần lặp

Một lần chạy 3 vòng vẫn là một mẫu độc lập (`n=1`). Nếu dùng kết quả để kết
luận thống kê, nên chạy ít nhất 3 lần khởi động độc lập cho mỗi
case/controller. Khi dùng `--controller all --repeat 3`, runner tự xoay thứ tự
controller giữa các repeat: Backstepping–BSMC–SMC, BSMC–SMC–Backstepping,
SMC–Backstepping–BSMC. Giữ nguyên pin,
pose đầu, vị trí dán tải, tốc độ góc và gain; đánh dấu chính xác vị trí tâm và
điểm lệch 5 cm trên robot.
