# Các lệnh riêng lẻ — Payload Circle set01

Mỗi lệnh dưới đây chỉ chạy đúng **một controller trong một case**. Runner tự
đưa metadata khối lượng, vị trí tải và phần tăng `Jz` vào CSV/summary. Sau khi
copy lệnh, kiểm tra tải và pose đầu rồi nhấn Enter.

## Node nền (bốn terminal riêng)

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amr_control robot_serial_bridge
```

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amr_control state_bridge
```

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amr_control custom_ekf_node
```

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run amr_control camera_circle_square
```

## Terminal chạy thí nghiệm

Chạy ba dòng này một lần trong terminal thứ năm:

```bash
cd /home/hoang/Paper1/Paper1-main
source /opt/ros/humble/setup.bash
source install/setup.bash
```

### 1. Nominal

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case nominal --controller backstepping --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case nominal --controller bsmc --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case nominal --controller smc --laps 3
```

### 2. Tải 0.5 kg đặt tâm

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m0p5_center --controller backstepping --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m0p5_center --controller bsmc --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m0p5_center --controller smc --laps 3
```

### 3. Tải 1.0 kg đặt tâm

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p0_center --controller backstepping --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p0_center --controller bsmc --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p0_center --controller smc --laps 3
```

### 4. Tải 1.5 kg đặt tâm

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p5_center --controller backstepping --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p5_center --controller bsmc --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p5_center --controller smc --laps 3
```

### 5. Tải 0.5 kg lệch 5 cm về phía trước

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m0p5_offset_5cm --controller backstepping --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m0p5_offset_5cm --controller bsmc --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m0p5_offset_5cm --controller smc --laps 3
```

### 6. Tải 1.0 kg lệch 5 cm về phía trước

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p0_offset_5cm --controller backstepping --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p0_offset_5cm --controller bsmc --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p0_offset_5cm --controller smc --laps 3
```

### 7. Tải 1.5 kg lệch 5 cm về phía trước

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p5_offset_5cm --controller backstepping --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p5_offset_5cm --controller bsmc --laps 3
```

```bash
ros2 run amr_control payload_circle_experiment --execute --session-id set01 --case m1p5_offset_5cm --controller smc --laps 3
```
