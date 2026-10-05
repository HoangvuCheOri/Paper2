# Payload Circle — set01

Mỗi điều kiện tải có một thư mục riêng. Ba run Backstepping, BSMC và SMC của
cùng điều kiện được lưu chung trong thư mục đó để pipeline có thể so sánh.

Các lệnh thí nghiệm ghi trực tiếp điều kiện vật lý vào từng hàng CSV và vào
khối `load_condition` của file `_summary.json`:

- khối lượng robot, tải và tổng khối lượng;
- vị trí tải, `d`, `dx`, `dy`;
- mô-men riêng của tải quanh COM;
- phần tăng mô-men quán tính của tải;
- tổng `Jz` nếu đã biết `Jz` robot không tải.

Giá trị hiện tại dùng mô hình tải điểm:

```text
payload_delta_Jz = payload_mass * d^2
total_Jz = base_Jz + payload_delta_Jz
```

Vì chưa có `base_Jz` đo từ CAD/thực nghiệm và chưa có kích thước thật của cục
tải, `base_jz_kg_m2` và `total_jz_kg_m2` được để trống. Không được diễn giải
giá trị trống thành zero. Bảng `experiment_plan.csv` giữ các đại lượng đã biết
để bổ sung `Jz` tuyệt đối sau mà không sửa CSV đo gốc.
