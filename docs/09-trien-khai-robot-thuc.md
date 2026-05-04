# Triển khai lên Robot Thực

Sau khi policy hoạt động tốt trong mô phỏng, bước cuối cùng là triển khai lên robot M2v6 thật. Tài liệu này mô tả quy trình chung.

> **Lưu ý:** Triển khai trên robot thật cần sự giám sát của kỹ sư có kinh nghiệm. Luôn đảm bảo an toàn.

---

## 1. Xuất Policy sang ONNX

### File ONNX là gì?

ONNX (Open Neural Network Exchange) là định dạng chuẩn để chia sẻ mô hình AI. File ONNX chứa:
- Kiến trúc mạng neural
- Trọng số đã huấn luyện
- Metadata mô tả đầu vào/đầu ra

### Tìm file ONNX

File ONNX được tạo tự động khi huấn luyện hoàn thành, nằm trong thư mục log:

```
mjlab/logs/rsl_rl/m2v6_tracking/<timestamp>/
└── <timestamp>.onnx
```

### Kiểm tra ONNX

Trích xuất metadata để xác nhận đầu vào/đầu ra đúng:

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab
uv run python load_metadata_onnx.py
```

Kết quả CSV sẽ cho biết:
- Tên và kích thước observation
- Tên và kích thước action
- Thông tin normalization (mean, std)

---

## 2. Chuẩn bị Robot Thật

### Phần cứng cần có
- Robot M2v6 đã lắp ráp và hiệu chuẩn
- Máy tính điều khiển (kết nối với robot)
- Hệ thống dừng khẩn cấp (E-stop)

### Phần mềm
- ONNX Runtime (để chạy inference trên máy tính điều khiển)
- Driver giao tiếp với motor robot
- Bộ lọc trạng thái (state estimator) nếu dùng task có state estimation

---

## 3. Vấn đề An toàn

### Trước khi chạy

- [ ] Kiểm tra tất cả kết nối cáp
- [ ] Đảm bảo E-stop hoạt động
- [ ] Có ít nhất 2 người giám sát
- [ ] Không gian xung quanh robot trống (bán kính ≥ 2m)
- [ ] Bắt đầu với robot treo trên giá (không chạm đất)

### Quy trình chạy an toàn

1. **Bước 1:** Treo robot trên giá, bật motor ở chế độ nhẹ
2. **Bước 2:** Chạy policy, quan sát chuyển động tay/chân
3. **Bước 3:** Nếu ổn, hạ robot xuống mặt đất, vẫn có người đỡ
4. **Bước 4:** Từ từ thả robot, sẵn sàng nhấn E-stop

### Các nguy cơ thường gặp

| Nguy cơ | Phòng tránh |
|---------|-------------|
| Robot ngã | Luôn có người đỡ, dùng dây treo ban đầu |
| Motor quá nóng | Giới hạn thời gian chạy, kiểm tra nhiệt độ |
| Chuyển động bất ngờ | Bắt đầu với action scale nhỏ |
| Sim-to-real gap | Domain randomization đã giúp, nhưng cần tinh chỉnh |

---

## 4. Sim-to-Real Transfer

### Thách thức
Policy học trong mô phỏng, nhưng thực tế có nhiều khác biệt:
- Mô hình vật lý không hoàn hảo
- Cảm biến có nhiễu thực
- Độ trễ (latency) khi giao tiếp
- Ma sát sàn thực tế khác mô phỏng

### Domain Randomization đã giúp gì?

Trong quá trình huấn luyện, ta đã ngẫu nhiên hóa:
- Ma sát sàn: 0.3 — 1.2
- Stiffness/damping khớp: ±20%
- Trọng tâm: ±5cm
- Nhiễu encoder: ±0.01 rad
- Đẩy robot ngẫu nhiên

Nhờ vậy, policy đã "quen" với nhiều điều kiện khác nhau → chuyển sang thực tế dễ hơn.

### Tinh chỉnh thêm (nếu cần)

Nếu robot thật chưa hoạt động tốt:
- Tăng cường domain randomization (phạm vi rộng hơn)
- Thêm latency giả trong mô phỏng
- Thu thập dữ liệu thực tế, fine-tune policy

---

## 5. Tóm tắt Toàn bộ Quy trình

```
Video người thật
    │
    ▼
GVHMR (vm_video2robot) → Tư thế SMPL 3D
    │
    ▼
Retargeting (vm_retargeting) → File CSV (27 khớp M2v6)
    │
    ▼
csv_to_npz_m2v6.py (mjlab) → File NPZ (motion + FK)
    │
    ▼
uv run train (mjlab + PPO) → Policy (mạng neural)
    │
    ▼
uv run play (đánh giá) → Checkpoint tốt nhất
    │
    ▼
Xuất ONNX → Triển khai robot thật
```

---

## Tài liệu Tham khảo

- [mjlab Documentation](https://mujocolab.github.io/mjlab/)
- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [GVHMR Paper](https://arxiv.org/abs/2409.06662)
- [PPO Paper (Schulman et al.)](https://arxiv.org/abs/1707.06347)
- [BeyondMimic](https://beyondmimic.github.io/)
