# Đánh giá Policy

Sau khi huấn luyện xong (hoặc đang huấn luyện), bạn cần đánh giá xem policy đã học được tốt chưa. Tài liệu này hướng dẫn cách chạy, xem và phân tích kết quả.

---

## 1. Chạy Policy Đã Huấn Luyện

### Từ checkpoint trên workstation

SSH vào workstation và chạy:

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --load-run logs/rsl_rl/m2v6_tracking/<ten-thu-muc-log>
```

### Từ WandB

Nếu đã upload checkpoint lên WandB, có thể tải và chạy trực tiếp:

```bash
uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --wandb-run-path your-org/mjlab/run-id
```

### Sanity check với Agent đơn giản

Trước khi chạy policy thật, có thể kiểm tra nhanh môi trường bằng các agent đơn giản:

Agent "zero" gửi toàn bộ action bằng 0 — robot giữ tư thế mặc định:

```bash
# Trên workstation:
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --agent zero \
  --env.commands.motion.motion-file motions/chao_mung.npz
```

Agent "random" gửi action ngẫu nhiên — robot sẽ co giật lung tung:

```bash
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --agent random \
  --env.commands.motion.motion-file motions/chao_mung.npz
```

Nếu các agent đơn giản chạy bình thường, nghĩa là môi trường và motion data đã đúng.

---

## 2. Trình xem MuJoCo

Khi chạy play, sẽ mở cửa sổ trình xem 3D (nếu có màn hình). Trong trình xem, bạn có thể:

### Điều khiển camera
- **Kéo chuột trái:** xoay góc nhìn
- **Kéo chuột phải:** di chuyển góc nhìn
- **Cuộn chuột:** zoom vào/ra
- **Nhấp đúp:** focus vào một bộ phận

### Chế độ render nền

Trên workstation (qua SSH, không có màn hình), luôn dùng MUJOCO_GL=egl:

```bash
# Trên workstation:
cd ~/Documents/vm_mjlab
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --load-run logs/rsl_rl/m2v6_tracking/<ten-thu-muc-log>
```

---

## 3. Các Chỉ số Đánh giá

### Chỉ số định lượng

Khi chạy play, hệ thống sẽ in ra các chỉ số trung bình:

| Chỉ số | Ý nghĩa | Giá trị tốt |
|--------|---------|-------------|
| Episode reward | Tổng reward trung bình | Càng cao càng tốt |
| Episode length | Thời lượng episode trung bình | Gần 10s (max) |
| Tracking body pos error | Sai số vị trí trung bình | < 0.05m |
| Tracking body ori error | Sai số hướng trung bình | < 0.2 rad |

### Đánh giá bằng mắt

Khi xem trong trình xem, chú ý:

| Tiêu chí | Tốt | Xấu |
|-----------|-----|------|
| Tổng thể | Robot bắt chước khá giống motion tham chiếu | Robot làm sai hướng hoặc đứng yên |
| Tay | Tay vẫy đúng nhịp và biên độ | Tay giật hoặc không giơ lên |
| Chân | Đứng vững, không trượt | Chân trượt trên mặt đất |
| Thân | Thẳng, ổn định | Nghiêng hoặc rung lắc |
| Chuyển tiếp | Mượt, tự nhiên | Giật, đột ngột |

---

## 4. So sánh Checkpoint

Nên chạy nhiều checkpoint khác nhau để chọn checkpoint tốt nhất. Thường checkpoint cuối cùng không phải lúc nào cũng tốt nhất (có thể bị overfit).

### Các checkpoint nên thử

| Checkpoint | Lý do |
|-----------|-------|
| model_5000.pt | Giai đoạn đầu — kiểm tra robot đã học cơ bản chưa |
| model_15000.pt | Giai đoạn giữa — cân bằng giữa tracking và ổn định |
| model_25000.pt | Giai đoạn cuối — chi tiết nhất |
| model_30000.pt | Cuối cùng — có thể overfitting |

---

## 5. Xuất Policy sang ONNX

Để triển khai trên robot thật hoặc chia sẻ, cần xuất policy sang định dạng ONNX. mjlab hỗ trợ xuất tự động khi hoàn thành huấn luyện.

File ONNX sẽ nằm cùng thư mục checkpoint:

```
logs/rsl_rl/m2v6_tracking/<timestamp>/
├── model_30000.pt        ← checkpoint PyTorch
└── <timestamp>.onnx      ← policy đã xuất
```

### Kiểm tra metadata ONNX

File ONNX chứa metadata mô tả observation và action. Dùng script có sẵn để trích xuất:

```bash
# Trên workstation:
cd ~/Documents/vm_mjlab
uv run python load_metadata_onnx.py
```

> Lưu ý: cần sửa đường dẫn trong script cho phù hợp với file ONNX của bạn.

---

## 6. Xử lý Khi Kết quả Chưa Tốt

### Robot không bắt chước được

| Nguyên nhân | Giải pháp |
|-------------|-----------|
| Motion NPZ sai | Kiểm tra lại video render từ bước chuyển đổi CSV → NPZ |
| FPS không khớp | Đảm bảo output-fps = 50 (khớp với tần số điều khiển) |
| Chưa đủ iteration | Huấn luyện thêm (tăng max-iterations) |

### Robot bắt chước nhưng giật

| Nguyên nhân | Giải pháp |
|-------------|-----------|
| Action rate penalty thấp | Tăng trọng số action_rate_l2 (ví dụ: -0.5 thay vì -0.1) |
| Motion tham chiếu giật | Áp dụng bộ lọc làm mượt cho CSV trước khi chuyển đổi |

### Robot ngã liên tục

| Nguyên nhân | Giải pháp |
|-------------|-----------|
| Motion quá khó | Bắt đầu với motion đơn giản hơn |
| Termination quá nghiêm | Tăng ngưỡng (threshold) trong cấu hình |
| Push robot quá mạnh | Giảm velocity_range trong events |

---

## 7. Script Đánh giá Sẵn

Một script shell tiện lợi đã được chuẩn bị:

```bash
# Trên workstation:
cd ~/Documents/vm_mjlab
bash scripts/03_chay_policy.sh
```

---

## Tiếp theo

Hãy đọc tiếp:
1. [Triển khai Robot thực](09-trien-khai-robot-thuc.md) — Chuyển policy sang robot thật
