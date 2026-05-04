# Huấn luyện Motion Imitation

Hướng dẫn thực hành từng bước để huấn luyện robot M2v6 bắt chước chuyển động tham chiếu, sử dụng ví dụ cụ thể là tư thế chào mừng.

---

## 1. Điều kiện Tiên quyết

Trước khi bắt đầu huấn luyện, đảm bảo bạn đã có:
- Môi trường mjlab đã cài đặt trên **workstation** (xem [doc 02](02-thiet-lap-moi-truong.md))
- File NPZ motion đã được tạo và copy lên workstation
- Tài khoản WandB đã đăng nhập trên workstation
- Workstation: RTX 5090 32GB VRAM (4096 envs)

> **Lưu ý:** Toàn bộ huấn luyện chạy trên workstation qua SSH.
> Kết nối: `ssh nguyenl3@10.148.255.113`
> Đường dẫn mjlab: `/home/nguyenl3/Documents/vm_mjlab/`

---

## 2. Các Task có sẵn cho M2v6

Trong mjlab, mỗi task là một cấu hình sẵn mô tả: robot nào, môi trường gì, reward ra sao. Cho M2v6, có 2 task tracking:

### Task 1: Mjlab-Tracking-Flat-M2v6
- Đầy đủ observation: bao gồm cả vị trí anchor và vận tốc tuyến tính
- Phù hợp khi có state estimation tốt (ví dụ: dùng bộ lọc Kalman)

### Task 2: Mjlab-Tracking-Flat-M2v6-No-State-Estimation
- Bỏ 2 observation: vị trí anchor tương đối và vận tốc tuyến tính
- Phù hợp hơn cho triển khai thực tế (robot thật khó đo chính xác 2 thông tin này)
- **Khuyến nghị dùng task này** cho phần lớn trường hợp

---

## 3. Chuyển đổi CSV sang NPZ

Trước khi huấn luyện, cần chuyển file CSV (từ retargeting) thành file NPZ mà mjlab hiểu.

### Giải thích các tham số

| Tham số | Ý nghĩa | Giá trị thường dùng |
|---------|---------|---------------------|
| input-file | Đường dẫn file CSV đầu vào | Đường dẫn tuyệt đối |
| output-file | Đường dẫn file NPZ đầu ra | motions/ten_motion.npz |
| input-fps | Tốc độ khung hình của file CSV | 30 (từ GVHMR) hoặc 120 |
| output-fps | Tốc độ khung hình mong muốn | 50 (khớp với tần số điều khiển) |
| render | Có tạo video kiểm tra không | True (lần đầu nên bật) |
| line-range | Chỉ xử lý một phần file CSV | Tùy chọn, ví dụ: (100, 500) |

Gửi file CSV lên workstation và chạy chuyển đổi:

```bash
# Từ laptop: gửi file CSV lên workstation
scp chao_mung.csv nguyenl3@10.148.255.113:~/Documents/vm_mjlab/motions/
```

```bash
# Trên workstation (SSH vào trước):
cd ~/Documents/vm_mjlab

MUJOCO_GL=egl uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
  --input-file motions/chao_mung.csv \
  --output-file motions/chao_mung.npz \
  --input-fps 30 \
  --output-fps 50 \
  --render True
```

Kết quả:
- File NPZ chứa: góc khớp, vận tốc, vị trí và hướng các bộ phận
- Video MP4 để kiểm tra bằng mắt (cùng thư mục, cùng tên)

---

## 4. Đăng tải Motion lên WandB

Sau khi tạo file NPZ, cần đăng tải lên WandB Registry để mjlab có thể tải khi huấn luyện.

Bạn có thể dùng giao diện web WandB hoặc dùng API. Tham khảo tài liệu WandB Registry cho cách upload artifact.

Hoặc, sử dụng trực tiếp file local bằng cách trỏ đường dẫn tuyệt đối thay vì registry name.

---

## 5. Bắt đầu Huấn luyện

### Chạy huấn luyện cơ bản

SSH vào workstation và chạy huấn luyện với 4096 robot song song:

```bash
# SSH vào workstation
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/chao_mung.npz
```

> **RTX 5090 (32GB VRAM):** chạy thoải mái 4096 envs. Thời gian ước tính: 3-4 giờ cho 30,000 iteration.

### Giảm số môi trường (nếu cần)

Nếu gặp vấn đề VRAM (không chắc sẽ xảy ra với 32GB):

```bash
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 2048 \
  --env.commands.motion.motion-file motions/chao_mung.npz
```

---

## 6. Theo dõi Huấn luyện

### Trên terminal

Trong quá trình huấn luyện, terminal sẽ hiển thị thông tin mỗi iteration:
- **Mean reward:** trung bình reward — con số này nên tăng dần
- **Mean episode length:** thời lượng trung bình episode — nên tăng (robot tồn tại lâu hơn)

### Trên WandB Dashboard

Mở trình duyệt, truy cập trang WandB của bạn để xem:
- Biểu đồ reward theo thời gian
- Biểu đồ từng thành phần reward riêng lẻ
- Biểu đồ episode length
- Thông tin học tập (learning rate, loss, v.v.)

### Các mốc quan trọng

| Iteration | Kỳ vọng |
|-----------|---------|
| 0 - 1000 | Robot bắt đầu đứng được, reward tăng nhanh |
| 1000 - 5000 | Robot bắt chước tổng thể (đúng hướng chung) |
| 5000 - 15000 | Chi tiết cải thiện dần (tay, chân chính xác hơn) |
| 15000 - 30000 | Tinh chỉnh, reward tăng chậm, đạt ổn định |

> Thời gian ước tính trên workstation RTX 5090: khoảng **3-4 giờ** với 4096 envs.

---

## 7. Checkpoint và Tiếp tục Huấn luyện

### Lưu tự động

Checkpoint được lưu tự động mỗi 500 iteration vào thư mục trên workstation:

```
/home/nguyenl3/Documents/vm_mjlab/logs/rsl_rl/m2v6_tracking/<timestamp>/
├── model_500.pt
├── model_1000.pt
├── ...
└── model_30000.pt
```

### Tiếp tục huấn luyện

Nếu quá trình bị gián đoạn hoặc muốn huấn luyện thêm, chỉ định checkpoint:

```bash
# Trên workstation:
cd ~/Documents/vm_mjlab

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/chao_mung.npz \
  --load-run logs/rsl_rl/m2v6_tracking/<ten-thu-muc-log>
```

---

## 8. Điều chỉnh Siêu tham số

### Khi nào cần điều chỉnh?

| Triệu chứng | Nguyên nhân có thể | Điều chỉnh |
|-------------|---------------------|------------|
| Reward không tăng | Learning rate quá cao/thấp | Thử 3e-4 hoặc 3e-3 |
| Reward dao động mạnh | Batch size nhỏ | Tăng num-envs hoặc num-steps-per-env |
| Robot giật mạnh | Action rate penalty quá nhẹ | Tăng trọng số action_rate_l2 |
| Robot "ăn gian" | Reward thiếu thành phần | Thêm/tăng trọng số tracking |
| Episode quá ngắn | Termination quá nghiêm | Tăng ngưỡng termination |

### Ưu tiên thử
1. Đầu tiên, chạy với config mặc định
2. Nếu không ổn, điều chỉnh learning rate
3. Tiếp theo, thử thay đổi số envs
4. Cuối cùng mới sửa reward weights

---

## 9. Script Huấn luyện Sẵn

Một script shell tiện lợi đã được chuẩn bị. Chỉ cần thay đường dẫn motion file và chạy:

Trên workstation:

```bash
cd ~/Documents/vm_mjlab
bash scripts/02_huan_luyen.sh
```

Hoặc từ laptop (copy script lên workstation và chạy qua SSH):

```bash
scp /home/nguyenld12/Documents/Humanoid_Tracking_Task/scripts/02_huan_luyen.sh nguyenl3@10.148.255.113:~/Documents/vm_mjlab/scripts/
echo '1' | ssh nguyenl3@10.148.255.113 "cd ~/Documents/vm_mjlab && bash scripts/02_huan_luyen.sh"
```

---

## Tiếp theo

Hãy đọc tiếp:
1. [Đánh giá Policy](08-danh-gia-policy.md) — Xem kết quả huấn luyện
2. [Triển khai Robot thực](09-trien-khai-robot-thuc.md) — Chuyển sang robot thật
