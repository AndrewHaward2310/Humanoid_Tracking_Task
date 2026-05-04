# Thiết lập Môi trường

Hướng dẫn cài đặt tất cả phần mềm cần thiết để chạy được toàn bộ quy trình: từ trích xuất tư thế, chuyển đổi motion, đến huấn luyện robot.

---

## 1. Quy trình 2 Máy

Chúng ta sử dụng **2 máy** với vai trò khác nhau:

### Máy Local (Laptop)

| Thông số | Giá trị |
|----------|--------|
| GPU | NVIDIA RTX 5070 Laptop (8GB VRAM) |
| Vai trò | Phát triển, GVHMR, retargeting, xem kết quả |
| Đường dẫn | `/home/nguyenld12/Documents/Humanoid_Tracking_Task/` |

### Workstation (Server huấn luyện)

| Thông số | Giá trị |
|----------|--------|
| GPU | **NVIDIA RTX 5090 (32GB VRAM)** |
| RAM | 188GB |
| Ổ cứng | 1.1TB trống |
| SSH | `ssh nguyenl3@10.148.255.113` |
| Vai trò | Huấn luyện RL, chuyển đổi CSV→NPZ, đánh giá policy |
| Đường dẫn mjlab | `/home/nguyenl3/Documents/vm_mjlab/` |

### Phân chia công việc

| Bước | Máy thực hiện | Lý do |
|------|--------------|-------|
| Quay video | Laptop | Có camera / nhận file |
| GVHMR (trích xuất 3D) | Laptop | Cần conda env riêng |
| Retargeting (SMPL→CSV) | Laptop | Nhẹ, CPU là đủ |
| CSV → NPZ | **Workstation** | Cần MuJoCo GPU cho FK |
| Huấn luyện RL | **Workstation** | Cần 32GB VRAM, 4096 envs |
| Đánh giá policy | **Workstation** | Headless, GPU render |
| Theo dõi WandB | Laptop | Trình duyệt web |

### Chuyển file giữa 2 máy

Gửi file từ laptop lên workstation:

```bash
scp file_motion.csv nguyenl3@10.148.255.113:~/Documents/vm_mjlab/motions/
```

Tải checkpoint từ workstation về laptop:

```bash
scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_tracking/<ten_log>/model_best.pt ./
```

### Yêu cầu chung
- **Hệ điều hành:** Linux (Ubuntu 22.04+)
- **RAM:** tối thiểu 16GB
- **Ổ cứng:** ít nhất 50GB trống

---

## 2. Cài đặt mjlab

mjlab dùng **uv** làm trình quản lý gói Python. Đây là công cụ hiện đại, nhanh hơn pip rất nhiều.

### 2.1. Cài đặt uv

Nếu máy bạn chưa có uv, chạy lệnh sau để cài đặt:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```

Sau khi cài xong, đóng và mở lại terminal, rồi kiểm tra:

```bash
uv --version
```

### 2.2. Cài đặt mjlab từ mã nguồn

Đi đến thư mục mjlab và cài đặt các thư viện phụ thuộc. Lệnh này sẽ tự tạo môi trường ảo và cài tất cả.

**Trên laptop** (nếu muốn chạy demo/đánh giá nhẹ):

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab
uv sync --extra cu128
```

**Trên workstation** (đã có sẵn tại `/home/nguyenl3/Documents/vm_mjlab/`):

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab
uv sync --extra cu128
```

> Workstation đã có sẵn uv và vm_mjlab với .venv. Chỉ cần chạy lại `uv sync` nếu có cập nhật code.

Giải thích:
- Lệnh trên sẽ đọc file cấu hình và cài đặt toàn bộ thư viện cần thiết
- Tùy chọn cu128 nghĩa là cài PyTorch với hỗ trợ CUDA 12.8

### 2.3. Kiểm tra cài đặt

Chạy demo để xác nhận mọi thứ hoạt động.

Trên laptop (có màn hình):

```bash
uv run demo
```

Trên workstation (qua SSH, không có màn hình):

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab
MUJOCO_GL=egl uv run demo
```

---

## 3. Cài đặt vm_video2robot (GVHMR)

GVHMR cần môi trường riêng vì dùng nhiều thư viện nặng (PyTorch3D, v.v.).

### 3.1. Tạo môi trường Conda

Ta cần tạo một môi trường Python riêng để tránh xung đột thư viện:

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/vm_video2robot

conda create -y -n video2robot python=3.10
conda activate video2robot
```

### 3.2. Cài đặt PyTorch

Cài PyTorch với hỗ trợ CUDA. Phiên bản nightly đảm bảo tương thích với GPU mới nhất:

```bash
pip install --pre torch torchvision torchaudio --index-url https://download.pytorch.org/whl/nightly/cu128
```

### 3.3. Cài đặt các thư viện phụ thuộc

```bash
pip install ninja setuptools cmake
pip install -r requirements.txt
```

### 3.4. Cài đặt PyTorch3D

PyTorch3D là thư viện xử lý 3D, cần biên dịch từ mã nguồn:

```bash
cd pytorch3d/
pip install -e . --no-build-isolation
cd ..
```

### 3.5. Cài đặt các gói còn lại

```bash
pip install chumpy --no-build-isolation
pip install -e .
pip install yacs
```

### 3.6. Tải các model checkpoint

GVHMR cần nhiều model đã huấn luyện sẵn. Bạn cần tải chúng về thủ công.

**Model SMPL/SMPL-X:** (cần đăng ký tài khoản)
- Đăng ký tại https://smpl.is.tue.mpg.de/ và https://smpl-x.is.tue.mpg.de/
- Tải file và đặt vào đúng vị trí theo cấu trúc sau:

```
vm_video2robot/inputs/checkpoints/
├── body_models/smplx/
│   └── SMPLX_{GENDER}.npz
└── body_models/smpl/
    └── SMPL_{GENDER}.pkl
```

**Các model khác:** tải từ Google Drive theo link trong file hướng dẫn gốc, đặt vào:

```
vm_video2robot/inputs/checkpoints/
├── dpvo/
│   └── dpvo.pth
├── gvhmr/
│   └── gvhmr_siga24_release.ckpt
├── hmr2/
│   └── epoch=10-step=25000.ckpt
├── vitpose/
│   └── vitpose-h-multi-coco.pth
└── yolo/
    └── yolov8x.pt
```

### 3.7. Kiểm tra GVHMR

Chạy demo với video mẫu có sẵn. Nếu không có lỗi, nghĩa là cài đặt thành công:

```bash
conda activate video2robot
python tools/demo/demo.py --video=docs/example_video/tennis.mp4 -s
```

---

## 4. Thiết lập Weights & Biases (WandB)

WandB dùng để lưu trữ motion data và theo dõi quá trình huấn luyện.

### 4.1. Tạo tài khoản

Truy cập https://wandb.ai/ và đăng ký tài khoản (có thể dùng tài khoản GitHub).

### 4.2. Đăng nhập

Chạy lệnh đăng nhập, hệ thống sẽ yêu cầu nhập API key (lấy từ trang cá nhân WandB):

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab
uv run wandb login
```

### 4.3. Tạo Registry cho Motion

Theo hướng dẫn trong tài liệu mjlab, bạn cần tạo một WandB registry để lưu trữ các file motion. Thực hiện theo hướng dẫn của BeyondMimic:

1. Vào trang WandB của tổ chức
2. Tạo một registry mới tên "motions"
3. Đây sẽ là nơi lưu trữ tất cả file NPZ motion

---

## 5. Kiểm tra Tổng thể

Sau khi hoàn tất tất cả bước trên, chạy các lệnh kiểm tra sau:

### Kiểm tra mjlab

Liệt kê tất cả các task có sẵn. Nếu thấy các task chứa "M2v6" trong danh sách, nghĩa là mjlab đã được cấu hình đúng:

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab
uv run list_envs
```

### Kiểm tra GVHMR

Đảm bảo tất cả checkpoint đã có đủ:

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/vm_video2robot
ls inputs/checkpoints/gvhmr/
ls inputs/checkpoints/vitpose/
ls inputs/checkpoints/yolo/
```

---

## 6. Xử lý Lỗi Thường gặp

### Lỗi "CUDA out of memory"
- Giảm số lượng môi trường song song khi huấn luyện
- Ví dụ: dùng 2048 thay vì 4096

### Lỗi "No module named ..."
- Đảm bảo bạn đang ở đúng môi trường:
  - mjlab: dùng uv (tự quản lý)
  - GVHMR: dùng conda (môi trường "video2robot")

### Lỗi "MUJOCO_GL"
- Nếu không có màn hình, luôn thêm biến môi trường EGL trước lệnh:

```bash
MUJOCO_GL=egl uv run <lệnh>
```

### Lỗi khi biên dịch PyTorch3D
- Đảm bảo đã cài ninja và cmake
- Kiểm tra phiên bản CUDA khớp với PyTorch

---

## Tiếp theo

Sau khi cài đặt xong, hãy đọc tiếp:
1. [Cấu trúc Robot M2v6](03-cau-truc-robot-m2v6.md) — Hiểu chi tiết robot bạn sẽ làm việc
2. [Hướng dẫn Quay Video](04-huong-dan-quay-video.md) — Chuẩn bị video đầu vào
