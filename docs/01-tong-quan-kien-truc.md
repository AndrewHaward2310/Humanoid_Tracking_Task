# Tổng quan Kiến trúc Hệ thống

Tài liệu này giải thích bức tranh tổng thể về cách huấn luyện một chuyển động mới (ví dụ: tư thế chào mừng) cho robot humanoid M2v6, từ đầu vào là video quay người thật cho đến khi robot học được chuyển động đó.

---

## 1. Bức tranh tổng thể

Toàn bộ quy trình có thể chia thành **4 giai đoạn chính**, mỗi giai đoạn sử dụng một công cụ riêng:

```
┌──────────────┐     ┌──────────────────┐     ┌──────────┐     ┌──────────────┐
│  Video quay  │     │  Trích xuất tư   │     │ Chuyển   │     │  Huấn luyện  │
│  người thật  │ ──> │  thế người 3D    │ ──> │ đổi sang │ ──> │  robot bắt   │
│              │     │  (GVHMR)         │     │ robot    │     │  chước       │
└──────────────┘     └──────────────────┘     └──────────┘     └──────────────┘
                      vm_video2robot           vm_retargeting       mjlab
```

### Mô tả từng giai đoạn

**Giai đoạn 1 — Quay video:** Ta quay một đoạn video người thật thực hiện động tác mong muốn (ví dụ: chào mừng). Đây là đầu vào duy nhất cần chuẩn bị.

**Giai đoạn 2 — Trích xuất tư thế 3D (vm_video2robot):** Dùng mô hình GVHMR để phân tích video và tái dựng lại chuyển động 3D của người trong video. Kết quả là một chuỗi các tư thế SMPL — một định dạng chuẩn mô tả cơ thể người bằng 24 khớp xương.

**Giai đoạn 3 — Chuyển đổi sang robot (vm_retargeting):** Cơ thể người có cấu trúc khác robot. Bước này "ánh xạ" các khớp của người sang các khớp tương ứng trên robot M2v6, tạo ra file CSV chứa góc quay của từng khớp robot qua mỗi khung hình.

**Giai đoạn 4 — Huấn luyện (mjlab):** File CSV được chuyển thành file NPZ (chứa thêm vận tốc và vị trí các bộ phận cơ thể). Sau đó, hệ thống dùng thuật toán PPO để huấn luyện robot trong mô phỏng MuJoCo, sao cho robot bắt chước được chuyển động tham chiếu.

---

## 2. Các repo và vai trò

Hệ thống gồm 3 repo chính, mỗi repo đảm nhận một phần trong quy trình:

### 2.1. vm_video2robot (GVHMR)

**Vai trò:** Trích xuất chuyển động 3D của người từ video.

**GVHMR là gì?**
- Viết tắt của "World-Grounded Human Motion Recovery via Gravity-View Coordinates"
- Là một mô hình AI (SIGGRAPH Asia 2024) có khả năng nhìn video 2D và "đoán" ra tư thế 3D của người
- Sử dụng hệ tọa độ Gravity-View, giúp kết quả ổn định và sát thực tế hơn các phương pháp cũ

**Cách hoạt động:**
1. Phát hiện người trong video (dùng YOLO)
2. Ước lượng tư thế 2D (dùng ViTPose)
3. Nâng cấp lên 3D (dùng mô hình HMR4D/GVHMR)
4. Ước lượng chuyển động camera (dùng SimpleVO)
5. Kết hợp tất cả để ra chuyển động 3D trong hệ tọa độ thế giới

**Đầu ra:** Chuỗi tư thế SMPL (SMPL-X) — mô tả cơ thể người bằng:
- Vị trí gốc (root position)
- Hướng quay gốc (root orientation)
- Góc quay 24 khớp xương

### 2.2. vm_retargeting

**Vai trò:** Chuyển đổi chuyển động từ mô hình người SMPL sang mô hình robot M2v6.

**Tại sao cần bước này?**
- Người có 24 khớp (SMPL), robot M2v6 chỉ có 26 khớp điều khiển được — nhưng cấu trúc hoàn toàn khác
- Tỉ lệ cơ thể khác: tay người dài hơn, chân robot có cấu trúc song song (parallel linkage)
- Một số khớp không có bản đồ 1-1: ví dụ bàn tay người có nhiều ngón, robot chỉ có cổ tay

**Đầu ra:** File CSV có cấu trúc:
- 3 cột đầu: vị trí gốc (x, y, z)
- 4 cột tiếp: hướng quay gốc (quaternion x, y, z, w)
- 26 cột còn lại: góc quay từng khớp robot

### 2.3. mjlab

**Vai trò:** Huấn luyện robot bắt chước chuyển động trong mô phỏng.

**mjlab là gì?**
- Là framework kết hợp API thiết kế môi trường của Isaac Lab với sức mạnh tính toán GPU của MuJoCo Warp
- Cho phép chạy hàng nghìn mô phỏng robot song song trên GPU
- Cung cấp các "khối xây dựng" sẵn có: quản lý cảnh, cảm biến, reward, observation, v.v.

**Các khái niệm quan trọng trong mjlab:**

| Khái niệm | Ý nghĩa |
|------------|---------|
| Scene | Bối cảnh mô phỏng gồm robot, mặt đất, và các đối tượng khác |
| Entity | Một đối tượng trong scene (ví dụ: robot M2v6) |
| Task | Định nghĩa bài toán: robot cần làm gì? đo lường thành công bằng cách nào? |
| Observation | Thông tin robot "cảm nhận" được: góc khớp, vận tốc, IMU, v.v. |
| Action | Hành động robot thực hiện: thay đổi góc khớp mục tiêu |
| Reward | Điểm thưởng/phạt giúp robot học hành vi đúng |
| Termination | Điều kiện kết thúc episode (ví dụ: robot ngã) |

---

## 3. Hiểu về MuJoCo và MuJoCo Warp

### MuJoCo
- Viết tắt của "Multi-Joint dynamics with Contact"
- Là phần mềm mô phỏng vật lý do DeepMind phát triển
- Chuyên mô phỏng robot: va chạm, ma sát, trọng lực, khớp nối, v.v.
- Mỗi robot được mô tả bằng file XML (MJCF format)

### MuJoCo Warp
- Phiên bản tăng tốc GPU của MuJoCo
- Cho phép chạy hàng nghìn mô phỏng song song cùng lúc
- Quan trọng cho huấn luyện RL vì cần rất nhiều dữ liệu mô phỏng

### So sánh với Isaac Lab
- Isaac Lab dùng PhysX (NVIDIA) làm engine vật lý
- mjlab dùng MuJoCo Warp — nhẹ hơn, ít phụ thuộc hơn
- mjlab "mượn" thiết kế API của Isaac Lab (manager-based) nhưng chạy trên MuJoCo

---

## 4. Cơ bản về Học tăng cường (RL)

Để robot học bắt chước chuyển động, ta dùng phương pháp **Học tăng cường** (Reinforcement Learning - RL).

### Ý tưởng cốt lõi
Hãy tưởng tượng bạn đang dạy một đứa trẻ nhảy múa: mỗi khi nó thực hiện đúng động tác, bạn khen (reward dương); khi sai, bạn chê (reward âm). Qua hàng triệu lần thử, nó dần học được cách nhảy đúng.

Robot cũng học tương tự:
1. Robot **quan sát** (observation): "tôi đang ở tư thế nào? chuyển động tham chiếu yêu cầu gì?"
2. Robot **hành động** (action): "tôi sẽ xoay các khớp như thế này"
3. Môi trường **đánh giá** (reward): "tay robot cách tay tham chiếu 5cm → trừ điểm"
4. Robot **cập nhật** chiến lược: "lần sau tôi sẽ xoay khớp vai thêm một chút"

### PPO (Proximal Policy Optimization)
- Là thuật toán RL phổ biến nhất cho điều khiển robot
- Hoạt động theo kiểu "on-policy": thu thập dữ liệu → học → thu thập lại
- Sử dụng hai mạng neural:
  - **Actor**: quyết định hành động (đầu vào: observation → đầu ra: action)
  - **Critic**: đánh giá tình huống hiện tại tốt hay xấu (giúp actor học tốt hơn)

### Tại sao cần chạy nhiều môi trường song song?
- PPO cần rất nhiều dữ liệu để học
- Thay vì chạy 1 robot trong 4096 bước, ta chạy 4096 robot trong 1 bước
- GPU rất giỏi xử lý song song → tốc độ huấn luyện tăng gấp hàng nghìn lần

---

## 5. Flow chi tiết cho task "Tư thế chào mừng M2v6"

Dưới đây là các bước cụ thể bạn sẽ thực hiện:

### Bước 1: Quay video
- Quay người thật thực hiện tư thế chào mừng
- Dùng camera tĩnh (tripod), ánh sáng đủ, phông nền đơn giản

### Bước 2: Chạy GVHMR
- Đưa video vào mô hình GVHMR để trích xuất tư thế 3D
- Kết quả: file chứa chuỗi tư thế SMPL

### Bước 3: Retargeting
- Chuyển đổi tư thế SMPL sang tư thế robot M2v6
- Kết quả: file CSV chứa góc quay các khớp robot qua từng khung hình

### Bước 4: Chuyển đổi CSV → NPZ
- Dùng script của mjlab để:
  - Nội suy tốc độ khung hình (ví dụ: từ 30fps lên 50fps)
  - Tính vận tốc tuyến tính và góc
  - Tính vị trí và hướng của từng bộ phận cơ thể robot (forward kinematics)
- Kết quả: file NPZ — định dạng mà mjlab dùng khi huấn luyện

### Bước 5: Huấn luyện
- Dùng mjlab để huấn luyện policy (mạng neural) cho robot
- Robot chạy trong 4096 môi trường mô phỏng song song
- Mỗi lần, robot cố gắng bắt chước chuyển động tham chiếu
- Reward dựa trên: sai số vị trí bộ phận cơ thể, sai số hướng quay, sai số vận tốc
- Quá trình huấn luyện được theo dõi trên Weights & Biases (WandB)

### Bước 6: Đánh giá
- Chạy policy đã huấn luyện để xem kết quả
- Xem trực quan trong trình xem MuJoCo
- Nếu chưa hài lòng: điều chỉnh reward, tham số, hoặc dữ liệu motion

### Bước 7: Triển khai (tùy chọn)
- Xuất policy sang định dạng ONNX
- Chạy trên robot thật

---

## 6. Cấu trúc thư mục

```
Humanoid_Tracking_Task/
├── mjlab/                     # Framework huấn luyện RL
│   ├── src/mjlab/
│   │   ├── asset_zoo/robots/M2v6/    # Định nghĩa robot M2v6
│   │   ├── tasks/tracking/           # Task motion imitation
│   │   │   ├── config/m2v6/          # Cấu hình riêng cho M2v6
│   │   │   └── mdp/                  # Reward, observation, termination
│   │   └── scripts/
│   │       ├── csv_to_npz_m2v6.py    # Chuyển đổi CSV → NPZ cho M2v6
│   │       ├── train.py              # Script huấn luyện
│   │       └── play.py               # Script đánh giá
│   └── pyproject.toml                # Quản lý phụ thuộc (dùng uv)
│
├── vm_video2robot/            # Trích xuất tư thế 3D từ video
│   ├── tools/demo/
│   │   ├── demo.py                   # Chạy GVHMR trên 1 video
│   │   └── demo_folder.py            # Chạy GVHMR trên cả thư mục
│   ├── hmr4d/                        # Mã nguồn chính GVHMR
│   └── docs/INSTALL.md              # Hướng dẫn cài đặt
│
├── vm_retargeting/            # Chuyển đổi motion người → robot
│
└── docs/                      # Tài liệu hướng dẫn (bạn đang đọc)
```

---

## 7. Thuật ngữ cần nhớ

| Thuật ngữ | Giải thích |
|-----------|-----------|
| SMPL/SMPL-X | Mô hình tham số hóa cơ thể người (24/55 khớp) |
| Retargeting | Chuyển đổi chuyển động từ một skeleton sang skeleton khác |
| NPZ | Định dạng lưu trữ của NumPy, chứa nhiều mảng dữ liệu |
| PPO | Thuật toán RL phổ biến cho điều khiển robot |
| Policy | Mạng neural quyết định hành động robot dựa trên observation |
| Episode | Một lần chạy mô phỏng từ đầu đến khi kết thúc |
| WandB | Weights & Biases — nền tảng theo dõi thí nghiệm ML |
| Forward Kinematics | Tính vị trí bộ phận cơ thể từ góc quay các khớp |
| Reward shaping | Thiết kế hàm reward để robot học hành vi mong muốn |
| Domain Randomization | Thay đổi ngẫu nhiên tham số mô phỏng để policy mạnh mẽ hơn |
| Decimation | Số bước mô phỏng vật lý cho mỗi bước điều khiển |
| ONNX | Định dạng xuất mô hình AI để triển khai trên nhiều nền tảng |

---

## Tiếp theo

Hãy đọc tiếp các tài liệu sau theo thứ tự:
1. [Thiết lập Môi trường](02-thiet-lap-moi-truong.md) — Cài đặt phần mềm cần thiết
2. [Cấu trúc Robot M2v6](03-cau-truc-robot-m2v6.md) — Hiểu chi tiết robot M2v6
3. [Hướng dẫn Quay Video](04-huong-dan-quay-video.md) — Chuẩn bị đầu vào
