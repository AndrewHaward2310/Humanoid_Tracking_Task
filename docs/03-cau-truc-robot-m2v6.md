# Cấu trúc Robot M2v6

Tài liệu này mô tả chi tiết cấu trúc cơ thể robot M2v6 — robot humanoid mà bạn sẽ huấn luyện thực hiện tư thế chào mừng.

---

## 1. Tổng quan

M2v6 là robot humanoid có cấu trúc tương tự người, gồm:
- **Hông (pelvis):** phần trung tâm, kết nối thân trên và thân dưới
- **2 chân:** mỗi chân có 6 khớp (hông, đầu gối, mắt cá)
- **Thân trên (torso):** kết nối với hông qua khớp eo
- **2 tay:** mỗi tay có 7 khớp (vai, khuỷu, cổ tay)

Robot đứng cao khoảng 1.1 mét, nặng khoảng 30kg. Toàn bộ robot được mô tả trong file XML theo định dạng MJCF (MuJoCo XML Format).

---

## 2. Hệ thống Khớp — 27 bậc tự do

M2v6 có tổng cộng **27 khớp điều khiển được**, chia theo 5 nhóm chính. Mỗi khớp chỉ xoay theo 1 trục (1 bậc tự do).

> Lưu ý: ngoài 27 khớp này, còn có 1 "freejoint" cho phần hông tự do di chuyển trong không gian 3D (6 bậc tự do: 3 tịnh tiến + 3 quay). Tổng cộng robot có 33 bậc tự do.

### 2.1. Chân trái (6 khớp)

| STT | Tên khớp | Trục quay | Chức năng | Lực tối đa (Nm) |
|-----|----------|-----------|-----------|-----------------|
| 1 | left_hip_pitch_joint | Y (trước/sau) | Đá chân trước/sau | 130 |
| 2 | left_hip_roll_joint | X (sang bên) | Dạng/khép chân | 330 |
| 3 | left_hip_yaw_joint | Z (xoay) | Xoay đùi | 70 |
| 4 | left_knee_joint | Y (trước/sau) | Gập/duỗi gối | 330 |
| 5 | left_ankle_pitch_joint | Y (trước/sau) | Gập/duỗi cổ chân | 60 |
| 6 | left_ankle_roll_joint | X (sang bên) | Nghiêng bàn chân | 60 |

### 2.2. Chân phải (6 khớp)

Cấu trúc giống chân trái (thay "left" bằng "right").

| STT | Tên khớp | Trục quay | Chức năng | Lực tối đa (Nm) |
|-----|----------|-----------|-----------|-----------------|
| 7 | right_hip_pitch_joint | Y | Đá chân trước/sau | 130 |
| 8 | right_hip_roll_joint | X | Dạng/khép chân | 330 |
| 9 | right_hip_yaw_joint | Z | Xoay đùi | 70 |
| 10 | right_knee_joint | Y | Gập/duỗi gối | 330 |
| 11 | right_ankle_pitch_joint | Y | Gập/duỗi cổ chân | 60 |
| 12 | right_ankle_roll_joint | X | Nghiêng bàn chân | 60 |

### 2.3. Eo (1 khớp)

| STT | Tên khớp | Trục quay | Chức năng | Lực tối đa (Nm) |
|-----|----------|-----------|-----------|-----------------|
| 13 | waist_joint | Z (xoay) | Xoay thân trên | 70 |

### 2.4. Tay trái (7 khớp)

| STT | Tên khớp | Trục quay | Chức năng | Lực tối đa (Nm) |
|-----|----------|-----------|-----------|-----------------|
| 14 | left_shoulder_pitch_joint | Y | Giơ/hạ tay | 90 |
| 15 | left_shoulder_roll_joint | X | Dạng/khép tay | 36 |
| 16 | left_shoulder_yaw_joint | Z | Xoay cánh tay | 36 |
| 17 | left_elbow_joint | Y | Gập/duỗi khuỷu | 36 |
| 18 | left_wrist_yaw_joint | Z | Xoay cổ tay | 36 |
| 19 | left_wrist_pitch_joint | Y | Gập/ngửa cổ tay | 36 |
| 20 | left_wrist_roll_joint | X | Lật cổ tay | 36 |

### 2.5. Tay phải (7 khớp)

Cấu trúc giống tay trái (thay "left" bằng "right").

| STT | Tên khớp | Trục quay | Chức năng | Lực tối đa (Nm) |
|-----|----------|-----------|-----------|-----------------|
| 21 | right_shoulder_pitch_joint | Y | Giơ/hạ tay | 90 |
| 22 | right_shoulder_roll_joint | X | Dạng/khép tay | 36 |
| 23 | right_shoulder_yaw_joint | Z | Xoay cánh tay | 36 |
| 24 | right_elbow_joint | Y | Gập/duỗi khuỷu | 36 |
| 25 | right_wrist_yaw_joint | Z | Xoay cổ tay | 36 |
| 26 | right_wrist_pitch_joint | Y | Gập/ngửa cổ tay | 36 |
| 27 | right_wrist_roll_joint | X | Lật cổ tay | 36 |

---

## 3. Sơ đồ Phân cấp Khớp

Dưới đây là cấu trúc cây khớp của M2v6, thể hiện mối quan hệ cha-con:

```
pelvis_link (hông) ─── freejoint [gốc tự do]
├── left_hip_pitch_link
│   └── left_hip_roll_link
│       └── left_hip_yaw_link
│           └── left_knee_link
│               └── left_ankle_pitch_link
│                   └── left_ankle_roll_link  ← bàn chân trái
│
├── right_hip_pitch_link
│   └── right_hip_roll_link
│       └── right_hip_yaw_link
│           └── right_knee_link
│               └── right_ankle_pitch_link
│                   └── right_ankle_roll_link  ← bàn chân phải
│
└── torso_link (thân trên) ─── waist_joint
    ├── left_shoulder_pitch_link
    │   └── left_shoulder_roll_link
    │       └── left_shoulder_yaw_link
    │           └── left_elbow_link
    │               └── left_wrist_yaw_link
    │                   └── left_wrist_pitch_link
    │                       └── left_wrist_roll_link  ← bàn tay trái
    │
    └── right_shoulder_pitch_link
        └── right_shoulder_roll_link
            └── right_shoulder_yaw_link
                └── right_elbow_link
                    └── right_wrist_yaw_link
                        └── right_wrist_pitch_link
                            └── right_wrist_roll_link  ← bàn tay phải
```

---

## 4. Nhóm Actuator (Bộ điều khiển)

Các khớp được nhóm lại thành 6 nhóm actuator, mỗi nhóm có cùng đặc tính cơ học:

| Nhóm | Các khớp | Lực tối đa | Đặc điểm |
|------|----------|------------|----------|
| Hip Pitch | hip_pitch (2 khớp) | 130 Nm | Chuyển động chính của chân |
| Hip Roll + Knee | hip_roll, knee (4 khớp) | 330 Nm | Mạnh nhất — cơ cấu song song |
| Hip Yaw + Waist | hip_yaw, waist (3 khớp) | 70 Nm | Xoay đùi và thân |
| Ankle | ankle_pitch, ankle_roll (4 khớp) | 60 Nm | Giữ thăng bằng bàn chân |
| Shoulder Pitch | shoulder_pitch (2 khớp) | 90 Nm | Giơ/hạ tay |
| Arm | shoulder_roll/yaw, elbow, wrist (12 khớp) | 36 Nm | Tay — lực nhẹ hơn |

### Bộ điều khiển PD

Tất cả khớp dùng bộ điều khiển vị trí PD (Proportional-Derivative):
- **Stiffness (Kp):** lực kéo khớp về vị trí mục tiêu — càng cao, robot càng "cứng"
- **Damping (Kd):** lực cản chuyển động — giúp tránh dao động
- **Tần số tự nhiên:** 3.4 Hz (tương đương chu kỳ ~0.3 giây)
- **Tỉ số cản:** 1.8 (quá cản — giúp chuyển động mượt, không rung)

Khi huấn luyện, policy (mạng neural) sẽ xuất ra **góc khớp mục tiêu**. Bộ điều khiển PD sẽ tính lực cần thiết để đưa khớp về vị trí mục tiêu đó.

---

## 5. Các Bộ phận Cơ thể được Theo dõi (Tracking Bodies)

Khi huấn luyện motion imitation, không phải tất cả bộ phận đều được theo dõi. Hệ thống chỉ so sánh **14 bộ phận chính** giữa robot và chuyển động tham chiếu:

| STT | Bộ phận | Vai trò trong tracking |
|-----|---------|----------------------|
| 1 | pelvis_link | Điểm neo chính (anchor) — vị trí và hướng gốc |
| 2 | left_hip_roll_link | Tracking đùi trái |
| 3 | left_knee_link | Tracking đầu gối trái |
| 4 | left_ankle_roll_link | Tracking bàn chân trái |
| 5 | right_hip_roll_link | Tracking đùi phải |
| 6 | right_knee_link | Tracking đầu gối phải |
| 7 | right_ankle_roll_link | Tracking bàn chân phải |
| 8 | torso_link | Tracking thân trên |
| 9 | left_shoulder_roll_link | Tracking vai trái |
| 10 | left_elbow_link | Tracking khuỷu tay trái |
| 11 | left_wrist_roll_link | Tracking bàn tay trái |
| 12 | right_shoulder_roll_link | Tracking vai phải |
| 13 | right_elbow_link | Tracking khuỷu tay phải |
| 14 | right_wrist_roll_link | Tracking bàn tay phải |

Với task chào mừng, các bộ phận tay (shoulder, elbow, wrist) đặc biệt quan trọng vì đó là nơi thể hiện động tác.

---

## 6. So sánh với Mô hình Người SMPL

Khi chuyển đổi chuyển động từ người sang robot (retargeting), cần hiểu sự khác biệt:

| Đặc điểm | SMPL (người) | M2v6 (robot) |
|-----------|-------------|-------------|
| Số khớp | 24 | 27 |
| Bàn tay | Nhiều ngón | Cố định (không ngón) |
| Cổ/đầu | Có (2 khớp) | Không điều khiển khi tracking |
| Xương sống | Nhiều đốt | 1 khớp eo (waist) |
| Cổ tay | 2 bậc tự do | 3 bậc tự do (yaw + pitch + roll) |
| Chân | Liên tục | Cơ cấu song song (parallel linkage) |
| Tỉ lệ cơ thể | Theo người thật | Khác — cần scaling |

Các khớp **không có** trên robot nhưng có trên SMPL sẽ bị bỏ qua trong quá trình retargeting. Ngược lại, robot M2v6 có thêm khớp cổ tay chi tiết hơn người.

---

## 7. Giới hạn Khớp

Mỗi khớp có giới hạn góc quay (đơn vị: radian). Đây là thông tin quan trọng khi thiết kế chuyển động — nếu motion tham chiếu yêu cầu góc vượt quá giới hạn, robot sẽ không thể thực hiện.

| Khớp | Giới hạn dưới (rad) | Giới hạn trên (rad) | Giới hạn dưới (độ) | Giới hạn trên (độ) |
|------|-------|-------|-------|-------|
| hip_pitch | -2.71 | 2.87 | -155° | 164° |
| hip_roll (trái) | -0.52 | 2.97 | -30° | 170° |
| hip_yaw | -2.88 | 2.88 | -165° | 165° |
| knee | -0.09 | 2.88 | -5° | 165° |
| ankle_pitch | -0.87 | 0.52 | -50° | 30° |
| ankle_roll | -0.26 | 0.26 | -15° | 15° |
| waist | -2.44 | 2.44 | -140° | 140° |
| shoulder_pitch | -3.14 | 2.44 | -180° | 140° |
| shoulder_roll (trái) | -0.21 | 3.18 | -12° | 182° |
| shoulder_yaw | -2.79 | 2.79 | -160° | 160° |
| elbow | -2.44 | 1.57 | -140° | 90° |
| wrist_yaw | -2.44 | 2.44 | -140° | 140° |
| wrist_pitch | -1.66 | 1.66 | -95° | 95° |
| wrist_roll | -1.92 | 1.92 | -110° | 110° |

> Lưu ý: hip_roll và shoulder_roll của trái và phải có giới hạn đối xứng ngược nhau.

---

## 8. File Cấu hình Quan trọng

Khi làm việc với M2v6, bạn sẽ thường xuyên tham khảo các file sau:

| File | Nội dung |
|------|---------|
| `mjlab/src/mjlab/asset_zoo/robots/M2v6/M2v6.xml` | Mô hình MJCF — định nghĩa hình học, khớp, va chạm |
| `mjlab/src/mjlab/asset_zoo/robots/M2v6/m26_constants.py` | Hằng số actuator, keyframe mặc định |
| `mjlab/src/mjlab/tasks/tracking/config/m2v6/env_cfgs.py` | Cấu hình môi trường tracking cho M2v6 |
| `mjlab/src/mjlab/tasks/tracking/config/m2v6/rl_cfg.py` | Cấu hình thuật toán PPO |
| `mjlab/src/mjlab/scripts/csv_to_npz_m2v6.py` | Chuyển đổi CSV sang NPZ |

---

## 9. Xem Robot trong MuJoCo Viewer

Bạn có thể mở trình xem 3D để xem robot M2v6 trực tiếp. Chạy lệnh sau từ thư mục mjlab:

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab
uv run python -m mjlab.asset_zoo.robots.M2v6.m26_constants
```

Trong trình xem, bạn có thể:
- Kéo chuột để xoay góc nhìn
- Cuộn để zoom
- Nhấp đúp vào khớp để xem thông tin chi tiết

---

## Tiếp theo

Hãy đọc tiếp:
1. [Hướng dẫn Quay Video](04-huong-dan-quay-video.md) — Chuẩn bị video đầu vào
2. [Hướng dẫn Retargeting](05-huong-dan-retargeting.md) — Chuyển đổi motion người sang robot
