# Hướng dẫn Motion Retargeting

Sau khi trích xuất tư thế 3D từ video (GVHMR), ta cần chuyển đổi chuyển động đó từ mô hình người (SMPL) sang mô hình robot M2v6. Quá trình này gọi là **Motion Retargeting**.

---

## 1. Tại sao cần Retargeting?

### Vấn đề
Mô hình SMPL mô tả cơ thể người với 24 khớp — nhưng robot M2v6 có cấu trúc hoàn toàn khác:
- Số khớp khác (24 vs 27)
- Tỉ lệ cơ thể khác (chiều dài tay, chân, v.v.)
- Một số khớp không có đối ứng (ví dụ: ngón tay người vs tay cố định robot)
- Giới hạn góc quay khác nhau

### Giải pháp
Retargeting giải quyết bằng cách:
1. Xác định **ánh xạ** giữa khớp người và khớp robot
2. **Chuyển đổi** góc quay, có tính đến tỉ lệ cơ thể
3. **Cắt gọn** các giá trị vượt quá giới hạn khớp robot
4. **Làm mượt** kết quả để tránh chuyển động giật

---

## 2. Ánh xạ Khớp SMPL → M2v6

Dưới đây là bảng ánh xạ giữa các khớp chính:

| Khớp SMPL (người) | Khớp M2v6 (robot) | Ghi chú |
|-------------------|-------------------|---------|
| Left Hip | left_hip_pitch/roll/yaw | Tách thành 3 khớp |
| Left Knee | left_knee_joint | Ánh xạ trực tiếp |
| Left Ankle | left_ankle_pitch/roll | Tách thành 2 khớp |
| Right Hip | right_hip_pitch/roll/yaw | Tách thành 3 khớp |
| Right Knee | right_knee_joint | Ánh xạ trực tiếp |
| Right Ankle | right_ankle_pitch/roll | Tách thành 2 khớp |
| Spine (nhiều đốt) | waist_joint | Gộp lại thành 1 khớp |
| Left Shoulder | left_shoulder_pitch/roll/yaw | Tách thành 3 khớp |
| Left Elbow | left_elbow_joint | Ánh xạ trực tiếp |
| Left Wrist | left_wrist_yaw/pitch/roll | Tách thành 3 khớp |
| Right Shoulder | right_shoulder_pitch/roll/yaw | Tách thành 3 khớp |
| Right Elbow | right_elbow_joint | Ánh xạ trực tiếp |
| Right Wrist | right_wrist_yaw/pitch/roll | Tách thành 3 khớp |
| Head/Neck | (không sử dụng) | Robot có đầu nhưng không tracking |
| Fingers | (không có) | Robot tay cố định |

### Cách chuyển đổi hướng quay

SMPL dùng **axis-angle** để biểu diễn hướng quay của mỗi khớp (1 vector 3D mô tả trục quay và góc quay). Quá trình retargeting phân tách vector này thành các góc quay riêng lẻ theo trục X, Y, Z — tương ứng với các khớp roll, pitch, yaw trên robot.

---

## 3. Xử lý Khớp Đặc biệt

### Xương sống → Eo
- Người có nhiều đốt xương sống (Spine1, Spine2, Spine3 trong SMPL)
- Robot chỉ có 1 khớp eo (waist_joint) xoay quanh trục Z
- Giải pháp: tổng hợp góc xoay yaw từ tất cả đốt sống

### Bàn tay
- Người có ngón tay với nhiều bậc tự do
- Robot M2v6 có tay cố định (không gập ngón)
- Bỏ qua toàn bộ chuyển động ngón tay

### Giới hạn góc quay
- Sau khi chuyển đổi, cần kiểm tra xem góc quay có nằm trong giới hạn của robot không
- Nếu vượt quá, cắt gọn (clamp) về giá trị gần nhất trong phạm vi cho phép
- Ví dụ: nếu tính ra ankle_roll = 0.35 rad nhưng giới hạn là ±0.26 rad → cắt về 0.26 rad

---

## 4. Định dạng File CSV Đầu ra

File CSV sau retargeting có cấu trúc mỗi dòng là một khung hình, mỗi cột là một giá trị:

```
Cột 0-2:   Vị trí gốc (x, y, z)               — mét
Cột 3-6:   Hướng quay gốc (qx, qy, qz, qw)   — quaternion (xyzw)
Cột 7-33:  Góc quay 27 khớp                    — radian
```

Thứ tự 27 khớp trong file CSV theo quy ước Unitree:

```
 0: left_hip_pitch_joint
 1: left_hip_roll_joint
 2: left_hip_yaw_joint
 3: left_knee_joint
 4: left_ankle_pitch_joint
 5: left_ankle_roll_joint
 6: right_hip_pitch_joint
 7: right_hip_roll_joint
 8: right_hip_yaw_joint
 9: right_knee_joint
10: right_ankle_pitch_joint
11: right_ankle_roll_joint
12: waist_joint
13: left_shoulder_pitch_joint
14: left_shoulder_roll_joint
15: left_shoulder_yaw_joint
16: left_elbow_joint
17: left_wrist_yaw_joint
18: left_wrist_pitch_joint
19: left_wrist_roll_joint
20: right_shoulder_pitch_joint
21: right_shoulder_roll_joint
22: right_shoulder_yaw_joint
23: right_elbow_joint
24: right_wrist_yaw_joint
25: right_wrist_pitch_joint
26: right_wrist_roll_joint
```

> Lưu ý quan trọng: quaternion trong file CSV theo thứ tự (x, y, z, w) — khác với MuJoCo dùng (w, x, y, z). Script chuyển đổi CSV → NPZ sẽ tự động hoán vị.

---

## 5. Kiểm tra Kết quả Retargeting

Sau khi có file CSV, cần kiểm tra xem chuyển động có hợp lý không trước khi tiến hành huấn luyện.

### Kiểm tra bằng mắt

Dùng script chuyển đổi của mjlab với tùy chọn render để xem trực quan. Script này sẽ phát lại motion trên robot trong mô phỏng MuJoCo:

```bash
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab

MUJOCO_GL=egl uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
  --input-file /duong/dan/toi/file/chao_mung.csv \
  --output-file motions/chao_mung.npz \
  --input-fps 30 \
  --output-fps 50 \
  --render True
```

Lệnh này vừa chuyển đổi, vừa tạo video để kiểm tra.

### Các vấn đề thường gặp

| Vấn đề | Nguyên nhân | Giải pháp |
|--------|-------------|-----------|
| Robot xuyên qua mặt đất | Vị trí gốc z quá thấp | Điều chỉnh offset z trong retargeting |
| Tay/chân giật | Nhiễu trong trích xuất GVHMR | Áp dụng bộ lọc làm mượt |
| Chân quay ngược | Sai ánh xạ hướng quay | Kiểm tra dấu góc hip_roll |
| Robot co rúm | Giới hạn khớp quá chặt | Kiểm tra giá trị clamp |

---

## 6. Quy trình Tóm tắt

```
Video gốc
    │
    ▼
GVHMR trích xuất tư thế SMPL
    │
    ▼
Retargeting: SMPL → M2v6 (CSV)
    │
    ▼
Kiểm tra bằng mắt (csv_to_npz_m2v6 --render)
    │
    ├── Tốt → Sang bước huấn luyện
    └── Chưa tốt → Điều chỉnh retargeting / quay lại video
```

---

## Tiếp theo

Hãy đọc tiếp:
1. [Script chuyển đổi CSV → NPZ](../scripts/01_chuyen_csv_sang_npz.sh) — Chuyển đổi và chuẩn bị cho huấn luyện
2. [Cơ bản huấn luyện RL](06-co-ban-huan-luyen-rl.md) — Hiểu thuật toán huấn luyện
