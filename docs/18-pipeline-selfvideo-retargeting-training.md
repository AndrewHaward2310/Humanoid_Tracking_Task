# Pipeline: Video tự quay → Retargeting → Training (M2v6)

Hướng dẫn đầy đủ từ video tự quay đến motion NPZ sẵn sàng train — dùng robot **M2v6** với ràng buộc **tay không chạm đất** (wrist_pitch).

---

## Tổng quan quy trình

```
[Bước 1] Quay video tự quay
          │  (trên laptop/điện thoại)
          ▼
[Bước 2] Chạy GVHMR trích xuất tư thế 3D
          │  vm_video2robot/ → outputs/demo/<tên>/hmr4d_results.pt
          ▼
[Bước 3] Retargeting SMPL-X → M2v6 (wrist_pitch)
          │  vm_retargeting/ → output/<tên>.pkl + .npz
          ▼
[Bước 4] Kiểm tra visual bằng MuJoCo
          │  csv_to_npz_m2v6 --render
          ▼
[Bước 5] Copy NPZ lên server → Train
          │  ssh + scp → vm_mjlab/motions/
          ▼
[Bước 6] Theo dõi training trên WandB
```

---

## Bước 1: Quay video

> Xem chi tiết: [04-huong-dan-quay-video.md](04-huong-dan-quay-video.md)

**Checklist nhanh:**
- [ ] Quay chính diện hoặc chéo 45°, tránh quay sau lưng
- [ ] Toàn thân luôn trong khung hình
- [ ] Camera cố định (tripod hoặc đặt trên bàn) — bật cờ `-s` khi chạy GVHMR
- [ ] Đồng phục ôm sát, nền đơn sắc
- [ ] 1080p 30fps, thời lượng 3–10 giây
- [ ] Bắt đầu và kết thúc ở tư thế đứng thẳng

**Cắt video và đặt vào thư mục đầu vào:**

```bash
# Cắt phần cần thiết (tùy chọn)
ffmpeg -i original.mp4 -ss 00:00:02 -to 00:00:08 -c copy motion_clip.mp4

# Copy vào thư mục input GVHMR
cp motion_clip.mp4 ~/Documents/Humanoid_Tracking_Task/vm_video2robot/inputs/demo/
```

---

## Bước 2: Chạy GVHMR (Video → SMPL-X `.pt`)

```bash
cd ~/Documents/Humanoid_Tracking_Task/vm_video2robot

# Kích hoạt môi trường GVHMR
conda activate video2robot   # hoặc tên môi trường của bạn

# Chạy GVHMR — dùng -s vì camera cố định
python tools/demo/demo.py \
    --video=inputs/demo/motion_clip.mp4 \
    -s
```

**Output**: `outputs/demo/motion_clip/hmr4d_results.pt`

Kiểm tra kết quả:
```bash
ls outputs/demo/motion_clip/
# Phải thấy: hmr4d_results.pt, 0_input_video.mp4, 1_incam.mp4, 2_global.mp4
```

Xem video `2_global.mp4` để đánh giá chất lượng SMPL-X. Nếu tư thế bị lệch nhiều → quay lại video với điều kiện tốt hơn.

---

## Bước 3: Retargeting SMPL-X → M2v6 (với wrist_pitch)

### Tại sao dùng `m2_v6_wrist_pitch`?

Robot M2v6 có chuỗi khớp cổ tay: `wrist_yaw → wrist_pitch → wrist_roll`.

- Config gốc `m2_v6`: IK target là `wrist_roll_link` (terminal) → 3 khớp đều tham gia, dễ bị wrist_roll cực đoan
- Config mới `m2_v6_wrist_pitch`: IK target là `wrist_pitch_link` → chỉ yaw + pitch tham gia, wrist_roll giữ ở 0

Kết quả: cổ tay có **độ quay pitch** để giữ lòng bàn tay không hướng xuống đất, phòng tránh tay chạm đất khi robot cúi xuống.

```bash
cd ~/Documents/Humanoid_Tracking_Task/vm_retargeting

# Kích hoạt môi trường GMR
conda activate gmr

# Chạy retargeting
python scripts/gvhmr_to_robot.py \
    --gvhmr_pred_file ~/Documents/Humanoid_Tracking_Task/vm_video2robot/outputs/demo/motion_clip/hmr4d_results.pt \
    --robot m2_v6_wrist_pitch \
    --save_path output/motion_clip.pkl \
    --tgt_fps 30 \
    --output_fps 50

python scripts/gvhmr_to_robot.py \
    --gvhmr_pred_file ~/Documents/Humanoid_Tracking_Task/vm_video2robot/outputs/demo/lying_new_1/hmr4d_results.pt \
    --robot m2_v6 \
    --save_path output/lying_new_style.pkl \
    --tgt_fps 30 \
    --output_fps 50

cd /home/nguyenld12/Documents/Humanoid_Tracking_Task && python3 -c "
import numpy as np
d = np.load('vm_retargeting/output/lying_new_style.npz', allow_pickle=True)
out = {}
for k in d.files:
    v = d[k]
    if v.dtype == object:
        # Replace None/object with empty string array
        print(f'  Fixing {k}: dtype=object → empty string array')
        out[k] = np.array([], dtype=str)
    else:
        out[k] = v

np.savez('vm_retargeting/output/lying_new_style.npz', **out)
print('Done — saved with pickle-safe arrays')

# Verify
d2 = np.load('vm_retargeting/output/lying_new_style.npz', allow_pickle=False)
print('Verify load with allow_pickle=False: OK')
print('Keys:', sorted(d2.files))
"
```

**Giải thích args:**

| Arg | Giá trị | Lý do |
|-----|---------|-------|
| `--robot` | `m2_v6_wrist_pitch` | M2v6 với IK target wrist_pitch |
| `--tgt_fps` | `30` | FPS xử lý SMPL-X (khớp với fps video) |
| `--output_fps` | `50` | FPS output NPZ — phải là **50** để khớp mjlab |
| `--save_path` | `output/*.pkl` | Script tự tạo cả `.pkl` và `.npz` |

**Output:**
```
output/motion_clip.pkl
output/motion_clip.npz    ← dùng file này cho training
```

---

## Bước 4: Kiểm tra visual

Xem lại motion trên robot M2v6 trong MuJoCo để xác nhận:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

MUJOCO_GL=egl uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
    --input-file ~/Documents/Humanoid_Tracking_Task/mjlab/data/0505/csv_01/lying_down_standing_up_002.csv \
    --output-file ~/Documents/Humanoid_Tracking_Task/mjlab/data/0505/npz_01/lying_down_standing_up_002.npz \
    --input-fps 30 \
    --output-fps 50 \
    --render True
```

**Những thứ cần kiểm tra:**
- [ ] Tay không chạm đất (wrist_pitch hoạt động đúng)
- [ ] Robot không xuyên qua mặt đất
- [ ] Chuyển động mượt, không giật
- [ ] Tư thế đầu/cuối giống tư thế đứng thẳng

**Vấn đề thường gặp:**

| Triệu chứng | Nguyên nhân | Xử lý |
|---|---|---|
| Tay vẫn chạm đất | SMPL-X tay quá thấp | Tăng weight rotation cho `left_wrist_pitch_link` trong IK config |
| Robot xuyên đất | Offset z sai | Điều chỉnh `ground_height` trong `smplx_to_m26_wrist_pitch.json` |
| Chuyển động giật | Nhiễu GVHMR | Xem xét lọc smoothing sau retargeting |
| Cổ tay cứng đờ | wrist_pitch bị clamp | Kiểm tra giới hạn joint (`±1.658 rad`) |

---

## Bước 5: Copy NPZ lên server và chuẩn bị training

```bash
# Copy NPZ từ laptop lên workstation
scp ~/Documents/Humanoid_Tracking_Task/vm_retargeting/output/motion_clip.npz \
    nguyenl3@10.148.255.113:~/Documents/vm_mjlab/motions/

# SSH vào server để verify
ssh nguyenl3@10.148.255.113   # pass: 1
cd ~/Documents/vm_mjlab

# Kiểm tra file
python -c "
import numpy as np
d = np.load('motions/motion_clip.npz')
print('Keys:', list(d.keys()))
print('Frames:', d['dof_pos'].shape[0], '@ 50fps =', d['dof_pos'].shape[0]/50, 's')
print('DOF:', d['dof_pos'].shape[1], '(phải là 27)')
"
```

---

## Bước 6: Train trên server

> Xem chi tiết training: [13-full-pipeline-train-to-sim-deploy.md](13-full-pipeline-train-to-sim-deploy.md)

```bash
# Trên workstation (RTX 5090)
cd ~/Documents/vm_mjlab

# Train với motion mới, warm-start từ standup
uv run python -m mjlab.scripts.train \
    task=Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
    motion_file=motions/motion_clip.npz \
    num_envs=4096 \
    max_iterations=20000 \
    load_run=m26_standingup_normal   # warm-start để tránh diverge
```

**Monitor WandB:** https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab

---

## Tóm tắt files liên quan

| File | Mục đích |
|------|---------|
| `vm_video2robot/tools/demo/demo.py` | Chạy GVHMR trích xuất SMPL-X |
| `vm_retargeting/scripts/gvhmr_to_robot.py` | Retargeting SMPL-X → robot |
| `vm_retargeting/ik_configs/smplx_to_m26_wrist_pitch.json` | IK config M2v6 với target wrist_pitch |
| `vm_retargeting/ik_configs/smplx_to_m26.json` | IK config M2v6 gốc (target wrist_roll) |
| `mjlab/scripts/csv_to_npz_m2v6.py` | Kiểm tra + render motion |

---

## Tiếp theo

- [05-huong-dan-retargeting.md](05-huong-dan-retargeting.md) — Chi tiết kỹ thuật retargeting
- [13-full-pipeline-train-to-sim-deploy.md](13-full-pipeline-train-to-sim-deploy.md) — Full pipeline training → deploy sim
- [11-huong-dan-doc-wandb-dashboard.md](11-huong-dan-doc-wandb-dashboard.md) — Theo dõi WandB
