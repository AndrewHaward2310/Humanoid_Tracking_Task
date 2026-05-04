# Xử Lý MoCap CSV Data → Train M2v6

> Hướng dẫn xử lý dataset CSV mới nhận (đã retarget cho M2v6) thành NPZ sẵn sàng train.

---

## Mục lục

1. [Hiểu format dataset](#1-hiểu-format-dataset)
2. [Workflow tổng quan](#2-workflow-tổng-quan)
3. [Bước 1 — Inspect dataset](#3-bước-1--inspect-dataset)
4. [Bước 2 — Convert CSV → NPZ](#4-bước-2--convert-csv--npz)
5. [Bước 3 — Validate NPZ](#5-bước-3--validate-npz)
6. [Bước 4 — Add metadata cho motion editor](#6-bước-4--add-metadata-cho-motion-editor)
7. [Bước 5 — Cleanup & filter](#7-bước-5--cleanup--filter)
8. [Bước 6 — Quyết định strategy training](#8-bước-6--quyết-định-strategy-training)
9. [Bước 7 — Train](#9-bước-7--train)
10. [Checklist](#10-checklist)

---

## 1. Hiểu format dataset

### Cấu trúc CSV

Mỗi file `.csv` là **1 motion clip**.

### 📐 Cấu trúc N rows × 34 cols

#### Rows = số frames (timesteps)

Mỗi row là 1 snapshot pose tại 1 thời điểm.

| Tham số | Ví dụ (file 742 rows) | Ý nghĩa |
|---|---|---|
| Rows | 742 | Số mẫu motion |
| Input FPS (MoCap) | 30 fps | Tốc độ ghi gốc |
| **Duration** | **742 / 30 ≈ 24.7 giây** | Độ dài motion thực tế |
| Output FPS (sau convert) | 50 fps | Tốc độ train mjlab |
| **Output frames** | **~1237 frames** (24.7 × 50) | Sau interpolate FPS |

#### Cols = state pose đầy đủ M2v6 (34 = 3 + 4 + 27)

```
┌──────────────┬───────────────┬───────────────────────────────┐
│ Cols 0-2     │ Cols 3-6      │ Cols 7-33                     │
│  base_pos    │  base_quat    │  joint_pos (27 DOF)           │
│  (x, y, z)   │  (x, y, z, w) │  góc khớp theo thứ tự M2v6    │
│   3 cols     │   4 cols      │      27 cols                  │
└──────────────┴───────────────┴───────────────────────────────┘
```

#### Chi tiết 27 joints (cols 7-33)

| Col | Joint | Body |
|---|---|---|
| 7-12 | hip_pitch_L, hip_roll_L, hip_yaw_L, knee_L, ankle_pitch_L, ankle_roll_L | Chân trái (6) |
| 13-18 | hip_pitch_R, hip_roll_R, hip_yaw_R, knee_R, ankle_pitch_R, ankle_roll_R | Chân phải (6) |
| 19 | waist | Thắt lưng (1) |
| 20-26 | sho_pitch_L, sho_roll_L, sho_yaw_L, elbow_L, wrist_yaw_L, wrist_pitch_L, wrist_roll_L | Tay trái (7) |
| 27-33 | sho_pitch_R, sho_roll_R, sho_yaw_R, elbow_R, wrist_yaw_R, wrist_pitch_R, wrist_roll_R | Tay phải (7) |

**Tổng: 6 + 6 + 1 + 7 + 7 = 27 DOF** (M2v6).

### 📖 Ví dụ đọc 1 row cụ thể

Frame 0 của file `stand_up_lying_R_002__A472.csv`:

```
-0.004, 0.071, 0.062,                       ← base_pos = (~0, 0.07, 0.06)
-0.483, -0.456, -0.503, 0.553,              ← base_quat (xyzw)
-0.153, 0.112, 0.232, 0.106, 0.375, ...     ← 27 joint_pos (rad)
```

**Đọc giá trị:**
- `base_pos.z = 0.062 m` → pelvis chỉ cao 6 cm → **robot đang nằm trên sàn**
- `base_quat = [-0.48, -0.46, -0.50, 0.55]` (xyzw) → xoay ~90-120° từ vertical → **nằm ngang**
- Sau ~25s (frame 742) sẽ thành `base_pos.z ≈ 0.9 m`, `base_quat ≈ [0,0,0,1]` → **đứng dậy**

### 🔄 Sau convert NPZ chứa gì

Script `csv_to_npz_m2v6.py` xử lý:
1. **Load** 34 cols → split thành `base_pos_w`, `base_quat_w`, `joint_pos`
2. **Reorder quat** từ `xyzw` (CSV) → `wxyz` (mjlab convention)
3. **Interpolate** 30fps → 50fps (output ~1237 frames cho file 742 rows)
4. **Compute velocities** (joint_vel, body_lin_vel_w, body_ang_vel_w) bằng finite difference
5. **Forward Kinematics** → tính `body_pos_w` và `body_quat_w` cho **30 bodies** (pelvis + 29 links)

NPZ output keys:

```
joint_pos       (T, 27)             ← từ CSV col 7-33
joint_vel       (T, 27)             ← finite diff
base_pos_w      (T, 3)              ← từ CSV col 0-2
base_quat_w     (T, 4)              ← từ CSV col 3-6 (reorder wxyz)
body_pos_w      (T, 30, 3)          ← FK compute
body_quat_w     (T, 30, 4)          ← FK compute
body_lin_vel_w  (T, 30, 3)          ← finite diff
body_ang_vel_w  (T, 30, 3)          ← finite diff
fps             [50.0]
```

(`T` = output frames sau interpolate ≈ rows × output_fps / input_fps)

→ Format chuẩn mjlab tracking, sẵn sàng train.

### Quy ước đặt tên file

```
[prefix]__A[number]_[suffix].csv
```

Phân tích pattern dataset hiện có:

**Folder `data/231010/` (32 files):**

| Prefix | Mô tả | Số take |
|---|---|---|
| `stand_up_lying_R_002` | Đứng dậy từ tư thế nằm ngửa | 4 takes (A472-A475) |
| `stand_up_lying_side_R_002` | Đứng dậy từ nằm nghiêng | 4 takes |
| `stand_up_lying_stomach_R_002` | Đứng dậy từ nằm sấp | 4 takes |
| `faint_stand_up_lying_puke_walk_ff_180_R_001` | Ngất → nôn → đi xoay 180° | 4 takes |

**Folder `data/231024/` (24 files):**

| Prefix | Mô tả |
|---|---|
| `faint_stand_up_lying_R_001` | Ngất rồi đứng dậy (nằm ngửa) |
| `faint_stand_up_lying_side_R_001` | Ngất rồi đứng dậy (nằm nghiêng) |
| `faint_stand_up_lying_stomach_R_001` | Ngất rồi đứng dậy (nằm sấp) |

### Suffix đặc biệt

| Suffix | Nghĩa |
|---|---|
| `A472`, `A473`, ... | Take number / actor ID |
| `_M` | **Mirror** — bản đối xứng trái-phải (data augmentation) |

→ Mỗi base motion có **8 versions**: 4 takes × 2 (orig + mirror).

### Frame rate

Mặc định MoCap thường **30 fps**. Cần verify với team data:

```bash
F=~/Documents/Humanoid_Tracking_Task/mjlab/data/231010/stand_up_lying_R_002__A472.csv
ROWS=$(wc -l < $F)
echo "Frames: $ROWS"
# Nếu MoCap 30fps → duration = $ROWS / 30 sec
# Nếu MoCap 50fps → duration = $ROWS / 50 sec
```

---

## 2. Workflow tổng quan

```
CSV (đã retarget M2v6) ──► [csv_to_npz_m2v6.py] ──► NPZ (motion data)
                                                       │
                                                       ▼
                                       [npz_add_base_fields.py] (optional)
                                                       │
                                                       ▼ NPZ + joint_names
                                                       │
                              ┌────────────────────────┼────────────────────────┐
                              ▼                        ▼                        ▼
                        [Single train]          [Multi-motion train]     [Motion editor]
                        1 NPZ → 1 policy        N NPZ → 1 policy         Edit → FK rebuild
```

---

## 3. Bước 1 — Inspect dataset

### Đếm + check format

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab/data

# Đếm files
ls 231010/*.csv | wc -l
ls 231024/*.csv | wc -l

# Check 1 file
F=231010/stand_up_lying_R_002__A472.csv
echo "Rows:  $(wc -l < $F)"
echo "Cols:  $(head -1 $F | tr ',' '\n' | wc -l)"
echo "Sample:"
head -3 $F | cut -c 1-100
```

Format đúng: **34 cols** (M2v6).

### Categorize files

```bash
# Nhóm theo prefix (loại bỏ A### và _M)
ls 231010/*.csv | xargs -n1 basename | sed 's/__A[0-9]*\(_M\)*\.csv//' | sort | uniq -c
```

---

## 4. Bước 2 — Convert CSV → NPZ

### 4.1 Tool: `mjlab/src/mjlab/scripts/csv_to_npz_m2v6.py` (có sẵn)

mjlab đã có sẵn script convert cho M2v6. Args:
- `--input-file <csv>` — path đến CSV
- `--output-file <npz>` — path đầy đủ đến NPZ output
- `--input-fps 30` (default)
- `--output-fps 50` (default — match training)
- `--render` (optional, render video MP4)
- `--line-range (start, end)` (optional, crop frames)

⚠️ Script **không có batch mode** — cần loop bash để convert nhiều file.

### 4.2 Setup môi trường

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
# Verify mjlab + M2v6 config import được
uv run python -c "from mjlab.tasks.tracking.config.m2v6.env_cfgs import m2v6_flat_tracking_env_cfg; print('OK')"
```

### 4.3 Convert 1 file (test)

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
  --input-file ~/Documents/Humanoid_Tracking_Task/mjlab/data/231010/stand_up_lying_R_002__A472.csv \
  --output-file ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010/stand_up_lying_R_002__A472.npz \
  --input-fps 30 \
  --output-fps 50
```

**Output:** `data/npz/231010/stand_up_lying_R_002__A472.npz`

NPZ chứa các keys chuẩn: `joint_pos`, `joint_vel`, `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w`, `fps`.

### 4.4 Batch convert toàn folder (bash loop)

Vì script không có batch mode, dùng bash loop:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

# Helper function — convert 1 folder
batch_convert() {
  local SRC_DIR="$1"
  local DST_DIR="$2"
  local FPS="${3:-30}"

  mkdir -p "$DST_DIR"
  for f in "$SRC_DIR"/*.csv; do
    NAME=$(basename "$f" .csv)
    OUT="$DST_DIR/${NAME}.npz"
    if [ -f "$OUT" ]; then
      echo "  ⏭  $NAME  (đã có, skip)"
      continue
    fi
    echo "  ▶  $NAME"
    uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
      --input-file "$f" --output-file "$OUT" \
      --input-fps "$FPS" --output-fps 50 2>&1 | tail -2
  done
}

# Convert 2 folder
batch_convert \
  ~/Documents/Humanoid_Tracking_Task/mjlab/data/231010 \
  ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010 \
  30

batch_convert \
  ~/Documents/Humanoid_Tracking_Task/mjlab/data/231024 \
  ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231024 \
  30
```

Output structure:

```
data/npz/231010/
├── stand_up_lying_R_002__A472.npz
├── stand_up_lying_R_002__A472_M.npz
├── stand_up_lying_R_002__A473.npz
└── ... (32 files NPZ)
```

### 4.5 Lưu ý input_fps

Nếu MoCap không phải 30fps → input_fps sai → motion sẽ play lệch tốc độ. Verify:

```bash
# Test convert với fps khác nhau, preview xem cái nào trông tự nhiên
# Hoặc hỏi team data chính xác fps gốc
```

---

## 5. Bước 3 — Validate NPZ

### 5.1 Check stats từng file

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python << 'EOF'
import numpy as np
from pathlib import Path

npz_dir = Path.home() / "Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010"

print(f"{'name':<50} {'frames':>7} {'dur':>7} {'max_jp':>8}")
for npz in sorted(npz_dir.glob("*.npz")):
    data = np.load(npz)
    jp = data['joint_pos']
    fps = float(np.asarray(data['fps']).flat[0])
    print(f"{npz.stem:<50} {jp.shape[0]:>7} {jp.shape[0]/fps:>6.1f}s {np.abs(np.rad2deg(jp)).max():>6.1f}°")
EOF
```

Cờ đỏ:
- `frames` quá ngắn (<100) → motion lỗi hoặc CSV trống
- `max_jp > 180°` → joint outlier, cần check
- `dur > 30s` → motion dài bất thường (combo, nên split)

### 5.2 Preview motion trong MuJoCo

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python scripts/play_motion.py \
  --npz ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010/stand_up_lying_R_002__A472.npz \
  --loop
```

Kiểm tra:
- [ ] Motion mượt, không giật cục
- [ ] Robot không bay lên trời / chìm dưới sàn
- [ ] Hình dáng motion match expectation (lying → sit → standup)
- [ ] Không có frame NaN

---

## 6. Bước 4 — Add metadata cho motion editor

NPZ output từ `csv_to_npz_m2v6.py` **không có** `joint_names` / `body_names` → motion editor sẽ không hiển thị joints (xem doc M2V6_TRAINING_GUIDE Section 3 — "Format NPZ — 2 dạng phổ biến").

Fix bằng `npz_add_base_fields.py`:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

# Process 1 file
uv run python scripts/npz_add_base_fields.py \
  --npz ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010/stand_up_lying_R_002__A472.npz \
  --out ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010/stand_up_lying_R_002__A472_editor.npz

# Batch all (thêm suffix _editor cho file output)
for f in ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010/*.npz; do
  # Skip file đã có suffix _editor
  case "$f" in *_editor.npz) continue;; esac
  OUT="${f%.npz}_editor.npz"
  if [ ! -f "$OUT" ]; then
    uv run python scripts/npz_add_base_fields.py --npz "$f" --out "$OUT" 2>&1 | tail -1
  fi
done
```

---

## 7. Bước 5 — Cleanup & filter

### 7.1 Loại file kém chất lượng

Sau preview manual, tạo file blacklist:

```bash
# Tạo file blacklist
cat > /tmp/blacklist.txt << 'EOF'
stand_up_lying_R_002__A472
faint_stand_up_lying_puke_walk_ff_180_R_001__A475_M
EOF

# Move blacklist sang folder _excluded
mkdir -p ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/_excluded
while read name; do
  mv ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010/$name \
     ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/_excluded/ 2>/dev/null && \
     echo "Moved: $name"
done < /tmp/blacklist.txt
```

### 7.2 Smooth motion (nếu giật)

```bash
# Tham khảo script có sẵn
ls ~/Documents/Humanoid_Tracking_Task/vm_retargeting/scripts/sub/smooth_kick_csv.py
```

### 7.3 Crop frames không cần thiết

Nếu motion có frame đầu/cuối chỉ đứng yên không cần thiết:

```bash
ls ~/Documents/Humanoid_Tracking_Task/vm_retargeting/scripts/crop_npz_frames.py
```

### 7.4 Fix initial pose mismatch (nếu deploy lying-style)

Nếu deploy lên robot từ Standing → motion frame 0 lệch → **prepend warmup frames**:

```bash
uv run python scripts/prepend_warmup_frames.py \
  --npz <input>.npz \
  --out <output>.npz \
  --warmup-frames 60
```

(Xem M2V6_TRAINING_GUIDE Section 14 — "Robot ngã ngay khi bắt đầu motion".)

---

## 8. Bước 6 — Quyết định strategy training

Có 2 chiến lược chính:

### Strategy A — Single motion training (truyền thống)

Train **1 policy cho 1 motion** (như standup hiện tại).

**Pros:** Tracking sát motion, reward cao
**Cons:** Policy chỉ làm được 1 motion

```bash
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 1024 \
  --env.commands.motion.motion-file <path>/motion.npz \
  --agent.experiment-name "m2v6_<motion>" \
  --agent.max-iterations 25000
```

→ Cần chọn 1 motion đại diện hoặc merge nhiều motion thành 1 sequence.

### Strategy B — Multi-motion training (khuyến nghị cho dataset này)

Train **1 policy có thể làm nhiều motion**, sample random motion mỗi episode.

**Pros:** 
- Policy generalize tốt — học variation từ nhiều take + mirror
- Robust với initial pose khác nhau

**Cons:** Reward thấp hơn từng motion riêng

Task: `Mjlab-Tracking-Flat-M2v6-Multi` (cần check đăng ký chưa — hiện task list chỉ có Single).

```bash
# Nếu task multi đã có
uv run train Mjlab-Tracking-Flat-M2v6-Multi \
  --motion-dir ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010 \
  --env.scene.num-envs 4096 \
  --agent.experiment-name "m2v6_standup_lying_multi" \
  --agent.max-iterations 30000
```

> ⚠️ Task `Mjlab-Tracking-Flat-M2v6-Multi` **có thể chưa đăng ký** trong mjlab. Check:
>
> ```bash
> grep -r "M2v6-Multi" ~/Documents/Humanoid_Tracking_Task/mjlab/src/
> ```
>
> Nếu chưa có, cần đăng ký bằng cách clone từ `Mjlab-Tracking-Flat-M23-Multi` (đã có) → đổi env_cfg M23 → M26. Tham khảo `mjlab/src/mjlab/tasks/tracking/config/m23/__init__.py`.

### Strategy C — Curriculum (nâng cao)

1. Pretrain trên 1 motion easy (ví dụ `stand_up_lying_R_002__A472` — nằm ngửa, đơn giản nhất)
2. Finetune với multi-motion từ checkpoint pretrain

---

## 9. Bước 7 — Train

### Local (laptop, 1024 envs, ~5h cho 25k iter)

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

# Single motion (chọn 1 motion test trước)
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 1024 \
  --env.commands.motion.motion-file \
    ~/Documents/Humanoid_Tracking_Task/mjlab/data/npz/231010/stand_up_lying_R_002__A472/motion.npz \
  --agent.experiment-name "m2v6_lying_R002_A472" \
  --agent.max-iterations 25000 \
  --agent.logger wandb
```

### Server (4096 envs, ~2-3h)

Sửa `prep_motion_v1_scratch_server.sh` template — đổi `LOCAL_MOTION` + `EXP_NAME`. Xem M2V6_TRAINING_GUIDE Section 7.

### Theo dõi & deploy

Theo flow chuẩn ở M2V6_TRAINING_GUIDE:

```bash
# Check peak
uv run python scripts/check_reward_curve_local.py m2v6_lying_R002_A472

# Deploy với versioning
bash ~/Documents/vm_packages/vm_ctrl/scripts/deploy_with_version.sh \
  lyingdown v3 \
  logs/rsl_rl/m2v6_lying_R002_A472/<run>/model_<peak>.pt \
  data/npz/231010/stand_up_lying_R_002__A472/motion.npz
```

---

## 10. Checklist

### Khi nhận data mới

- [ ] Đếm số file CSV, đối chiếu với expectation
- [ ] Check format: 34 cols (M2v6)? Số rows hợp lý?
- [ ] Verify input_fps với team data (30 hay 50?)
- [ ] Categorize theo prefix → biết có bao nhiêu motion type
- [ ] Note suffix `_M` (mirror) — coi như augmentation

### Convert

- [ ] Chạy `csv_to_npz_m2v6.py` cho 1 file test trước
- [ ] Preview NPZ bằng `play_motion.py` — motion trông OK?
- [ ] Batch convert toàn folder
- [ ] Check stats (frames, max_jp) toàn dataset
- [ ] Add metadata bằng `npz_add_base_fields.py` (cho motion editor)

### Cleanup

- [ ] Manual review từng motion (preview lần lượt)
- [ ] Loại file kém (di chuyển sang `_excluded/`)
- [ ] Smooth/crop nếu cần
- [ ] Backup raw CSV trước khi sửa NPZ

### Training

- [ ] Quyết định strategy (single vs multi)
- [ ] Test 1 motion local trước với 5k iter (smoke test)
- [ ] Nếu OK → train full 25k iter (local) hoặc server (4096 envs)
- [ ] Check peak reward, deploy

### Deploy

- [ ] Dùng `deploy_with_version.sh` với version mới
- [ ] Update NOTES.md
- [ ] Commit + tag (xem VM_PACKAGES_PROJECT_MANAGEMENT.md)

---

## 📎 Doc liên quan

- **[M2V6_TRAINING_GUIDE.md](./M2V6_TRAINING_GUIDE.md)** — pipeline train + deploy đầy đủ
- **[VM_PACKAGES_PROJECT_MANAGEMENT.md](./VM_PACKAGES_PROJECT_MANAGEMENT.md)** — quản lý version policy
- **[05-huong-dan-retargeting.md](./05-huong-dan-retargeting.md)** — nếu cần retarget từ SMPL
- **[18-pipeline-selfvideo-retargeting-training.md](./18-pipeline-selfvideo-retargeting-training.md)** — pipeline từ video → train

---

*Cập nhật lần cuối: 2026-04-24*
