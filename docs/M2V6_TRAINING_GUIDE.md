# Hướng Dẫn Train Motion Mới Cho M2v6

> Hướng dẫn đầy đủ từ A → Z: chuẩn bị motion → train → deploy → kiểm tra trong sim.  
> Tự train mà không cần hỗ trợ thêm.

---

## Mục lục

> 📎 **Doc liên quan:** [Quản lý dự án vm_packages & Policy Versioning](./VM_PACKAGES_PROJECT_MANAGEMENT.md) — workflow submodule (vm_packages monorepo + vm_ctrl), pull master an toàn ở cả 2 level, versioning policy với symlink `current/`, tag release, rollback.

1. [Tổng quan pipeline](#1-tổng-quan-pipeline)
2. [Chuẩn bị môi trường](#2-chuẩn-bị-môi-trường)
3. [Bước 1 – Chuẩn bị Motion File](#3-bước-1--chuẩn-bị-motion-file)
4. [Bước 2 – Kiểm tra Motion trước khi train](#4-bước-2--kiểm-tra-motion-trước-khi-train)
5. [Bước 3 – Cấu hình Training](#5-bước-3--cấu-hình-training)
6. [Bước 4 – Train ở Local](#6-bước-4--train-ở-local)
7. [Bước 5 – Train trên Server](#7-bước-5--train-trên-server)
8. [Bước 6 – Theo dõi Training](#8-bước-6--theo-dõi-training)
9. [Bước 7 – Deploy vào Sim](#9-bước-7--deploy-vào-sim)
   - 9.6. [Phím bấm để trigger từng motion](#96-phím-bấm-để-trigger-từng-motion-trong-sim)
10. [Bước 8 – Cập nhật NOTES.md](#10-bước-8--cập-nhật-notesmd)
11. [Chiến lược Training](#11-chiến-lược-training)
12. [Tất cả scripts có sẵn](#12-tất-cả-scripts-có-sẵn)
13. [Cấu hình quan trọng – vị trí file](#13-cấu-hình-quan-trọng--vị-trí-file)
14. [Troubleshooting thường gặp](#14-troubleshooting-thường-gặp)

---

## 1. Tổng quan pipeline

```
Motion Editor  →  FK Rebuild  →  [Fix penetration/hand]  →  Train mjlab
                                                                  ↓
                                                         Export ONNX (.pt → .onnx)
                                                                  ↓
                                                         Deploy to vm_packages
                                                                  ↓
                                                         Restart sim → kiểm tra
```

**Thư mục quan trọng:**

| Vai trò | Đường dẫn |
|---|---|
| mjlab (training) | `~/Documents/Humanoid_Tracking_Task/mjlab` |
| vm_packages (sim) | `~/Documents/vm_packages` |
| Checkpoints M2v6 | `~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/` |
| Config sim M2v6 | `~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/` |
| Training logs | `~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/` |
| Server | `nguyenl3@10.148.255.113:~/Documents/vm_mjlab` |

---

## 2. Chuẩn bị môi trường

### PATH cần thiết

```bash
export PATH="$HOME/.local/bin:$PATH"   # cho uv
```

### Kiểm tra uv hoạt động

```bash
uv run python --version   # phải ra Python 3.x
uv run train --help       # phải thấy usage
```

### Nếu `uv` không found

```bash
export PATH="$HOME/.local/bin:$PATH"
# hoặc dùng full path:
~/.local/bin/uv run train ...
```

---

## 3. Bước 1 – Chuẩn bị Motion File

### ⚠️ Format NPZ — 2 dạng phổ biến

Có 2 dạng NPZ motion khác nhau trong pipeline:

| Dạng | Keys cần thiết | Dùng cho |
|---|---|---|
| **Editor-compat** | `base_pos_w`, `base_quat_w`, `joint_names` | Motion Editor v2 |
| **FK-pipeline** | `body_pos_w`, `body_quat_w` (gộp root vào body arrays, không có `joint_names`) | Output của GMR, fk_rebuild_and_hold.py, train mjlab |

**Triệu chứng khi load file FK-pipeline vào motion editor:**

| Triệu chứng | Nguyên nhân |
|---|---|
| Robot đứng im hoàn toàn, kéo frame slider không thấy gì đổi | Thiếu `base_pos_w` + `base_quat_w` → editor kẹt root ở origin |
| **Root di chuyển (người bay xuống nằm) nhưng chân tay đứng yên, không động khớp** | Thiếu `joint_names` → editor dùng tên default `joint_0`, `joint_1`... không khớp tên URDF → `jointsMap[name] = undefined` → không update joint nào |
| `play_motion.py` báo `KeyError: 'joint_names'` | Thiếu các key cũ |

**Fix — 1 lệnh duy nhất thêm đủ tất cả key cần thiết:**

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python scripts/npz_add_base_fields.py \
  --npz /path/to/motion.npz \
  --out /path/to/motion_editor.npz
  # Tự động thêm:
  #   - base_pos_w  ← body_pos_w[:, 0, :]   (pelvis)
  #   - base_quat_w ← body_quat_w[:, 0, :]
  #   - joint_names ← 27 tên joint URDF của M2v6 đúng thứ tự
  #   - body_names  ← 30 tên link của M2v6
```

Sau đó load file `_editor.npz` vào Motion Editor v2 — motion sẽ hiển thị đầy đủ root + joints.

> **Lưu ý quan trọng:** Motion Editor v2 **match joint theo tên** (`jointsMap[joint_names[i]].setJointValue(...)` trong `templates/index.html:720`). Nếu tên không khớp URDF, joint không được update. Đây là lý do cùng 1 file có motion rõ ràng khi preview bằng `play_motion.py` (dùng index theo thứ tự XML) nhưng editor lại chỉ có root di chuyển — 2 tool dùng cơ chế mapping khác nhau.

`play_motion.py` đã được patch để **hỗ trợ cả 2 format** (tự fallback sang body_pos_w nếu thiếu base_pos_w, assume identity joint order nếu thiếu joint_names).

---

### A. Nếu có motion từ motion editor

Motion editor lưu file `.npz` đã chỉnh sửa `joint_pos`. **Bắt buộc phải FK rebuild** vì các trường `body_pos_w`, `body_quat_w`, velocity chưa được cập nhật.

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python scripts/fk_rebuild_and_hold.py \
  --npz /path/to/edited_motion.npz \
  --out /path/to/motion_rebuilt.npz
  # Tùy chọn: --hold-frames 150   (thêm frame hold cuối để học đứng vững)
  # Tùy chọn: --xml src/mjlab/asset_zoo/m2v6/m2v6.xml
```

**Khi nào cần thêm `--hold-frames`?**

Chỉ cần khi **tư thế cuối KHÔNG tự ổn định** (active balance). Gravity không giữ được → policy phải học "giữ tư thế" → cần hold frame làm target tracking.

| Tư thế cuối | Tự ổn định? | Cần hold? | Gợi ý |
|---|---|---|---|
| Đứng thẳng | ❌ (active balance) | ✅ Cần | 150-500 frame (3-10s) |
| Đứng 1 chân | ❌ (rất bất ổn) | ✅ Rất cần | 300-500 frame |
| Squat sâu | ⚠️ Tương đối ổn | Optional | 0-100 frame |
| **Nằm (lying)** | ✅ **Gravity giữ, tiếp xúc rộng** | ❌ **Không cần** | 0 |
| Ngồi xếp bằng | ✅ CoM thấp, base rộng | ❌ Không cần | 0 |
| Motion loop (dance, walk) | N/A (cyclic) | ❌ Không | 0 |

**Nguyên tắc:** Hold = "dạy policy duy trì tư thế bất ổn". Tư thế đã ổn định tự nhiên thì thêm hold chỉ tốn data, không học được gì mới.

### B. Nếu motion bị chìm xuống sàn

```bash
uv run python scripts/shift_motion_to_floor.py \
  --npz motion_rebuilt.npz \
  --out motion_floor_fixed.npz
```

### C. Nếu tay chạm sàn gây penalty

```bash
# Nâng cổ tay lên
uv run python scripts/lift_hand_motion.py \
  --npz motion_floor_fixed.npz \
  --out motion_hand_fixed.npz

# Hoặc uốn cổ tay lên (giữ nguyên forearm)
uv run python scripts/flex_wrist_motion.py \
  --npz motion_floor_fixed.npz \
  --out motion_hand_fixed.npz
```

### D. Lưu motion vào đúng vị trí

Đặt motion vào thư mục checkpoint tương ứng:

```bash
cp motion_hand_fixed.npz \
  ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/<tên_motion>/motion_v1.npz
```

> **Quy ước đặt tên:** `motion_v1.npz`, `motion_v2.npz` ... mỗi lần chỉnh sửa tăng version.

---

## 4. Bước 2 – Kiểm tra Motion trước khi train

Preview motion trong MuJoCo viewer để xác nhận đúng file và đúng chuyển động:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python scripts/play_motion.py \
  --npz /path/to/motion_v1.npz \
  --loop
  # Phím F3: toggle geom visualization
  # Phím F2: contact forces
```

**Kiểm tra các điểm sau:**
- [ ] Robot thực hiện đúng động tác mong muốn
- [ ] Không có body nào chìm dưới sàn
- [ ] Chuyển động mượt mà, không giật cục
- [ ] Frame count đúng (xem `npz['joint_pos'].shape[0]`)

### Kiểm tra nhanh nội dung NPZ

```bash
uv run python -c "
import numpy as np
d = np.load('/path/to/motion_v1.npz')
print('Keys:', list(d.keys()))
print('joint_pos shape:', d['joint_pos'].shape)
print('Frames:', d['joint_pos'].shape[0])
fps = float(d.get('fps', d.get('framerate', [50]))[0]) if hasattr(d.get('fps', d.get('framerate', 50)), '__len__') else d.get('fps', d.get('framerate', 50))
print('FPS:', fps)
"
```

---

## 5. Bước 3 – Cấu hình Training

### 5.1 Tham số Training (trong script .sh)

| Tham số | Mô tả | Giá trị thông thường |
|---|---|---|
| `EXP_NAME` | Tên experiment (tạo folder log) | `m2v6_<tên_motion>` |
| `NUM_ENVS` | Số environment song song | Local: 1024, Server: 4096 |
| `MAX_ITERATIONS` | Số iteration tối đa | 30000 |
| `MOTION_FILE` | Path đến file .npz | Full path tuyệt đối |
| `WARMSTART_CKPT` | Checkpoint khởi đầu (nếu warmstart) | Path đến .pt file |

### 5.2 Task ID

Luôn dùng task này cho M2v6 (không có state estimation → dễ deploy hơn):

```
Mjlab-Tracking-Flat-M2v6-No-State-Estimation
```

### 5.3 Điều chỉnh Reward (nếu cần)

**File:** `~/Documents/Humanoid_Tracking_Task/mjlab/src/mjlab/tasks/tracking/config/m2v6/env_cfgs.py`

Các reward cần chú ý:

```python
# Penalty khi tay chạm sàn — tăng nếu robot hay để tay chạm sàn
"hand_ground_contact": {"weight": -0.15}   # mặc định -0.15

# Penalty joint limit violation
"joint_limit": {"weight": -10.0}           # không nên thay đổi

# Self collision
"self_collisions": {"weight": -10.0}       # không nên thay đổi
```

### 5.4 Warmstart hay Scratch?

| Trường hợp | Quyết định |
|---|---|
| Motion mới hoàn toàn khác old motion | **Train scratch** — tránh bias từ weights cũ |
| Motion cải tiến nhỏ từ motion cũ (ví dụ chỉ fix tay) | **Warmstart** — nhanh hơn, giữ được kiến thức cũ |
| Finetune một behavior cụ thể | **Warmstart** từ checkpoint tốt nhất |

---

## 6. Bước 4 – Train ở Local

### Tạo script train mới (từ template)

```bash
cp ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/train_standup_motion_v1_scratch.sh \
   ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/train_<tên_motion>_scratch.sh
```

Chỉnh sửa các biến trong script:

```bash
EXP_NAME="m2v6_<tên_motion>"
NUM_ENVS=1024                           # local: ít hơn để vừa VRAM
MAX_ITERATIONS=30000
MOTION_FILE="/full/path/to/motion.npz"
```

### Chạy train local

```bash
export PATH="$HOME/.local/bin:$PATH"
cd ~/Documents/Humanoid_Tracking_Task/mjlab
bash scripts/train_<tên_motion>_scratch.sh
```

### Hoặc chạy lệnh train trực tiếp

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 1024 \
  --env.commands.motion.motion-file /path/to/motion.npz \
  --agent.experiment-name "m2v6_ten_motion" \
  --agent.max-iterations 30000 \
  --agent.logger wandb

cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 2048 \
  --env.commands.motion.motion-file /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/data/0505/npz/lying_down_standing_up_002_editor_trim100-164_fk_hold.npz \
  --agent.experiment-name "m2v6_standing_up_optitrack_v1" \
  --agent.max-iterations 30000 \
  --agent.logger wandb

cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/data/0505/npz_01/vansunhuy_video.npz \
  --agent.experiment-name "m2v6_vansunhuy" \
  --agent.max-iterations 25000 \
  --agent.logger wandb
Experiment:    m2v6_lying_down_002_softlanding_v1
Tmux session:  lying_softland
Wandb URL:     https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab/runs/ra9hs0eh
Log file:      /tmp/train_softland_20260430_221229.log
Iter time:     ~1.6s/iter
ETA:           ~15h cho 25k iter
GPU:           ~2.5GB used (free 5.2GB)
```

### Warmstart (nếu cần)

```bash
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 1024 \
  --env.commands.motion.motion-file /path/to/motion.npz \
  --agent.experiment-name "m2v6_ten_motion_finetune" \
  --agent.max-iterations 35000 \
  --agent.resume True\
  --agent.load-run "m2v6_ten_motion_base/2026-xx-xx_xx-xx-xx" \
  --agent.load-checkpoint 30000 \
  --agent.logger wandb
```

Ví dụ: 

```bash
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4048 \
  --env.commands.motion.motion-file /home/nguyenld12/Downloads/edited_10_balanced.npz \
  --agent.experiment-name m2v6_tracking \
  --agent.max-iterations 20000 \
  --agent.resume True \
  --agent.load-run "2026-05-02_01-03-14" \
  --agent.load-checkpoint model_4000.pt \
  --agent.logger wandb
```

### Logs và checkpoints

```
logs/rsl_rl/<EXP_NAME>/<timestamp>/
  ├── model_500.pt
  ├── model_1000.pt
  ├── ...
  ├── model_<best>.pt
  └── params/
      ├── env.yaml
      └── agent.yaml
```

---

## 7. Bước 5 – Train trên Server

### 7.1 Tạo script server

```bash
cp ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/$TRAIN_SCRIPT \
   ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/train_<tên_motion>_scratch_server.sh

# Chỉnh sửa NUM_ENVS=4096 và các biến khác
```

### 7.2 Tạo prep script (upload + launch)

```bash
cp ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/prep_motion_v1_scratch_server.sh \
   ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/prep_<tên_motion>_server.sh
```

Nội dung cần điều chỉnh:

```bash
SERVER="nguyenl3@10.148.255.113"
LOCAL_MJLAB="$HOME/Documents/Humanoid_Tracking_Task/mjlab"
SERVER_MJLAB="/home/nguyenl3/Documents/vm_mjlab"
LOCAL_MOTION="/path/to/local/motion.npz"        # ← đổi path này

# Tên tmux session và tên script server
TMUX_SESSION="<tên_motion>_scratch"
TRAIN_SCRIPT="train_<tên_motion>_scratch_server.sh"
```

### 7.3 Chạy prep + launch

```bash
chmod +x scripts/prep_<tên_motion>_server.sh
bash scripts/prep_<tên_motion>_server.sh
```

Script sẽ:
1. Upload motion file lên server
2. Upload train script lên server
3. Kill tmux session cũ (nếu có)
4. Tạo tmux session mới và chạy training

### 7.4 Kiểm tra training đã chạy chưa

```bash
ssh nguyenl3@10.148.255.113 "tmux ls"
# Phải thấy: <tên_session>: 1 windows (created ...)
```

---

## 8. Bước 6 – Theo dõi Training

### Xem log realtime (server)

```bash
ssh nguyenl3@10.148.255.113 "tail -f /tmp/<tên_session>.log"
```

### Attach vào tmux session (server)

```bash
ssh nguyenl3@10.148.255.113
tmux attach -t <tên_session>
# Ctrl+B D để detach (không kill)
```

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab && uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation   --num-envs 1   --motion-file data/0505/npz/lying_down_standing_up_002_01_edited9_ready.npz   --checkpoint-file logs/rsl_rl/m2v6_standup_002_01_ready/2026-04-30_11-06-23/model_25500.pt --no-terminations True

cd ~/Documents/Humanoid_Tracking_Task/mjlab && uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation   --num-envs 1   --motion-file /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/data/0505/npz_01/lying_down_standing_up_002_v3.npz  --checkpoint-file /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_lying_down_v2/2026-05-01_15-02-14/model_6000.pt --no-terminations True

cd ~/Documents/Humanoid_Tracking_Task/mjlab && uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation   --num-envs 1   --motion-file /home/nguyenld12/Downloads/edited_11_balanced.npz  --checkpoint-file /home/nguyenld12/Downloads/model_21500.pt --no-terminations True

cd ~/Documents/Humanoid_Tracking_Task/mjlab && uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation   --num-envs 1   --motion-file /home/nguyenld12/Downloads/edited_11_balanced.npz  --checkpoint-file /home/nguyenld12/Downloads/model_5500.pt --no-terminations True


uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation --num-envs 1 --motion-file /home/nguyenl3/Downloads/edited_10_balanced.npz --checkpoint-file /home/nguyenl3/Documents/vm_mjlab/logs/rsl_rl/m2v6_tracking/2026-05-02_08-59-25/model_4500.pt  --no-terminations True
ssh -L 8082:localhost:8080 nguyenl3@10.148.255.113

```
### Kiểm tra reward curve & tìm Peak Checkpoint (local)

Sau khi train xong, luôn chạy step này để xác định checkpoint nào peak — dùng để deploy.

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

# Chỉ định tên experiment (auto-pick run mới nhất)
uv run python scripts/check_reward_curve_local.py m2v6_<tên_motion>

# Hoặc không có arg → default "m2v6_standup_laptop"
uv run python scripts/check_reward_curve_local.py
```

**Output mẫu:**

```
Run dir: logs/rsl_rl/m2v6_lying_down_v1/2026-04-22_01-12-06
Total points: 15000  |  step 0 → 14999
PEAK reward: iter 14937 → 34.843

  iter |   reward |  ep_len |  body_pos |  root_pos |  hand_gnd |  self_col
------------------------------------------------------------------------------------
     0 |   -2.629 |    17.1 |    0.0165 |    0.0082 |   -0.0073 |   -0.0525
  4995 |   27.141 |   425.6 |    0.8462 |    0.3007 |   -0.0079 |   -0.0199
  6993 |   32.010 |   478.9 |    0.9088 |    0.3015 |   -0.0057 |   -0.0279
 14937 |   34.843 |   489.9 |    0.9342 |    0.3444 |   -0.0068 |   -0.0241  <- PEAK
```

### Diễn giải các cột

| Cột | Ý nghĩa | Giá trị tốt |
|---|---|---|
| `iter` | Iteration số mấy | — |
| `reward` | Tổng mean_reward | Tuỳ motion, 25-40 là tốt |
| `ep_len` | Độ dài episode trung bình (max 500) | > 450 (robot không chết sớm) |
| `body_pos` | Motion body tracking reward | > 0.85 (track limb tốt) |
| `root_pos` | Motion global root tracking | > 0.30 |
| `hand_gnd` | Penalty tay chạm sàn | > -0.05 (càng gần 0 càng tốt) |
| `self_col` | Penalty self-collision | > -0.05 |

### Dấu hiệu training tốt

- Reward tăng dần trong 5000-10000 iter đầu
- `ep_len` tiến dần tới 500 (robot sống hết episode)
- `body_pos > 0.85` cuối training → track motion tốt
- Penalty (`hand_gnd`, `self_col`) ≥ -0.05 → robot không có behavior xấu

### Chọn Peak Checkpoint để Deploy

Checkpoint save interval = 500 iter (config trong `rl_cfg.py`). Sau khi biết peak iter, tìm **checkpoint gần nhất đã save**:

```bash
# Check các checkpoint đã save
ls logs/rsl_rl/<exp>/<run>/model_*.pt | sort -V

# Quy tắc: chọn checkpoint gần peak_iter nhất
# Ví dụ peak=14937 → model_14999.pt (cách 62) hoặc model_14500.pt (cách 437)
#        → chọn 14999 (gần hơn)
```

**Lưu ý:** Cẩn thận nếu peak ở rất sớm (ví dụ iter 500) nhưng sau đó reward duy trì cao — có thể chọn checkpoint muộn hơn để có robustness tốt hơn (training lâu hơn thường → explore nhiều scenario hơn).

### Benchmark reward cho các motion M2v6

| Motion | Peak reward điển hình | Ghi chú |
|---|---|---|
| Standup | 38-40 | Motion ngắn, dễ track |
| Lying down | 33-35 | Motion dài, transition lớn |
| Walk/Cyclic | 30-40 | Tuỳ complexity |
| Neymar dance | 25-35 | Motion phức tạp nhiều limb |

Nếu peak của bạn **thấp hơn benchmark > 10 reward** → có thể motion có issue (joint limit violation, quá nhanh, v.v.) hoặc cần train thêm.

### Kiểm tra reward curve & tìm Peak Checkpoint (trên SERVER)

Training trên server → tfevents nằm ở server, không có local. 3 cách xử lý:

#### Cách 1 (khuyến nghị): Upload script + chạy trên server

```bash
# Bước 1: Upload check_reward_curve_local.py lên server (chỉ cần làm 1 lần)
scp ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/check_reward_curve_local.py \
    nguyenl3@10.148.255.113:~/Documents/vm_mjlab/scripts/

# Bước 2: Chạy trên server (uv env server có đủ tensorboard)
ssh nguyenl3@10.148.255.113 "export PATH=\$HOME/.local/bin:\$PATH && \
  cd ~/Documents/vm_mjlab && \
  uv run python scripts/check_reward_curve_local.py <tên_experiment>"
```

**Ví dụ thực tế (check training `m2v6_standup_motion_v1` đang chạy):**

```bash
ssh nguyenl3@10.148.255.113 "export PATH=\$HOME/.local/bin:\$PATH && \
  cd ~/Documents/vm_mjlab && \
  uv run python scripts/check_reward_curve_local.py m2v6_standup_motion_v1"
```

Có thể chạy **real-time trong khi training đang chạy** (tensorboard event file được flush liên tục).


### Pull peak checkpoint từ server về local

Sau khi xác định peak iter, pull .pt về local để deploy:

```bash
EXP="m2v6_standup_motion_v1"
RUN="2026-04-21_17-48-23"
ITER="27999"   # checkpoint gần peak nhất

mkdir -p ~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/$EXP/$RUN
scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/$EXP/$RUN/model_${ITER}.pt \
  ~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/$EXP/$RUN/
```

Xong, dùng file này chạy `deploy_motion.sh` như thường.

### Download checkpoint tốt nhất về local (từ WandB)

```bash
uv run python scripts/download_checkpoint.py <iter_number> \
  logs/rsl_rl/<tên_experiment>/ \
  <tên_experiment>
```

### Pull checkpoint từ server manual

```bash
scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/<exp>/<run>/model_<iter>.pt \
  ~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/<exp>/<run>/
```

---

## 9. Bước 7 – Deploy vào Sim

### 9.1 Export ONNX từ checkpoint

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python scripts/reexport_onnx.py \
  logs/rsl_rl/<exp>/<run>/model_<iter>.pt \
  /path/to/motion.npz \
  /tmp/policy_new.onnx
```

> **Lưu ý:** Motion NPZ được **bake vào trong ONNX** (như hằng số). Deploy ONNX là deploy cả motion.

### 9.2 Deploy ONNX vào vm_packages

**Cách 1 (khuyến nghị): Dùng `deploy_motion.sh` — generic cho mọi motion**

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

bash scripts/deploy_motion.sh <motion_name> <checkpoint.pt> <motion.npz>
```

**Trong đó:**
- `<motion_name>`: tên folder trong `vm_packages/.../mimic/` — ví dụ `standup`, `lyingdown`, `neymar`
- `<checkpoint.pt>`: đường dẫn file .pt peak đã chọn
- `<motion.npz>`: motion NPZ sẽ được bake vào ONNX

**Script tự động:**
1. Re-export ONNX từ .pt + NPZ
2. Generate CSV metadata
3. Copy vào `vm_packages/.../mimic/<motion_name>/policy.onnx` + `param_for_m26_<motion_name>.csv`
4. Tính số frame và **in gợi ý `motion.end` + `motion.hold`** cần cập nhật YAML

**Ví dụ thực tế — deploy lying_down_v1:**

```bash
bash scripts/deploy_motion.sh lyingdown \
  logs/rsl_rl/m2v6_lying_down_v1/2026-04-22_01-12-06/model_14999.pt \
  ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/lyingdown/m26_lying_down_v1.npz
```

**Ví dụ deploy standup từ motion_v1:**

```bash
bash scripts/deploy_motion.sh standup \
  logs/rsl_rl/m2v6_standup_motion_v1/<timestamp>/model_<iter>.pt \
  ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/standup/motion_v1.npz
```

---

**Cách 2 (legacy): Deploy script riêng cho từng motion cũ**

Trước đây có các script riêng (cứng parameters):
- `deploy_standup.sh` — chỉ cho standup, path hardcoded
- `deploy_neymar.sh` — chỉ cho neymar

→ Giờ đã có `deploy_motion.sh` thay thế. Chỉ dùng các script cũ nếu cần pipeline cũ.

---

**Cách 3: Manual** (nếu muốn kiểm soát từng bước)

```bash
# Bước a: Export ONNX
uv run python scripts/reexport_onnx.py \
  <ckpt.pt> <motion.npz> /tmp/policy_new.onnx

# Bước b: Generate CSV metadata
uv run python load_metadata_onnx.py /tmp/policy_new.onnx \
  /tmp/param_new.csv

# Bước c: Copy vào vm_packages
DEST=~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/<tên_motion>
mkdir -p "$DEST"
cp /tmp/policy_new.onnx "$DEST/policy.onnx"
cp /tmp/param_new.csv "$DEST/param_for_m26_<tên_motion>.csv"
```

### 9.3 Cập nhật config YAML sim

**File:** `~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_<tên_motion>.yaml`

> ⚠️ **LUÔN KIỂM TRA** file YAML hiện có có phải template M2v3 cũ không! Nhiều file trong folder `config/M2v6/` được copy từ M2v3 có sẵn **dof=23, obs_per_frame=124, model_path trỏ sang Checkpoints/M2v3/...** — robot sẽ load sai policy. Xem [Troubleshooting: YAML trỏ sang policy M2v3 cũ](#-yaml-config-trỏ-sang-policy-m2v3-cũ-23-dof-thay-vì-m2v6-mới-27-dof).

**Kiểm tra nhanh file YAML có đúng M2v6 không:**

```bash
grep -E "^(model_path|dof|obs_per_frame)" \
  ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_<tên_motion>.yaml
# Đúng M2v6:  dof: 27, obs_per_frame: 144, model_path chứa "M2v6/mimic"
# Sai (M2v3): dof: 23, obs_per_frame: 124, model_path chứa "M2v3/track"
```

**Scan toàn bộ folder để tìm file sai (khuyến nghị chạy sau khi clone repo mới):**

```bash
cd ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/
for f in track_*.yaml; do
  DOF=$(grep "^dof:" "$f" | awk '{print $2}')
  OBS=$(grep "^obs_per_frame:" "$f" | awk '{print $2}')
  if [ "$DOF" != "27" ] || [ "$OBS" != "144" ]; then
    echo "  ⚠️  $f  →  dof=$DOF  obs_per_frame=$OBS"
  else
    echo "  ✅ $f"
  fi
done
```

Output cho biết file nào cần fix. Triệu chứng khi load file sai:

```
[FSM] Failed to load RL 'track_taskN' from .../track_XXX.yaml :
default_joint_pos size mismatch. Skipping this state.
```

→ Khi đó ấn phím trigger task đó sẽ **không có gì xảy ra** (state bị skip).

Nếu file hiện có là template M2v3 hoặc chưa có → **overwrite từ template M2v6 standup** (đã chuẩn):

```bash
cp ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_standingup.yaml \
   ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_<tên_motion>.yaml
```

**Các trường cần cập nhật:**

```yaml
model_path: "../RLController/Checkpoints/M2v6/mimic/<tên_motion>/policy.onnx"
metadata:
  path: "../RLController/Checkpoints/M2v6/mimic/<tên_motion>/param_for_m26_<tên_motion>.csv"

motion:
  time_rate: 1.0
  init: 0
  end: <N-1>    # N = số frame trong motion (= joint_pos.shape[0] - 1)
  hold: <N-1>   # giống end nếu muốn hold tư thế cuối
```

**Cách lấy N (số frame):**

```bash
python3 -c "import numpy as np; d=np.load('/path/to/motion.npz'); print(d['joint_pos'].shape[0])"
```

### 9.4 Restart Sim

```bash
cd ~/Documents/vm_packages

# Kill session cũ
tmux kill-session -t Sim_test 2>/dev/null || true

# Khởi động lại
ROBOT=M2v6 MODE=sim tmuxp load start.yaml
```

> **Quan trọng:** Sim phải được restart để pick up ONNX mới và config YAML mới. Chỉ copy file thôi là chưa đủ.

### 9.5 Kiểm tra trong sim

- Quan sát robot có thực hiện đúng motion không
- Kiểm tra robot có đứng vững sau khi motion kết thúc không
- Nếu robot ngã hoặc motion sai → xem log, kiểm tra `motion.end` và `motion.hold` trong YAML

### 9.6 Phím bấm để trigger từng motion trong sim

**Chọn input method** trong `vm_ctrl/Config/sim/M2v6/properties.yaml`:

```yaml
input_method: 1  # 0 xbox, 1 keyboard, 2 ps
```

#### Bảng mapping đầy đủ (track_task1 → task9)

| Task | Motion file | Keyboard | PS4/PS5 | Xbox | Từ state |
|---|---|---|---|---|---|
| task1 | `track_ces_long_boxing.yaml` | `←` | L2 + D-Pad ◄ | ❌ chưa impl | Standing / RL_walk |
| task2 | `track_neymar_celebrate.yaml` | `↑` | L2 + D-Pad ▲ | ❌ | Standing / RL_walk |
| task3 | `track_dancing_2.yaml` (pubg) | `→` | L2 + D-Pad ► | ❌ | Standing / RL_walk |
| task4 | `track_lyingdown.yaml` | `↓` | L2 + D-Pad ▼ | ❌ | **RL_walk only** (từ Standing sẽ ngã) |
| task5 | `track_standingup.yaml` | `=` | D-Pad ► (no combi) | ❌ | **Passive** |
| task6 | `track_kick_combo_1.yaml` (spin_kick) | `t` | L1 + D-Pad ◄ ⚠️ | ❌ | Standing / RL_walk |
| task7 | `track_kick_combo_2.yaml` (qa_kick1) | `e` | L1 + D-Pad ▲ ⚠️ | ❌ | Standing / RL_walk |
| task8 | `track_kick_combo_3.yaml` (kien_kick1) | `y` | L1 + D-Pad ► ⚠️ | ❌ | Standing / RL_walk |
| task9 | `track_kick_combo_4.yaml` (kien_kick2) | `r` | L1 + D-Pad ▼ ⚠️ | ❌ | Standing / RL_walk |

> **Lưu ý state transition (quan sát thực tế):**
> - Sau khi standup (task5) xong → robot về **Standing** (PD standing tại chỗ), KHÔNG tự vào RL_walk.
> - Muốn vào RL_walk từ Standing: ấn **`2`** (L2_A), chờ ~2s transition.
> - Code `FSMState_RL_tracking.cpp:56-62` có nhánh auto-transit nếu `motion_finished == true`, nhưng trong thực tế `motion.hold = motion.end` giữ motion ở frame cuối → `motion_finished` không fire → robot về Standing thay vì RL_walk.

Ghi chú:
- **task4 (lying) bắt buộc đi qua RL_WALK** — không đi thẳng từ Standing vì initial pose mismatch (motion frame 0 có shoulder -36° vs Standing default 0°) sẽ gây ngã. Luồng thông thường: `=` → standup → Standing → **`2`** → RL_WALK → `↓` → lying. Xem "Vì sao task4 phải đi qua RL_WALK" bên dưới.
- **⚠️** task6-9 qua PS4: code `ps4.cpp` line 167-170 hiện map `BTN_TL` (L1) → `STATE_SUCESS` thay vì set combi_key → **L1 + D-Pad chưa hoạt động qua PS4**. Cần sửa tương tự BTN_TL2 nếu muốn dùng.
- **Xbox (`xbox.cpp`)**: D-Pad + combi_key handlers đang EMPTY → **không dùng được task1-9 qua Xbox**. Chỉ dùng được keyboard hoặc PS4.

#### Flow chuẩn: từ boot → thực hiện motion

```
Sim khởi động → Passive state
   │
   │  [Keyboard: =]  (STANDUP command)
   ▼
task5 standingup  (robot đứng dậy)
   │
   ▼
Standing state  (PD standing, robot đứng vững tại chỗ)
   │
   │  [Keyboard: 2]  (L2_A — chờ ~2s transition)
   ▼
RL_27DOF_WALK  (policy walk đang chạy, arms ở dynamic pose)
   │
   ├─ [Keyboard: ←]        → task1 boxing
   ├─ [Keyboard: ↑]        → task2 neymar
   ├─ [Keyboard: →]        → task3 pubg
   ├─ [Keyboard: ↓]        → task4 lying down  ⭐
   └─ [Keyboard: t/e/y/r]  → task6-9 kick combos
```

> **Quan trọng (thực tế quan sát):** Sau khi standup xong, robot về **Standing** (không auto vào RL_walk dù code có nhánh auto-transit). **PHẢI ấn `2` để vào RL_walk** trước khi ấn mũi tên trigger motion khác.
>
> Nguyên nhân có thể: `motion.hold = motion.end` trong YAML giữ motion ở frame cuối → `motion_finished` không fire → nhánh auto trong `FSMState_RL_tracking.cpp:56-62` không chạy. Có thể override bằng cách set `motion.hold` < `motion.end` nếu muốn auto-transit.

#### Vì sao task4 (lying) phải đi qua RL_WALK, KHÔNG enable trực tiếp từ Standing

**Lý do giữ nguyên code comment trong `Standing.cpp:347`:**

Standing state có robot ở **default PD pose** (all joints = 0). Motion lying_down frame 0 có **shoulder yaw ở ~-36°** (và nhiều joint khác lệch) → transition thẳng Standing → lying bị **initial pose mismatch lớn** → policy yank mạnh tay → CoM lệch → **robot ngã đầu motion**.

Trong khi đó, `RL_WALK` state chạy policy walk có robot ở **dynamic pose** (arms swing, joints dao động tự nhiên). Khi transit từ RL_WALK → lying, pose khởi đầu match tốt hơn với motion frame 0 → policy không bị yank → **robot thực hiện lying mượt**.

**Flow đúng để lying (3 phím):**

```
Passive  →  [=]  →  task5 standup  →  Standing
                                         │
                                         │  [2]  (L2_A, chờ ~2s transition)
                                         ▼
                                     RL_27DOF_WALK
                                         │
                                         │  [↓]  (TL2_DOWN_ARROW)
                                         ▼
                                     task4 lying down   ✓
```

3 phím: `=` → `2` → `↓`.

**Nếu muốn ấn ↓ từ Standing thẳng vào lying** (không khuyến nghị — sẽ ngã), uncomment block sau trong `src/FSM/FSMState_Standing.cpp:347`:

```cpp
else if(_lowState->userCmd == UserCommand::TL2_DOWN_ARROW){
    if (!_data->_highCmd->idle) {
        std::cout << "Upperbody in use, can not change to Tracking" << std::endl;
        return FSMStateName::STANDING;
    }
    std::cout << "transition from Standing to RL Mimic 4 (lying down)" << std::endl;
    return FSMStateName::RL_27DOF_TRACKING_TASK4;
}
```

→ Chỉ nên uncomment **sau khi retrain policy với `prepend_warmup_frames.py`** (xem Section 14 "Robot ngã ngay khi bắt đầu motion").

Sau đó rebuild + restart sim:

```bash
tmux kill-session -t Sim_test 2>/dev/null
cd ~/Documents/vm_packages
ROBOT=M2v6 MODE=sim tmuxp load start.yaml
# start.yaml tự ./build.sh trước khi chạy vm_ctrl → binary được rebuild
```

#### Nguồn source code (để trace/debug nếu cần)

| Logic | File |
|---|---|
| Keyboard → UserCommand | `src/interface/KeyBoard.cpp` |
| PS4/PS5 → UserCommand | `src/interface/ps4.cpp` |
| Xbox → UserCommand | `src/interface/xbox.cpp` (chưa impl TL/TL2) |
| Standing → task1-3,6-9 | `src/FSM/FSMState_Standing.cpp` |
| Passive → task5 (standup) | `src/FSM/FSMState_Passive.cpp` |
| RL_walk → task1-4,6-9 | `src/FSM/FSMState_RL_walk.cpp` |
| Task registry (load yaml) | `src/FSM/FSM.cpp` (load_rl_safe calls) |
| Enum UserCommand | `include/common/enumClass.h` |
| Task name → yaml file | `Config/sim/M2v6/properties.yaml` (rl_yamls block) |

---

## 10. Bước 8 – Cập nhật NOTES.md

**Sau mỗi lần train xong và deploy**, cập nhật NOTES.md tại thư mục checkpoint:

```
~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/<tên_motion>/NOTES.md
```

**Template:**

```markdown
# <Tên Motion> — Version Notes

## v<N> — <ngày>

**Motion file:** `motion_v<N>.npz`
**Checkpoint:** `policy.onnx` (peak iter <X>, reward <Y>)
**Nguồn gốc:** <mô tả cách tạo motion>
**Frames:** <số frame> @ 50fps (~<giây>s)
**DOF:** 27

**Thay đổi so với trước:**
- <thay đổi 1>
- <thay đổi 2>

**Training:** <scratch / warmstart từ ...>
**Run:** `logs/rsl_rl/<exp>/<timestamp>` — <N> iter, <envs> envs
**Peak:** iter <X> — reward <Y>
**Deploy sim:** _(chưa test / OK)_
**Deploy real:** _(chưa test / OK)_
```

---

## 11. Chiến lược Training

### Chiến lược tiêu chuẩn (motion hoàn toàn mới)

```
Giai đoạn 1 — Scratch base (30k iter, 4096 envs)
  └─ Học motion cơ bản, không warmstart
  └─ Dùng motion NPZ đã FK rebuild

Giai đoạn 2 — Stability finetune (nếu cần, +5k iter)
  └─ Warmstart từ best iter của giai đoạn 1
  └─ Tăng hand_ground penalty nếu cần: -0.15 → -0.3
  └─ Tăng push_robot intensity nếu muốn robust hơn

Giai đoạn 3 — Hold training (nếu motion kết thúc ở tư thế tĩnh)
  └─ Extend motion với --hold-frames 300-500
  └─ Warmstart từ best iter giai đoạn 2
```

### Chiến lược cải tiến motion cũ

```
Giai đoạn 1 — Warmstart từ checkpoint tốt nhất
  └─ --agent.resume + --agent.load-checkpoint <best_iter>
  └─ Chạy thêm 5k-10k iter
  └─ Xem reward có tăng không
```

### Khi nào chọn số envs?

| Máy | NUM_ENVS |
|---|---|
| Laptop (RTX 5070, 8GB VRAM) | 1024 |
| Server (A100/3090) | 4096 |
| Quick test | 512 |

### Chọn max iterations?

- **Quick test:** 10k (xem policy có học không)
- **Full train:** 30k
- **Finetune:** +5k từ best checkpoint

---

## 12. Tất cả scripts có sẵn

### Scripts trong `mjlab/scripts/`

#### Training scripts

| Script | Mô tả |
|---|---|
| `train_standup_motion_v1_scratch.sh` | Train scratch motion_v1, local 1024 envs |
| `$TRAIN_SCRIPT` | Train scratch motion_v1, server 4096 envs |
| `train_standup_v9_server.sh` | Train v9 warmstart từ iter30000, server |
| `train_standup_handfix.sh` | Finetune fix hand contact |
| `train_standup_stable.sh` | Train stability (extended hold frames) |
| `train_neymar_laptop.sh` | Train Neymar motion, laptop |
| `train_warmstart_laptop.sh` | Warmstart generic, laptop |

#### Prep + Server scripts

| Script | Mô tả |
|---|---|
| `prep_motion_v1_scratch_server.sh` | Upload motion_v1 + launch scratch train |
| `prep_v9_server.sh` | Upload motion_v1 + warmstart iter30000 + launch |
| `prep_v8_server.sh` | Upload config + warmstart + launch v8 |

#### Deploy scripts

| Script | Mô tả |
|---|---|
| **`deploy_motion.sh`** ⭐ | **GENERIC deploy cho mọi motion** — truyền `<motion_name> <ckpt> <npz>` |
| `deploy_standup.sh` | (Legacy) Hardcoded cho standup |
| `deploy_neymar.sh` | (Legacy) Hardcoded cho neymar |
| `pull_deploy_run.sh` | Pull từ server + deploy + restart sim |

#### Motion processing scripts

| Script | Input | Output | Mô tả |
|---|---|---|---|
| `fk_rebuild_and_hold.py` | edited.npz | rebuilt.npz | Rebuild body kinematics từ joint_pos |
| `extend_npz_hold.py` | motion.npz | motion_hold.npz | Thêm N frame hold cuối |
| `shift_motion_to_floor.py` | motion.npz | floor_fixed.npz | Fix robot bị chìm dưới sàn |
| `lift_hand_motion.py` | motion.npz | hand_fixed.npz | Nâng trajectory cổ tay |
| `flex_wrist_motion.py` | motion.npz | hand_fixed.npz | Uốn cổ tay để không chạm sàn |
| `fix_hand_flex.py` | motion.npz | fixed.npz | Fix finger/hand dưới z=0 |
| `npz_add_base_fields.py` | motion.npz (no base_*) | motion_editor.npz | Thêm `base_pos_w`/`base_quat_w` để motion editor đọc được |
| `prepend_warmup_frames.py` ⭐ | motion.npz | motion_warmup.npz | Thêm N frame blend từ Standing pose → motion[0] (fix initial pose mismatch gây ngã đầu motion) |

#### Visualization scripts

| Script | Mô tả |
|---|---|
| `play_motion.py` | Preview motion trong MuJoCo viewer |

#### Monitoring scripts

| Script | Mô tả |
|---|---|
| `check_reward_curve_local.py` | Xem reward curve từ local tensorboard logs |
| `check_reward_curve.py` | Xem reward curve từ WandB |
| `check_running_runs.py` | List WandB runs đang chạy |
| `download_checkpoint.py` | Download checkpoint từ WandB |

#### Export scripts

| Script | Mô tả |
|---|---|
| `reexport_onnx.py` | Export ONNX từ .pt checkpoint + motion NPZ |

---

## 13. Cấu hình quan trọng – vị trí file

### mjlab (training framework)

| File | Mô tả |
|---|---|
| `src/mjlab/tasks/tracking/config/m2v6/env_cfgs.py` | Reward weights, randomization, observation config |
| `src/mjlab/tasks/tracking/config/m2v6/rl_cfg.py` | PPO hyperparameters, network architecture |
| `src/mjlab/tasks/tracking/config/m2v6/__init__.py` | Đăng ký task names |
| `src/mjlab/tasks/tracking/tracking_env_cfg.py` | Base config (reward definitions) |

### PPO hyperparameters quan trọng (rl_cfg.py)

```python
clip_param: 0.2           # PPO clip
learning_rate: 1.0e-3     # lr (adaptive schedule)
desired_kl: 0.01          # KL divergence target
num_learning_epochs: 5    # update epochs per rollout
num_mini_batches: 4       # mini-batches
gamma: 0.99               # discount factor
lam: 0.95                 # GAE lambda
entropy_coef: 0.005       # exploration bonus
```

### vm_packages (sim deployment)

| File | Mô tả |
|---|---|
| `vm_ctrl/RLController/config/M2v6/track_<motion>.yaml` | Config sim cho từng motion |
| `vm_ctrl/RLController/Checkpoints/M2v6/mimic/<motion>/policy.onnx` | Model đã deploy |
| `vm_ctrl/RLController/Checkpoints/M2v6/mimic/<motion>/param_for_m26_<motion>.csv` | Metadata |
| `start.yaml` | TMuxp config để khởi động sim |

### Trường `motion` trong track_*.yaml

```yaml
motion:
  time_rate: 1.0    # tốc độ phát motion (1.0 = 50fps)
  init: 0           # frame bắt đầu (thường = 0)
  end: <N-1>        # frame cuối (= total_frames - 1)
  hold: <N-1>       # frame hold sau khi kết thúc (thường = end)
```

---

## 14. Troubleshooting thường gặp

### "uv: command not found"

```bash
export PATH="$HOME/.local/bin:$PATH"
```

### Sim không hiển thị motion mới

Sim **phải được restart** sau khi thay ONNX:
```bash
tmux kill-session -t Sim_test
ROBOT=M2v6 MODE=sim tmuxp load start.yaml
```

### Robot thực hiện motion cũ trong sim

- Kiểm tra `policy.onnx` đã được copy đúng chưa
- Kiểm tra `track_<motion>.yaml` có đúng path không (xem mục kế tiếp)
- Restart sim

### ⚠️ FSM báo "default_joint_pos size mismatch. Skipping this state"

Log trong terminal vm_ctrl:

```
[FSM] Failed to load RL 'track_taskN' from .../track_XXX.yaml :
default_joint_pos size mismatch. Skipping this state.
```

→ State bị skip → ấn phím trigger task đó không có gì xảy ra.

**Nguyên nhân:** Trong file YAML, `dof` và số phần tử của `default_joint_pos` không khớp. Thường do file lẫn template M2v3 (dof=23) nhưng `default_joint_pos` đã được update thành 27 elements cho M2v6 → mismatch.

**Fix:** Sửa `dof: 23 → 27` và `obs_per_frame: 124 → 144`. Xem mục kế tiếp.

### ⚠️ YAML config trỏ sang policy M2v3 cũ (23 DOF) thay vì M2v6 mới (27 DOF)

**Triệu chứng:** Đã deploy policy M2v6 mới nhưng robot trong sim vẫn chạy sai (motion cũ, crash, hoặc robot không react đúng).

**Nguyên nhân:** Nhiều file `track_*.yaml` trong `vm_packages/vm_ctrl/RLController/config/M2v6/` **thừa hưởng từ M2v3 cũ** — model_path trỏ về `../RLController/Checkpoints/M2v3/...` thay vì M2v6, DOF = 23 thay vì 27.

**Cách kiểm tra nhanh:**

```bash
grep -E "^(model_path|metadata|dof|obs_per_frame)" \
  ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_<motion>.yaml
```

Output ĐÚNG (M2v6):
```
model_path: "../RLController/Checkpoints/M2v6/mimic/<motion>/policy.onnx"
metadata.path: ".../M2v6/mimic/<motion>/param_for_m26_<motion>.csv"
dof: 27
obs_per_frame: 144
```

Output SAI (M2v3):
```
model_path: "../RLController/Checkpoints/M2v3/track/<motion>/<timestamp>.onnx"   ← SAI!
dof: 23                                                                            ← SAI!
obs_per_frame: 124                                                                 ← SAI!
```

**Fix:** Rewrite YAML theo template `track_standingup.yaml` (đã là M2v6 27DOF chuẩn). Các trường cần đồng bộ:

| Field | Giá trị đúng cho M2v6 |
|---|---|
| `model_path` | `"../RLController/Checkpoints/M2v6/mimic/<motion>/policy.onnx"` |
| `metadata.path` | `"../RLController/Checkpoints/M2v6/mimic/<motion>/param_for_m26_<motion>.csv"` |
| `dof` | `27` |
| `obs_per_frame` | `144` |
| `default_joint_pos` | 27 × `0.0` |
| `action_scale` | `0.5` |
| `pos_map`, `vel_map`, `action_map` | `[0, 1, ..., 26]` identity |
| `legs.kp` | `[40.317, 173.548, 36.414, 173.548, 24.477, 24.477]` |
| `legs.kd` | `[5.775, 24.859, 5.216, 24.859, 3.506, 3.506]` |
| `waist.kp / waist.kd` | `36.414` / `5.216` |
| `arms.kp` | `[20.056, 14.735, 14.735, 14.735, 14.735, 14.735, 14.735]` |
| `arms.kd` | `[2.873, 2.111, 2.111, 2.111, 2.111, 2.111, 2.111]` |
| `motion.init` | `0` (không phải 1) |
| `motion.end` / `motion.hold` | `<N-1>` (N = số frame trong NPZ) |
| `joint_limit.lower/upper` | `[-10]*27` / `[10]*27` |

**Lệnh copy nhanh template:**

```bash
cd ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6
cp track_standingup.yaml track_<motion>.yaml
# Sau đó sửa:
#   - model_path → đổi "standup" thành "<motion>"
#   - metadata.path → đổi "standup" thành "<motion>"
#   - motion.end/hold → set theo số frame của motion mới
```

**Ví dụ case thực tế (lying down v1):**

File `track_lyingdown.yaml` thừa hưởng từ M2v3 với dof=23, path M2v3 — robot load sai policy. Đã rewrite hoàn toàn theo template M2v6 standup, set `motion.end: 534` (motion 535 frames), trỏ về `Checkpoints/M2v6/mimic/lyingdown/policy.onnx`.

### Robot ngã ngay khi bắt đầu motion trong sim (initial pose mismatch)

**Triệu chứng:** Transition từ Standing → motion → vài frame đầu robot vẫy tay/gập người mạnh rồi mất thăng bằng sập xuống. Không phải lúc nào cũng sập (tuỳ initial pose ngẫu nhiên).

**Nguyên nhân:** Motion frame 0 có joint position **khác xa Standing default** (all zeros). Ví dụ nếu motion bắt đầu với `shoR_y = -36°` (vai xoay mạnh) mà Standing ở 0° → policy tracking thúc mạnh tay để match → CoM lệch → ngã.

**Chẩn đoán nhanh:**

```bash
uv run python -c "
import numpy as np
d = np.load('/path/to/motion.npz')
jp = d['joint_pos']
# Check mismatch ở frame 0 vs Standing default (all zeros)
max_deg = np.abs(np.rad2deg(jp[0])).max()
names = ['hipL_p','hipL_r','hipL_y','kneeL','ankL_p','ankL_r',
        'hipR_p','hipR_r','hipR_y','kneeR','ankR_p','ankR_r','waist',
        'shoL_p','shoL_r','shoL_y','elbowL','wrL_y','wrL_p','wrL_r',
        'shoR_p','shoR_r','shoR_y','elbowR','wrR_y','wrR_p','wrR_r']
print(f'Max joint deviation @ frame 0: {max_deg:.1f}° ({names[np.abs(jp[0]).argmax()]})')
print(f'Warning nếu > 15° — policy sẽ kéo mạnh khớp đó → mất thăng bằng')
"
```

**2 level fix (từ nhẹ → mạnh):**

> ⚠️ **KHÔNG dùng `motion.init` để skip frame đầu** — đây là hiểu lầm phổ biến.
> Nếu mismatch là **offset cố định** ở joint nào đó (ví dụ `shoR_y` luôn ở -36° suốt motion), skip frame chỉ làm robot ngã **nhanh hơn** vì:
> - Frame 30 mismatch vẫn ~36° (offset không đổi)
> - Frame 100+ còn có thêm hip/knee lệch → **mismatch tích luỹ lớn hơn**
>
> `motion.init` chỉ work nếu vài frame đầu có **glitch thoáng qua** (NaN, outlier 1-2 frame) rồi stabilize. Với motion có **arm pose đặc trưng cố định** → không giải quyết được.
>
> **Cách kiểm tra motion có dùng motion.init được không:**
> ```bash
> uv run python -c "
> import numpy as np
> d = np.load('/path/motion.npz')
> jp = d['joint_pos']
> # So sánh frame 0 vs các frame sau
> for f in [0, 30, 60, 100, 150]:
>     max_abs = np.abs(np.rad2deg(jp[f])).max()
>     print(f'f{f:3d}: max |joint| = {max_abs:.1f}°')
> "
> ```
> Nếu max|joint| tăng đều (hoặc giữ lớn) theo frame → skip không help. Phải dùng Level 1 hoặc 2 bên dưới.

#### Level 1 — Edit motion trong Motion Editor (10 phút, không retrain)

Mở NPZ editor-compat trong Motion Editor v2:

1. Chọn frame 0 → set các joint mismatch cao (shoulder, elbow...) = 0
2. Set keyframe **K1** tại frame 0 (pose Standing-like)
3. Đi tới frame ~30-60 → set **K2** (giữ nguyên giá trị gốc motion tại đó)
4. Bấm **Interpolate K1→K2 (linear)** → blend mượt từ Standing pose vào motion thật
5. Download NPZ, FK rebuild, redeploy

```bash
uv run python scripts/fk_rebuild_and_hold.py \
  --npz edited.npz --out motion_v2.npz

bash scripts/deploy_motion.sh <motion_name> <ckpt.pt> motion_v2.npz
```

**Hạn chế:** policy đã train với motion cũ → motion mới sai khác → tracking không hoàn hảo. Chỉ work nếu edit rất nhỏ.

#### Level 2 — Prepend warmup frames + retrain (5h, đúng bài, khuyến nghị)

Có sẵn script `scripts/prepend_warmup_frames.py` — thêm N frames blend tuyến tính từ Standing pose (all joints = 0, pelvis_z = 0.95) về motion[0]:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

# 0. Nếu motion gốc thiếu joint_names → thêm trước (fk_rebuild yêu cầu joint_names)
uv run python scripts/npz_add_base_fields.py \
  --npz /path/to/motion.npz \
  --out /tmp/motion_with_names.npz

# 1. Prepend 60 frame warmup (1.2s blend)
uv run python scripts/prepend_warmup_frames.py \
  --npz /tmp/motion_with_names.npz \
  --out /tmp/motion_warmup.npz \
  --warmup-frames 60

# 2. FK rebuild để body arrays consistent
uv run python scripts/fk_rebuild_and_hold.py \
  --npz /tmp/motion_warmup.npz \
  --out /path/to/motion_v2.npz

# 3. Retrain từ scratch (EXP name mới) hoặc warmstart từ checkpoint cũ
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4048 \
  --env.commands.motion.motion-file motions/boxing.npz  \
  --agent.experiment-name "m2v6_boxing_v2_warmup" \
  --agent.max-iterations 15000 \
  --agent.logger wandb

# 4. Check peak, deploy như thường
uv run python scripts/check_reward_curve_local.py m2v6_<motion>_v2_warmup
bash scripts/deploy_motion.sh <motion_name> <new_ckpt.pt> /path/to/motion_v2.npz
```

**Tham số `--warmup-frames`:**
- `30` (~0.6s) — nhẹ nhàng, đủ cho mismatch nhỏ <15°
- `60` (~1.2s) — **khuyến nghị default**, xử lý mismatch 15-30°
- `120` (~2.4s) — mismatch rất lớn >30° hoặc transition phức tạp

**Cơ chế:** Motion mới có N frame đầu = interp tuyến tính từ zero-pose → motion[0]. Policy train thấy transition Standing → motion mượt, không bị yank đột ngột.

### Các checks khác khi robot ngã đầu motion

- Kiểm tra `motion.end` trong YAML có đúng = `total_frames - 1` không
- Kiểm tra NPZ đã được FK rebuild chưa (`fk_rebuild_and_hold.py`)
- Kiểm tra joint limit violation trong motion (có thể gây reward penalty đột biến → policy rối)

### Robot không đứng vững sau khi hoàn thành motion

- Cần train thêm với `hold_frames` dài hơn (thêm 300-500 frame hold vào motion)
- Hoặc train thêm giai đoạn stability với push_robot mạnh hơn

### Ấn phím trong sim không trigger được motion

1. **Check `input_method` trong properties.yaml** — phải = 1 (keyboard), 2 (PS4) hoặc 0 (xbox — chưa impl)
2. **Check state hiện tại của robot:**
   - task5 (standup) chỉ trigger được từ **Passive** state (ấn `=`)
   - task1,2,3,6,7,8,9 chỉ trigger từ **Standing** hoặc **RL_WALK**
   - task4 (lying) mặc định chỉ từ **RL_WALK** — muốn từ Standing phải uncomment code trong `FSMState_Standing.cpp:347`
3. **Focus vào terminal vm_ctrl khi dùng keyboard** — nếu không focus terminal sẽ không nhận phím
4. **Sau khi sửa file .cpp → phải rebuild** — sim chạy binary `build/vm_ctrl`. Restart sim qua `start.yaml` sẽ tự `./build.sh`
5. **Xbox hoàn toàn không dùng được task1-9** — `xbox.cpp` chưa implement D-Pad combi handlers. Dùng keyboard hoặc PS4 thay thế

Chi tiết mapping phím → xem Section 9.6.

### Training reward không tăng sau 5000 iter

- Kiểm tra motion NPZ có hợp lệ không: `play_motion.py`
- Thử giảm `NUM_ENVS` (nếu OOM)
- Thử scratch thay vì warmstart (nếu warmstart từ motion khác quá)
- Kiểm tra reward weights trong `env_cfgs.py`

### Motion editor load NPZ: 2 triệu chứng khác nhau

**Triệu chứng A — Robot đứng im hoàn toàn, kéo frame không đổi gì:**
→ Thiếu `base_pos_w` + `base_quat_w` → editor kẹt root ở origin + identity quat.

**Triệu chứng B — Root di chuyển (người bay xuống nằm) nhưng chân tay đứng yên:**
→ Thiếu `joint_names`. Motion Editor v2 match joint theo **tên** (`jointsMap[joint_names[i]].setJointValue(...)` trong `templates/index.html:720`). Khi không có `joint_names`, app.py fallback về `["joint_0", "joint_1", ...]` — không trùng tên URDF (`left_hip_pitch_joint`, ...) → `jointsMap["joint_0"] = undefined` → joint không được update.

**Fix cho cả 2 triệu chứng — cùng 1 lệnh:**

```bash
uv run python scripts/npz_add_base_fields.py \
  --npz /path/to/motion.npz \
  --out /path/to/motion_editor.npz
```

Script tự thêm `base_pos_w`, `base_quat_w`, `joint_names` (27 tên URDF M2v6), `body_names`.

> **⚠️ Sau khi convert file, BẮT BUỘC hard refresh browser** (`Ctrl + Shift + R`) rồi load lại URDF + NPZ mới. Nếu chỉ upload file mới mà không refresh, browser sẽ dùng lại `motionData` cached → vẫn thấy motion sai dù file đã đúng.

**Sau khi convert file phải hard refresh browser (`Ctrl + Shift + R`)** rồi load lại URDF + NPZ. Nếu không refresh, browser cache `motionData` cũ → editor vẫn hiển thị sai.

**Debug nhanh trong DevTools Console (F12):**

```js
// 1. NPZ đã có joint_names đúng chưa?
console.log("NPZ joint_names:", motionData.joint_names);

// 2. URDF parse ra được joint nào?
console.log("URDF joints:", Object.keys(jointsMap));

// 3. Bao nhiêu joint match?
const matched = motionData.joint_names.filter(n => jointsMap[n]);
console.log(`Matched ${matched.length}/${motionData.joint_names.length}`);
```

Matched = 27/27 → mọi thứ OK. Matched = 0 → check xem NPZ joint_names có phải `["joint_0", ...]` không (load nhầm file) hay tên đúng mà URDF không match (URDFLoader parse khác).

**Cách kiểm tra nhanh 1 NPZ có đủ key editor không:**

```bash
uv run python -c "
import numpy as np
d = np.load('/path/to/motion.npz')
keys = set(d.files)
need = {'base_pos_w', 'base_quat_w', 'joint_names'}
print('Keys:', sorted(keys))
print('Missing for editor:', sorted(need - keys) or '(none — ready)')
"
```

### `play_motion.py` báo KeyError: 'joint_names'

NPZ format mới không có `joint_names`/`base_pos_w`/`base_quat_w`. Script đã được patch để fallback:
- Không có `joint_names` → assume identity joint order (27 joints đúng thứ tự MuJoCo)
- Không có `base_pos_w`/`base_quat_w` → dùng `body_pos_w[:, 0, :]` / `body_quat_w[:, 0, :]` (body index 0 = pelvis)

Nếu vẫn lỗi → kéo code mới nhất từ `scripts/play_motion.py`.

### Joint limit violation trong motion

```bash
uv run python -c "
import numpy as np
d = np.load('/path/to/motion.npz')
jp = d['joint_pos']
# kiểm tra giới hạn joint (tùy robot)
print('min joint_pos:', jp.min(axis=0))
print('max joint_pos:', jp.max(axis=0))
"
```

Nếu vi phạm nhỏ (< 0.1 rad) thường policy vẫn học được.  
Nếu vi phạm lớn → cần sửa lại trong motion editor.

### Server training bị kill

```bash
ssh nguyenl3@10.148.255.113 "tmux ls"
# Nếu session đã mất → phải chạy lại prep script
```

---

## Quick Reference Card

```bash
# 1. FK rebuild sau khi edit motion
uv run python scripts/fk_rebuild_and_hold.py --npz edited.npz --out rebuilt.npz

# 2. Preview motion
uv run python scripts/play_motion.py --npz rebuilt.npz --loop

# 3. Train local (scratch)
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 1024 \
  --env.commands.motion.motion-file /path/motion.npz \
  --agent.experiment-name "m2v6_ten_motion" \
  --agent.max-iterations 30000 \
  --agent.logger wandb

# 4. Check peak checkpoint sau khi train
uv run python scripts/check_reward_curve_local.py m2v6_ten_motion
# Ghi lại peak iter → chọn model_<gần_peak>.pt

# 5. Deploy ONE-LINER (generic cho mọi motion)
bash scripts/deploy_motion.sh <motion_name> <ckpt.pt> <motion.npz>
# Ví dụ:
bash scripts/deploy_motion.sh lyingdown \
  logs/rsl_rl/m2v6_lying_down_v1/2026-04-22_01-12-06/model_14999.pt \
  ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/lyingdown/m26_lying_down_v1.npz

# 6. Cập nhật motion.end/hold trong YAML (script sẽ print gợi ý)
#   File: ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_<motion_name>.yaml
#   Set end = hold = N-1

# 7. Restart sim
tmux kill-session -t Sim_test && cd ~/Documents/vm_packages && ROBOT=M2v6 MODE=sim tmuxp load start.yaml

# 8. Theo dõi server
ssh nguyenl3@10.148.255.113 "tail -f /tmp/<session>.log"
```

---

*Cập nhật lần cuối: 2026-04-22*
