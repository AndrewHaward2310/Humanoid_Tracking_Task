# Full Pipeline — Train mjlab → Deploy vào sim `vm_packages` (M2v6)

Hướng dẫn end-to-end, làm thủ công 100 %. Dùng **warm-start** từ `m26_standingup_normal` để tránh policy diverge (như đã xảy ra với run `m2v6_neymar_v1` hôm 17/04 — reward âm suốt 15 000 iter).

Toàn bộ verification compat mjlab ↔ vm_packages đã xong (xem §0). Nếu theo đúng guide này, policy sẽ plug-and-play vào `mj_sim`.

---

## 0. Kiểm tra compat đã verify (tham khảo)

| Điểm | Giá trị mjlab | Giá trị C++ / mj_sim | Match |
|---|---|---|---|
| Obs dim | 144 | 144 | ✅ |
| Policy rate | 50 Hz (`sim.dt=0.005 × decimation=4`) | 50 Hz (`dt=0.002 × stride=10`) | ✅ |
| Motion fps | 50 | `motion_step += 1.0/tick` ở 50 Hz | ✅ |
| ONNX input | `obs [1,144]`, `time_step [1,1]` | giống | ✅ |
| ONNX output | 7 tên, body_* có 14 bodies | 7 tên, buf_bpos=42, buf_bquat=56 | ✅ |
| Anchor body | `pelvis_link` (index 0) | body[0] | ✅ |
| Joint order (27) | L-leg 6, R-leg 6, waist 1, L-arm 7, R-arm 7 | `pos_map=[0..26]` identity | ✅ |
| Action formula | `q_des = default + raw × action_scale` | giống | ✅ |
| Metadata ONNX | 8 key tự nhúng qua `get_base_metadata()` | `try_override_from_csv` đọc 5 key | ✅ |

**→ Kết luận:** train với task `Mjlab-Tracking-Flat-M2v6-No-State-Estimation`, giữ nguyên config mặc định, dùng NPZ 50 fps → ONNX export xong **plug thẳng vào vm_packages** không cần sửa gì ngoài path.

---

## 1. Chuẩn bị (một lần)

### 1.1. Motion file

Bạn đã có 2 NPZ trên server sau bước trước:

```
~/Documents/vm_mjlab/motions/
├── m26_standingup_normal.npz   (309 frames @ 50 fps = 6.18s)   ← motion mới
├── standing_up_m26.npz         (430 frames @ 50 fps = 8.60s)   ← motion cũ, vẫn giữ làm backup
├── neymar_celebrate.npz        (849 frames @ 50 fps = 16.98s)
└── neymar_celebrate.csv
```

Nếu server thiếu `m26_standingup_normal.npz`, copy từ laptop:
```bash
scp ~/Documents/Humanoid_Tracking_Task/experiments/neymar_samba/DATA/m26_standingup_normal.npz \
    nguyenl3@10.148.255.113:~/Documents/vm_mjlab/motions/
```

### 1.2. Sanity check code server (đã làm)

Verify 1 lần trước khi chạy:
```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

# stochastic=True thay distribution_cfg
grep -q 'stochastic=True' src/mjlab/tasks/tracking/config/m2v6/rl_cfg.py || \
  echo "!! LỖI: rl_cfg.py chưa được update"

# 3 stub safefall
for f in base_height head_orientation self_collision_penalty; do
  grep -q "def $f" src/mjlab/tasks/safefall/mdp/rewards.py || \
    echo "!! LỖI: thiếu stub $f"
done

# Motion files có đủ
for f in m26_standingup_normal.npz neymar_celebrate.npz; do
  [ -f motions/$f ] && echo "OK: motions/$f" || echo "!! THIẾU: motions/$f"
done
```

Nếu báo lỗi, quay lại §1–2 của `docs/10-huong-dan-train-neymar-celebrate.md` để fix.

### 1.3. WandB

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab
uv run wandb login    # bỏ qua nếu đã login
```

Dashboard: https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab

Nếu muốn skip WandB: đặt `WANDB_MODE=disabled` trước mọi lệnh `uv run train …`.

---

## 1.bis. Patch hand collision (đã làm — ghi lại để biết)

> [!IMPORTANT]
> Training gốc có rubber_hand mesh nhưng collision bị comment out → sim không biết hand có thể va đất → khi deploy real có gắn tay, hand có nguy cơ va đất lúc đứng dậy.

Đã patch 3 nơi (local + sync server):

**A. `mjlab/src/mjlab/asset_zoo/robots/M2v6/M2v6.xml`** — uncomment 2 block hand collision (left + right), 2 body geom capsule fromto `0.08 → 0.13` size 0.05, kèm visual mesh `{left|right}_rubber_hand` và site palm.

**B. `mjlab/src/mjlab/tasks/tracking/config/m2v6/env_cfgs.py`** — thêm:
- `ContactSensorCfg` tên `hand_ground` (primary: geom `^(left|right)_hand_collision$`, secondary: body `terrain`)
- Reward term `hand_ground_contact` (weight **−0.5**, threshold 20 N) dùng `self_collision_cost` — **penalty nhẹ** cho va đất, **không** termination (vì standup cần chống tay đất để đẩy người lên)
- Nâng `nconmax` 70→90, `contact_sensor_maxmatch` 100→120, `njmax` 400→450

**C. Obs dim VẪN 144 sau patch** → ONNX schema không đổi → **compat với C++ `RL27DofTrackingController` giữ nguyên**, không phải sửa vm_packages.

Verify trên server:
```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab
uv run python -c "
from mjlab.tasks.tracking.config.m2v6.env_cfgs import m2v6_flat_tracking_env_cfg
c = m2v6_flat_tracking_env_cfg(has_state_estimation=False)
print('sensors:', [s.name for s in c.scene.sensors])
print('rewards:', list(c.rewards.keys()))
"
# Mong đợi thấy: sensors=['self_collision','hand_ground']; 'hand_ground_contact' trong rewards
```

---

## 2. Giai đoạn A — Pretrain `m26_standingup_normal` (warm-start từ run cũ)

**Mục tiêu:** có 1 checkpoint biết đứng vững (`Train/mean_reward ≥ 30`). Sau đó dùng làm warm-start cho tất cả motion phức tạp khác.

### 2.1. Kiểm tra run cũ có thể warm-start từ đâu

Run cũ đã train 15 k iter **không có hand collision**. Vẫn có thể warm-start vì:
- Obs/action shape không đổi (144 / 27)
- Reward có 1 term mới (hand_ground_contact) nhưng không ảnh hưởng weights của actor/critic
- Physics khác → policy sẽ adapt trong ~2–5 k iter đầu

Checkpoint tốt nhất để warm-start:
```bash
ssh nguyenl3@10.148.255.113
ls ~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_v5/
# Ghi lại timestamp — ví dụ 2026-04-17_14-50-41
```

### 2.2. Khởi động training warm-start trong tmux

```bash
ssh nguyenl3@10.148.255.113
tmux kill-session -t standup 2>/dev/null    # xóa session cũ nếu có
tmux new -s standup                         # session mới — Ctrl+B D để detach
cd ~/Documents/vm_mjlab

# Thay TS_OLD bằng timestamp của run không-tay trước đó (ví dụ 2026-04-17_14-50-41)
TS_OLD=<timestamp-run-cu-khong-tay>

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/m26_standingup_normal.npz \
  --agent.experiment-name m2v6_standup_v5 \
  --agent.load-run $TS_OLD \
  --agent.resume True
```

> [!NOTE]
> `--agent.experiment-name` phải **trùng với run cũ** (`m2v6_standup_v5`) để rsl_rl tìm được checkpoint trong `logs/rsl_rl/m2v6_standup_v5/$TS_OLD/`. Thư mục mới (timestamp mới) sẽ được tạo trong cùng experiment.
>
> Nếu muốn train **from scratch** với hand collision mới (không warm-start):
> ```bash
> uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
>   --env.scene.num-envs 4096 \
>   --env.commands.motion.motion-file motions/m26_standingup_normal.npz \
>   --agent.experiment-name m2v6_standup_hand
> ```

Save path: `logs/rsl_rl/m2v6_standup_normal/<timestamp>/`. Ghi lại `<timestamp>` ngay khi thấy ở dòng đầu terminal.

Detach tmux: `Ctrl+B` → `D`.

### 2.3. Mốc kỳ vọng (warm-start có hand collision)

| Iter (kể từ resume) | `Train/mean_reward` | Ghi chú |
|---|---|---|
| 0 | ~30 (kế thừa từ run cũ) | |
| 100 – 500 | Có thể **tụt xuống 10–20** | Policy adapt với contact physics mới — bình thường |
| 1 000 | 20 – 28 | Lấy lại đà |
| 3 000 | 30 – 35 | Đã quen contact mới |
| 5 000 | ≥ 33 | Stable |
| 10 000+ | 35 – 40 | Converge |

**Theo dõi `Episode_Reward/hand_ground_contact`** trên WandB:
- Phase đầu standup: hand chống đất **bình thường** — reward_hand_ground có giá trị âm nhẹ (−0.5 × vài frames)
- Khi đã đứng: hand_ground phải **= 0** — nếu còn âm, policy chưa nhấc tay đủ cao

### 2.3. Theo dõi từ laptop (không cần SSH interactive)

```bash
# Reward + iter gần nhất
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "tmux capture-pane -t standup -p -S -100 | \
   grep -E 'Learning iteration|Mean reward|Mean episode|ETA'"

# Checkpoint mới nhất
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "ls -t ~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_normal/*/model_*.pt | head -3"
```

Hoặc mở WandB, filter theo experiment_name `m2v6_standup_normal`.

### 2.4. Khi nào nên dừng

- Tối thiểu: **iter 5 000 + `Train/mean_reward ≥ 30`** — dùng làm warm-start được
- Khuyến nghị: chờ đến 10 000–15 000 iter để policy ổn định hơn

Dừng: `tmux attach -t standup` → `Ctrl+C` → `tmux kill-session -t standup`.

### 2.5. Render video để verify

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

# Thay <timestamp> thành thư mục thật (ls logs/rsl_rl/m2v6_standup_normal/ để lấy)
TS=<timestamp>
CKPT=model_15000   # hoặc model checkpoint cuối cùng

MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --checkpoint-file logs/rsl_rl/m2v6_standup_normal/$TS/$CKPT.pt \
  --motion-file motions/m26_standingup_normal.npz \
  --num-envs 1 --video True --video-length 309 \
  --video-width 1920 --video-height 1080 --camera side-right
```

Video ra tại: `logs/rsl_rl/m2v6_standup_normal/$TS/videos/play/rl-video-step-0.mp4`.

Pull về laptop:
```bash
sshpass -p '1' scp \
  nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_normal/$TS/videos/play/rl-video-step-0.mp4 \
  ~/Documents/Humanoid_Tracking_Task/data/videos/standup_normal_$CKPT.mp4
```

Xem video: robot phải đứng dậy được từ tư thế nằm → trụ vững. Nếu robot ngã, chưa đủ train — chờ thêm.

---

### 2.7. Troubleshoot hand collision

| Triệu chứng | Nguyên nhân | Fix |
|---|---|---|
| `Invalid contact_sensor_maxmatch` | `nconmax` / `contact_sensor_maxmatch` quá thấp | Đã nâng 90/120 — nếu vẫn lỗi, tăng tiếp trong env_cfgs.py |
| `hand_ground_contact` reward rất âm (−50+) cả run | Hand va đất liên tục, policy chưa học | Tăng iter, hoặc hạ weight từ −0.5 → −0.2 |
| Robot không chống tay đứng dậy được | Penalty hand_ground quá mạnh | Hạ weight (−0.5 → −0.1) trong `env_cfgs.py` |
| `self_collisions` reward tăng mạnh sau patch | Hand va vào body khác | Kiểm tra motion reference — có thể wrist góc kỳ quặc |

---

## 3. Giai đoạn B — Warm-start train `neymar_samba`

**Nguyên lý warm-start:** load toàn bộ trọng số từ checkpoint standup (đã biết đứng), rồi tiếp tục optimize với motion samba. Policy không phải học lại kỹ năng cân bằng từ đầu.

### 3.1. Lấy tên thư mục standup

```bash
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "ls ~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_normal/"
# Ghi lại <timestamp> — ví dụ 2026-04-18_09-12-34
```

### 3.2. Train samba với resume

```bash
ssh nguyenl3@10.148.255.113
tmux new -s neymar
cd ~/Documents/vm_mjlab

# Thay TS_STANDUP bằng timestamp lấy ở §3.1
TS_STANDUP=<timestamp-standup>

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/neymar_celebrate.npz \
  --agent.experiment-name m2v6_standup_normal \
  --agent.load-run $TS_STANDUP \
  --agent.resume True
```

> [!IMPORTANT]
> `--agent.experiment-name` phải **giữ nguyên là `m2v6_standup_normal`** vì rsl_rl tìm `load_run` trong thư mục `logs/rsl_rl/<experiment_name>/`. Một thư mục mới sẽ được tạo trong cùng experiment (timestamp mới).
>
> Nếu muốn tên thư mục riêng, copy thủ công checkpoint rồi dùng `--agent.experiment-name m2v6_neymar_warmstart` + `--agent.load-run …`.

Detach: `Ctrl+B` → `D`.

### 3.3. Mốc kỳ vọng cho neymar (warm-start)

| Iter (kể từ resume) | `Train/mean_reward` | Ghi chú |
|---|---|---|
| 0 (= iter warm-start base) | ~30 | Kế thừa từ standup |
| 500 | có thể tụt xuống 10–20 | Adapting sang motion mới — bình thường |
| 2 000 | quay lại 25–30 | |
| 5 000 | 32 – 38 | |
| 10 000 | **≥ 35** | Neymar samba đã nét |
| 15 000 | ≥ 38 | Mượt |

**Nếu reward rớt âm sau iter 1 000** → warm-start đã fail, kiểm tra:
- `--agent.load-run` đúng timestamp không?
- `experiment_name` giống run gốc không?
- File `model_*.pt` có trong thư mục không?

### 3.4. Render video neymar

```bash
TS_NEYMAR=<timestamp-warm-start-mới>
CKPT=model_15000

MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --checkpoint-file logs/rsl_rl/m2v6_standup_normal/$TS_NEYMAR/$CKPT.pt \
  --motion-file motions/neymar_celebrate.npz \
  --num-envs 1 --video True --video-length 849 \
  --video-width 1920 --video-height 1080 --camera side-right
```

> `--video-length 849` = đúng 1 chu kỳ motion. Dài hơn → video lặp (do play mode dùng `sampling_mode=start` + `episode_length_s=1e9`).

---

## 4. Giai đoạn C — Export sang vm_packages sim

Sau mỗi run, rsl_rl đã tự export ONNX tại `logs/rsl_rl/<exp>/<timestamp>/<timestamp>.onnx` (xem `runner.py:export_policy_to_onnx`). ONNX này đã có metadata đủ 8 key. Chỉ cần 4 bước copy/generate/edit.

### 4.1. Pull ONNX về laptop

```bash
# Trên laptop:
mkdir -p ~/Documents/Humanoid_Tracking_Task/mjlab/params/m26_standingup_normal
mkdir -p ~/Documents/Humanoid_Tracking_Task/mjlab/params/neymar_celebrate

# Thay TS_STANDUP / TS_NEYMAR đúng folder đã train
TS_STANDUP=<timestamp-standup>
TS_NEYMAR=<timestamp-neymar-warmstart>

sshpass -p '1' scp \
  nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_normal/$TS_STANDUP/$TS_STANDUP.onnx \
  ~/Documents/Humanoid_Tracking_Task/mjlab/params/m26_standingup_normal/policy.onnx

sshpass -p '1' scp \
  nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_normal/$TS_NEYMAR/$TS_NEYMAR.onnx \
  ~/Documents/Humanoid_Tracking_Task/mjlab/params/neymar_celebrate/policy.onnx
```

### 4.2. Sinh CSV metadata từ ONNX

Sửa biến đầu file `~/Documents/Humanoid_Tracking_Task/mjlab/load_metadata_onnx.py` trước khi chạy:

```python
extract_metadata_to_wide_csv(
    '/home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/params/m26_standingup_normal/policy.onnx',
    '/home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/params/m26_standingup_normal/param_for_m26_standup.csv')
```

Chạy:
```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run python load_metadata_onnx.py
```

Lặp lại cho neymar (đổi path trong file và chạy lại). Hoặc viết script nhỏ chạy cả 2 lần:
```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run python -c "
from load_metadata_onnx import extract_metadata_to_wide_csv as F
base = 'params'
F(f'{base}/m26_standingup_normal/policy.onnx', f'{base}/m26_standingup_normal/param_for_m26_standup.csv')
F(f'{base}/neymar_celebrate/policy.onnx',      f'{base}/neymar_celebrate/param_for_m26_neymar.csv')
"
```

### 4.3. Đẩy ONNX + CSV vào vm_packages

```bash
# Standup
cp ~/Documents/Humanoid_Tracking_Task/mjlab/params/m26_standingup_normal/policy.onnx \
   ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/standup/policy.onnx

cp ~/Documents/Humanoid_Tracking_Task/mjlab/params/m26_standingup_normal/param_for_m26_standup.csv \
   ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/standup/param_for_m26_standup.csv

# Neymar
cp ~/Documents/Humanoid_Tracking_Task/mjlab/params/neymar_celebrate/policy.onnx \
   ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/neymar/policy.onnx

cp ~/Documents/Humanoid_Tracking_Task/mjlab/params/neymar_celebrate/param_for_m26_neymar.csv \
   ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/neymar/param_for_m26_neymar.csv
```

### 4.4. Cập nhật YAML motion range

Motion mới `m26_standingup_normal.npz` có **309 frames** (chứ không phải 430 như `standing_up_m26.npz` cũ). Phải sửa `track_standingup.yaml`.

File: `~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_standingup.yaml`

```yaml
motion:
  time_rate: 1.0
  init: 0
  end: 308        # <- đổi từ 408 thành 308 (= num_frames - 1)
  hold: 308       # <- đổi từ 408 thành 308
```

Neymar giữ nguyên (vẫn 849 frames như cũ):
```yaml
# track_neymar_celebrate.yaml — không đổi
motion:
  time_rate: 1.0
  init: 0
  end: 848
  hold: 848
```

### 4.5. Verify FSM wiring (đã wire sẵn — chỉ kiểm tra)

File: `~/Documents/vm_packages/vm_ctrl/Config/sim/M2v6/properties.yaml`

```yaml
rl_yamls:
  track_task2: "track_neymar_celebrate.yaml"  # neymar — đã có
  track_task5: "track_standingup.yaml"         # standup — đã có
```

---

## 5. Giai đoạn D — Build & chạy mj_sim

### 5.1. Build lần đầu

```bash
cd ~/Documents/vm_packages/mj_sim
bash build.sh
```

```bash
cd ~/Documents/vm_packages/vm_ctrl
bash build.sh
```

### 5.2. Chạy sim

```bash
cd ~/Documents/vm_packages
# (lệnh cụ thể tuỳ theo start.yaml / run.sh — xem README của vm_packages)
```

Trong sim: chọn tracking task 2 (neymar) hoặc 5 (standup) qua joystick / keyboard input đã wire trong FSM.

### 5.3. Log kiểm tra runtime

Trong terminal chạy vm_ctrl, sẽ thấy dòng:
```
ONNX Tracking Inference: 123 us
motion step: 1
motion step: 2
...
```

Nếu `motion step` tăng đều mỗi tick policy → ONNX load OK, pipeline chạy. Nếu crash khi chọn task → check:

| Triệu chứng | Nguyên nhân | Fix |
|---|---|---|
| `YAML must have dof=27, obs_per_frame=144, num_frames=1` | Sai task, sai YAML | Dùng `Mjlab-Tracking-Flat-M2v6-No-State-Estimation` khi train |
| `default_joint_pos size mismatch` | CSV lỗi | Re-generate CSV từ ONNX bước §4.2 |
| `ORT ... session creation failed` | ONNX corrupt | Copy lại từ laptop |
| Robot vung loạn rồi ngã | Policy chưa đủ reward | Train thêm |
| Robot đứng im, không theo motion | Joint_sign lật sai | Thêm `joint_sign: [...]` vào YAML (27 giá trị ±1) |

---

## 6. Checklist chạy thủ công (in ra giấy, tick từng mục)

**Giai đoạn A — standup (6.18s motion) — warm-start có hand collision:**
- [ ] XML + env_cfgs.py đã sync server (§1.bis)
- [ ] Verify sensors + rewards trên server bằng lệnh trong §1.bis
- [ ] NPZ `m26_standingup_normal.npz` đã trên server
- [ ] Chọn `TS_OLD` (timestamp run cũ không-tay) để warm-start
- [ ] `tmux new -s standup`
- [ ] Chạy train với `--agent.load-run $TS_OLD --agent.resume True --agent.experiment-name m2v6_standup_v5`
- [ ] Detach `Ctrl+B D`
- [ ] Monitor: reward hồi phục về ≥ 30 tại iter 3 000 (kể từ resume)
- [ ] Monitor: `Episode_Reward/hand_ground_contact` gần 0 trong phase đứng
- [ ] Chờ iter ≥ 10 000 kể từ resume
- [ ] Ghi lại `TS_STANDUP` (timestamp thư mục mới)

**Giai đoạn B — neymar samba (17s motion):**
- [ ] `tmux new -s neymar`
- [ ] Chạy train với `--agent.load-run $TS_STANDUP --agent.resume True` (cùng experiment name)
- [ ] Detach
- [ ] Monitor: reward > 30 tại iter 10 000 kể từ resume
- [ ] Chờ iter ≥ 20 000 kể từ resume
- [ ] Ghi lại `TS_NEYMAR` (thư mục mới trong cùng experiment)

**Giai đoạn C — export:**
- [ ] Render 2 video play để verify bằng mắt
- [ ] Pull 2 ONNX về laptop
- [ ] Sinh 2 CSV metadata
- [ ] Copy ONNX + CSV vào 2 thư mục `vm_packages/.../mimic/{standup,neymar}/`
- [ ] Sửa `track_standingup.yaml`: `end/hold = 308`
- [ ] Kiểm tra `track_neymar_celebrate.yaml`: `end/hold = 848`
- [ ] Kiểm tra properties.yaml đã wire task2, task5

**Giai đoạn D — sim:**
- [ ] Build `mj_sim` và `vm_ctrl`
- [ ] Chạy sim, kích hoạt tracking task 5 (standup) → robot đứng dậy
- [ ] Chạy sim, kích hoạt tracking task 2 (neymar) → robot nhảy samba

---

## 7. Đề phòng — nếu warm-start fail

Nếu reward neymar rơi âm sau iter 1 000–2 000 khi resume, có 2 cách fallback:

### Cách 1: curriculum — giảm motion complexity

Train samba từ standup nhưng với `--env.episode-length-s` nhỏ (ví dụ 4.0 thay vì 10.0) trong vài nghìn iter đầu → robot chỉ cần track ngắn → dễ adapt hơn.

### Cách 2: chain warm-start

Pretrain 1 checkpoint "đứng" rất sâu (30 k iter), rồi:
1. Warm-start 1 motion trung gian dễ hơn neymar (ví dụ dancing_1)
2. Từ kết quả đó warm-start sang neymar samba

### Cách 3: giảm num-envs, tăng entropy

Đổi `entropy_coef` trong `rl_cfg.py` từ 0.005 → 0.01 để policy explore mạnh hơn khi adapt motion mới. Lưu ý phải edit trên server (cả `src/.../rl_cfg.py`), rồi chạy train.

---

## 8. Tham khảo chéo

- [10 — neymar_celebrate setup chi tiết](10-huong-dan-train-neymar-celebrate.md) — bước sync code mjlab lên server (đã làm rồi)
- [12 — guide cơ bản standup + neymar](12-guide-train-m2v6-standup-neymar.md) — phiên bản ngắn hơn, không warm-start
- [11 — đọc WandB](11-huong-dan-doc-wandb-dashboard.md)
- `rl_tracking_27dof.h` / `.cpp` — nguồn authoritative về 144-D obs layout và body layout
- `runner.py:export_policy_to_onnx` (mjlab) — nguồn authoritative về ONNX output names
- `exporter_utils.py:get_base_metadata` — nguồn authoritative về metadata keys
