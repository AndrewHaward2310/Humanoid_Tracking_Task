# Training Neymar Samba trên laptop (warm-start)

Hướng dẫn train lại policy cho motion `neymar_celebrate` (neymar samba) trên laptop RTX 5070 — warm-start để tiết kiệm thời gian.

**Ngữ cảnh**: ONNX neymar cũ từ server (run `2026-04-16_11-59-42`, reward 38.9) deploy được nhưng mất thăng bằng sau 1 lúc. Cần policy robust hơn — train với wider RSI (đã có sẵn trong `env_cfgs.py` sau khi patch cho standup).

---

## Prerequisites — đã sẵn trên laptop

| Mục | Trạng thái |
|---|---|
| GPU RTX 5070 8GB | ✅ |
| mjlab env | ✅ (hoạt động tốt, đã train standup robust) |
| NPZ motion | ✅ `experiments/neymar_samba/DATA/neymar_celebrate.npz` (849 frames) |
| env_cfgs.py đã patch | ✅ Wider RSI đã có (roll/pitch ±0.2, yaw ±0.3) |
| hand collision XML | ✅ Đã uncomment |
| Standup robust checkpoint | ✅ `logs/rsl_rl/m2v6_standup_robust/.../model_15000.pt` |

---

## Chiến lược warm-start — 2 lựa chọn

### Option A — Warm-start từ standup robust (khuyên)

- Policy đã biết body dynamics, giữ thăng bằng từ training standup
- Train thêm ~15k iter để học motion neymar samba (cần lâu hơn standup vì motion phức tạp + chu kỳ)
- **Ưu**: base robust
- **Nhược**: phải học motion mới từ đầu → cần nhiều iter

### Option B — Warm-start từ neymar ONNX cũ (nếu có .pt)

- Policy đã biết motion neymar (reward 38.9)
- Chỉ cần train thêm ~5-8k iter để robust với widened RSI
- **Ưu**: motion đã thuộc, train ít hơn
- **Nhược**: phải download .pt từ WandB artifact — xem có sẵn không

Kiểm tra Option B khả thi không:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run python scripts/download_checkpoint.py 22000 ~/Downloads m2v6_tracking 2>&1 | tail -10
```

Nếu download OK → Option B. Nếu không → Option A.

---

## Bước 1 — Verify YAML motion file

File: `mjlab/src/mjlab/tasks/tracking/config/m2v6/env_cfgs.py` phải có wider RSI (đã patch từ train standup robust). Verify:

```bash
grep -A4 'pose_range =' ~/Documents/Humanoid_Tracking_Task/mjlab/src/mjlab/tasks/tracking/config/m2v6/env_cfgs.py
```

Mong đợi thấy:
```
motion_cmd.pose_range = {
    "x": (-0.08, 0.08),
    ...
    "roll": (-0.2, 0.2),
    "pitch": (-0.2, 0.2),
    "yaw": (-0.3, 0.3),
}
```

Nếu chưa có thì patch (xem doc chuẩn train standup robust).

---

## Bước 2 — Tạo script train neymar warm-start

Script đã được tạo tại `mjlab/scripts/train_neymar_laptop.sh`. Sửa biến phía trên theo Option A hoặc B.

### Option A (warm-start từ standup robust)

```bash
WARMSTART_CKPT="$HOME/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_standup_robust/2026-04-19_02-50-03/model_15000.pt"
EXP_NAME="m2v6_neymar_laptop"
MAX_ITERATIONS=15000                     # cần nhiều iter vì học motion mới
```

### Option B (warm-start từ neymar .pt)

```bash
WARMSTART_CKPT="$HOME/Downloads/model_22000.pt"     # neymar .pt từ WandB
EXP_NAME="m2v6_neymar_robust"
MAX_ITERATIONS=8000                      # ít iter hơn vì chỉ cần robust hóa
```

---

## Bước 3 — Chạy training

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
tmux new -s neymar
bash scripts/train_neymar_laptop.sh
```

Detach: `Ctrl+B D`.

**Thời gian ước tính** (RTX 5070 + 1024 envs):
- Option A (15k iter): ~6-8 giờ
- Option B (8k iter): ~3-4 giờ

Chạy qua đêm, cắm sạc, disable suspend.

---

## Bước 4 — Theo dõi tiến độ

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run python scripts/check_reward_curve_local.py m2v6_neymar_laptop    # Option A
# hoặc
uv run python scripts/check_reward_curve_local.py m2v6_neymar_robust    # Option B
```

Kỳ vọng:
- Option A: iter 0-3000 reward thấp (đang học motion), iter 5000+ >20, iter 10000+ ≥ 30
- Option B: iter 0-500 có thể dip (adapt wider RSI), iter 1000+ quay lại 30+, iter 5000+ ≥ 33

Nếu reward không tăng sau 5k iter → train lỗi, check WandB / logs.

---

## Bước 5 — Deploy checkpoint tốt nhất

Sau khi train, chọn iter peak rồi deploy:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

EXP=m2v6_neymar_laptop      # hoặc m2v6_neymar_robust
TS=$(ls -t logs/rsl_rl/$EXP/ | grep -v warmstart | head -1)
BEST_ITER=???               # iter peak từ bước 4

CKPT="logs/rsl_rl/$EXP/$TS/model_${BEST_ITER}.pt"
NPZ="$HOME/Documents/Humanoid_Tracking_Task/experiments/neymar_samba/DATA/neymar_celebrate.npz"

# Re-export ONNX
uv run python scripts/reexport_onnx.py "$CKPT" "$NPZ" params/neymar_celebrate/policy.onnx

# Gen CSV
uv run python load_metadata_onnx.py \
  params/neymar_celebrate/policy.onnx \
  params/neymar_celebrate/param_for_m26_neymar.csv

# Deploy vào vm_packages
DEST=~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/neymar
cp params/neymar_celebrate/policy.onnx $DEST/
cp params/neymar_celebrate/param_for_m26_neymar.csv $DEST/
```

**KHÔNG cần đổi** `track_neymar_celebrate.yaml` (motion range đã đúng: end=848, hold=848).

  Deploy standup                                                                                                                 
                                                         
  cd ~/Documents/Humanoid_Tracking_Task/mjlab
  bash scripts/deploy_standup.sh logs/rsl_rl/m2v6_standup_robust/2026-04-19_02-50-03/model_14500.pt                              
                                                                                                                                 
  Deploy neymar                                                                                                                  
                                                                                                                                 
  bash scripts/deploy_neymar.sh logs/rsl_rl/m2v6_neymar_laptop/2026-04-19_05-07-54/model_20000.pt
bash scripts/deploy_standup.sh logs/rsl_rl/m2v6_standup_stable/2026-04-19_15-46-10/model_18500.pt

---

## Bước 6 — Test trong sim

```bash
tmux kill-session -t Sim_test 2>/dev/null
cd ~/Documents/vm_packages
ROBOT=M2v6 MODE=sim tmuxp load start.yaml
```

Trong pane vm_ctrl:
1. `=` — standup (dùng standup policy đã deploy sẵn)
2. Chờ `motion step: 308`
3. `2` — vào walk mode
4. `^` (Shift+6) — trigger neymar samba (task2)
5. Robot nhảy ~17 giây (849 frames @ 50fps)

Kỳ vọng policy mới **giữ thăng bằng lâu hơn** neymar cũ.

---

## Troubleshoot

| Triệu chứng | Fix |
|---|---|
| OOM VRAM | Giảm `NUM_ENVS` 1024 → 512 trong script |
| Reward đứng yên ≤ 10 sau 3k iter | Warm-start checkpoint sai → verify Option A vs B |
| Training crash import error | `uv run python -c "import mjlab.tasks.tracking.config.m2v6.env_cfgs"` để check syntax YAML vừa edit |
| Robot load OK nhưng policy không nhảy neymar | Check NPZ motion path trong script đúng `neymar_celebrate.npz` |
| Neymar deploy xong robot không chuyển động | CSV format sai, re-gen bằng `load_metadata_onnx.py` bản mới |

---

## Thời gian ước tính toàn bộ

| Option | Thời gian | Kỳ vọng |
|---|---|---|
| A — warm-start standup robust | 6-8h train + 15m deploy | Ổn định, đủ reward |
| B — warm-start neymar .pt cũ | 3-4h train + 15m deploy | Ổn định, motion đã thuộc |

Option A an toàn hơn (không phụ thuộc WandB artifact có hay không). Option B nhanh hơn nếu có .pt sẵn.
