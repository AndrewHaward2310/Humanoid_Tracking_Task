# Hướng dẫn Training: neymar_celebrate → M2v6

Hướng dẫn từng bước để chạy training motion imitation cho video `neymar_celebrate` trên workstation RTX 5090.

> [!IMPORTANT]
> Server `vm_mjlab` chạy phiên bản **cũ hơn** so với laptop. Nhiều module M2v6 chưa có trên server và cần được sync thủ công. Tài liệu này ghi chép đầy đủ các lỗi gặp phải và cách khắc phục.

---

## Điều kiện tiên quyết

- ✅ Retargeting đã hoàn thành trên laptop
- File CSV: `/home/nguyenld12/Documents/Humanoid_Tracking_Task/vm_retargeting/output/neymar_celebrate.csv`
- File PKL: `/home/nguyenld12/Documents/Humanoid_Tracking_Task/vm_retargeting/output/neymar_celebrate.pkl`
- Video kiểm tra: `/home/nguyenld12/Documents/Humanoid_Tracking_Task/vm_retargeting/videos/m2_v6_neymar_celebrate.mp4`

### Thông tin server

| Thông tin | Giá trị |
|-----------|---------|
| SSH | `ssh nguyenl3@10.148.255.113` |
| Password | `1` |
| GPU | NVIDIA GeForce RTX 5090 (32GB VRAM) |
| mjlab path | `~/Documents/vm_mjlab/` |
| mjlab version (server) | v1.1.1 (commit `6cc4e29`) |
| Python | 3.13.9 |
| CUDA | 12.9 |

---

## Bước 1: Copy CSV lên server

Từ laptop:

```bash
scp /home/nguyenld12/Documents/Humanoid_Tracking_Task/vm_retargeting/output/neymar_celebrate.csv \
    nguyenl3@10.148.255.113:~/Documents/vm_mjlab/motions/
```

---

## Bước 2: Sync module M2v6 lên server

> [!WARNING]
> Server `vm_mjlab` (commit `6cc4e29`) **chưa có** module M2v6. Cần copy từ laptop lên server trước khi chạy bất kỳ script nào liên quan.

### 2.1. Copy robot M2v6 asset

Server chỉ có `M2v3`, `unitree_g1`, `unitree_go1` — không có `M2v6`:

```bash
# Từ laptop:
scp -r /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/src/mjlab/asset_zoo/robots/M2v6 \
    nguyenl3@10.148.255.113:~/Documents/vm_mjlab/src/mjlab/asset_zoo/robots/M2v6
```

### 2.2. Copy tracking config M2v6

Server chỉ có config cho `g1` và `m23` — không có `m2v6`:

```bash
scp -r /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/src/mjlab/tasks/tracking/config/m2v6 \
    nguyenl3@10.148.255.113:~/Documents/vm_mjlab/src/mjlab/tasks/tracking/config/m2v6
```

### 2.3. Fix version mismatch trong `rl_cfg.py`

Laptop code mới hơn dùng `distribution_cfg` trong `RslRlModelCfg`, nhưng server không hỗ trợ tham số này. Thay vào đó, server dùng `stochastic=True`.

SSH vào server và sửa file:

```bash
ssh nguyenl3@10.148.255.113
cat > ~/Documents/vm_mjlab/src/mjlab/tasks/tracking/config/m2v6/rl_cfg.py << 'EOF'
"""RL configuration for M2v6 tracking task."""

from mjlab.rl import (
  RslRlModelCfg,
  RslRlOnPolicyRunnerCfg,
  RslRlPpoAlgorithmCfg,
)


def m2v6_tracking_ppo_runner_cfg() -> RslRlOnPolicyRunnerCfg:
  """Create RL runner configuration for M2v6 tracking task."""
  return RslRlOnPolicyRunnerCfg(
    actor=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
      stochastic=True,
    ),
    critic=RslRlModelCfg(
      hidden_dims=(512, 256, 128),
      activation="elu",
      obs_normalization=True,
    ),
    algorithm=RslRlPpoAlgorithmCfg(
      value_loss_coef=1.0,
      use_clipped_value_loss=True,
      clip_param=0.2,
      entropy_coef=0.005,
      num_learning_epochs=5,
      num_mini_batches=4,
      learning_rate=1.0e-3,
      schedule="adaptive",
      gamma=0.99,
      lam=0.95,
      desired_kl=0.01,
      max_grad_norm=1.0,
    ),
    experiment_name="m2v6_tracking",
    save_interval=500,
    num_steps_per_env=24,
    max_iterations=20_000,
  )
EOF
```

> [!NOTE]
> **Sự khác biệt chính giữa laptop vs server:**
> | Tham số | Laptop (mới) | Server (cũ) |
> |---------|-------------|-------------|
> | Actor distribution | `distribution_cfg={...}` | `stochastic=True` |
> | Critic | Không stochastic | Không stochastic |
> | `max_iterations` | 30,000 | 20,000 (đã giảm) |

---

## Bước 3: Fix lỗi SafeFall module

> [!WARNING]
> Khi import bất kỳ module tracking nào, `mjlab.tasks.__init__` sẽ **tự động walk tất cả packages** (bao gồm `safefall`). Module `safefall` trên server thiếu nhiều hàm trong `rewards.py`, gây `ImportError`.

### Cơ chế gây lỗi

```
csv_to_npz_m2v6.py
  → import m2v6_flat_tracking_env_cfg
    → mjlab.tasks.__init__ → import_packages()
      → walk ALL packages → import safefall
        → safefall/mdp/__init__.py
          → from rewards import base_height  ← THIẾU!
```

### Cách fix: Thêm stub functions

SSH vào server và thêm các hàm stub vào `rewards.py`:

```bash
ssh nguyenl3@10.148.255.113

printf '\n\ndef base_height(*args, **kwargs):
    """Stub function to fix import error."""
    return 0.0


def head_orientation(*args, **kwargs):
    """Stub function to fix import error."""
    return 0.0


def self_collision_penalty(*args, **kwargs):
    """Stub function to fix import error."""
    return 0.0
' >> ~/Documents/vm_mjlab/src/mjlab/tasks/safefall/mdp/rewards.py
```

**Danh sách đầy đủ các hàm cần stub** (check `safefall/mdp/__init__.py`):

| Hàm | Trạng thái trên server |
|-----|------------------------|
| `contact_force_weighted_penalty` | ✅ Có sẵn (class) |
| `joint_constraint_force_penalty` | ✅ Có sẵn (class) |
| `torque_limit_penalty` | ✅ Có sẵn (class) |
| `regularization_penalty` | ✅ Có sẵn (function) |
| `arm_extension` | ⚠️ Stub đã thêm trước đó |
| `base_height` | ❌ **Cần thêm stub** |
| `head_orientation` | ❌ **Cần thêm stub** |
| `self_collision_penalty` | ❌ **Cần thêm stub** |

---

## Bước 4: Cài đặt & đăng nhập WandB

WandB chưa được cấu hình sẵn trên server:

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

# Cài wandb (nếu chưa có)
uv add wandb

# Đăng nhập
uv run wandb login
# Paste API key từ https://wandb.ai/authorize
```

Thông tin WandB sau đăng nhập:
- **User:** `nguyen-ld2310`
- **Org:** `nguyen-ld2310-hanoi-university-of-science-and-technology`
- **Project:** `mjlab`
- **Dashboard:** https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab

> [!TIP]
> Nếu muốn bỏ qua WandB, thêm `WANDB_MODE=disabled` trước lệnh train.

---

## Bước 5: Chuyển CSV → NPZ (trên server)

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

MUJOCO_GL=egl uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
  --input-file motions/neymar_celebrate.csv \
  --output-file motions/neymar_celebrate.npz \
  --input-fps 30 \
  --output-fps 50 \
  --render True
```

### Giải thích tham số

| Tham số | Giá trị | Lý do |
|---------|---------|-------|
| `MUJOCO_GL=egl` | - | Server không có display, dùng EGL để render offscreen |
| `input-fps` | 30 | GVHMR xuất video ở 30 FPS |
| `output-fps` | 50 | Tần số điều khiển robot M2v6 là 50 Hz |
| `render` | True | Tạo video MP4 để kiểm tra bằng mắt |

### Kết quả mong đợi

```
Motion interpolated, input frames: 510, input fps: 30.0, output frames: 849, output fps: 50.0
Processing frames (t=16.0s): 100%|████████████████████| 849/849 [00:01<00:00, 432.63frame/s]
Saved motion data to motions/neymar_celebrate.npz
Video saved to motions/neymar_celebrate.mp4
```

File output:
- `motions/neymar_celebrate.npz` — Dữ liệu: `joint_pos`, `joint_vel`, `body_pos_w`, `body_quat_w`, `body_lin_vel_w`, `body_ang_vel_w`, `fps`
- `motions/neymar_celebrate.mp4` — Video kiểm tra

### Quá trình nội bộ của script

```
CSV (510 frames @ 30fps, 17s video)
  │
  ├── 1. Load CSV: pos(3) + quat(4) + joints(27) per frame
  │     └── Quaternion: CSV dùng xyzw → script convert sang wxyz (MuJoCo)
  │
  ├── 2. Interpolate: 30fps → 50fps (SLERP cho quaternion, LERP cho vị trí)
  │     └── 510 frames → 849 frames
  │
  ├── 3. Compute velocities: finite differences
  │     ├── Linear velocity: torch.gradient()
  │     ├── Angular velocity: SO(3) derivative
  │     └── Joint velocity: torch.gradient()
  │
  ├── 4. Replay on MuJoCo simulation
  │     └── Set root state + joint state → forward() → record body states
  │
  └── 5. Save NPZ + render MP4
```

---

## Bước 6: Huấn luyện (trên server)

```bash
cd ~/Documents/vm_mjlab

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/neymar_celebrate.npz
```

### Thông tin training

| Thông số | Giá trị |
|----------|---------|
| **Task** | `Mjlab-Tracking-Flat-M2v6-No-State-Estimation` |
| **Lý do chọn task** | Không cần state estimation → phù hợp robot thật |
| **Num envs** | 4096 (song song trên RTX 5090) |
| **Max iterations** | 20,000 |
| **Steps per env** | 24 |
| **Total steps/iteration** | 4096 × 24 = 98,304 |
| **Checkpoint** | Tự động lưu mỗi 500 iterations |
| **Thời gian/iteration** | ~0.72s |
| **ETA** | ~4 giờ |
| **Save path** | `logs/rsl_rl/m2v6_tracking/<timestamp>/` |

### RL Config (PPO)

| Tham số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| Actor | 512→256→128 + ELU | 3 hidden layers, stochastic output |
| Critic | 512→256→128 + ELU | 3 hidden layers, deterministic |
| Learning rate | 1e-3 | Adaptive schedule |
| Clip param | 0.2 | PPO clipping |
| Entropy coef | 0.005 | Khuyến khích exploration |
| GAE (γ, λ) | 0.99, 0.95 | Discount + advantage estimation |
| Desired KL | 0.01 | Auto-adjust learning rate |

---

## Bước 7: Theo dõi huấn luyện

### Metrics trên terminal

Mỗi iteration hiển thị:

```
Learning iteration 164/20000
  Mean reward: 1.48           ← Nên tăng dần
  Mean episode length: 51.81  ← Nên tăng (robot sống lâu hơn)
  Mean action noise std: 0.54 ← Giảm dần (policy ổn định hơn)
  Steps per second: 137182    ← Throughput
```

### Reward components (ý nghĩa)

| Component | Ý nghĩa | Tốt khi |
|-----------|---------|---------|
| `motion_global_root_pos` | Tracking vị trí gốc | Tăng |
| `motion_global_root_ori` | Tracking hướng quay gốc | Tăng |
| `motion_body_pos` | Tracking vị trí các body | Tăng |
| `motion_body_ori` | Tracking hướng quay body | Tăng |
| `motion_body_lin_vel` | Tracking vận tốc tuyến tính | Tăng |
| `motion_body_ang_vel` | Tracking vận tốc góc | Tăng |
| `action_rate_l2` | Penalty: thay đổi action quá nhanh | Gần 0 |
| `joint_limit` | Penalty: vượt giới hạn khớp | Gần 0 |
| `self_collisions` | Penalty: tự va chạm | Gần 0 |

### Termination reasons

| Termination | Ý nghĩa |
|-------------|---------|
| `time_out` | Hết thời gian episode (tốt) |
| `anchor_pos` | Vị trí anchor lệch quá xa reference |
| `anchor_ori` | Hướng quay anchor lệch quá xa |
| `ee_body_pos` | Tay/chân lệch quá xa reference |

> [!NOTE]
> Ban đầu `ee_body_pos` termination sẽ rất cao (ví dụ: 62/86 episodes) — robot chưa biết bắt chước, tay chân bị lệch quá xa → bị reset. Con số này nên giảm dần theo training.

### WandB Dashboard

Truy cập https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab để xem biểu đồ realtime.

### Các mốc quan trọng

| Iteration | Kỳ vọng |
|-----------|---------|
| 0 - 1,000 | Robot bắt đầu đứng được |
| 1,000 - 5,000 | Bắt chước tổng thể |
| 5,000 - 10,000 | Chi tiết cải thiện |
| 10,000 - 20,000 | Tinh chỉnh, ổn định |

---

## Bước 8: Chạy trong tmux (quan trọng!)

> [!WARNING]
> Nếu chạy training qua SSH thông thường, **tắt laptop = mất training**. Luôn dùng `tmux`.

### Chạy training mới trong tmux

```bash
ssh nguyenl3@10.148.255.113

# Tạo tmux session
tmux new -s training

# Bên trong tmux, chạy training
cd ~/Documents/vm_mjlab
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/neymar_celebrate.npz

# Detach (thoát mà không kill): Ctrl+B rồi nhấn D
```

### Resume training từ checkpoint (trong tmux)

```bash
tmux new -s training
cd ~/Documents/vm_mjlab

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/neymar_celebrate.npz \
  --agent.load-run <ten-thu-muc-run> \
  --agent.resume True
```

> [!NOTE]
> - `--agent.load-run` chỉ cần **tên thư mục** (VD: `2026-04-16_10-02-59`), không cần full path
> - `--agent.resume True` phải viết hoa chữ `True`
> - Resume sẽ load weights rồi chạy thêm `max_iterations` nữa (không phải tiếp tục đếm cũ)

### Quản lý tmux session

```bash
# Xem danh sách sessions
tmux ls

# Attach lại vào session
tmux attach -t training

# Detach: Ctrl+B rồi D

# Kill session
tmux kill-session -t training
```

---

## Bước 9: Theo dõi training từ xa

### Lệnh kiểm tra nhanh (chạy trên laptop)

```bash
# Xem iteration + reward + ETA
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "tmux capture-pane -t training -p -S -100 | grep -E 'Learning iteration|Mean reward|Mean episode|Time elapsed|ETA'"

# Xem toàn bộ output gần nhất
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "tmux capture-pane -t training -p | tail -25"

# Xem checkpoint mới nhất
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "ls -t ~/Documents/vm_mjlab/logs/rsl_rl/m2v6_tracking/*/model_*.pt | head -3"
```

### WandB Dashboard (xem biểu đồ realtime)

Truy cập https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab

Xem chi tiết cách đọc biểu đồ: [Hướng dẫn đọc WandB Dashboard](11-huong-dan-doc-wandb-dashboard.md)

---

## Bước 10: Đánh giá kết quả (play)

### Tạo video đánh giá trên server (headless)

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

# Góc trước mặt
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --checkpoint-file logs/rsl_rl/m2v6_tracking/2026-04-16_11-59-42/model_20500.pt \
  --motion-file motions/neymar_celebrate.npz \
  --num-envs 1 --video True --video-length 849 \
  --video-width 1920 --video-height 1080 \
  --camera side-right
```

> [!NOTE]
> - `--video-length 849` = số frames của motion file (17s × 50fps)
> - Video lưu tại: `logs/rsl_rl/m2v6_tracking/<run-dir>/videos/play/rl-video-step-0.mp4`
> - Syntax play **khác** syntax train: dùng `--checkpoint-file`, `--motion-file`, `--num-envs` (không có `--env.` hay `--agent.`)

### Xem interactive (Viser 3D viewer)

```bash
# Terminal 1 trên laptop: Port forward
sshpass -p '1' ssh -L 8080:localhost:8080 nguyenl3@10.148.255.113 -N &

# Terminal 2 trên server (hoặc tmux):
cd ~/Documents/vm_mjlab
# Góc trước mặt
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --checkpoint-file logs/rsl_rl/m2v6_tracking/2026-04-16_11-59-42/model_20500.pt \
  --motion-file motions/neymar_celebrate.npz \
  --num-envs 1 --video True --video-length 849 \
  --video-width 1920 --video-height 1080 \
  --camera side-right


# Mở browser → http://localhost:8080
# Robot xám = policy output (robot thật)
# Robot xanh trong suốt = reference motion (mẫu cần bắt chước)
```

### Tải video về laptop (chạy lệnh ở local)

```bash
sshpass -p '1' scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_tracking/2026-04-16_11-59-42/videos/play/rl-video-step-0.mp4 ~/Documents/Humanoid_Tracking_Task/data/videos/neymar_celebrate_eval_22000.mp4
```

---

## Tổng hợp lỗi đã gặp & cách fix

### Lỗi 1: `ImportError: cannot import name 'arm_extension'`

**Nguyên nhân:** Module `safefall` trên server thiếu hàm `arm_extension` trong `rewards.py`.

**Fix:** Thêm stub function vào `rewards.py` (xem Bước 3).

### Lỗi 2: `ImportError: cannot import name 'base_height'`

**Nguyên nhân:** Sau khi fix `arm_extension`, lại thiếu `base_height`, `head_orientation`, `self_collision_penalty`.

**Fix:** Thêm tất cả stub cùng lúc (xem Bước 3). **Bài học: kiểm tra toàn bộ `__init__.py` trước khi fix từng cái một.**

### Lỗi 3: `ModuleNotFoundError: No module named 'mjlab.tasks.tracking.config.m2v6'`

**Nguyên nhân:** Server cũ không có config M2v6.

**Fix:** Copy từ laptop (xem Bước 2.2).

### Lỗi 4: `ModuleNotFoundError: No module named 'mjlab.asset_zoo.robots.M2v6'`

**Nguyên nhân:** Server cũ không có robot M2v6 assets.

**Fix:** Copy từ laptop (xem Bước 2.1).

### Lỗi 5: `TypeError: RslRlModelCfg.__init__() got an unexpected keyword argument 'distribution_cfg'`

**Nguyên nhân:** Laptop code mới dùng `distribution_cfg`, server code cũ không hỗ trợ.

**Fix:** Thay bằng `stochastic=True` trong actor config (xem Bước 2.3).

### Lỗi 6: `AttributeError: 'NoneType' object has no attribute 'log_prob'`

**Nguyên nhân:** Actor model không có distribution → `self.distribution` là `None`. Do bước fix lỗi 5 ban đầu chỉ xóa `distribution_cfg` mà quên thêm `stochastic=True`.

**Fix:** Thêm `stochastic=True` vào actor config. Critic **không** cần stochastic.

### Lỗi 7: `uv: command not found` (khi SSH non-interactive)

**Nguyên nhân:** SSH non-interactive không load `~/.bashrc`, `uv` nằm ở `~/.local/bin/` không trong PATH.

**Fix:** Thêm `export PATH=/home/nguyenl3/.local/bin:$PATH` hoặc dùng interactive SSH.

---

## Thứ tự thực hiện (checklist)

```
[ ] 1. Copy CSV lên server
[ ] 2. Sync M2v6 assets (robot + config + fix rl_cfg.py)
[ ] 3. Fix safefall stubs (3 hàm)
[ ] 4. Đăng nhập WandB
[ ] 5. Chạy CSV → NPZ
[ ] 6. Kiểm tra video NPZ
[ ] 7. Bắt đầu training (trong tmux!)
[ ] 8. Theo dõi trên WandB + terminal
[ ] 9. Đánh giá kết quả (play + video)
[ ] 10. Tải video về laptop
```
