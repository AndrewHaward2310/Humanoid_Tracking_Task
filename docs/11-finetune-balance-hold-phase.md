# 11. Finetune Balance — Kéo dài Hold Phase

Hướng dẫn fix vấn đề robot đứng dậy được nhưng mất thăng bằng / drift ngã sau khi hoàn tất standup.

## Bối cảnh

Sau khi train policy đứng dậy (handfix v5) với motion `m26_standingup_handfix_v3_hold3s.npz` (309 standup + 150 hold = 459 frames, hold 3s), robot:

- ✅ Đứng lên được
- ✅ Tay không còn chạm sàn
- ❌ Mất thăng bằng, drift 1 đoạn rồi ngã sau pha đứng

### Root cause

- Motion training chỉ có 150 hold frames (~3s tĩnh)
- Sim freeze tại frame cuối sau khi hết motion → policy phải giữ balance vô thời hạn trên target tĩnh
- Policy không được train đủ trên long-hold nên drift accumulate không corrective

## Các hướng giải quyết

| # | Cách | Ưu | Nhược |
|---|---|---|---|
| 1 | **Kéo hold frames 150 → 500** | Dễ, chỉ regenerate motion | Không teach active balance |
| 2 | Push disturbance DR (random impulse base) | Teach active balance | Cần sửa code DR term |
| 3 | Stability reward (penalize lin/ang vel) | Direct signal | Phải sửa `env_cfgs.py` + rewards |

Hướng dẫn dưới đi theo **cách 1** (nhẹ nhất).

## Chuẩn bị: fix PATH cho `uv`

Nếu shell (zsh/bash) báo `uv: command not found`, chạy lệnh dưới 1 lần để fix vĩnh viễn:

```bash
# zsh
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.zshrc && source ~/.zshrc

# bash
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc && source ~/.bashrc
```

Hoặc tạm thời dùng full path: `~/.local/bin/uv run python ...`

Training script `train_standup_handfix_v6.sh` đã có `export PATH` sẵn nên chạy `bash scripts/...` sẽ OK — nhưng các lệnh `uv` gõ tay vẫn cần PATH đúng.

## Quy trình

### Bước 1: Tạo motion mới với hold dài

Dùng script `fk_rebuild_and_hold.py` (có sẵn trong `mjlab/scripts/`):

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run python scripts/fk_rebuild_and_hold.py \
  --npz "/home/nguyenld12/Downloads/edited (7).npz" \
  --out /tmp/m26_standingup_handfix_hold10s.npz \
  --hold-frames 500
```

→ Output: 309 + 500 = **809 frames** (~16s motion, 10s hold tĩnh ở cuối).

Script này:
- Load joint_pos từ NPZ nguồn
- Rebuild body_pos_w / body_quat_w / body_lin_vel_w / body_ang_vel_w qua `mj_forward`
- Extend thêm `--hold-frames` frames copy từ frame cuối cùng

### Bước 2: Setup warmstart folder

Copy checkpoint peak từ v5 vào folder warmstart mới:

```bash
mkdir -p ~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_standup_handfix/warmstart_iter25000
cp ~/Downloads/model_25000.pt \
   ~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_standup_handfix/warmstart_iter25000/
```

`model_25000.pt` = peak v5 (train với hand capsule mở rộng + hand_ground_contact weight -0.07 + body_mass DR).

### Bước 3: Tạo training script

```bash
cat > ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/train_standup_handfix_v6.sh << 'EOF'
#!/bin/bash
# v6: finetune balance — extend hold frames 150→500 (~10s) + warmstart v5 peak
set -e
export PATH="$HOME/.local/bin:$PATH"
MJLAB_DIR="$HOME/Documents/Humanoid_Tracking_Task/mjlab"
cd "$MJLAB_DIR"

EXP_NAME="m2v6_standup_handfix"
WARMSTART_TS="warmstart_iter25000"
NUM_ENVS=1024               # giảm từ 4096 (server) theo GPU local
MAX_ITERATIONS=3000
MOTION_FILE="/tmp/m26_standingup_handfix_hold10s.npz"

[ ! -f "$MOTION_FILE" ] && { echo "motion missing"; exit 1; }
[ ! -f "logs/rsl_rl/$EXP_NAME/$WARMSTART_TS/model_25000.pt" ] && { echo "warmstart missing"; exit 1; }

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs "$NUM_ENVS" \
  --env.commands.motion.motion-file "$MOTION_FILE" \
  --agent.experiment-name "$EXP_NAME" \
  --agent.max-iterations "$MAX_ITERATIONS" \
  --agent.load-run "$WARMSTART_TS" \
  --agent.resume True \
  --agent.logger wandb
EOF
chmod +x ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/train_standup_handfix_v6.sh
```

### Bước 4: Launch training

```bash
bash ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/train_standup_handfix_v6.sh
```

### Bước 5: Deploy sau khi train

```bash
# Pick checkpoint gần peak (xem wandb)
bash ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/deploy_standup.sh \
  logs/rsl_rl/m2v6_standup_handfix/<timestamp>/model_<iter>.pt \
  /tmp/m26_standingup_handfix_hold10s.npz
```

Lưu ý motion path **phải khớp với motion đã train**. Nếu dùng sai motion → policy track sai target.

## Tham số khuyến nghị

| Param | Giá trị | Ghi chú |
|---|---|---|
| `num_envs` | 1024 (local) / 4096 (server) | Theo VRAM |
| `max_iterations` | 3000 | Finetune ngắn vì warmstart đã tốt |
| `hold-frames` | 500 | ~10s. Có thể tăng 1000 nếu cần hold lâu hơn |
| `--agent.resume True` | bắt buộc | Để load warmstart checkpoint |

## Monitor convergence

Dùng `scripts/check_reward_curve.py` với wandb run name:

```bash
uv run python scripts/check_reward_curve.py <run_name>
```

Metric quan tâm:
- **`Episode_Reward/motion_body_pos`**: ≥ 0.92 là tốt
- **`Episode_Termination/time_out`**: nên tăng lên (ít bị terminate sớm = đứng vững hơn)
- **`Episode_Termination/anchor_pos` / `anchor_ori`**: giảm (ít ngã ra khỏi ngưỡng tracking)
- **`Episode_Reward/hand_ground_contact`**: giữ ~0 (không được tệ đi so với v5)

Dừng sớm khi metrics plateau 500+ iter.

## Troubleshooting

| Triệu chứng | Nguyên nhân | Fix |
|---|---|---|
| Motion file not found | `/tmp` bị xóa | Chạy lại Bước 1 |
| `uv: command not found` | `~/.local/bin` không trong PATH | Xem phần "Chuẩn bị" ở đầu doc — thêm vào `~/.zshrc` hoặc `~/.bashrc` |
| `motion missing` khi chạy script | Chưa tạo NPZ (skip bước 1) | Quay lại Bước 1 chạy `fk_rebuild_and_hold.py` trước |
| `warmstart missing` | Chưa copy model_25000.pt | Quay lại Bước 2 |
| Training không tiến triển | Warmstart policy đã quá fit cho hold ngắn | Tăng max_iterations lên 5000 hoặc giảm num_envs để học chậm hơn |
| Robot vẫn ngã sau hold dài | Motion hold không đủ, cần active balance | Chuyển sang cách 2 hoặc 3 trong bảng trên |

## Tham khảo thêm

- `scripts/fk_rebuild_and_hold.py` — motion surgery utility
- `scripts/deploy_standup.sh` — deploy pipeline (.pt → ONNX + CSV → vm_packages)
- `scripts/check_reward_curve.py` — analyze wandb run
- Session log: `~/Documents/Claude/Projects/mjlab_local/conversations/handfix_session_2026-04-19_to_20.md`
