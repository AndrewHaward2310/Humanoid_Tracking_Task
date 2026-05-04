# Pretrain Standup — fix 2 issues

Hai vấn đề cần khắc phục sau khi deploy standup:
1. **Hand va đất** lúc chống push-up → cần bàn tay gập lên, chỉ wrist chạm đất
2. **Forward drift** sau khi đứng dậy → phải đứng im tại chỗ, không mất cân bằng

Hai lần pretrain **tuần tự** — vấn đề 1 trước (chấp nhận một chút drift), rồi fix vấn đề 2 trên nền policy đã fix hand.

---

## Pretrain #1 — Fix hand collision

### Vấn đề với lần fix cũ

- Penalty weight **−0.5** quá nặng → policy học "tránh mọi contact tay-đất" → không dám chống đất → không đứng được (iter 7k+ fail).

### Fix lần này

- Weight **−0.15** (nhẹ hơn 3x)
- `force_threshold` 20 → **30 N** (chỉ penalty khi va đất mạnh, không phạt touch nhẹ)
- **Warm-start** từ policy đã biết standup (`m2v6_standup_robust/model_15000`) → chỉ adjust wrist flex, không quên standup
- Sensor đã target đúng `hand_collision` geom (không phạt wrist_roll_collision)

### Chạy

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run wandb login    # (1 lần, lần đầu)

tmux new -s standup_v2
bash scripts/train_standup_nohand_v2.sh
# Ctrl+B D để detach
```

**Thời gian**: ~3h (8k iter, 1024 envs).

### Theo dõi

```bash
uv run python scripts/check_reward_curve_local.py m2v6_standup_nohand_v2
```

Metric cần xem:
- `Episode_Reward/hand_ground_contact` — nên giảm về ~0 sau iter 3000-5000 (policy học flex wrist)
- `Train/mean_reward` — giữ ổn định 25-35, không tụt âm
- `Episode_Reward/motion_body_pos` — giữ 0.85+ (standup vẫn tracking OK)
- `Train/mean_episode_length` — giữ 450+/500

### Deploy sau pretrain #1

Chọn iter peak rồi deploy:
```bash
TS=$(ls -t logs/rsl_rl/m2v6_standup_nohand_v2/ | grep -v warmstart | head -1)
BEST=???  # iter peak

CKPT="logs/rsl_rl/m2v6_standup_nohand_v2/$TS/model_${BEST}.pt"
NPZ="$HOME/Documents/Humanoid_Tracking_Task/experiments/neymar_samba/DATA/m26_standingup_normal.npz"

uv run python scripts/reexport_onnx.py "$CKPT" "$NPZ" params/m26_standingup_normal/policy.onnx
uv run python load_metadata_onnx.py \
  params/m26_standingup_normal/policy.onnx \
  params/m26_standingup_normal/param_for_m26_standup.csv

DEST=~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/standup
cp params/m26_standingup_normal/policy.onnx $DEST/
cp params/m26_standingup_normal/param_for_m26_standup.csv $DEST/
```

Test sim: `1` → `=`. Verify:
- Robot đứng dậy thành công
- **Không có bàn tay chạm đất** (bật `F` trong MuJoCo để xem contact force)
- Drift forward vẫn có (fix sau ở pretrain #2)

---

## Pretrain #2 — Fix forward drift

### Vấn đề

Motion `m26_standingup_normal.npz` (309 frames) — frame cuối là robot vừa đứng lên, chưa có phase "đứng im". Policy không được train để maintain pose frame 308.

### Fix

- **Extend NPZ**: append 150 frame copy của frame 308, zero velocity → tổng 459 frames (9.18s). Policy thấy frame 308 lặp 3s → học hold.
- Retrain warm-start từ policy pretrain #1 (đã fix hand).
- Update YAML `end/hold = 458` (match frame count mới).

### Bước 1 — Tạo NPZ extended

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run python scripts/extend_npz_hold.py \
  ~/Documents/Humanoid_Tracking_Task/experiments/neymar_samba/DATA/m26_standingup_normal.npz \
  ~/Documents/Humanoid_Tracking_Task/experiments/neymar_samba/DATA/m26_standingup_hold3s.npz \
  150
```

### Bước 2 — Tạo train script

Copy `train_standup_nohand_v2.sh` thành `train_standup_stable.sh`, sửa:
- `SRC_CKPT` → checkpoint peak của pretrain #1
- `EXP_NAME="m2v6_standup_stable"`
- `MOTION_FILE` → path `m26_standingup_hold3s.npz`
- `MAX_ITERATIONS=5000` (đủ để learn hold)

### Bước 3 — Chạy

```bash
tmux new -s standup_stable
bash scripts/train_standup_stable.sh
```

### Bước 4 — Deploy

Re-export + copy vào vm_packages như thường.

**Update YAML** `track_standingup.yaml`:
```yaml
motion:
  time_rate: 1.0
  init: 0
  end: 458        # 459 frames - 1
  hold: 458       # hold tại pose cuối (bây giờ pose này đã được học HOLD sẵn trong training)
```

---

## Bảng tổng hợp

| Pretrain | Mục tiêu | Thay đổi | Iter | Thời gian |
|---|---|---|---|---|
| #1 | Fix hand va đất | Weight penalty −0.5→−0.15, threshold 20→30, warm-start | 8000 | ~3h |
| #2 | Fix forward drift | Extend NPZ +150 frame hold, warm-start từ #1 | 5000 | ~2h |

Tổng: ~5h train + deploy test.

---

## Lưu ý quan trọng

1. **KHÔNG train cả 2 issue cùng lúc** — khó debug nếu regression. Fix sequentially.
2. **Warm-start có dependency chain**:
   - Pretrain #1 warm-start từ `m2v6_standup_robust/model_15000` (policy biết standup)
   - Pretrain #2 warm-start từ `m2v6_standup_nohand_v2/model_<peak>` (policy biết standup + không va hand)
3. **Theo dõi hand_ground_contact reward** — nếu âm mạnh kéo dài → penalty quá cao, cần giảm weight tiếp.
4. **Nếu robot không đứng được sau pretrain #1** → giảm weight xuống −0.1 thậm chí −0.05, retrain.
5. **Motion NPZ extended** chỉ dùng cho training. Deploy vẫn dùng NPZ gốc 309 frames cho gọn, không sao vì ONNX đã bake motion vào.

---

## Tham khảo chéo

- [13 — full pipeline](13-full-pipeline-train-to-sim-deploy.md)
- [14 — export checkpoint](14-export-checkpoint-to-vm-packages.md)
- `scripts/train_standup_nohand_v2.sh` — pretrain #1 script
- `scripts/extend_npz_hold.py` — extend NPZ tool
- `mjlab/src/mjlab/tasks/tracking/config/m2v6/env_cfgs.py:48-52` — reward config
