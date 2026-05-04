# Guide đầy đủ — Train mimic M2v6: `m26_standingup_normal` và `neymar_samba`

Hướng dẫn thủ công từng bước để huấn luyện motion imitation cho robot **M2v6** với 2 động tác:
1. **m26_standingup_normal** — robot đứng dậy từ tư thế nằm
2. **neymar_samba** (tên file: `neymar_celebrate`) — điệu nhảy ăn mừng của Neymar

Toàn bộ training chạy trên **workstation RTX 5090** qua SSH. Laptop chỉ dùng để theo dõi và tải kết quả.

---

## 0. Tổng quan kiến trúc

```
┌──────────── LAPTOP (RTX 5070, 8GB) ────────────┐
│  /home/nguyenld12/Documents/Humanoid_Tracking_Task/  │
│   ├── mjlab/                   ← code gốc (mới nhất) │
│   ├── editor_motion/             ← NPZ tham chiếu       │
│   ├── vm_retargeting/output/   ← CSV từ GVHMR         │
│   └── docs/                    ← tài liệu             │
└──────────────────────────────────────────────────┘
                      │ SSH + SCP
                      ▼
┌─────────── WORKSTATION (RTX 5090, 32GB) ───────────┐
│  nguyenl3@10.148.255.113   (password: 1)             │
│  /home/nguyenl3/Documents/vm_mjlab/                  │
│   ├── src/mjlab/…              ← code (cũ hơn chút)  │
│   ├── motions/                 ← NPZ/CSV đã sẵn sàng │
│   │    ├── neymar_celebrate.npz  (849 frames)        │
│   │    ├── neymar_celebrate.csv                      │
│   │    └── standing_up_m26.npz   (430 frames)        │
│   └── logs/rsl_rl/…            ← checkpoint + onnx   │
└──────────────────────────────────────────────────┘
```

**Thông tin training task:**

| Mục | Giá trị |
|---|---|
| Task ID | `Mjlab-Tracking-Flat-M2v6-No-State-Estimation` |
| Control frequency | 50 Hz (motion NPZ phải ở 50 fps) |
| Num envs (RTX 5090) | 4096 |
| PPO steps/env | 24 → 98 304 samples / iteration |
| Actor/Critic | MLP 512-256-128 + ELU |
| Save interval | 500 iter |
| Max iterations (server) | 20 000 |
| ETA | 3-4 giờ cho ~20 000 iter |

---

## 1. Kiểm tra điều kiện tiên quyết

Tất cả các phần chuẩn bị sau đây **đã được hoàn tất trước đó** — mục này chỉ để bạn verify trước khi bắt đầu.

### 1.1. SSH và thư mục server

```bash
ssh nguyenl3@10.148.255.113         # pass: 1
cd ~/Documents/vm_mjlab
nvidia-smi                          # phải thấy RTX 5090, 32GB
uv --version                        # uv đã cài
```

### 1.2. Kiểm tra module M2v6 đã được sync lên server

```bash
# Các đường dẫn sau đều PHẢI tồn tại:
ls ~/Documents/vm_mjlab/src/mjlab/asset_zoo/robots/M2v6/        # robot asset
ls ~/Documents/vm_mjlab/src/mjlab/tasks/tracking/config/m2v6/   # tracking config
ls ~/Documents/vm_mjlab/src/mjlab/scripts/csv_to_npz_m2v6.py    # script convert
```

Nếu thiếu, xem lại **doc 10** (đã có full hướng dẫn sync) — quick recap:

```bash
# Từ laptop (chỉ làm khi server thiếu file):
scp -r ~/Documents/Humanoid_Tracking_Task/mjlab/src/mjlab/asset_zoo/robots/M2v6 \
  nguyenl3@10.148.255.113:~/Documents/vm_mjlab/src/mjlab/asset_zoo/robots/

scp -r ~/Documents/Humanoid_Tracking_Task/mjlab/src/mjlab/tasks/tracking/config/m2v6 \
  nguyenl3@10.148.255.113:~/Documents/vm_mjlab/src/mjlab/tasks/tracking/config/
```

### 1.3. Kiểm tra `rl_cfg.py` đang dùng API cũ (`stochastic=True`)

Server dùng phiên bản rsl_rl cũ — nếu code local mới dùng `distribution_cfg` thì phải chuyển sang `stochastic=True`:

```bash
grep -E 'distribution_cfg|stochastic' \
  ~/Documents/vm_mjlab/src/mjlab/tasks/tracking/config/m2v6/rl_cfg.py
# Mong đợi: chỉ thấy `stochastic=True`, KHÔNG có `distribution_cfg`
```

### 1.4. Kiểm tra stub safefall đã được patch

```bash
grep -E 'def (base_height|head_orientation|self_collision_penalty)' \
  ~/Documents/vm_mjlab/src/mjlab/tasks/safefall/mdp/rewards.py
# Phải thấy cả 3 hàm
```

### 1.5. Motion files đã sẵn trên server

```bash
ls -la ~/Documents/vm_mjlab/motions/
# Mong đợi:
#   neymar_celebrate.csv       (199 KB)
#   neymar_celebrate.npz       (1.5 MB, 849 frames @ 50 fps)
#   neymar_celebrate.mp4       (video tham chiếu)
#   standing_up_m26.npz        (1.4 MB, 430 frames @ 50 fps)
```

Nếu thiếu `standing_up_m26.npz`, có bản gốc ở laptop:
```bash
scp ~/Documents/Humanoid_Tracking_Task/editor_motion/standing_up_m26.npz \
  nguyenl3@10.148.255.113:~/Documents/vm_mjlab/motions/
```

### 1.6. WandB đã đăng nhập trên server

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab
uv run wandb login   # paste API key từ https://wandb.ai/authorize
# Dashboard: https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab
```

Nếu không muốn dùng WandB, đặt `WANDB_MODE=disabled` trước lệnh train.

---

## 2. (Tuỳ chọn) Tạo lại NPZ từ CSV

Bỏ qua phần này nếu đã có sẵn NPZ. Nếu muốn tạo lại (ví dụ, sửa FPS hay cắt khúc):

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

# neymar_samba (neymar_celebrate)
MUJOCO_GL=egl uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
  --input-file  motions/neymar_celebrate.csv \
  --output-file motions/neymar_celebrate.npz \
  --input-fps 30 --output-fps 50 --render True

# m26_standingup_normal (nếu có CSV — thường standing_up_m26.npz được tạo sẵn từ motion-editor)
# MUJOCO_GL=egl uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
#   --input-file  motions/standing_up_m26.csv \
#   --output-file motions/standing_up_m26.npz \
#   --input-fps 30 --output-fps 50 --render True
```

Sau khi chạy sẽ có thêm file `.mp4` cùng thư mục để kiểm tra bằng mắt.

---

## 3. Training — `m26_standingup_normal`

### 3.1. Vào tmux (BẮT BUỘC — đừng chạy SSH trần!)

```bash
ssh nguyenl3@10.148.255.113
tmux new -s standup
cd ~/Documents/vm_mjlab
```

### 3.2. Lệnh training

```bash
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/standing_up_m26.npz \
  --agent.experiment-name m2v6_standup_v6
```

Giải thích flag:

| Flag | Vai trò |
|---|---|
| `Mjlab-Tracking-Flat-M2v6-No-State-Estimation` | Task không yêu cầu state estimation (phù hợp robot thật) |
| `--env.scene.num-envs 4096` | 4096 robot song song (tận dụng 32GB VRAM) |
| `--env.commands.motion.motion-file <npz>` | NPZ motion tham chiếu |
| `--agent.experiment-name m2v6_standup_v6` | Tên thư mục log riêng (tăng số version mỗi lần chạy mới) |

Checkpoint lưu tại: `logs/rsl_rl/m2v6_standup_v6/<timestamp>/model_*.pt`.

### 3.3. Detach tmux và theo dõi

Nhấn `Ctrl+B` → `D` để thoát tmux mà vẫn giữ training chạy nền.

Attach lại bất kỳ lúc nào:
```bash
ssh nguyenl3@10.148.255.113
tmux attach -t standup
```

---

## 4. Training — `neymar_samba` (neymar_celebrate)

### 4.1. Tạo tmux session riêng (tách khỏi standup)

```bash
ssh nguyenl3@10.148.255.113
tmux new -s neymar
cd ~/Documents/vm_mjlab
```

> [!WARNING]
> **Không** chạy 2 training cùng lúc trên 1 GPU — sẽ hết VRAM. Chờ `standup` xong rồi mới bắt đầu `neymar`, hoặc giảm mỗi bên xuống 2048 envs.

### 4.2. Lệnh training

```bash
uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/neymar_celebrate.npz \
  --agent.experiment-name m2v6_neymar_v2
```

Checkpoint lưu tại: `logs/rsl_rl/m2v6_neymar_v2/<timestamp>/model_*.pt`.

### 4.3. Detach

`Ctrl+B` → `D`.

---

## 5. Theo dõi training

### 5.1. Nhanh — từ laptop (không cần SSH interactive)

```bash
# Iteration + reward + ETA (standup)
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "tmux capture-pane -t standup -p -S -100 | \
   grep -E 'Learning iteration|Mean reward|Mean episode|ETA'"

# Tương tự cho neymar
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "tmux capture-pane -t neymar -p -S -100 | \
   grep -E 'Learning iteration|Mean reward|Mean episode|ETA'"

# Checkpoint mới nhất
sshpass -p '1' ssh nguyenl3@10.148.255.113 \
  "ls -t ~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_v6/*/model_*.pt | head -3"
```

### 5.2. WandB dashboard

https://wandb.ai/nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab

Filter theo `experiment_name` (`m2v6_standup_v6`, `m2v6_neymar_v2`) để xem biểu đồ riêng.

### 5.3. Các metric cần nhìn

| Metric | Xu hướng tốt |
|---|---|
| `Mean reward` | Tăng đều |
| `Mean episode length` | Tăng (robot sống lâu hơn) |
| `motion_global_root_pos` / `_ori` | Tăng về ~1 |
| `motion_body_pos` / `_ori` | Tăng về ~1 |
| `action_rate_l2` | Gần 0 |
| `self_collisions` | Gần 0 |
| `Termination/ee_body_pos` | Giảm theo thời gian |

### 5.4. Mốc kỳ vọng

| Iter | Standup | Neymar samba |
|---|---|---|
| 0-1 000 | Robot tập đứng được trên đất | Robot cân bằng, chưa theo beat |
| 1 000-5 000 | Bắt chước tổng thể động tác | Tay chân theo tổng quan điệu nhảy |
| 5 000-10 000 | Gối/hông chính xác hơn | Nhịp tay khớp với mẫu |
| 10 000-20 000 | Tinh chỉnh, chân đặt chắc | Chuyển động mượt, ít giật |

---

## 6. Resume training từ checkpoint

Nếu bị ngắt giữa chừng hoặc muốn train thêm:

```bash
ssh nguyenl3@10.148.255.113
tmux new -s standup   # hoặc attach lại session cũ
cd ~/Documents/vm_mjlab

uv run train Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --env.scene.num-envs 4096 \
  --env.commands.motion.motion-file motions/standing_up_m26.npz \
  --agent.experiment-name m2v6_standup_v6 \
  --agent.load-run 2026-04-17_14-50-41 \
  --agent.resume True
```

Lưu ý:
- `--agent.load-run` chỉ cần **tên thư mục timestamp**, không cần full path.
- `--agent.resume True` — `T` viết hoa.
- Resume load weights rồi chạy tiếp `max_iterations` mới (reset counter).

---

## 7. Đánh giá policy (play + video)

Sau khi train xong (hoặc đã có checkpoint tốt):

### 7.1. Render video headless trên server

```bash
ssh nguyenl3@10.148.255.113
cd ~/Documents/vm_mjlab

# Ví dụ với standup, model_15000 — chỉnh path cho phù hợp
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --checkpoint-file logs/rsl_rl/m2v6_standup_v6/<timestamp>/model_15000.pt \
  --motion-file motions/standing_up_m26.npz \
  --num-envs 1 --video True --video-length 430 \
  --video-width 1920 --video-height 1080 \
  --camera side-right

# neymar
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --checkpoint-file logs/rsl_rl/m2v6_neymar_v2/<timestamp>/model_15000.pt \
  --motion-file motions/neymar_celebrate.npz \
  --num-envs 1 --video True --video-length 849 \
  --video-width 1920 --video-height 1080 \
  --camera side-right
```

`--video-length` phải bằng số frame NPZ (`standing_up_m26.npz` = 430, `neymar_celebrate.npz` = 849).

Video được ghi tại: `logs/rsl_rl/<exp>/<timestamp>/videos/play/rl-video-step-0.mp4`.

Lưu ý syntax khác train: dùng `--checkpoint-file`, `--motion-file`, `--num-envs` (không có `--env.*` hay `--agent.*`).

### 7.2. Xem interactive 3D (Viser) — tuỳ chọn

Terminal 1, trên laptop:
```bash
sshpass -p '1' ssh -L 8080:localhost:8080 nguyenl3@10.148.255.113 -N
```

Terminal 2, trên server:
```bash
cd ~/Documents/vm_mjlab
MUJOCO_GL=egl uv run play Mjlab-Tracking-Flat-M2v6-No-State-Estimation \
  --checkpoint-file logs/rsl_rl/m2v6_standup_v6/<timestamp>/model_15000.pt \
  --motion-file motions/standing_up_m26.npz \
  --num-envs 1
```

Mở `http://localhost:8080` trên trình duyệt laptop.

### 7.3. Tải video về laptop

```bash
# Chạy trên laptop
sshpass -p '1' scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_v6/<timestamp>/videos/play/rl-video-step-0.mp4 \
  ~/Documents/Humanoid_Tracking_Task/data/videos/standup_v6_15000.mp4

sshpass -p '1' scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_neymar_v2/<timestamp>/videos/play/rl-video-step-0.mp4 \
  ~/Documents/Humanoid_Tracking_Task/data/videos/neymar_v2_15000.mp4
```

---

## 8. Export ONNX để deploy robot thật

Mỗi run đã tự export 1 file `.onnx` tại thư mục log (`<timestamp>.onnx`). Lấy file đó copy về:

```bash
# Trên laptop
sshpass -p '1' scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_standup_v6/<timestamp>/<timestamp>.onnx \
  ~/Documents/Humanoid_Tracking_Task/mjlab/params/m26_standingup_normal/

sshpass -p '1' scp nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/m2v6_neymar_v2/<timestamp>/<timestamp>.onnx \
  ~/Documents/Humanoid_Tracking_Task/mjlab/params/neymar_celebrate/
```

---

## 9. Checklist hoàn chỉnh (dán ra giấy, tick từng bước)

- [ ] SSH được vào server, `nvidia-smi` OK
- [ ] `src/mjlab/asset_zoo/robots/M2v6/` tồn tại
- [ ] `src/mjlab/tasks/tracking/config/m2v6/` tồn tại
- [ ] `rl_cfg.py` dùng `stochastic=True` (không `distribution_cfg`)
- [ ] Stub `base_height / head_orientation / self_collision_penalty` có trong `safefall/mdp/rewards.py`
- [ ] `motions/standing_up_m26.npz` tồn tại
- [ ] `motions/neymar_celebrate.npz` tồn tại
- [ ] WandB đăng nhập (hoặc đã đặt `WANDB_MODE=disabled`)
- [ ] Tmux session `standup` đã chạy train
- [ ] Tmux session `neymar` đã chạy train (sau khi standup xong hoặc giảm envs)
- [ ] Đã theo dõi WandB / tmux, reward tăng ổn định
- [ ] Checkpoint `model_15000.pt` trở lên đã có
- [ ] Đã render video play cho cả 2 motion
- [ ] Đã copy ONNX về laptop

---

## 10. Xử lý sự cố thường gặp

| Triệu chứng | Nguyên nhân | Giải pháp |
|---|---|---|
| `ImportError: base_height` khi import tracking | Stub safefall chưa patch | Xem §1.4, thêm stub |
| `unexpected keyword distribution_cfg` | `rl_cfg.py` dùng API mới | Thay bằng `stochastic=True` (§1.3) |
| CUDA OOM khi train | 4096 quá nhiều / train 2 lúc | Giảm `--env.scene.num-envs 2048` |
| Reward không tăng sau 2000 iter | Motion lỗi hoặc fps sai | Render video `.mp4` NPZ để kiểm |
| Episode quá ngắn | Termination `ee_body_pos` nghiêm khắc | Bình thường ở early training, sẽ giảm dần |
| Mất SSH, training vẫn chạy không | Không dùng tmux | Kill session — tạo tmux mới lần sau |
| `play` báo lỗi flag `--env.*` | Syntax play khác train | Dùng `--checkpoint-file --motion-file --num-envs` |

---

## 11. Tham khảo chéo

- [02 — Thiết lập môi trường](02-thiet-lap-moi-truong.md)
- [03 — Cấu trúc robot M2v6](03-cau-truc-robot-m2v6.md)
- [07 — Huấn luyện Motion Imitation](07-huan-luyen-motion-imitation.md)
- [08 — Đánh giá policy](08-danh-gia-policy.md)
- [10 — Hướng dẫn train neymar_celebrate (đã ghi đầy đủ lỗi gặp phải)](10-huong-dan-train-neymar-celebrate.md)
- [11 — Đọc WandB dashboard](11-huong-dan-doc-wandb-dashboard.md)
