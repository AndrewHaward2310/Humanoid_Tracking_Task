# Export checkpoint mjlab → vm_packages sim (M2v6)

Guide quy trình pull ONNX từ server, sinh CSV metadata, deploy vào `vm_packages` để test sim. Dựa trên các lệnh đã chạy thực tế cho motion `m26_standingup_normal` ngày 17/04/2026.

**Pre-requisite:** đã train xong policy trong mjlab, ONNX file đã tồn tại ở `logs/rsl_rl/<exp>/<ts>/<ts>.onnx` trên server.

---

## Tổng quan workflow

```
SERVER (train)                          LAPTOP (deploy)
┌──────────────────────┐                ┌─────────────────────────────────┐
│ logs/rsl_rl/<exp>/   │   (1) scp      │ mjlab/params/<motion>/          │
│   <ts>/<ts>.onnx     │  ───────────►  │   policy.onnx                    │
└──────────────────────┘                │                                  │
                                         │ (2) uv run python ← parse ONNX   │
                                         │ metadata → CSV                   │
                                         │                                  │
                                         │ mjlab/params/<motion>/           │
                                         │   param_for_m26_<motion>.csv    │
                                         │                                  │
                                         │ (3) cp ONNX+CSV                  │
                                         │                                  │
                                         │ vm_packages/.../mimic/<motion>/ │
                                         │   policy.onnx                    │
                                         │   param_for_m26_<motion>.csv    │
                                         │                                  │
                                         │ (4) edit YAML: end/hold frames  │
                                         │ vm_packages/.../config/M2v6/    │
                                         │   track_<motion>.yaml           │
                                         └─────────────────────────────────┘
                                                      │
                                                      ▼ (5) restart sim
                                          ┌──────────────────────────┐
                                          │  vm_ctrl loads ONNX+CSV  │
                                          │  bấm phím → trigger task │
                                          └──────────────────────────┘
```

---

## Biến cần thay trước khi chạy

Tuỳ motion bạn export, đặt 3 biến này đầu session (chạy **trên laptop**):

```bash
# Tên motion (dùng cho đường dẫn local, không phải tên motion file server)
MOTION=m26_standingup_normal

# Experiment name trong logs/rsl_rl/ trên server
EXP=m2v6_standup_normal

# Timestamp của run cụ thể
TS=2026-04-17_19-15-19

# Tên thư mục đích trong vm_packages/.../mimic/
# (thường là 'standup' hoặc 'neymar')
DEST=standup
```

Các lệnh sau dùng 4 biến này — copy-paste trực tiếp.

---

## Bước 1: Pull ONNX từ server về laptop

```bash
# Đảm bảo thư mục đích tồn tại
mkdir -p ~/Documents/Humanoid_Tracking_Task/mjlab/params/$MOTION

# Pull ONNX — rsl_rl export tự động tại save_interval (mỗi 500 iter)
sshpass -p '1' scp \
  nguyenl3@10.148.255.113:~/Documents/vm_mjlab/logs/rsl_rl/$EXP/$TS/$TS.onnx \
  ~/Documents/Humanoid_Tracking_Task/mjlab/params/$MOTION/policy.onnx

# Verify
ls -la ~/Documents/Humanoid_Tracking_Task/mjlab/params/$MOTION/policy.onnx
```

---

## Bước 2: Parse ONNX metadata → CSV

Dùng script `load_metadata_onnx.py` (đã sửa bug pad empty cells, không nên dùng bản cũ):

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

uv run python load_metadata_onnx.py \
  params/$MOTION/policy.onnx \
  params/$MOTION/param_for_m26_$DEST.csv

# Verify format — phải thấy 11 dòng: 1 comment + 10 metadata rows
cat params/$MOTION/param_for_m26_$DEST.csv | head -5
wc -l params/$MOTION/param_for_m26_$DEST.csv
```

Format CSV đúng (quan trọng — C++ parser rất strict):
```
# generated from ONNX metadata of ...
run_path,<timestamp>
joint_names,left_hip_pitch_joint,...,right_wrist_roll_joint
joint_stiffness,29.129,...,10.646
joint_damping,4.909,...,1.794
default_joint_pos,-0.000,...,0.000
command_names,motion
observation_names,command,motion_anchor_ori_b,base_ang_vel,joint_pos,joint_vel,actions
action_scale,1.116,...,0.845
anchor_body_name,pelvis_link
body_names,pelvis_link,...,right_wrist_roll_link
```

> [!WARNING]
> **KHÔNG** pad trailing empty cells (`,,,,,,`). Nếu có, C++ `to_floats` parse `""` → throw `CSV parse error: ''` → state load fail silent → robot không cử động. Script mới đã fix; nếu dùng script cũ (`extract_metadata_to_wide_csv` với `max_values=29`) phải bỏ.

---

## Bước 3: Copy ONNX + CSV vào vm_packages

```bash
DEST_DIR=~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/$DEST
mkdir -p $DEST_DIR

cp ~/Documents/Humanoid_Tracking_Task/mjlab/params/$MOTION/policy.onnx \
   $DEST_DIR/policy.onnx

cp ~/Documents/Humanoid_Tracking_Task/mjlab/params/$MOTION/param_for_m26_$DEST.csv \
   $DEST_DIR/param_for_m26_$DEST.csv

ls -la $DEST_DIR
```

---

## Bước 4: Cập nhật YAML motion range

YAML file tương ứng với task ID trong `properties.yaml`. Wiring đã có sẵn cho standup (task5) và neymar (task2).

File: `~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_<motion>.yaml`

Các config file có sẵn:
| Motion | YAML | DEST dir |
|---|---|---|
| standup | `track_standingup.yaml` | `mimic/standup/` |
| neymar samba | `track_neymar_celebrate.yaml` | `mimic/neymar/` |

**Cập nhật `end` và `hold` match số frame NPZ motion** (NPZ.shape[0] - 1):

```yaml
motion:
  time_rate: 1.0
  init: 0
  end: 308        # standup motion 309 frames → 308
  hold: 308       # giữ ở frame cuối (standup thường cần chain → walk)
```

Ví dụ mapping:
| Motion | NPZ frames | `end` / `hold` |
|---|---|---|
| m26_standingup_normal | 309 | 308 |
| standing_up_m26 (cũ) | 430 | 429 (hoặc 408 tuỳ version) |
| neymar_celebrate | 849 | 848 |

Verify sau khi sửa:
```bash
grep -A4 'motion:' ~/Documents/vm_packages/vm_ctrl/RLController/config/M2v6/track_standingup.yaml
```

> [!NOTE]
> Wiring task → yaml file nằm ở `Config/sim/M2v6/properties.yaml` dưới key `rl_yamls:`. Nếu thêm motion mới, cần thêm dòng (ví dụ `track_task0: "track_my_new.yaml"`). Đã có:
> - `track_task2: "track_neymar_celebrate.yaml"` (neymar samba)
> - `track_task5: "track_standingup.yaml"` (standup)

---

## Bước 5: Build và restart sim

Nếu chỉ đổi ONNX/CSV/YAML (không đổi C++ source): **không cần rebuild** vm_ctrl — nó load YAML runtime.

Nếu vm_ctrl đang chạy, restart để đọc file mới:

```bash
# Kill session cũ
tmux kill-session -t Sim_test 2>/dev/null

# Restart
cd ~/Documents/vm_packages
ROBOT=M2v6 MODE=sim tmuxp load start.yaml
```

Hoặc nếu không dùng tmuxp:
```bash
# Terminal 1 — simulator
cd ~/Documents/vm_packages/mj_sim/build
./vm_mujoco M2v6

# Terminal 2 — controller
cd ~/Documents/vm_packages/vm_ctrl/build
./vm_ctrl M2v6 sim
```

---

## Bước 6: Verify load thành công

Kiểm tra log startup của `vm_ctrl` (trước khi bấm phím trigger):

```bash
tmux capture-pane -t Sim_test:Experiment -p -S -500 | \
  grep -iE 'fail|missing|track_task5|standup|cannot|exception' | head -20
```

**Log lành mạnh:** KHÔNG có dòng nào chứa `Failed to load RL 'track_task5'` hay `CSV parse error`.

**Log bệnh:** nếu thấy:
```
[FSM] Failed to load RL 'track_task5' from ...track_standingup.yaml: CSV parse error: ''. Skipping this state.
```
→ CSV format sai (thường do pad empty cells). Regenerate CSV qua bước 2.

---

## Bước 7: Trigger motion trong sim

1. **Click cửa sổ MuJoCo** để focus (keyboard chỉ nghe khi MuJoCo có focus)
2. **Bấm `1`** để về state PASSIVE (nếu không đã ở đó)
3. **Bấm `=`** để trigger STANDUP → TASK5 (standup policy)
4. Robot nằm → chống tay → đứng dậy (6.18s)
5. **Bấm `2`** sau khi motion kết thúc (log `motion step: 308` dừng tăng) → chuyển sang RL_27DOF_WALK policy giữ đứng ổn định

| Phím | Tác dụng |
|---|---|
| `1` | → PASSIVE (reset state) |
| `2` | → RL_27DOF_WALK (giữ đứng, sau khi motion_hold) |
| `=` | PASSIVE → TASK5 (standup) |
| `Space` | Pause/play physics |
| `F` | Hiện contact force |
| `Backspace` | Reset robot pose |

Log mong đợi trong terminal vm_ctrl:
```
transition from Passive to RL Mimic 5
ONNX Tracking Inference: xxx us
motion step: 1
motion step: 2
...
motion step: 308    ← hold ở đây nếu không bấm `2`
```

---

## Troubleshoot

| Triệu chứng | Nguyên nhân | Fix |
|---|---|---|
| Log `Failed to load RL 'track_task5'` + `CSV parse error: ''` | CSV có empty cells | Regenerate với script mới (bước 2) |
| Log `Failed to load ...` + `YAML exception` | YAML format sai | Check indent, key names trong yaml |
| Log `ORT session creation failed` | ONNX corrupt hoặc opset không support | Pull lại ONNX từ server |
| Log `default_joint_pos size mismatch` | CSV thiếu row hoặc số giá trị ≠ 27 | Regenerate CSV |
| Bấm `=` → log `transition from Passive to RL Mimic 5` nhưng không có `motion step` | rl_track_task5 = nullptr (load fail) | Check log startup (có `[FSM] Failed`) |
| Bấm `=` → motion step tăng đều nhưng robot không cử động | Obs sign flip / joint_sign mismatch | Kiểm `LegController_M2v6_sim` sign flips, override `joint_sign` trong YAML |
| Robot đứng dậy thành công rồi ngã | Motion final pose không stable, cần chain sang walk | Bấm `2` sau khi motion hold để chuyển RL_WALK |
| Robot lăn loạn khi trigger | Policy chưa đủ converge | Train thêm, reward nên ≥ 35 |

---

## Checklist dán giấy

- [ ] ONNX pulled về `mjlab/params/<motion>/policy.onnx`
- [ ] CSV sinh bằng `load_metadata_onnx.py` (script mới, 2 args)
- [ ] CSV verify: 11 dòng, không có trailing `,,`
- [ ] ONNX + CSV copy vào `vm_packages/.../mimic/<DEST>/`
- [ ] `track_<motion>.yaml`: `end`/`hold` match NPZ frames - 1
- [ ] Wiring `properties.yaml` đã có task number tương ứng
- [ ] Restart sim qua tmuxp
- [ ] Startup log KHÔNG có `Failed to load RL` hay `CSV parse error`
- [ ] MuJoCo focus → `1` → `=` → motion step in terminal tăng đều
- [ ] Sau motion complete → bấm `2` để giữ đứng bằng walk policy

---

## Re-export ONNX từ `.pt` ở laptop (khi không SSH được server)

Trường hợp WandB có `.onnx` quá cũ so với `.pt` (do `wandb.save` không re-upload reliable), bạn có thể:

1. Download `.pt` mới nhất từ WandB artifact
2. Re-export ONNX ở laptop (cần GPU + NPZ motion)
3. Tiếp tục bước 2–3 của doc này

### Script

Đã sẵn: `mjlab/scripts/reexport_onnx.py`. Chạy:

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab

# Biến
CKPT=~/Downloads/model_5000.pt                                       # đường dẫn .pt đã tải về
NPZ=~/Documents/Humanoid_Tracking_Task/experiments/neymar_samba/DATA/m26_standingup_normal.npz
OUT=params/m26_standingup_normal/policy.onnx

# Re-export
uv run python scripts/reexport_onnx.py "$CKPT" "$NPZ" "$OUT"
```

Mong đợi output: `[OK] exported params/m26_standingup_normal/policy.onnx (1265 KB)` — kèm metadata đầy đủ 10 keys.

### Download `.pt` từ WandB artifact (nếu chưa có)

```bash
cd ~/Documents/Humanoid_Tracking_Task/mjlab
uv run python << 'PY'
import wandb
api = wandb.Api()
runs = api.runs(
    "nguyen-ld2310-hanoi-university-of-science-and-technology/mjlab",
    filters={"config.experiment_name": "m2v6_standup_normal"},
    order="-created_at",
)
run = runs[0]
print(f"Run: {run.name}, state: {run.state}")
for a in run.logged_artifacts():
    if a.type == "model":
        print(f"Downloading {a.name} (v{a.version})...")
        a.download(root="params/m26_standingup_normal/ckpt/")
        break
PY
# File sẽ nằm tại params/m26_standingup_normal/ckpt/<name>.pt
```

### Task ID mặc định

Script default `Mjlab-Tracking-Flat-M2v6-No-State-Estimation`. Nếu motion khác task, truyền tham số thứ 4:

```bash
uv run python scripts/reexport_onnx.py "$CKPT" "$NPZ" "$OUT" Mjlab-Tracking-Flat-M2v6
```

---

## Tham khảo chéo

- [13 — Full pipeline train-to-sim](13-full-pipeline-train-to-sim-deploy.md) — big picture
- `vm_packages/vm_ctrl/RLController/include/rl_params.h:100` — hàm `load_keyrow_csv` (nguồn authoritative về CSV format)
- `vm_packages/vm_ctrl/RLController/src/rl_tracking_27dof.cpp` — C++ policy inference
- `mjlab/src/mjlab/rl/exporter_utils.py:22` — `get_base_metadata()` sinh metadata keys
- `mjlab/src/mjlab/tasks/tracking/rl/runner.py:61` — `export_policy_to_onnx()`
