# Humanoid_Tracking_Task — Root index

End-to-end humanoid motion pipeline: **video → human pose → robot retarget → motion edit → RL training → sim/real deploy**.

## Layout

| Path | Role | CLAUDE.md |
|------|------|-----------|
| [`vm_video2robot/`](vm_video2robot/CLAUDE.md) | Video → SMPL-X human pose (GVHMR) | yes |
| [`vm_retargeting/`](vm_retargeting/CLAUDE.md) | Human pose → robot motion (IK retargeting, mink) | yes |
| [`editor_motion/`](editor_motion/CLAUDE.md) | Web editor: visualize / smooth / fix robot NPZ motion (was `robot-motion-editor-v3`) | yes |
| [`mjlab/`](mjlab/CLAUDE.md) | RL training (rsl_rl) → ONNX export, deploy scripts | yes |
| `docs/` | Numbered Vietnamese guides + `command.md` | — |
| `notebooks/` | Pipeline notebooks (01 GVHMR, 02 motion conversion, 03 training monitor) | — |
| `scripts/` | Pipeline shell scripts (`01_chuyen_csv_sang_npz.sh`, `02_huan_luyen.sh`, `03_chay_policy.sh`) + helpers | — |
| `data/videos/` | Source videos for retargeting | — |
| `experiments/standup/` | Standup motion experiment data (was `StandUp/`) | — |
| `experiments/neymar_samba/` | Neymar samba motion data + checkpoints (was `neymar_samba_mimic/`) | — |
| `archive/` | Old `robot-motion-editor` v1/v2 + zip — frozen | — |

Deploy target lives outside this repo: `~/Documents/vm_packages` (sim + real-time controller).

## Pipeline data flow

```
data/videos/*.mp4
   │
   ▼  vm_video2robot/tools/demo/demo.py  (conda env: video2robot)
outputs/<name>/hmr4d_results.pt           (SMPL-X)
   │
   ▼  vm_retargeting/scripts/gvhmr_to_robot.py  (conda env: gmr)
robot motion .pkl / .npz @ 50 fps
   │
   ▼  editor_motion/app.py (Flask, port 5000)  ← smooth / fix joints
refined .npz
   │
   ▼  mjlab/scripts/train_*.sh                 ← RL training (uv)
checkpoint .pt
   │
   ▼  mjlab/scripts/deploy_*.sh                ← export ONNX + metadata.csv
   ▼  scp → ~/Documents/vm_packages/...        (deploy to sim/hardware)
```

## Key docs (most-used)

- `docs/command.md` — quick command reference across all 4 stages
- `docs/13-full-pipeline-train-to-sim-deploy.md` — end-to-end pipeline
- `docs/14-export-checkpoint-to-vm-packages.md` — ONNX export to vm_packages
- `docs/18-pipeline-selfvideo-retargeting-training.md` — own-video → train flow
- `docs/M2V6_TRAINING_GUIDE.md` — M2v6 robot training specifics
- `docs/VM_PACKAGES_PROJECT_MANAGEMENT.md` — vm_packages deployment notes

## Common reference data

- Default motion data dir: `experiments/neymar_samba/DATA/` (e.g. `m26_standingup_normal.npz`, `neymar_celebrate.npz`).
- Default robot: `m2_v6_wrist_pitch` (M2v6 with wrist_pitch constraint).
- Policy rate: 50 Hz; observation dim: 144; ONNX metadata: 8 keys via `get_base_metadata()`.

## Environments

| Tool | Manager | Activation |
|------|---------|------------|
| `vm_video2robot` | conda | `conda activate video2robot` |
| `vm_retargeting` | conda | `conda activate gmr` |
| `editor_motion` | pip (Flask + numpy) | `python app.py` |
| `mjlab` | `uv` | `uv run …` (never bare `python`) |

## Recent reorg (2026-05-04)

Root cleanup applied: `robot-motion-editor[-v2]` + `.zip` moved to `archive/`; `robot-motion-editor-v3` renamed to `editor_motion`; `videos/` + `src_video/` merged into `data/videos/`; `StandUp/` and `neymar_samba_mimic/` moved under `experiments/`. Path references in `docs/`, `mjlab/scripts/`, and `vm_retargeting/docs/` were updated to match.
