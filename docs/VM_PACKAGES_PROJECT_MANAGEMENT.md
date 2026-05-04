# Quản Lý Dự Án vm_packages — Git Workflow & Policy Versioning

> Workflow quản lý monorepo `vm_packages` (chứa submodules `vm_ctrl`, `mj_sim`, `robots`, ...) để tránh mất policy/fix/config khi pull master hoặc thử version mới.

---

## Mục lục

1. [Cấu trúc repo](#1-cấu-trúc-repo)
2. [Vấn đề gặp phải](#2-vấn-đề-gặp-phải)
3. [Cấu trúc branch khuyến nghị](#3-cấu-trúc-branch-khuyến-nghị)
4. [Submodule workflow cơ bản](#4-submodule-workflow-cơ-bản)
5. [Pull master an toàn — từng level](#5-pull-master-an-toàn--từng-level)
6. [Policy version management](#6-policy-version-management)
7. [NOTES.md — bắt buộc mọi version](#7-notesmd--bắt-buộc-mọi-version)
8. [Commit & tag release sau deploy](#8-commit--tag-release-sau-deploy)
9. [Scripts tự động](#9-scripts-tự-động)
10. [Checklist khi nhận policy mới](#10-checklist-khi-nhận-policy-mới)

---

## 1. Cấu trúc repo

`vm_packages` là **monorepo** với nhiều submodules — mỗi submodule là 1 git repo riêng:

```
vm_packages/                                     ⬅ REPO CHÍNH (remote: vm_packages.git)
│   branch hiện tại: feat/m2v6-standup-neymar-integration
│
├── vm_ctrl/               → submodule, repo vm_ctrl.git      (branch: AI_LOCO/test_policy_new)
├── mj_sim/                → submodule, repo mujoco_sim.git   (branch: master)
├── robots/                → submodule, repo vm_robot.git     (branch: main)
├── vm_control_dashboard/  → submodule, repo vm_control_dashboard.git
├── ros2_ws/               → KHÔNG phải submodule, tracked trong vm_packages
│   └── src/vmo_ros2_interface/  → submodule, repo vmo_ros2_interface.git
│
├── start.yaml             → tracked trong vm_packages root
├── Dockerfile
└── experiment.xml
```

### Kiểm tra cấu trúc

```bash
cd ~/Documents/vm_packages

# Root repo
git branch --show-current
git remote get-url origin

# List submodules
cat .gitmodules
git submodule status

# Submodule vm_ctrl
cd vm_ctrl
git branch --show-current
git remote get-url origin
```

### Hệ quả của cấu trúc submodule

1. **Mỗi submodule có git/branch/history RIÊNG** — push/pull độc lập
2. **vm_packages tracked "pointer" (commit SHA) của mỗi submodule** — pointer là 1 phần history của vm_packages
3. **Cần pull 2 tầng** khi cập nhật: pull vm_packages → init/update submodules
4. **Commit 2 lần khi sửa submodule**: commit trong submodule trước, rồi commit pointer trong vm_packages

---

## 2. Vấn đề gặp phải

| Vấn đề | Hậu quả | Level |
|---|---|---|
| Pull `vm_packages/master` ghi đè submodule pointer | Submodule rollback về commit cũ → mất code | vm_packages |
| Pull `vm_ctrl/master` ghi đè `policy.onnx` local | Mất policy vừa train | vm_ctrl |
| Motion NPZ các version (v1, v2, warmup) rải rác | Dùng nhầm motion cũ | vm_ctrl |
| `track_*.yaml` bị merge ghi đè fix dof=27 | Standup fail lại | vm_ctrl |
| Overwrite `policy.onnx` khi deploy version mới | Mất khả năng rollback | vm_ctrl |
| Commit submodule nhưng quên commit pointer ở vm_packages | Team khác không thấy code mới | vm_packages + vm_ctrl |
| Update `ros2_ws/` nhưng không commit | Mất code ros2 | vm_packages |

---

## 3. Cấu trúc branch khuyến nghị

### Mỗi repo 1 branch cá nhân riêng

Vì mỗi submodule là repo riêng biệt, cần **branch cá nhân ở CẢ 2 level**:

```
vm_packages/                        ← branch: dev/<user>/m2v6
  vm_ctrl/                          ← branch: dev/<user>
  mj_sim/                           ← giữ master (thường không cần fork)
  robots/                           ← giữ main
```

### Quy ước đặt tên branch

| Prefix | Dùng cho | Ví dụ |
|---|---|---|
| `dev/<user>` | Work cá nhân hằng ngày (submodule) | `dev/nguyenld12` |
| `dev/<user>/<feature>` | Branch vm_packages root ghi nhớ context | `dev/nguyenld12/m2v6` |
| `feature/<tên>` | Feature ghép vào team | `feature/lying_down_v2` |
| `exp/<tên>` | Experiment chưa chắc | `exp/warmup_frames` |
| `hotfix/<mô_tả>` | Fix khẩn | `hotfix/standup_dof` |

### Tạo branch cá nhân ở cả 2 level

```bash
# 1. Branch cá nhân cho vm_ctrl submodule (chỗ sửa nhiều nhất)
cd ~/Documents/vm_packages/vm_ctrl
git checkout AI_LOCO/test_policy_new
git pull origin AI_LOCO/test_policy_new
git checkout -b dev/$(whoami)
git push -u origin dev/$(whoami)

# 2. Branch cá nhân cho vm_packages root
cd ~/Documents/vm_packages
git checkout feat/m2v6-standup-neymar-integration
git pull origin feat/m2v6-standup-neymar-integration
git checkout -b dev/$(whoami)/m2v6
git push -u origin dev/$(whoami)/m2v6

# 3. Các submodule khác (mj_sim, robots): giữ nguyên master/main
```

**Lợi ích:**
- Fix/experiment ở submodule không ảnh hưởng team
- Pull master ở vm_packages không reset submodule về team pointer (vì bạn đang ở branch riêng)
- Dễ rollback / cherry-pick sau này

---

## 4. Submodule workflow cơ bản

### Sync submodule sau khi clone / pull vm_packages

```bash
cd ~/Documents/vm_packages

# Init lần đầu
git submodule update --init --recursive

# Pull vm_packages + auto update submodules
git pull --recurse-submodules

# Hoặc 2 bước:
git pull
git submodule update --recursive
```

### Quy trình sửa code trong submodule

```bash
# B1: Vào submodule, commit code
cd ~/Documents/vm_packages/vm_ctrl
git checkout dev/nguyenld12      # đảm bảo ở branch cá nhân
# ... sửa file ...
git add .
git commit -m "fix: dof=27 cho track_standingup.yaml"
git push origin dev/nguyenld12

# B2: Quay về vm_packages, commit pointer mới của submodule
cd ~/Documents/vm_packages
git status
# Sẽ thấy: modified: vm_ctrl (new commits)
git add vm_ctrl
git commit -m "bump vm_ctrl: dof fix cho standup"
git push origin dev/$(whoami)/m2v6
```

> ⚠️ **Nếu quên B2:** submodule pointer ở vm_packages vẫn trỏ commit cũ → team pull về sẽ không thấy fix.

### Check trạng thái submodule

```bash
cd ~/Documents/vm_packages
git submodule status
# Output:
#  <sha>     vm_ctrl (heads/dev/nguyenld12)         ← ở branch dev/nguyenld12
# +<sha>     mj_sim (v1.2.3)                        ← "+" = submodule có commit mới so với pointer tracked
# -<sha>     vm_control_dashboard                    ← "-" = chưa init

# Chi tiết diff
git diff --submodule
```

### Switch branch trong submodule

```bash
cd ~/Documents/vm_packages/vm_ctrl

# Submodule ở "detached HEAD" sau khi pull → cần checkout branch explicit
git branch --show-current      # có thể empty (detached)
git checkout dev/nguyenld12    # về branch cá nhân
git pull                       # sync latest
```

---

## 5. Pull master an toàn — từng level

### Scenario A — Chỉ pull `vm_ctrl/master` (submodule level)

Dùng khi chỉ cập nhật từ team vm_ctrl:

```bash
cd ~/Documents/vm_packages/vm_ctrl
bash scripts/safe_pull_master.sh
```

Script: backup → stash → merge master → hướng dẫn restore.

### Scenario B — Pull `vm_packages/master` + update submodules

Dùng khi sync tổng thể (ROS2 update, Dockerfile update, submodule pointer mới):

```bash
cd ~/Documents/vm_packages
bash scripts/safe_pull_all.sh
```

Chi tiết script: xem Section 9.

### Scenario C — Pull master của 1 submodule (không đụng pointer)

```bash
cd ~/Documents/vm_packages/vm_ctrl
git fetch origin master
git merge origin/master --no-edit
# Pointer trong vm_packages vẫn chưa update — cần cd .. && git add vm_ctrl && commit
```

---

## 6. Policy version management

(Áp dụng cho `vm_ctrl/RLController/Checkpoints/M2v6/mimic/`)

### Vấn đề với layout hiện tại

```
Checkpoints/M2v6/mimic/standup/
├── policy.onnx            ← Ghi đè mỗi lần deploy → mất history
├── param_for_m26_standup.csv
└── motion.npz             ← Version nào?
```

### Layout khuyến nghị — sub-folder version + symlink `current/`

```
Checkpoints/M2v6/mimic/standup/
├── NOTES.md                        ← Bắt buộc, log mọi version
├── v1/
│   ├── policy.onnx
│   ├── param_for_m26_standup.csv
│   ├── motion.npz
│   └── meta.yaml                   ← experiment, ckpt_iter, reward, date
├── v2/
│   └── ...
├── v2_warmup/
│   └── ...
└── current -> v2                   ← symlink trỏ version đang active
```

### Dùng symlink `current`

```bash
cd ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/standup

# Deploy version mới → tạo folder + đổi symlink
mkdir v2
cp /path/policy.onnx v2/
cp /path/motion.npz v2/
cp /path/param.csv v2/

rm current
ln -s v2 current
```

`track_standingup.yaml` trỏ tới `current/`:

```yaml
model_path: "../RLController/Checkpoints/M2v6/mimic/standup/current/policy.onnx"
metadata:
  path: "../RLController/Checkpoints/M2v6/mimic/standup/current/param_for_m26_standup.csv"
```

**Rollback 1 lệnh:**

```bash
rm current && ln -s v1 current
```

### Quy tắc bất di bất dịch

1. **KHÔNG overwrite** file có sẵn — luôn tạo version mới
2. **Mỗi version có `meta.yaml`** ghi nguồn gốc
3. **Update `NOTES.md`** mỗi khi tạo version
4. Chỉ `current` symlink là "active"

---

## 7. NOTES.md — bắt buộc mọi version

**Vị trí:** `vm_ctrl/RLController/Checkpoints/M2v6/mimic/<motion>/NOTES.md`

**Template:**

```markdown
# <Motion Name> — Version Notes

## v<N> — <YYYY-MM-DD>

**Status:** deployed / archived / broken
**Active:** true / false

### Motion
- **File:** `v<N>/motion.npz`
- **Frames:** <N> @ 50fps (<s>s)
- **Nguồn gốc:** <edit motion editor / GMR retarget / prepend warmup / ...>
- **Preprocessing:**
  - [ ] FK rebuild
  - [ ] Hold frames cuối: N
  - [ ] Warmup frames đầu: N
  - [ ] Shift to floor
  - [ ] Lift hand

### Checkpoint
- **File:** `v<N>/policy.onnx`
- **Experiment:** `m2v6_<tên>`
- **Run:** `logs/rsl_rl/<exp>/<timestamp>`
- **Iter:** <X> (peak) / <Y> (deployed)
- **Reward:** <peak_reward>
- **Training:**
  - Num envs: <1024 local / 4096 server>
  - Max iter: <N>
  - Warmstart: <path hoặc "scratch">

### Thay đổi so với v<N-1>
- <thay đổi>

### Deploy status
- **Sim:** OK / Chưa test / Fail (mô tả)
- **Real:** OK / Chưa test / Fail (mô tả)

### Issues known
- <issue>: <workaround>
```

---

## 8. Commit & tag release sau deploy

Sau mỗi lần deploy policy + test OK, commit 2 lần:

### Lần 1 — trong submodule (vm_ctrl)

```bash
cd ~/Documents/vm_packages/vm_ctrl

# Commit YAML + NOTES (không commit binary nếu đã dùng symlink current/ + .gitignore)
git add RLController/config/M2v6/track_<motion>.yaml
git add RLController/Checkpoints/M2v6/mimic/<motion>/NOTES.md
git commit -m "deploy <motion> v<N>: <desc>

- Peak iter: <X>, reward: <Y>
- Motion: <frames> frames
- Sim test: OK
"

# Tag
git tag -a "deploy/<motion>_v<N>_$(date +%Y%m%d)" \
  -m "Deployed <motion> v<N> — reward <Y>"

git push origin dev/$(whoami)
git push origin "deploy/<motion>_v<N>_$(date +%Y%m%d)"
```

### Lần 2 — trong vm_packages (root) — update submodule pointer

```bash
cd ~/Documents/vm_packages
git status
# Sẽ thấy: modified: vm_ctrl (new commits)

git add vm_ctrl
git commit -m "bump vm_ctrl: deploy <motion> v<N>"
git push origin dev/$(whoami)/m2v6

# (Optional) Tag cùng ở vm_packages level — snapshot toàn bộ hệ thống
git tag -a "deploy/<motion>_v<N>_$(date +%Y%m%d)" \
  -m "vm_packages snapshot với <motion> v<N>"
git push origin "deploy/<motion>_v<N>_$(date +%Y%m%d)"
```

### List + rollback

```bash
# List tag
cd ~/Documents/vm_packages/vm_ctrl && git tag -l "deploy/*"
cd ~/Documents/vm_packages && git tag -l "deploy/*"

# Rollback vm_ctrl về tag cũ
cd ~/Documents/vm_packages/vm_ctrl
git checkout deploy/standup_v1_20260421

# Rollback toàn bộ vm_packages (cả submodule pointer) về tag
cd ~/Documents/vm_packages
git checkout deploy/standup_v1_20260421
git submodule update --recursive
```

### ⚠️ Binary files — cân nhắc

vm_ctrl hiện commit cả `.onnx` + `.npz` (dung lượng lớn). Options:

1. **Ignore binary** (`.gitignore` trong vm_ctrl):
   ```
   RLController/Checkpoints/**/*.onnx
   RLController/Checkpoints/**/*.npz
   RLController/Checkpoints/**/*.pt
   ```
   Backup binary qua cloud (Google Drive / S3 / external disk).

2. **Git LFS** — nếu team cấp quota:
   ```bash
   cd vm_ctrl && git lfs track "*.onnx" "*.npz" "*.pt"
   ```

3. **Giữ như hiện tại** — dễ rollback nhưng repo phình to.

Thảo luận với team lead.

---

## 9. Scripts tự động

### Script đã có trong `vm_ctrl/scripts/`

| Script | Mô tả |
|---|---|
| `safe_pull_master.sh` | Backup + stash + merge `vm_ctrl/master` vào branch hiện tại |
| `deploy_with_version.sh` | Deploy policy với sub-folder `v<N>/` + symlink `current/` + `meta.yaml` |

### Script khuyến nghị tạo thêm ở `vm_packages/scripts/`

Tạo folder + `safe_pull_all.sh` (pull root + sync submodules):

```bash
mkdir -p ~/Documents/vm_packages/scripts
```

**`vm_packages/scripts/safe_pull_all.sh`:**

```bash
#!/bin/bash
# Pull vm_packages master + update submodules.
# Backup cả root + submodules trước khi pull.
set -e
cd ~/Documents/vm_packages

BACKUP=/tmp/vm_packages_backup_$(date +%Y%m%d_%H%M%S)
mkdir -p "$BACKUP"

echo "=== 1. Backup root + submodules ==="
# Root
git status --porcelain | awk '{print $2}' | while read f; do
  mkdir -p "$BACKUP/$(dirname $f)"
  [ -f "$f" ] && cp "$f" "$BACKUP/$f" 2>/dev/null
done
# Submodules
for sub in vm_ctrl mj_sim robots vm_control_dashboard; do
  [ -d "$sub/.git" ] && (cd "$sub" && git status --porcelain 2>/dev/null | \
    awk -v s="$sub" '{print s "/" $2}' | while read f; do
    mkdir -p "$BACKUP/$(dirname $f)"
    [ -f "$f" ] && cp "$f" "$BACKUP/$f" 2>/dev/null
  done)
done
echo "$BACKUP" > /tmp/vm_packages_last_backup.txt
echo "  Backup: $BACKUP"

echo ""
echo "=== 2. Fetch ==="
git fetch origin master
NEW=$(git log --oneline HEAD..origin/master | wc -l)
echo "  vm_packages master: $NEW commit mới"
[ "$NEW" -eq 0 ] && { echo "Nothing to pull. Exit."; exit 0; }

echo ""
echo "=== 3. Stash root ==="
git stash push -u -m "safe_pull_all $(date +%Y-%m-%d)" 2>&1 | grep -v "^No local"

echo ""
echo "=== 4. Merge master ==="
git merge origin/master --no-edit

echo ""
echo "=== 5. Update submodules ==="
git submodule update --recursive

echo ""
echo "=== 6. Check submodule branch ==="
git submodule foreach 'echo "  $path: $(git branch --show-current || echo DETACHED)"'

echo ""
echo "============================================"
echo "  DONE. Backup: $BACKUP"
echo "  Submodule detached? cd <sub> && git checkout <branch>"
echo "============================================"
```

### Scripts mjlab liên quan

Ở `~/Documents/Humanoid_Tracking_Task/mjlab/scripts/` (đã có):
- `deploy_motion.sh` — deploy generic 1-liner
- `prepend_warmup_frames.py` — fix initial pose mismatch
- `npz_add_base_fields.py` — thêm joint_names/base_* cho motion editor
- `fk_rebuild_and_hold.py` — rebuild body arrays sau khi edit motion
- `check_reward_curve_local.py` — check peak từ tensorboard

---

## 10. Checklist khi nhận policy mới

### Trước khi deploy

- [ ] Check peak: `check_reward_curve_local.py m2v6_<exp>`
- [ ] Preview motion: `play_motion.py --npz <file>.npz --loop`
- [ ] Ghi nhớ `current/` đang trỏ version nào (để rollback nếu cần)

### Deploy

- [ ] `deploy_with_version.sh <motion> <v_new> <ckpt.pt> <motion.npz>`
- [ ] Verify `current/` đã update
- [ ] Check `track_<motion>.yaml` trỏ `current/` (không hardcode version)
- [ ] Check `motion.end`, `motion.hold` = `<N_frames - 1>`

### Test

- [ ] Rebuild + restart sim:
  ```bash
  tmux kill-session -t Sim_test
  cd ~/Documents/vm_packages && ROBOT=M2v6 MODE=sim tmuxp load start.yaml
  ```
- [ ] Log vm_ctrl **không có** `default_joint_pos size mismatch`
- [ ] Test flow đầy đủ: Passive → `=` standup → `2` walking → arrow → task
- [ ] Ghi lại bất thường (ngã / giật / motion sai)

### Sau deploy (2 lần commit!)

**Submodule:**
- [ ] Update `NOTES.md`
- [ ] Commit YAML + NOTES ở `vm_ctrl`
- [ ] Tag `deploy/<motion>_v<N>_<date>`
- [ ] Push branch + tag

**vm_packages root:**
- [ ] `cd ~/Documents/vm_packages && git add vm_ctrl && git commit`
- [ ] Push vm_packages branch

### Rollback plan

```bash
# Rollback policy (không đụng code):
cd ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/<motion>
rm current && ln -s <v_cũ> current
tmux kill-session -t Sim_test
cd ~/Documents/vm_packages && ROBOT=M2v6 MODE=sim tmuxp load start.yaml

# Rollback code vm_ctrl về tag:
cd ~/Documents/vm_packages/vm_ctrl
git checkout deploy/<motion>_v<N>_<date>

# Rollback toàn bộ vm_packages + submodules:
cd ~/Documents/vm_packages
git checkout <tag>
git submodule update --recursive
```

---

## Phụ lục A — Migration layout hiện tại sang sub-folder version

```bash
cd ~/Documents/vm_packages/vm_ctrl/RLController/Checkpoints/M2v6/mimic/standup

# 1. Tạo sub-folder cho version hiện tại
mkdir v1
mv motion_v1.npz v1/motion.npz
mv motion_v2.npz v1/motion_v2.npz     # giữ nếu có
mv policy.onnx v1/policy.onnx
mv param_for_m26_standup.csv v1/param_for_m26_standup.csv

# 2. Tạo symlink
ln -s v1 current

# 3. Update track_standingup.yaml:
#    model_path: "../RLController/Checkpoints/M2v6/mimic/standup/current/policy.onnx"
#    metadata.path: "../RLController/Checkpoints/M2v6/mimic/standup/current/param_for_m26_standup.csv"

# 4. Restart sim + test
tmux kill-session -t Sim_test
cd ~/Documents/vm_packages && ROBOT=M2v6 MODE=sim tmuxp load start.yaml
```

Lặp lại cho: `lyingdown/`, `neymar/`, `boxing/`, ...

---

## Phụ lục B — Troubleshooting submodule

### Submodule đang ở "detached HEAD"

```bash
cd ~/Documents/vm_packages/vm_ctrl
git branch --show-current      # empty = detached
git checkout dev/nguyenld12
```

### vm_packages báo "modified: vm_ctrl" không biết vì sao

```bash
cd ~/Documents/vm_packages
git diff --submodule
# Cho thấy submodule SHA hiện tại vs SHA tracking.
# Nếu bạn vừa pull submodule mới hơn → git add vm_ctrl && commit pointer.
```

### Pull submodule về sai branch

```bash
cd ~/Documents/vm_packages/vm_ctrl
git fetch --all
git checkout <branch đúng>
git pull origin <branch đúng>
cd ..
git add vm_ctrl && git commit -m "fix vm_ctrl branch pointer"
```

### Reset submodule về đúng pointer tracking

```bash
cd ~/Documents/vm_packages
git submodule update --recursive --force
```

### Khi pull vm_packages master, submodule pointer bị lùi về cũ

```bash
cd ~/Documents/vm_packages
git log -p -- vm_ctrl | head -20   # xem pointer master tracking commit nào

# Nếu muốn giữ pointer local (commit mới hơn):
cd vm_ctrl
git checkout dev/<user>            # về branch cá nhân (có commit mới)
cd ..
git add vm_ctrl
git commit -m "restore vm_ctrl pointer to dev/<user> latest"
```

---

*Cập nhật lần cuối: 2026-04-24*
