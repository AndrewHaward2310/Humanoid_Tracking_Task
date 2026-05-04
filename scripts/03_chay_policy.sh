#!/bin/bash
# ============================================================================
# Script chạy và đánh giá policy đã huấn luyện
#
# Cách dùng:
#   1. Sửa các biến bên dưới cho phù hợp
#   2. Chạy: bash scripts/03_chay_policy.sh
# ============================================================================

# === CẦN SỬA: Đường dẫn thư mục log chứa checkpoint ===
# Ví dụ: logs/rsl_rl/m2v6_tracking/2026-04-15_14-30-00_m26_chao_mung
LOG_DIR="logs/rsl_rl/m2v6_tracking/<ten-thu-muc-log>"

# === Task ID (phải giống task khi huấn luyện) ===
TASK_ID="Mjlab-Tracking-Flat-M2v6-No-State-Estimation"

# === Chế độ render ===
# "gui"     : mở cửa sổ 3D (cần màn hình)
# "egl"     : render nền (SSH, không màn hình)
RENDER_MODE="egl"

# ============================================================================
# KHÔNG CẦN SỬA PHẦN DƯỚI
# ============================================================================

# Nếu chạy trên workstation:
MJLAB_DIR="/home/nguyenl3/Documents/vm_mjlab"
# Nếu chạy trên laptop (bỏ comment dòng trên, mở comment dòng dưới):
# MJLAB_DIR="/home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab"

echo "============================================"
echo "  Đánh giá Policy - M2v6"
echo "============================================"
echo ""
echo "  Task:     $TASK_ID"
echo "  Log dir:  $LOG_DIR"
echo "  Render:   $RENDER_MODE"
echo ""

cd "$MJLAB_DIR" || exit 1

# Kiểm tra thư mục log tồn tại
if [ ! -d "$LOG_DIR" ]; then
    echo "[LỖI] Không tìm thấy thư mục log: $LOG_DIR"
    echo ""
    echo "Các thư mục log có sẵn:"
    ls -la logs/rsl_rl/m2v6_tracking/ 2>/dev/null || echo "  (chưa có)"
    echo ""
    echo "Hãy sửa biến LOG_DIR ở đầu script."
    exit 1
fi

if [ "$RENDER_MODE" = "egl" ]; then
    export MUJOCO_GL=egl
fi

uv run play "$TASK_ID" \
  --load-run "$LOG_DIR"
