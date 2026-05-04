#!/bin/bash
# ============================================================================
# Script huấn luyện Motion Imitation cho robot M2v6
#
# Cách dùng:
#   1. Sửa các biến bên dưới cho phù hợp
#   2. Chạy: bash scripts/02_huan_luyen.sh
# ============================================================================

# === CẦN SỬA: Đường dẫn file NPZ motion ===
# Lưu ý: đường dẫn trên workstation (nguyenl3@10.148.255.113)
MOTION_FILE="motions/chao_mung.npz"

# === Task ID (chọn 1 trong 2) ===
# "Mjlab-Tracking-Flat-M2v6"                    : đầy đủ state estimation
# "Mjlab-Tracking-Flat-M2v6-No-State-Estimation" : không cần state estimation (khuyến nghị)
TASK_ID="Mjlab-Tracking-Flat-M2v6-No-State-Estimation"

# === Số lượng môi trường song song ===
# RTX 5090 (32GB - workstation): 4096 (khuyến nghị)
# RTX 5070 (8GB - laptop):       2048
NUM_ENVS=4096

# ============================================================================
# KHÔNG CẦN SỬA PHẦN DƯỚI (trừ khi muốn tinh chỉnh)
# ============================================================================

# Nếu chạy trên workstation:
MJLAB_DIR="/home/nguyenl3/Documents/vm_mjlab"
# Nếu chạy trên laptop (bỏ comment dòng trên, mở comment dòng dưới):
# MJLAB_DIR="/home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab"

echo "============================================"
echo "  Huấn luyện Motion Imitation - M2v6"
echo "============================================"
echo ""
echo "  Task:          $TASK_ID"
echo "  Motion file:   $MOTION_FILE"
echo "  Số môi trường: $NUM_ENVS"
echo ""

# Kiểm tra file motion tồn tại
if [ ! -f "$MOTION_FILE" ]; then
    echo "[LỖI] Không tìm thấy file motion: $MOTION_FILE"
    echo "Hãy sửa biến MOTION_FILE ở đầu script."
    echo "Nếu chưa có, chạy script 01_chuyen_csv_sang_npz.sh trước."
    exit 1
fi

cd "$MJLAB_DIR" || exit 1

echo "Bắt đầu huấn luyện..."
echo "Theo dõi trên WandB dashboard hoặc terminal."
echo "Nhấn Ctrl+C để dừng (checkpoint đã lưu sẽ không mất)."
echo ""

uv run train "$TASK_ID" \
  --env.scene.num-envs "$NUM_ENVS" \
  --env.commands.motion.motion-file "$MOTION_FILE"
