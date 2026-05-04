#!/bin/bash
# ============================================================================
# Script chuyển đổi file CSV motion sang NPZ cho robot M2v6
#
# Cách dùng:
#   1. Sửa các biến bên dưới cho phù hợp
#   2. Chạy: bash scripts/01_chuyen_csv_sang_npz.sh
# ============================================================================

# === CẦN SỬA: Đường dẫn file CSV đầu vào ===
# Lưu ý: đường dẫn trên workstation (copy CSV lên trước bằng scp)
INPUT_FILE="motions/chao_mung.csv"

# === CẦN SỬA: Đường dẫn file NPZ đầu ra ===
OUTPUT_FILE="motions/chao_mung.npz"

# === Tốc độ khung hình file CSV (thường là 30 từ GVHMR, hoặc 120) ===
INPUT_FPS=30

# === Tốc độ khung hình đầu ra (phải là 50 để khớp với tần số điều khiển) ===
OUTPUT_FPS=50

# === Bật render video để kiểm tra bằng mắt (True/False) ===
RENDER=True

# === GPU sử dụng ===
DEVICE="cuda:0"

# ============================================================================
# KHÔNG CẦN SỬA PHẦN DƯỚI
# ============================================================================

# Nếu chạy trên workstation:
MJLAB_DIR="/home/nguyenl3/Documents/vm_mjlab"
# Nếu chạy trên laptop (bỏ comment dòng trên, mở comment dòng dưới):
# MJLAB_DIR="/home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab"

echo "============================================"
echo "  Chuyển đổi CSV -> NPZ cho M2v6"
echo "============================================"
echo ""
echo "  File đầu vào:  $INPUT_FILE"
echo "  File đầu ra:   $OUTPUT_FILE"
echo "  FPS vào/ra:    $INPUT_FPS -> $OUTPUT_FPS"
echo "  Render video:  $RENDER"
echo "  Device:        $DEVICE"
echo ""

# Kiểm tra file đầu vào tồn tại
if [ ! -f "$INPUT_FILE" ]; then
    echo "[LỖI] Không tìm thấy file: $INPUT_FILE"
    echo "Hãy sửa biến INPUT_FILE ở đầu script."
    exit 1
fi

cd "$MJLAB_DIR" || exit 1

MUJOCO_GL=egl uv run python -m mjlab.scripts.csv_to_npz_m2v6 \
  --input-file "$INPUT_FILE" \
  --output-file "$OUTPUT_FILE" \
  --input-fps "$INPUT_FPS" \
  --output-fps "$OUTPUT_FPS" \
  --render "$RENDER" \
  --device "$DEVICE"

echo ""
echo "============================================"
echo "  Hoàn thành!"
echo "  File NPZ: $OUTPUT_FILE"
if [ "$RENDER" = "True" ]; then
    VIDEO_FILE="${OUTPUT_FILE%.npz}.mp4"
    echo "  Video:    $VIDEO_FILE"
fi
echo "============================================"
