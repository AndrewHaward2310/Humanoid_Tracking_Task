Peak checkpoint handfix 20/04 
logs/rsl_rl/m2v6_standup_handfix/peak_v4_iter23000/
bash /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/scripts/deploy_standup.sh /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_standup_handfix/peak_v4_iter23000/model_23000.pt /home/nguyenld12/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_standup_handfix/peak_v4_iter23000/motion.npz 2>&1 | tail -6
# deploy 1 checkpoint
bash ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/deploy_standup.sh \
  ~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_standup_handfix/2026-04-21_06-27-41/model_27500.pt \
  /tmp/m26_standingup_handfix_v3_hold3s.npz

# deploy v5 warmstart goc
bash ~/Documents/Humanoid_Tracking_Task/mjlab/scripts/deploy_standup.sh \
  ~/Documents/Humanoid_Tracking_Task/mjlab/logs/rsl_rl/m2v6_standup_handfix/warmstart_iter25000/model_25000.pt \
  /tmp/m26_standingup_handfix_v3_hold3s.npz

# Cách mở motion editor v2
cd /home/nguyenld12/Documents/Humanoid_Tracking_Task/archive/robot-motion-editor-v2
python app.py
M2v6/urdf/m2v6.urdf
/home/nguyenld12/Documents/TextOp/TextOpDeploy/src/textop_ctrl/models/motion.npz

# video2robot
conda activate video2robot 
python tools/demo/demo.py \
    --video=inputs/demo/motion_clip.mp4 \
    -s

# vm_retargeting
conda activate gmr 
cd ~/Documents/Humanoid_Tracking_Task/vm_retargeting
python scripts/gvhmr_to_robot.py \
    --gvhmr_pred_file ~/Documents/Humanoid_Tracking_Task/vm_video2robot/outputs/demo/motion_clip/hmr4d_results.pt \
    --robot m2_v6_wrist_pitch \
    --save_path output/motion_clip.pkl \
    --tgt_fps 30 \
    --output_fps 50
    -- 
<-- 
  --gvhmr_pred_file: link đến file .pt sau khi chạy video2robot
  --robot choices=["m2_v6", "m2_v6_wrist_pitch", "m2_v3_toe", "m2_v3",  "m2_v3_23dof", "m2_v3_27dof", "m2_v2_toe","m2_v2_27dof", "unitree_g1", "unitree_g1_with_hands", "unitree_h1", "unitree_h1_2","booster_t1", "booster_t1_29dof","stanford_toddy", "fourier_n1","engineai_pm01","kuavo_s45", "hightorque_hi", "galaxea_r1pro","berkeley_humanoid_lite", "booster_k1","pnd_adam_lite", "openloong", "tienkung"],
  --save_path: link đến file .pkl sau khi chạy vm_retargeting
  --loop: có lặp lại hay không
  --record_video: có record vidoe không (default: false)
  --rate_limit: giới hạn tốc độ của robot để giống với tốc độ của con người (default: false)
  --tgt_fps: target FPS cho motion sau khi lưu (default: 30)
  --output_fps: Output FPS cho npz trajectory. Defaults to tgt_fps if not specified (default: 50)
  --extract_contact: có extract contact không (default: false)
  --use_urdf_calibration: có sử dụng urdf calibration không (default: false)
  --visualize_calib: có visualize calibration không (default: false)

--!>