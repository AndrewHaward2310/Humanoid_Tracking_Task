# Phân tích nguyên nhân robot ngã sau khi standup trong sim

Tình huống: policy `m26_standingup_normal` (iter 6500+, reward 39.5) deploy vào `vm_packages` sim → robot **đứng dậy được** nhưng **ngã** khi hoàn thành motion hoặc trong quá trình.

Doc này liệt kê các nguyên nhân có thể theo mức độ khả thi + cách diagnose + cách fix.

---

## Tổng kết nhanh

**Nguyên nhân chính nghi nhất**: **ankle parallel linkage dynamics mismatch** giữa training XML và deploy XML. Training dùng direct servo ankle, deploy dùng cơ cấu 4-thanh với 2 motor (hardware convention) + ball joint anchor.

**Cùng với**: policy có thể chưa đủ converge (mới iter ~7000 / 22500 max) — cần train thêm để robust với sim-to-sim gap.

---

## 1. So sánh XML training vs deploy

| Thành phần | Training (`mjlab/asset_zoo/robots/M2v6/M2v6.xml`) | Deploy (`vm_packages/robots/M2v6/M2v6_real.xml`) |
|---|---|---|
| `ankle_pitch_joint` | **ACTIVE** (actuator class `ankle_pitch`) | **PASSIVE** (class `passive_joint`) |
| `ankle_roll_joint` | **ACTIVE** (actuator class `ankle_roll`) | **PASSIVE** |
| `ankle_motor_1/2_joint` | KHÔNG tồn tại | **ACTIVE** (class `EC-A6408-P2-25`) |
| Ball joints | KHÔNG | `left/right-anchor_joint_1/2` (damping=0) |
| Cơ cấu | Servo trực tiếp 2-DoF | Parallel linkage 4-bar với 2 motor |
| Hand collision | **Có** (đã uncomment) | Không rõ, cần check |

**Hậu quả**:
- Policy output `ankle_pitch_qDes, ankle_roll_qDes` (27 joints) như training
- `LegController_M2v6_real::joint2motorMapping()` convert qua **IK** sang motor_1/motor_2 torque
- MuJoCo simulate parallel linkage (có thêm ball joints, inertia motor links, compliance)
- IK không phải 100% chính xác, dynamics parallel linkage khác servo trực tiếp → robot response ankle sai lệch so với training → mất cân bằng

**Code reference**:
- `vm_ctrl/src/common/LegControllers/LegController_M2v6_real.cpp:344-386`: ankle IK + torque mapping
- `LegController_M2v6_sim.cpp:267-280`: phiên bản servo trực tiếp (không dùng vì `ankle_mode: 1` trong `properties.yaml`)

## 2. Các nguyên nhân khác — xếp theo xác suất

| # | Nguyên nhân | Mức độ | Cách diagnose | Cách fix |
|---|---|---|---|---|
| 1 | **Ankle parallel linkage** | Cao | Xem kp/kd ankle trong log, quan sát ankle wobble lúc ngã | Copy mjlab XML sang vm_packages hoặc train với parallel linkage |
| 2 | Policy undertrained | Cao | Train tiếp đến iter ≥ 15k, test lại | Tiếp tục `--agent.resume True` |
| 3 | State estimator drift (InEKF) | Trung bình | In `est.position`, `est.rpy` mỗi N tick | Tune InEKF hoặc bypass (dùng GT từ sim) |
| 4 | Initial pose mismatch | Thấp | Policy chạy OK từ frame 0 → loại trừ | N/A |
| 5 | Gain kp/kd mismatch | Trung bình | CSV gains từ ONNX = training ⇒ nên đúng. Check deploy có override không | Đã verify CSV đúng |
| 6 | Friction floor khác | Trung bình | So sánh `<geom>` terrain trong 2 XML | Align friction |
| 7 | No domain randomization | Trung bình | Training `mjlab` có `events["foot_friction"]` và `base_com`, có noise | Thêm DR cho ankle motor gains |
| 8 | Control jitter | Thấp | 50Hz target; log timing per step | Đã verify |
| 9 | Joint limit clamping | Thấp | YAML ±10 rad — rất rộng | N/A |
| 10 | Obs noise gap training-deploy | Trung bình | Training có `Unoise`; deploy clean | Đã verify obs dim 144 match |

## 3. Chiến lược diagnose hệ thống

Thứ tự thử, từ rẻ → đắt:

### Bước 3.1 — Xác định thời điểm ngã (rẻ, nhanh)

```bash
tmux capture-pane -t Sim_test:Experiment -p -S -300 | grep -E 'motion step|transition' | tail -30
```

- **Ngã TRƯỚC frame 308** → policy chưa converged cho đoạn motion đó → train thêm
- **Ngã AT frame 308** (motion_hold) → final pose không stable → train thêm hoặc policy không được shaped để giữ tĩnh
- **Ngã SAU khi motion_hold vài giây** → accumulation error, ankle compliance drift → XML mismatch

### Bước 3.2 — So sánh joint pos obs vs reference (trung bình)

Thêm log vào `rl_tracking_27dof.cpp` tạm thời để print `q_internal_` vs `last_joint_pos_onnx_`:

```cpp
// trong step() trước khi session_.Run()
if ((tick_ / stride_) % 50 == 0) {  // log mỗi 1s
    std::cout << "  q_internal: ";
    for (int i = 0; i < 27; ++i) std::cout << q_internal_[i] << " ";
    std::cout << "\n  motion_ref: ";
    for (int i = 0; i < 27; ++i) std::cout << last_joint_pos_onnx_[i] << " ";
    std::cout << "\n";
}
```

- Nếu `q_internal` lệch xa `motion_ref` ở các joint ankle → bằng chứng ankle IK sai
- Nếu lệch xa đều ở tất cả joints → gain/physics mismatch tổng thể

### Bước 3.3 — Quan sát visual nguyên nhân ngã

Trong MuJoCo:
- Bật `F` (contact force) → xem foot có slip không, hand có chạm đất bất thường không
- Bật `J` (joints) → xem joint nào rung trước khi ngã
- Bấm `Space` pause rồi `←` từng frame để phát hiện joint đầu tiên bất ổn

### Bước 3.4 — Loại trừ parallel linkage (đắt)

**Sửa mj_sim để dùng XML training**:

```bash
# Copy mjlab XML + meshes sang vm_packages
cp ~/Documents/Humanoid_Tracking_Task/mjlab/src/mjlab/asset_zoo/robots/M2v6/M2v6.xml \
   ~/Documents/vm_packages/robots/M2v6/M2v6_direct_ankle.xml

# Tạo scene mới
cat > ~/Documents/vm_packages/robots/M2v6/scene_M2v6_direct.xml << 'EOF'
<mujoco model="m2 direct ankle">
  <include file="M2v6_direct_ankle.xml"/>
  <statistic center="0.5 0.5 1.2" extent="1.0"/>
  <visual>
    <headlight diffuse="0.6 0.6 0.6" ambient="0.3 0.3 0.3"/>
    <global azimuth="-130" elevation="-20"/>
  </visual>
  <worldbody>
    <light pos="0 0 1.5" dir="0 0 -1" directional="true"/>
    <geom name="floor" size="0 0 .05" type="plane" material="groundplane"/>
  </worldbody>
</mujoco>
EOF

# Thay scene trong mj_sim config.yaml
sed -i 's|scene_M2v6_real.xml|scene_M2v6_direct.xml|g' \
   ~/Documents/vm_packages/mj_sim/config.yaml   # verify path trước khi sửa!
```

Sau đó set `ankle_mode: 0` trong `Config/sim/M2v6/properties.yaml` để dùng `LegController_M2v6_sim`. Rebuild, restart sim.

**Rủi ro**: meshes/collision paths trong mjlab XML có thể reference khác — cần check sau khi thay.

### Bước 3.5 — Train lâu hơn với domain randomization (đắt nhất)

Trong `tracking_env_cfg.py` hoặc `m2v6/env_cfgs.py`, thêm DR cho ankle joint gains và mass:

```python
from mjlab.managers.event_manager import EventTermCfg
from mjlab.envs.mdp.events import randomize_actuator_gains

cfg.events["ankle_gains_dr"] = EventTermCfg(
    func=randomize_actuator_gains,
    mode="startup",
    params={
        "asset_cfg": SceneEntityCfg("robot", joint_names=".*ankle.*"),
        "stiffness_distribution_params": (0.8, 1.2),  # ±20%
        "damping_distribution_params": (0.8, 1.2),
        "operation": "scale",
    },
)
```

Train thêm với warm-start từ checkpoint hiện tại. Policy sẽ robust hơn với ankle dynamics khác nhau → dễ chuyển sang parallel linkage.

## 4. Đề xuất thứ tự thử (thực tế)

### Lựa chọn A — Ưu tiên thử rẻ trước

1. **Train thêm 5-10k iter** (~2-3 giờ) → xem reward có ổn ở 40+ không
2. **Re-deploy** checkpoint mới vào sim, test lại
3. Nếu vẫn ngã → làm Bước 3.1 + 3.3 để xác định kiểu ngã
4. Dựa vào kiểu ngã → chọn giữa XML mismatch (3.4) hay DR (3.5)

### Lựa chọn B — Tấn công gốc XML mismatch

1. Bước 3.4 — dùng XML training cho mj_sim
2. Nếu work → deploy thật (real hardware) sẽ có vấn đề parallel linkage; cần train lại với XML real
3. Nếu không work → loại trừ ankle, quay lại options khác

### Lựa chọn C — Pragmatic (khuyên dùng nếu gấp)

1. Train thêm iter đến 15k
2. Deploy vào sim, chấp nhận đứng ~5s (thường đủ để deploy demo)
3. Chain sang walk policy để giữ đứng dài hơn (không được chain tự động nếu `hold=end`)

## 5. Kiểm tra hiện trạng sau khi ngã

Sau khi sim chạy lại và ngã, thu thập:

```bash
# Full log stretch
tmux capture-pane -t Sim_test:Experiment -p -S -500 > /tmp/ctrl_fall.log
grep -E 'motion step|transition|Inference|Warning|Error' /tmp/ctrl_fall.log | tail -40

# Robot position timeline — cần thêm print vào C++ hoặc dùng msgpack log:
ls -la ~/Documents/vm_packages/vm_ctrl/log/stateLogging_*.msgpack | tail -3
```

MSGpack log chứa full state — có thể phân tích offline bằng Python để plot joint trajectory, CoM drift, v.v.

## 6. Tài liệu tham khảo

- [14 — export checkpoint](14-export-checkpoint-to-vm-packages.md) — workflow deploy
- `vm_ctrl/src/common/LegControllers/LegController_M2v6_real.cpp` — ankle IK code
- `vm_ctrl/RLController/src/rl_tracking_27dof.cpp:257-362` — policy step()
- `mjlab/src/mjlab/asset_zoo/robots/M2v6/M2v6.xml` — training robot XML
- `vm_packages/robots/M2v6/M2v6_real.xml` — deploy robot XML
- `vm_packages/vm_ctrl/Config/sim/M2v6/properties.yaml:5` — `ankle_mode: 1`
