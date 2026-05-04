# Hướng dẫn đọc WandB Workspace — mjlab Motion Tracking

Hướng dẫn cách đọc và hiểu WandB Workspace khi training với mjlab framework.

---

## 1. Tổng quan Workspace

Khi mở WandB Workspace, bạn sẽ thấy giao diện gồm 2 phần chính:

![Workspace Overview](/home/nguyenld12/.gemini/antigravity/brain/4d4bb2b1-ac80-46aa-8cd2-7877a36c18eb/wandb_workspace_top_1776311185644.png)

### Sidebar trái: Danh sách Runs

- Mỗi run là **1 lần chạy training**, đặt tên theo timestamp (VD: `2026-04-16_10-02-59`)
- 🟢 Chấm xanh = đang chạy | ⚫ Chấm xám = đã kết thúc | 🔴 Chấm đỏ = lỗi
- Bấm vào 👁️ icon để ẩn/hiện run trên biểu đồ — giúp so sánh các lần training

### Khu vực chính: Biểu đồ theo nhóm

Workspace chia thành **10 nhóm** biểu đồ. Cách đọc từng nhóm:

| Nhóm | Số chart | Quan trọng | Ý nghĩa |
|------|----------|:----------:|---------|
| **Train** | 2 | ⭐⭐⭐ | Mean reward & episode length — **xem đầu tiên** |
| **Episode_Reward** | 21 | ⭐⭐⭐ | Chi tiết từng reward component |
| **Episode_Termination** | 5 | ⭐⭐ | Lý do robot bị reset |
| **Metrics** | 20 | ⭐⭐ | Sai số tracking thực tế (mét, radian) |
| **Loss** | 4 | ⭐ | Loss functions (value, surrogate, entropy, lr) |
| **Policy** | 2 | ⭐ | Noise std — mức exploration |
| **Curriculum** | 6 | ⭐ | Target velocities (nếu dùng curriculum) |
| **Episode_Metrics** | 1 | - | Mean action acceleration |
| **Perf** | 3 | - | Performance (steps/s, timing) |
| **System** | 23 | - | CPU/GPU/RAM/Network usage |

> [!TIP]
> **Xem nhanh:** Chỉ cần nhìn nhóm **Train** (2 charts) + **Episode_Termination** là biết training tốt hay không.

---

## 2. Nhóm Train — 2 biểu đồ quan trọng nhất

> Cuộn xuống cuối workspace hoặc tìm bằng search box `Train`

| Biểu đồ | Xu hướng tốt | Xu hướng xấu |
|----------|:------------:|:------------:|
| `Train/mean_reward` | 📈 Tăng đều | Đi ngang sớm hoặc giảm |
| `Train/mean_episode_length` | 📈 Tăng tới max | Thấp = robot ngã sớm |

**Cách so sánh runs:** Nếu nhiều đường cùng hiện trên biểu đồ, đường nào **cao hơn** là run có performance tốt hơn.

---

## 3. Nhóm Episode_Reward — Chi tiết phần thưởng

![Episode Reward](/home/nguyenld12/.gemini/antigravity/brain/4d4bb2b1-ac80-46aa-8cd2-7877a36c18eb/wandb_workspace_scroll_1_1776311198026.png)

21 biểu đồ chia thành 2 loại:

### Tracking rewards (dương — càng cao càng tốt)

| Chart | Ý nghĩa | Mô tả |
|-------|---------|-------|
| `motion_global_root_pos` | Vị trí gốc (pelvis) | Robot đứng đúng vị trí reference? |
| `motion_global_root_ori` | Hướng quay gốc | Robot quay mặt đúng hướng? |
| `motion_body_pos` | Vị trí tay/chân/đầu | Các bộ phận đúng vị trí? |
| `motion_body_ori` | Hướng quay body parts | Tay chân quay đúng góc? |
| `motion_body_lin_vel` | Vận tốc di chuyển | Tốc độ khớp với reference? |
| `motion_body_ang_vel` | Vận tốc xoay | Tốc độ xoay khớp? |

### Penalty rewards (âm — càng gần 0 càng tốt)

| Chart | Ý nghĩa |
|-------|---------|
| `action_rate_l2` | Phạt thay đổi action đột ngột (giật) |
| `joint_limit` | Phạt vượt giới hạn khớp |
| `self_collisions` | Phạt tay đâm vào thân |

---

## 4. Nhóm Episode_Termination — Lý do robot bị reset

Nhóm này cho biết **tại sao** episode kết thúc. Mỗi chart ghi số lần terminate vì lý do đó:

| Chart | Ý nghĩa | Khi training tốt |
|-------|---------|:-----------------:|
| `time_out` | Sống đến hết episode | 📈 **Tăng** (TỐT!) |
| `ee_body_pos` | Tay/chân lệch quá xa | 📉 Giảm |
| `anchor_pos` | Pelvis lệch quá xa | 📉 Giảm |
| `anchor_ori` | Hướng quay pelvis lệch | 📉 Giảm |
| `fell_over` | Robot ngã | 📉 Giảm |

**Quy luật:** Training thành công = `time_out` chiếm đa số, các lý do khác gần 0.

---

## 5. Nhóm Metrics — Sai số thực tế

![Error Metrics](/home/nguyenld12/.gemini/antigravity/brain/4d4bb2b1-ac80-46aa-8cd2-7877a36c18eb/wandb_workspace_scroll_3_1776311205760.png)

20 biểu đồ sai số motion tracking — **tất cả phải giảm dần**:

| Nhóm sai số | Charts | Đơn vị | Giá trị mục tiêu |
|-------------|--------|--------|:-----------------:|
| **Anchor (pelvis)** | `error_anchor_pos`, `error_anchor_rot`, `error_anchor_lin_vel`, `error_anchor_ang_vel` | m, rad, m/s, rad/s | < 0.1, < 0.2, < 0.5, < 1.0 |
| **Body (toàn thân)** | `error_body_pos`, `error_body_rot`, `error_body_lin_vel`, `error_body_ang_vel` | m, rad, m/s, rad/s | < 0.1, < 0.3, < 0.5, < 1.5 |
| **Joint (khớp)** | `error_joint_pos`, `error_joint_vel` | rad, rad/s | < 0.5, < 5.0 |
| **Sampling** | `sampling_entropy`, `sampling_top1_prob`, `sampling_top1_bin` | - | Tham khảo |
| **Other** | `landing_force_mean` | N | Tham khảo |

> [!NOTE]
> Khi so sánh runs: đường nào có **sai số thấp hơn** = run đó bắt chước chính xác hơn.

---

## 6. Nhóm Loss — Ổn định training

![Loss & Metrics](/home/nguyenld12/.gemini/antigravity/brain/4d4bb2b1-ac80-46aa-8cd2-7877a36c18eb/wandb_workspace_scroll_2_1776311202043.png)

| Chart | Ý nghĩa | Xu hướng bình thường |
|-------|---------|:--------------------:|
| `Loss/value` | Critic loss | Giảm → ổn định |
| `Loss/surrogate` | PPO policy loss | Dao động nhỏ gần 0 |
| `Loss/learning_rate` | Learning rate hiện tại | Giảm dần (adaptive schedule) |
| `Loss/entropy` | Mức exploration của policy | Giảm chậm dần |

**⚠️ Cảnh báo:** Nếu `value loss` tăng đột ngột → training bất ổn, cần giảm learning rate.

---

## 7. Nhóm Policy & Curriculum

### Policy (2 charts)
| Chart | Ý nghĩa |
|-------|---------|
| `Policy/mean_std` | Std dev output (nhìn chung) |
| `Policy/mean_noise_std` | Nhiễu thêm vào action |

Cả 2 nên **giảm dần** từ ~1.0 → ~0.2-0.3, cho thấy policy ngày càng tự tin.

### Curriculum (6 charts)
Các chart `command_vel/lin_vel_*` và `command_vel/ang_vel_*` hiện target velocities. Nếu project dùng curriculum learning, các giá trị này sẽ thay đổi theo bậc thang. Với task tracking motion, nhóm này có thể không thay đổi.

---

## 8. So sánh nhiều Runs

Workspace hiển thị **tất cả 17 runs** trên cùng biểu đồ — đây là sức mạnh chính của WandB workspace:

### Cách đọc kết quả

- **Mỗi đường màu** = 1 training run (xem chú thích phía trên biểu đồ)
- **Run nào cao hơn trong reward** = performance tốt hơn
- **Run nào thấp hơn trong error** = tracking chính xác hơn
- Bấm 👁️ bên sidebar để **ẩn/hiện** run, giúp dễ so sánh

### Pattern trong workspace hiện tại

Từ screenshots, ta thấy:
- **Run mới nhất** (`2026-04-16_10-02-59`, đường xanh dương) đang có reward cao nhất
- **Các run cũ** (tháng 3/2026) là các experiment trước đó, có thể dùng config/robot khác
- **Tất cả tracking rewards** đều cho thấy xu hướng tăng — framework hoạt động tốt

---

## 9. Checklist đánh giá nhanh

Khi mở WandB, kiểm tra theo thứ tự:

```
1. ✅ Mean reward tăng đều?        → Nhóm Train
2. ✅ Episode length tăng tới max? → Nhóm Train
3. ✅ time_out termination tăng?   → Nhóm Episode_Termination
4. ✅ Error metrics giảm?          → Nhóm Metrics
5. ✅ Noise std giảm?              → Nhóm Policy
```

Nếu cả 5 đều ✅ → Training đang **tốt**, cứ để chạy tiếp.

---

## 10. Dấu hiệu cảnh báo

| Dấu hiệu | Nguyên nhân | Cách fix |
|-----------|-------------|----------|
| Reward đi ngang sớm (< 5K iter) | LR quá thấp hoặc reward shaping chưa tốt | Tăng LR lên 3e-3 |
| Reward giảm đột ngột | LR quá cao, policy collapse | Giảm LR xuống 3e-4, giảm `desired_kl` |
| Episode length không tăng | Robot không đứng được | Kiểm tra reference motion |
| Error tăng trong khi reward tăng | Reward "ăn gian" (exploit penalty) | Tăng weight tracking rewards |
| Entropy giảm quá nhanh về 0 | Kẹt local optima | Tăng `entropy_coef` |
| `ee_body_pos` termination không giảm | Tay chân dao động mạnh | Tăng `action_rate_l2` penalty |
