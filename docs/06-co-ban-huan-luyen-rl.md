# Cơ bản về Huấn luyện RL

Tài liệu này giải thích nền tảng lý thuyết về Học tăng cường (Reinforcement Learning - RL) được áp dụng trong bài toán motion tracking. Không cần kiến thức toán học nâng cao — mọi thứ được giải thích bằng trực giác.

---

## 1. Học tăng cường là gì?

### Ý tưởng
Hãy tưởng tượng bạn đang tập đi xe đạp lần đầu:
- Bạn **quan sát**: "xe đang nghiêng trái, tốc độ chậm"
- Bạn **hành động**: "đánh lái phải, đạp mạnh hơn"
- Bạn **nhận phản hồi**: "ổn! không ngã" (thưởng) hoặc "ngã rồi!" (phạt)
- Qua nhiều lần, bạn **học được** cách giữ thăng bằng

Robot học theo cách tương tự — nhưng thay vì vài ngày, nó học trong vài giờ nhờ chạy hàng nghìn mô phỏng song song.

### Các thành phần chính

| Thành phần | Trong bài toán tracking | Ví dụ cụ thể |
|------------|------------------------|--------------|
| Agent (tác nhân) | Robot M2v6 | Con robot trong mô phỏng |
| Environment (môi trường) | Thế giới MuJoCo | Mặt phẳng + trọng lực |
| State (trạng thái) | Tư thế hiện tại của robot | Góc các khớp, vận tốc |
| Observation (quan sát) | Thông tin robot "thấy" | IMU, encoder khớp, motion tham chiếu |
| Action (hành động) | Lệnh điều khiển | Góc khớp mục tiêu (27 giá trị) |
| Reward (phần thưởng) | Đánh giá chất lượng | Sai số vị trí tay/chân so với tham chiếu |
| Episode | 1 lần chạy mô phỏng | 10 giây (500 bước, mỗi bước 0.02 giây) |

---

## 2. Observation — Robot "nhìn thấy" gì?

Mỗi bước, robot nhận một vector số thực chứa các thông tin sau:

### Nhóm Actor (cái robot thấy)

Đây là thông tin đầu vào cho mạng neural quyết định hành động:

| Thông tin | Mô tả | Nhiễu |
|-----------|-------|-------|
| Command (motion tham chiếu) | Vị trí và hướng cần đạt của từng bộ phận | Không |
| Vị trí anchor tương đối | Khoảng cách giữa robot và điểm neo tham chiếu | ±0.25 |
| Hướng anchor tương đối | Sai lệch hướng quay giữa robot và tham chiếu | ±0.15 |
| Vận tốc tuyến tính (IMU) | Tốc độ di chuyển của thân robot | ±0.5 |
| Vận tốc góc (IMU) | Tốc độ quay của thân robot | ±0.2 |
| Góc khớp | Góc hiện tại của 27 khớp (so với vị trí mặc định) | ±0.01 |
| Vận tốc khớp | Tốc độ quay của 27 khớp | ±0.5 |
| Hành động trước | Lệnh điều khiển ở bước trước | Không |

> **Nhiễu (noise):** Trong quá trình huấn luyện, hệ thống cố tình thêm nhiễu vào các quan sát. Mục đích là để policy học cách xử lý khi cảm biến không chính xác — giống thực tế trên robot thật. Khi đánh giá, nhiễu sẽ bị tắt.

### Nhóm Critic (dùng khi huấn luyện)

Critic có thêm thông tin "đặc quyền" mà robot thật không có — giúp đánh giá tình huống chính xác hơn:

- Vị trí và hướng các bộ phận cơ thể (không nhiễu)
- Tất cả thông tin của actor (không nhiễu)

---

## 3. Action — Robot "làm" gì?

### Điều khiển vị trí khớp

Mỗi bước, mạng neural xuất ra **27 giá trị** — mỗi giá trị là lượng thay đổi góc mong muốn cho 1 khớp.

Cách tính góc khớp mục tiêu:

```
Góc mục tiêu = Góc mặc định + (Action × Action Scale)
```

Trong đó:
- **Góc mặc định:** tư thế đứng thẳng (đầu gối hơi cong)
- **Action:** giá trị từ mạng neural (thường trong khoảng -1 đến 1)
- **Action Scale:** hệ số khuếch đại — khác nhau cho từng nhóm khớp

Sau đó, bộ điều khiển PD trong MuJoCo sẽ tính lực cần thiết để đưa khớp về góc mục tiêu.

### Tại sao dùng Action Scale?

Mỗi nhóm khớp có lực tối đa và stiffness khác nhau. Action Scale được tính sao cho action = 1.0 tương ứng với khoảng 25% lực tối đa, đảm bảo robot hoạt động ổn định.

### Decimation (bước nhảy)

Một điểm quan trọng: mô phỏng vật lý chạy ở 200Hz (mỗi bước 0.005 giây), nhưng policy chỉ ra quyết định ở 50Hz (mỗi bước 0.02 giây). Nghĩa là:
- Cứ 4 bước vật lý, policy mới ra 1 hành động mới
- Trong 4 bước vật lý đó, hành động giữ nguyên
- Điều này giúp chuyển động mượt hơn và giảm tải tính toán

---

## 4. Reward — Đánh giá như thế nào?

Hàm reward là "la bàn" hướng dẫn robot học hành vi đúng. Trong motion tracking, reward gồm 2 nhóm: **thưởng** (khuyến khích bắt chước đúng) và **phạt** (trừ khi hành vi xấu).

### Nhóm Thưởng (Tracking)

Mỗi thành phần thưởng đo sai số giữa robot và chuyển động tham chiếu, rồi chuyển thành điểm từ 0 đến 1 bằng hàm mũ:

```
Reward = exp(-sai_số² / ngưỡng²)
```

Sai số nhỏ → reward gần 1 (tốt). Sai số lớn → reward gần 0 (kém).

| Thành phần reward | Trọng số | Đo lường | Ngưỡng |
|-------------------|----------|----------|--------|
| Vị trí gốc (anchor) | 0.5 | Khoảng cách giữa hông robot và hông tham chiếu | 0.3 m |
| Hướng gốc (anchor) | 0.5 | Sai lệch hướng quay hông | 0.4 rad |
| Vị trí các bộ phận | 1.0 | Trung bình sai số vị trí 14 bộ phận | 0.3 m |
| Hướng các bộ phận | 1.0 | Trung bình sai lệch hướng 14 bộ phận | 0.4 rad |
| Vận tốc tuyến tính | 1.0 | Sai số vận tốc di chuyển các bộ phận | 1.0 m/s |
| Vận tốc góc | 1.0 | Sai số vận tốc quay các bộ phận | 3.14 rad/s |

### Nhóm Phạt

| Thành phần phạt | Trọng số | Mục đích |
|------------------|----------|----------|
| Tốc độ thay đổi action | -0.1 | Tránh giật, chuyển động mượt |
| Giới hạn khớp | -10.0 | Phạt nặng khi khớp chạm giới hạn |
| Tự va chạm | -10.0 | Phạt nặng khi các bộ phận đâm vào nhau |

> **Trọng số âm** = phạt. Trọng số càng lớn (về giá trị tuyệt đối), ảnh hưởng càng mạnh. Giới hạn khớp và tự va chạm bị phạt rất nặng (-10.0) vì đây là hành vi nguy hiểm trên robot thật.

---

## 5. Termination — Khi nào dừng?

Một episode kết thúc sớm khi xảy ra một trong các điều kiện sau:

| Điều kiện | Ngưỡng | Ý nghĩa |
|-----------|--------|---------|
| Hết thời gian | 10 giây | Episode hoàn thành (tốt) |
| Sai lệch chiều cao gốc | > 0.25 m | Robot "lạc" quá xa tham chiếu theo chiều đứng |
| Sai lệch hướng gốc | > 0.8 | Hướng quay lệch quá nhiều (gần như ngã) |
| Sai lệch tay/chân | > 0.25 m | Một trong 4 chi quá xa vị trí tham chiếu |

Khi episode kết thúc sớm, robot được reset về vị trí ngẫu nhiên trong motion tham chiếu và bắt đầu lại. Qua hàng triệu episode, robot dần học cách duy trì tracking suốt 10 giây.

---

## 6. PPO — Thuật toán Huấn luyện

### Tổng quan

PPO (Proximal Policy Optimization) là thuật toán RL phổ biến nhất cho điều khiển robot liên tục. Nó hoạt động theo chu kỳ:

```
┌──────────────────────────────────────────────────┐
│  1. Thu thập dữ liệu (rollout)                   │
│     → Chạy 4096 robot song song, mỗi robot 24    │
│       bước → được 98304 mẫu dữ liệu              │
│                                                   │
│  2. Tính advantage                                │
│     → Critic đánh giá: "tình huống này tốt/xấu"  │
│                                                   │
│  3. Cập nhật mạng neural                          │
│     → 5 epochs, 4 mini-batches mỗi epoch          │
│     → Actor học hành động tốt hơn                 │
│     → Critic học đánh giá chính xác hơn           │
│                                                   │
│  4. Lặp lại                                       │
└──────────────────────────────────────────────────┘
```

### Cấu hình PPO cho M2v6

| Tham số | Giá trị | Ý nghĩa |
|---------|---------|---------|
| Actor hidden dims | (512, 256, 128) | 3 lớp ẩn, từ rộng đến hẹp |
| Critic hidden dims | (512, 256, 128) | Cùng kiến trúc với actor |
| Activation | ELU | Hàm kích hoạt |
| Learning rate | 0.001 | Tốc độ học (adaptive) |
| Gamma (γ) | 0.99 | Tầm nhìn xa (gần 1 = coi trọng tương lai) |
| Lambda (λ) | 0.95 | Hệ số GAE (cân bằng bias-variance) |
| Clip param | 0.2 | Giới hạn mức thay đổi policy mỗi lần cập nhật |
| Entropy coef | 0.005 | Khuyến khích khám phá (nhỏ = ít khám phá) |
| Num epochs | 5 | Số lần lặp trên cùng batch dữ liệu |
| Mini batches | 4 | Chia batch thành 4 phần nhỏ |
| Max iterations | 30,000 | Tổng số vòng lặp lớn |
| Save interval | 500 | Lưu checkpoint mỗi 500 iteration |

### Actor và Critic

**Actor:** Quyết định robot làm gì

```
Observation (vector số) → [512] → [256] → [128] → Action (27 giá trị)
```

- Nhận observation, xuất ra phân phối xác suất cho action
- Dùng GaussianDistribution: mỗi action được lấy mẫu từ phân phối chuẩn
- Ban đầu std = 1.0 (khám phá nhiều), giảm dần khi học

**Critic:** Đánh giá tình huống

```
Observation (vector số) → [512] → [256] → [128] → Value (1 giá trị)
```

- Nhận observation "đặc quyền" (không nhiễu, thêm thông tin)
- Xuất ra 1 số: ước lượng tổng reward từ bây giờ đến hết episode
- Giúp actor biết "tình huống hiện tại tốt hay xấu"

---

## 7. Domain Randomization — Học để Chống chịu

Trong quá trình huấn luyện, hệ thống cố tình thay đổi ngẫu nhiên các tham số mô phỏng:

| Tham số ngẫu nhiên | Phạm vi | Mục đích |
|---------------------|---------|----------|
| Ma sát bàn chân | 0.3 — 1.2 | Sàn trơn/nhám |
| Stiffness khớp | 80% — 120% | Motor mạnh/yếu |
| Damping khớp | 80% — 120% | Cản nhiều/ít |
| Trọng tâm | ±2.5cm (x), ±5cm (y,z) | Tải trọng lệch |
| Sai số encoder | ±0.01 rad | Cảm biến không chính xác |
| Đẩy robot | Mỗi 1-3 giây | Nhiễu loạn ngoại lực |

Mục đích: policy học được sẽ **mạnh mẽ** (robust) — hoạt động tốt dù điều kiện thay đổi. Đặc biệt quan trọng khi chuyển sang robot thật (sim-to-real transfer).

---

## 8. Tại sao 4096 Môi trường Song song?

### Bài toán
PPO cần rất nhiều dữ liệu để học. Nếu chạy 1 robot:
- 30,000 iterations × 24 bước = 720,000 bước
- Mỗi bước 0.02 giây → 14,400 giây mô phỏng = 4 giờ (real-time)

### Giải pháp
Chạy 4096 robot song song trên GPU:
- Mỗi iteration: 4096 × 24 = 98,304 bước
- GPU xử lý tất cả cùng lúc → thời gian gần như không tăng
- Tổng dữ liệu: 30,000 × 98,304 ≈ 3 tỷ bước — trong vài giờ

### Giới hạn VRAM
Với RTX 5070 (8GB VRAM), có thể cần giảm xuống 2048 hoặc 1024 môi trường. Thời gian huấn luyện sẽ tăng tỉ lệ tương ứng.

---

## 9. Tóm tắt Flow Huấn luyện

```
Bước 1: Khởi tạo 4096 robot trong MuJoCo Warp
    │
Bước 2: Mỗi robot nhận motion tham chiếu (NPZ)
    │
Bước 3: Robot quan sát → Actor quyết định → Robot hành động
    │
Bước 4: MuJoCo mô phỏng vật lý (4 bước × 0.005s)
    │
Bước 5: Tính reward (sai số tracking + phạt)
    │
Bước 6: Kiểm tra termination (ngã? lệch quá xa?)
    │
Bước 7: Sau 24 bước → PPO cập nhật Actor + Critic
    │
Bước 8: Lặp lại 30,000 lần → Policy hoàn thiện
```

---

## Tiếp theo

Hãy đọc tiếp:
1. [Huấn luyện Motion Imitation](07-huan-luyen-motion-imitation.md) — Thực hành huấn luyện
2. [Đánh giá Policy](08-danh-gia-policy.md) — Kiểm tra kết quả
