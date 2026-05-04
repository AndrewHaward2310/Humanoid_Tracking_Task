# Hướng dẫn Quay Video

Bước đầu tiên trong quy trình là quay video người thật thực hiện động tác mong muốn. Chất lượng video ảnh hưởng trực tiếp đến chất lượng motion được trích xuất.

---

## 1. Yêu cầu Thiết bị

### Camera
- Điện thoại thông minh (iPhone/Android) là đủ
- Nếu dùng iPhone: lens 1x (24mm) là lý tưởng nhất
- Không nên dùng lens góc rộng (0.5x) vì gây méo hình

### Chân máy (Tripod)
- **Rất khuyến nghị** dùng chân máy hoặc đặt điện thoại cố định
- Camera tĩnh giúp thuật toán hoạt động chính xác hơn
- Nếu camera tĩnh, khi chạy GVHMR sẽ bật cờ "tĩnh" để bỏ qua bước ước lượng chuyển động camera

---

## 2. Thiết lập Bối cảnh

### Ánh sáng
- Sử dụng ánh sáng đều, tránh bóng đổ mạnh
- Tốt nhất: ánh sáng ban ngày gần cửa sổ, hoặc đèn studio
- Tránh: quay ngược sáng (người đen, nền sáng)

### Phông nền
- Nên dùng nền đơn sắc, không có họa tiết phức tạp
- Tránh: nền có nhiều người khác (gây nhầm lẫn khi phát hiện người)
- Đảm bảo toàn bộ cơ thể người luôn nằm trong khung hình

### Không gian
- Đảm bảo người thực hiện có đủ chỗ để thực hiện động tác
- Giữ khoảng cách camera-người khoảng 2-4 mét

---

## 3. Hướng dẫn cho Người Thực hiện

### Trang phục
- Mặc đồ ôm sát hoặc vừa vặn
- Tránh quần áo rộng thùng thình (gây nhiễu khi ước lượng tư thế)
- Màu quần áo nên khác với phông nền

### Thực hiện động tác
- Bắt đầu ở tư thế đứng thẳng tự nhiên, giữ yên khoảng 1-2 giây
- Thực hiện động tác với tốc độ bình thường — không quá nhanh
- Kết thúc ở tư thế đứng thẳng, giữ yên 1-2 giây
- Hạn chế tự xoay (xoay quanh trục z) nếu không cần thiết

### Đặc biệt cho động tác chào mừng
- Giơ tay vẫy ở tầm vai hoặc cao hơn
- Giữ cánh tay duỗi thẳng hoặc hơi cong tự nhiên
- Chú ý: robot M2v6 không có ngón tay, nên chỉ cần tập trung vào cánh tay và thân

---

## 4. Góc quay Khuyến nghị

### Góc tốt nhất: chính diện hoặc chéo 45°
- Quay từ phía trước, camera ngang tầm hông hoặc ngực
- Hoặc quay chéo 45° để thấy được cả chiều sâu

### Góc nên tránh
- Quay từ phía sau (mất thông tin mặt và tay)
- Quay từ trên xuống hoặc dưới lên (méo tỉ lệ)
- Quay quá gần (không thấy hết cơ thể)

### Nhiều góc quay
- Nếu có thể, quay cùng một động tác từ 2-3 góc khác nhau
- Chọn kết quả trích xuất 3D tốt nhất để sử dụng

---

## 5. Thông số Video

| Thông số | Khuyến nghị | Tối thiểu |
|----------|-------------|-----------|
| Độ phân giải | 1080p (1920x1080) | 720p |
| Tốc độ khung hình | 30fps | 24fps |
| Định dạng | MP4 (H.264) | Bất kỳ |
| Thời lượng | 3-10 giây | 2 giây |

> Lưu ý: Video quá dài không cần thiết — chỉ cần đoạn chứa động tác chính. Bạn có thể cắt video sau khi quay.

---

## 6. Sau khi Quay

### Kiểm tra video
- Xem lại để đảm bảo toàn bộ cơ thể luôn trong khung hình
- Kiểm tra xem có bị mờ (motion blur) không
- Đảm bảo ánh sáng đủ, không quá tối

### Cắt video (nếu cần)
Chỉ giữ phần chứa động tác. Dùng ffmpeg để cắt nhanh:

```bash
# Cắt video từ giây thứ 2 đến giây thứ 8
ffmpeg -i nguon.mp4 -ss 00:00:02 -to 00:00:08 -c copy dong_tac_chao.mp4
```

### Đặt file video
Sao chép video đã cắt vào thư mục đầu vào của GVHMR:

```bash
cp dong_tac_chao.mp4 /home/nguyenld12/Documents/Humanoid_Tracking_Task/vm_video2robot/inputs/demo/
```

---

## 7. Checklist Trước khi Sang Bước Tiếp

- [ ] Video có độ phân giải ít nhất 720p
- [ ] Toàn bộ cơ thể luôn trong khung hình
- [ ] Ánh sáng đủ, không quá tối
- [ ] Chỉ có 1 người trong khung hình
- [ ] Camera cố định (hoặc ít di chuyển)
- [ ] Động tác bắt đầu và kết thúc ở tư thế đứng
- [ ] Video đã được cắt và đặt vào thư mục đúng

---

## Tiếp theo

Hãy đọc tiếp:
1. [Notebook trích xuất GVHMR](../notebooks/01_trich_xuat_gvhmr.ipynb) — Chạy GVHMR trên video
2. [Hướng dẫn Retargeting](05-huong-dan-retargeting.md) — Chuyển đổi sang robot
