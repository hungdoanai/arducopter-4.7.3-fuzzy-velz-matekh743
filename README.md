# ArduCopter 4.7.3 — Fuzzy-PID Sugeno Bậc 0 Vòng Vận Tốc Trục Độ Cao (Matek H743)

Dự án tích hợp bộ điều khiển **Sugeno bậc 0 đối xứng tâm (7×7 luật mờ, phép nhân đại số và trung bình có trọng số)** vào vòng điều khiển vận tốc trục độ cao (trục Z/Down — `AC_PosControl::D_update_controller()`) cho **ArduCopter 4.7.3**, hỗ trợ bật/tắt tức thì bằng **công tắc tay điều khiển TX** và cấu hình trực tiếp trên **Mission Planner**, biên dịch tự động ra firmware `.apj` nạp cho bo mạch **Matek H743**.

## Tham số cấu hình trên Mission Planner
Sau khi nạp file `arducopter.apj`, vào **Config → Full Parameter List** và tìm nhóm **`PSC_D_FZ_`**:

| Tham số | Mặc định | Dải giá trị | Ý nghĩa |
| :--- | :---: | :---: | :--- |
| `PSC_D_FZ_ENABLE` | `0` | `0` (Tắt) / `1` (Bật) | Bật/tắt Fuzzy khi `PSC_D_FZ_RC = 0` (`0` = chạy PID gốc ArduCopter 4.7.3) |
| `PSC_D_FZ_RC` | `0` | `0` hoặc `5 – 16` | Kênh công tắc RC trên tay điều khiển TX để bật/tắt Fuzzy khi đang bay (`PWM ≥ 1700`: Bật, `PWM ≤ 1300`: Tắt + hiện thông báo `Fuzzy VelZ: ON/OFF` lên Mission Planner/OSD; để `0` để tắt điều khiển qua TX) |
| `PSC_D_FZ_E_MAX` | `1.5` | `0.2 – 5.0` m/s | Ngưỡng chuẩn hóa sai số vận tốc trục Z về `[-1, 1]` |
| `PSC_D_FZ_ED_MAX` | `3.0` | `0.5 – 15.0` m/s² | Ngưỡng chuẩn hóa đạo hàm sai số vận tốc về `[-1, 1]` |
| `PSC_D_FZ_DP_MAX` | `2.0` | `0.0 – 5.0` | Biên độ hiệu chỉnh `±ΔKp` tối đa quanh `PSC_D_VEL_P` |
| `PSC_D_FZ_DI_MAX` | `0.5` | `0.0 – 3.0` | Biên độ hiệu chỉnh `±ΔKi` tối đa quanh `PSC_D_VEL_I` |
| `PSC_D_FZ_DD_MAX` | `0.10` | `0.0 – 1.0` | Biên độ hiệu chỉnh `±ΔKd` tối đa quanh `PSC_D_VEL_D` |
| `PSC_D_FZ_FILT` | `5.0` | `0.5 – 30.0` Hz | Tần số cắt bộ lọc thông thấp cho đạo hàm `de/dt` |

## Cách tải file `.apj` và nạp cho Matek H743
1. Vào tab **Actions** trên kho lưu trữ này → chọn lần chạy **Build ArduCopter 4.7.3 Fuzzy-VelZ (MatekH743)** mới nhất → tải mục **Artifacts → `ArduCopter-4.7.3-FuzzyVelZ-MatekH743.zip`**.
2. Giải nén lấy file **`arducopter.apj`**.
3. Mở **Mission Planner** → **Setup → Install Firmware → Load custom firmware** → chọn `arducopter.apj` để nạp cho bo Matek H743.
4. Gán kênh công tắc TX rảnh vào `PSC_D_FZ_RC` (ví dụ `7` cho kênh CH7), gạt thử trên mặt đất để thấy dòng `Fuzzy VelZ: ON / OFF` trên tab **Messages** của Mission Planner, rồi mới bay thử ở chế độ `AltHold` / `Loiter`.
