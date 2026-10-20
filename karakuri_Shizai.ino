#include <AccelStepper.h>

#define PUL_PIN A0 // kết nối với chân PUL động cơ bước  
#define DIR_PIN A1 // 
#define ENA_PIN A2

#define CAM_BIEN_12 12  // cảm biến tịm cận dụng để xác định vị trí quay của động cơ bước 
#define PIN_TIEN    24  // Nhận tín hiệu từ chân 11 AGV
#define PIN_LUI     22  // Nhận tín hiệu từ chân 12 AGV
#define PIN_TU_DONG 26  // Nhận tín hiệu từ chân 10 AGV

AccelStepper stepper(1, PUL_PIN, DIR_PIN);

const float STEPS_PER_REV = 1000.0;
float GOC_DO = 500.0; 

float TOC_DO_TIM_GOC = 500.0; 
float TOC_DO_LAM_VIEC = 1250.0;  
float GIA_TOC_LAM_VIEC = 1500.0; 
const unsigned long THOI_GIAN_CHONG_NHIEU = 10; 

// --- CÀI ĐẶT CHIỀU QUAY ---
// Tùy vào cách nối dây của bạn, hãy đổi dấu (1 hoặc -1) để động cơ quay đúng 
// chiều "Ngược kim đồng hồ" như bạn muốn nhé.
const int CHIEU_QUAY_TIEP = -1;   // Quay tiếp 1 vòng (Dành cho chân 10)
const int CHIEU_QUAY_NGUOC = 1; // Quay ngược lại (Dành cho chân 12)

bool dangXacNhanGoc = false;
unsigned long thoiGianBatDauLow = 0;
unsigned long thoiGianBatDauCho = 0;

// Khai báo các trạng thái của hệ thống
enum TrangThaiHeThong { 
  TIM_GOC_KHOI_DONG, 
  IDLE, 
  AUTO_DANG_TIEN, 
  AUTO_DANG_CHO, 
  TIM_GOC_SAU_AUTO, 
  DANG_TIEN_VA_GIU, 
  TIM_GOC_LUI       
};

// Vừa bật máy lên là phải đi tìm gốc trước
TrangThaiHeThong trangThai = TIM_GOC_KHOI_DONG;

void setup() {
  pinMode(CAM_BIEN_12, INPUT_PULLUP);
  pinMode(PIN_TIEN, INPUT_PULLUP);
  pinMode(PIN_LUI, INPUT_PULLUP);
  pinMode(PIN_TU_DONG, INPUT_PULLUP);

  pinMode(ENA_PIN, OUTPUT);
  digitalWrite(ENA_PIN, LOW); 

  stepper.setMaxSpeed(2000.0); 
}

// Hàm chuyên dùng để quay liên tục TÌM GỐC bằng mắt cảm biến (Không đếm xung)
void thucHienTimGoc(int chieuQuay) {
  // Xuất xung cho động cơ quay liên tục ở tốc độ dò tìm
  stepper.setSpeed(chieuQuay * TOC_DO_TIM_GOC);
  stepper.runSpeed(); 

  // Nếu cảm biến bị đè (LOW)
  if (digitalRead(CAM_BIEN_12) == LOW) {
    if (!dangXacNhanGoc) {
      dangXacNhanGoc = true;
      thoiGianBatDauLow = millis();
    } else if (millis() - thoiGianBatDauLow >= THOI_GIAN_CHONG_NHIEU) {
      // Đã xác nhận đè chắc chắn qua 10ms -> TÌM THẤY GỐC
      stepper.stop(); 
      stepper.setCurrentPosition(0); // LƯU TỌA ĐỘ 0 TẠI ĐÂY
      
      stepper.setMaxSpeed(TOC_DO_LAM_VIEC); 
      stepper.setAcceleration(GIA_TOC_LAM_VIEC);
      
      dangXacNhanGoc = false;
      trangThai = IDLE; // Thoát khỏi trạng thái tìm gốc, chuyển về chờ lệnh
    }
  } else {
    dangXacNhanGoc = false;
  }
}

void loop() {
  long xungLamViec = (GOC_DO * STEPS_PER_REV) / 360.0; 

  switch (trangThai) {
    
    // --------------------------------------------------------
    // TRẠNG THÁI KHỞI ĐỘNG (Dò tìm mốc 0 lúc mới cấp điện)
    // --------------------------------------------------------
    case TIM_GOC_KHOI_DONG:
      thucHienTimGoc(CHIEU_QUAY_NGUOC); 
      break;

    // --------------------------------------------------------
    // TRẠNG THÁI CHỜ LỆNH TỪ AGV
    // --------------------------------------------------------
    case IDLE:
      if (digitalRead(PIN_TU_DONG) == LOW) { // Nút 26 - Chân 10 kích
        stepper.moveTo(xungLamViec);
        trangThai = AUTO_DANG_TIEN;
      }
      else if (digitalRead(PIN_TIEN) == LOW) { // Nút 24 - Chân 11 kích
        stepper.moveTo(xungLamViec);
        trangThai = DANG_TIEN_VA_GIU;
      }
      else if (digitalRead(PIN_LUI) == LOW) { // Nút 22 - Chân 12 kích
        trangThai = TIM_GOC_LUI;
      }
      break;

    // --------------------------------------------------------
    // LUỒNG XỬ LÝ CHÂN 10 (AUTO XẢ HÀNG)
    // --------------------------------------------------------
    case AUTO_DANG_TIEN:
      stepper.run();
      // Nếu đã chạy tới đúng góc 550 độ
      if (stepper.distanceToGo() == 0) {
        thoiGianBatDauCho = millis(); // Bắt đầu bấm giờ
        trangThai = AUTO_DANG_CHO;
      }
      break;

    case AUTO_DANG_CHO:
      // Chờ đủ 6 giây
      if (millis() - thoiGianBatDauCho >= 6000) { 
        trangThai = TIM_GOC_SAU_AUTO; 
      }
      break;

    case TIM_GOC_SAU_AUTO:
      // Quay TỚI (Tiếp 1 vòng) để đè lại điểm gốc
      thucHienTimGoc(CHIEU_QUAY_TIEP); 
      break;

    // --------------------------------------------------------
    // LUỒNG XỬ LÝ CHÂN 11 (TIẾN VÀ ĐỨNG IM)
    // --------------------------------------------------------
    case DANG_TIEN_VA_GIU:
      stepper.run();
      // Tới nơi thì về IDLE, do không có lệnh quay về nên nó sẽ đứng cứng ngắc ở đó
      if (stepper.distanceToGo() == 0) {
        trangThai = IDLE; 
      }
      break;

    // --------------------------------------------------------
    // LUỒNG XỬ LÝ CHÂN 12 (QUAY NGƯỢC LẠI TÌM GỐC VÀ ĐỨNG IM)
    // --------------------------------------------------------
    case TIM_GOC_LUI:
      // Quay NGƯỢC LẠI quét cảm biến gốc
      thucHienTimGoc(CHIEU_QUAY_NGUOC); 
      break;
  }
}