#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); 
const byte address[6] = "12345";
int tg_phat_loa = 15000;
const int ledPin = 2; 
unsigned long lastReceiveTime = 0; 
unsigned long lastBlinkTime = 0;   
unsigned long lastButtonPress = 0; 
bool ledState = LOW;

bool lastResetButtonState = HIGH; 

// --- Biến phục vụ đếm thời gian 10s cho riêng chân A1 ---
unsigned long a1StartTime = 0;
bool isA1TimerActive = false;

void setup() {
  // Đã dọn sạch Serial để giải phóng bộ nhớ, chạy đầm hơn
  radio.begin();
  radio.openReadingPipe(1, address); 
  radio.setPALevel(RF24_PA_MIN); 
  radio.startListening(); 

  pinMode(A0, INPUT_PULLUP); 
  pinMode(ledPin, OUTPUT);   

  for (int i = A1; i <= A5; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH); 
  }
}

// Hàm Reset toàn bộ trạng thái về HIGH
void resetOutputs() {
  for (int i = A1; i <= A5; i++) {
    digitalWrite(i, HIGH);
  }
  isA1TimerActive = false; // Reset thì phải hủy luôn timer của A1 nếu đang chạy
}

void loop() {
  // 1. Kiểm tra nút Reset vật lý tại chỗ (A0)
  bool currentResetState = digitalRead(A0);
  if (currentResetState == LOW && lastResetButtonState == HIGH) {
    if (millis() - lastButtonPress > 50) { 
      resetOutputs();
      lastButtonPress = millis();
    }
  }
  lastResetButtonState = currentResetState;

  // 2. Xử lý dữ liệu sóng nRF24
  if (radio.available()) {
    byte receivedData = 0;
    radio.read(&receivedData, sizeof(receivedData)); 
    lastReceiveTime = millis(); 

    if (receivedData == 10) { 
      resetOutputs();
    } 
    else if (receivedData >= 1 && receivedData <= 5) {
      switch (receivedData) {
        case 1: 
          digitalWrite(A1, LOW); 
          a1StartTime = millis();    // Bắt đầu bấm giờ
          isA1TimerActive = true;    // Kích hoạt cờ timer báo đang đếm
          break;
        case 2: digitalWrite(A2, LOW); break; // Các chân khác vẫn chốt cứng LOW
        case 3: digitalWrite(A3, LOW); break;
        case 4: digitalWrite(A4, LOW); break;
        case 5: digitalWrite(A5, LOW); break;
      }
    }
  }

  // 3. XỬ LÝ NHẢ CHÂN A1 SAU 10 GIÂY (10000ms)
  if (isA1TimerActive && (millis() - a1StartTime >= tg_phat_loa)) {
    digitalWrite(A1, HIGH);    // Đủ 10s thì kéo lên lại mức HIGH
    isA1TimerActive = false;   // Tắt cờ timer
  }

  // 4. Đèn báo kết nối chân số 2 (Sáng nhấp nháy khi có sóng liên tục)
  if (millis() - lastReceiveTime < 1000) {
    if (millis() - lastBlinkTime >= 500) {
      ledState = !ledState; 
      digitalWrite(ledPin, ledState);
      lastBlinkTime = millis();
    }
  } else {
    digitalWrite(ledPin, LOW); 
  }
}
/*
VER 2
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8); 
const byte address[6] = "12345";

const int ledPin = 2; 
unsigned long lastReceiveTime = 0; 
unsigned long lastBlinkTime = 0;   
unsigned long lastButtonPress = 0; 
bool ledState = LOW;

// Mảng lưu trạng thái cũ của nút Reset vật lý trên mạch thu
bool lastResetButtonState = HIGH; 

void setup() {
  Serial.begin(9600);
  radio.begin();
  radio.openReadingPipe(1, address); 
  radio.setPALevel(RF24_PA_MIN); 
  radio.startListening(); 

  pinMode(A0, INPUT_PULLUP); 
  pinMode(ledPin, OUTPUT);   

  // Khởi tạo các chân A1 đến A5 ở mức HIGH (Tắt)
  for (int i = A1; i <= A5; i++) {
    pinMode(i, OUTPUT);
    digitalWrite(i, HIGH); 
  }
  
  Serial.println("Mach THU (Che do CHOT LOW) da san sang!");
}

// Hàm Reset toàn bộ trạng thái về HIGH
void resetOutputs() {
  for (int i = A1; i <= A5; i++) {
    digitalWrite(i, HIGH);
  }
  Serial.println("DA RESET: Tat ca cac chan da nha (Muc HIGH)");
}

void loop() {
  // 1. Kiểm tra nút Reset vật lý tại chỗ (bắt sườn âm)
  bool currentResetState = digitalRead(A0);
  if (currentResetState == LOW && lastResetButtonState == HIGH) {
    if (millis() - lastButtonPress > 50) { 
      resetOutputs();
      lastButtonPress = millis();
    }
  }
  lastResetButtonState = currentResetState;

  // 2. Xử lý dữ liệu sóng nRF24
  if (radio.available()) {
    byte receivedData = 0;
    radio.read(&receivedData, sizeof(receivedData)); 
    lastReceiveTime = millis(); 

    if (receivedData == 10) { 
      // Lệnh 10 từ chân A0 bên phát
      Serial.println(">>> Nhan lenh RESET tu ben phat!");
      resetOutputs();
    } 
    else if (receivedData >= 1 && receivedData <= 5) {
      Serial.print("Nhan lenh CHOT LOW chan: A");
      Serial.println(receivedData);

      // CHỐT TRẠNG THÁI: Thay vì đảo (!digitalRead), giờ ta ép thẳng xuống LOW
      // Dù có nhiễu sóng gửi 100 lần thì nó vẫn chỉ là LOW, cực kỳ ổn định.
      switch (receivedData) {
        case 1: digitalWrite(A1, LOW); break;
        case 2: digitalWrite(A2, LOW); break;
        case 3: digitalWrite(A3, LOW); break;
        case 4: digitalWrite(A4, LOW); break; // Mình giữ lại 4, 5 phòng khi bạn dùng tới
        case 5: digitalWrite(A5, LOW); break;
      }
    }
  }

  // 3. Đèn báo kết nối (Sáng nhấp nháy khi có sóng liên tục)
  if (millis() - lastReceiveTime < 1000) {
    if (millis() - lastBlinkTime >= 500) {
      ledState = !ledState; 
      digitalWrite(ledPin, ledState);
      lastBlinkTime = millis();
    }
  } else {
    digitalWrite(ledPin, LOW); 
  }
}
*/