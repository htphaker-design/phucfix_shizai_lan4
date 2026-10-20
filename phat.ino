#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

RF24 radio(7, 8);
const byte address[6] = "12345";

const int buttonPins[] = {A0, A1, A2, A3, A4, A5};
const byte buttonCodes[] = {10, 1, 2, 3, 4, 5}; 
const int numButtons = 6;

bool buttonState[numButtons]       = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; 
bool lastFlickerState[numButtons]  = {HIGH, HIGH, HIGH, HIGH, HIGH, HIGH}; 
unsigned long lastDebounceTime[numButtons] = {0, 0, 0, 0, 0, 0};           

unsigned long lastPingTime = 0;      
unsigned long lastConnectedTime = 0; 

void setup() {
  pinMode(2, OUTPUT);
  digitalWrite(2, HIGH); // Trạng thái mặc định khi mất sóng là HIGH
  
  for (int i = 0; i < numButtons; i++) {
    pinMode(buttonPins[i], INPUT_PULLUP);
  }
  
  radio.begin();
  radio.openWritingPipe(address);
  radio.setPALevel(RF24_PA_MIN); // Lên MAX nhớ đấu thêm tụ ở nguồn nRF24
  radio.stopListening();         
}

void loop() {
  // 1. TỰ ĐỘNG GỬI PING KIỂM TRA MẠCH THU (300ms/lần)
  if (millis() - lastPingTime >= 300) {
    lastPingTime = millis();
    byte pingData = 0; 
    
    // Nếu mạch thu phản hồi (Auto-Ack) -> Cập nhật thời gian có sóng
    if (radio.write(&pingData, sizeof(pingData))) {
      lastConnectedTime = millis(); 
    }
  }

  // 2. BỘ LỌC CHỐNG NHIỄU VÀ GỬI LỆNH TỪ NÚT BẤM
  for (int i = 0; i < numButtons; i++) {
    bool reading = digitalRead(buttonPins[i]);

    if (reading != lastFlickerState[i]) {
      lastDebounceTime[i] = millis(); 
    }

    if ((millis() - lastDebounceTime[i]) > 100) {
      if (reading != buttonState[i]) {
        buttonState[i] = reading; 

        // Chỉ bắn tín hiệu đi khi nút bị dập xuống (LOW)
        if (buttonState[i] == LOW) {
          byte dataToSend = buttonCodes[i]; 
          // Bắn lệnh đi, nếu mạch thu nhận được cũng tính là đang có kết nối
          if(radio.write(&dataToSend, sizeof(dataToSend))) {
            lastConnectedTime = millis(); 
          }
        }
      }
    }
    lastFlickerState[i] = reading;
  }

  // 3. XUẤT TÍN HIỆU CHÂN SỐ 2 (Báo trạng thái kết nối)
  if (millis() - lastConnectedTime < 1000) { 
    // Có kết nối liên tục -> Xuất mức LOW
    digitalWrite(2, LOW);  
  } else {
    // Quá 1 giây không thấy mạch thu -> Xuất mức HIGH
    digitalWrite(2, HIGH); 
  }
}