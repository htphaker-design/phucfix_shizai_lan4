#ifndef CONTROL_STEP_H
#define CONTROL_STEP_H

#include <Arduino.h> // Nên có dòng này để trình biên dịch hiểu các lệnh digitalWrite, micros...

#define PUL_TRAI  A0
#define DIR_TRAI  A1
#define ENA_TRAI  A2
#define PUL_PHAI  A3
#define DIR_PHAI  A4
#define ENA_PHAI  A5

// Thêm 'static' để tránh lỗi biên dịch khi include file này ở nhiều nơi
static bool trang_thai_xung_trai = false;
static bool trang_thai_xung_phai = false;
static unsigned long prevMicrosTrai = 0;
static unsigned long prevMicrosPhai = 0;

// Đổi kiểu dữ liệu tốc độ thành 'unsigned long' để đồng nhất với hàm micros()
static void step_dc(bool quay_trai_flag, bool quay_phai_flag, int CHIEU_QUAY_PHAI, int CHIEU_QUAY_TRAI, unsigned long toc_do_phai, unsigned long toc_do_trai)  
{
  digitalWrite(DIR_TRAI, CHIEU_QUAY_TRAI);  
  digitalWrite(DIR_PHAI, CHIEU_QUAY_PHAI);  
  unsigned long currentMicros = micros(); 

  //  ĐỘNG CƠ TRÁI
  if (quay_trai_flag) {
    if (currentMicros - prevMicrosTrai >= toc_do_trai) {
      prevMicrosTrai += toc_do_trai;                                   // cộng dồn: không mất phần trễ của vòng loop
      if (currentMicros - prevMicrosTrai >= toc_do_trai) prevMicrosTrai = currentMicros; // trễ quá 1 chu kỳ (vừa khởi động/dừng) thì bắt kịp, không bắn xung dồn
      trang_thai_xung_trai = !trang_thai_xung_trai;
      digitalWrite(PUL_TRAI, trang_thai_xung_trai);
    }
  } else {
    digitalWrite(PUL_TRAI, LOW);
  }

  // ĐỘNG CƠ PHẢI
  if (quay_phai_flag) {
    if (currentMicros - prevMicrosPhai >= toc_do_phai) {
      prevMicrosPhai += toc_do_phai;
      if (currentMicros - prevMicrosPhai >= toc_do_phai) prevMicrosPhai = currentMicros;
      trang_thai_xung_phai = !trang_thai_xung_phai;
      digitalWrite(PUL_PHAI, trang_thai_xung_phai);
    }
  } else {
    digitalWrite(PUL_PHAI, LOW);
  }
}

#endif