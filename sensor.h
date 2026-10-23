#include <Arduino.h>
#ifndef SENSOR_H
#define SENSOR_H

const int TG = 31;      const int PG = 30;
const int T1 = 32;      const int P1 = 29;
const int T2 = 33;      const int P2 = 28;
const int T3 = 34;      const int P3 = 27;
const int T4 = 35;      const int P4 = 26;
const int cb1 = 25; 

int g_ma_truoc = 0;          // Mã lệch hợp lệ gần nhất
unsigned int g_mau_la = 0;   // Mẫu 10 bit gần nhất bị trả 99 (dùng debug)

int DocMauCamBien()
{
  int ss1  = digitalRead(T4);
  int ss2  = digitalRead(T3);
  int ss3  = digitalRead(T2);
  int ss4  = digitalRead(T1);
  int ss5  = digitalRead(TG);
  int ss6  = digitalRead(PG);
  int ss7  = digitalRead(P1);
  int ss8  = digitalRead(P2);
  int ss9  = digitalRead(P3);
  int ss10 = digitalRead(P4);

  // =========================================================================
  // BƯỚC 1: KIỂM TRA CÁC MẪU CHUẨN & CASE ĐẶC BIỆT (14, 15, 12, 13) TRƯỚC TIÊN
  // (Đảm bảo 100% không bao giờ bỏ sót Case 14, Case 15)
  // =========================================================================

  // CASE 0: GIỮA LINE
  if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)) {return 0;}

  // LỆCH PHẢI (Giá trị dương)
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 1;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 0 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 2;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 0 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 3;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 0 && ss8 == 0 && ss9 == 1 && ss10 == 1)){return 4;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 0 && ss9 == 1 && ss10 == 1)){return 5;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 0 && ss9 == 0 && ss10 == 1)){return 6;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 0 && ss10 == 1)){return 7;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 0 && ss10 == 0)){return 8;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 0)){return 9;}

  // LỆCH TRÁI (Giá trị âm)
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -1;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 0     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -2;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 0     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -3;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 0 && ss4 == 0     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -4;}
  else if((ss1 == 1 && ss2 == 1 && ss3 == 0 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -5;}
  else if((ss1 == 1 && ss2 == 0 && ss3 == 0 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -6;}
  else if((ss1 == 1 && ss2 == 0 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -7;}
  else if((ss1 == 0 && ss2 == 0 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -8;}
  else if((ss1 == 0 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return -9;}

  // CASE 14: VẠCH TRẠM PHẢI (5 đến 9 mắt liên tiếp từ phải sang)
  else if((ss1 == 1 && ss2 == 1 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)){return 14;}

  // CASE 15: VẠCH TRẠM TRÁI (5 đến 9 mắt liên tiếp từ trái sang)
  else if((ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 1)||
          (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 1 && ss10 == 1)||
          (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
          (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
          (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 15;}

  else if(ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1){return 13;}
  else if(ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0){return 12;}

  // =========================================================================
  // BƯỚC 2: XỬ LÝ NGÃ RẼ (ƯU TIÊN ĐƯỜNG THẲNG, BỎ QUA ĐƯỜNG THỨ 2) & NGÃ CUA
  // =========================================================================
  int v[10] = {ss1, ss2, ss3, ss4, ss5, ss6, ss7, ss8, ss9, ss10};
  int so_doan = 0;
  int tot_dau = -1, tot_cuoi = -1, tot_len = 0, tot_kc = 100;
  int i = 0;

  while (i < 10) {
    if (v[i] == 0) {
      int j = i;
      while (j + 1 < 10 && v[j + 1] == 0) j++;
      int len = j - i + 1;
      int tam = -9 + i + j;
      int kc  = abs(tam - g_ma_truoc);
      so_doan++;

      // Chọn đoạn nằm gần vị trí line đang chạy (g_ma_truoc) nhất
      if (kc < tot_kc) {
        tot_kc = kc; tot_len = len; tot_dau = i; tot_cuoi = j;
      }
      i = j + 1;
    } else {
      i++;
    }
  }

  // TRƯỜNG HỢP A: ĐANG CHẠY ĐƯỜNG THẲNG (|g_ma_truoc| <= 2)
  if (abs(g_ma_truoc) <= 2) {
    // 1. Nếu mắt giữa (ss5 hoặc ss6) VẪN ĐANG THẤY LINE THẲNG:
    //    Bất kể có nhánh thứ 2 tách ra (so_doan >= 2) hay gốc nhánh rẽ làm phình vạch 3-4 mắt,
    //    ta coi phần phát sinh thêm là NHIỄU và chỉ giữ nguyên line thẳng ở giữa!
    if (ss5 == 0 && ss6 == 0) return 0;
    if (ss5 == 0)             return (g_ma_truoc == 0) ? 0 : -1;
    if (ss6 == 0)             return (g_ma_truoc == 0) ? 0 :  1;

    // 2. Nếu có từ 2 đường trở lên (so_doan >= 2, phát sinh nhánh rẽ):
    //    Chỉ nhận đoạn chính nếu nó rộng 1-2 mắt và sát vị trí cũ (tot_kc <= 2), bỏ qua hoàn toàn nhánh thứ 2.
    if (so_doan >= 2) {
      if (tot_len <= 2 && tot_kc <= 2) return -9 + tot_dau + tot_cuoi;
      return 99; // Coi nhánh phát sinh là nhiễu -> trả 99 để khóa thẳng lái vượt qua
    }

    // 3. Nếu chỉ có 1 đoạn (so_doan == 1) rộng <= 3 mắt và sát vị trí cũ (tot_kc <= 2):
    if (so_doan == 1 && tot_len <= 3 && tot_kc <= 2) {
      return -9 + tot_dau + tot_cuoi;
    }

    return 99;
  }

  // TRƯỜNG HỢP B: ĐANG ÔM NGÃ CUA (|g_ma_truoc| >= 3)
  // Chỉ khi có đúng 1 đường băng keo duy nhất (so_doan == 1, rộng 1..4 mắt) và liền kề mã cua trước đó (tot_kc <= 4)
  if (so_doan == 1 && tot_len >= 1 && tot_len <= 4 && tot_kc <= 4) {
    return -9 + tot_dau + tot_cuoi;
  }
  // Nếu trong cua mà phát sinh nhánh thứ 2 (so_doan >= 2), chỉ bám đoạn gần nhất nếu rộng 1-2 mắt và sát mã trước (tot_kc <= 2)
  if (so_doan >= 2 && tot_len <= 2 && tot_kc <= 2) {
    return -9 + tot_dau + tot_cuoi;
  }

  g_mau_la = 0;
  for (int k = 0; k < 10; k++) g_mau_la = (g_mau_la << 1) | v[k];
  return 99;
}

int In_SenSor()
{
  int ma = DocMauCamBien();
  if (ma >= -9 && ma <= 9) { g_ma_truoc = ma; }
  return ma;
}
#endif