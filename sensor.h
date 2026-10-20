#include <Arduino.h>
#ifndef SENSOR_H
#define SENSOR_H
const int TG = 31;      const int PG = 30;
const int T1 = 32;      const int P1 = 29;
const int T2 = 33;      const int P2 = 28;
const int T3 = 34;      const int P3 = 27;
const int T4 = 35;      const int P4 = 26;
const int cb1 = 25; 
int g_ma_truoc = 0;          // mã lệch hợp lệ gần nhất (dùng chọn đoạn line khi có nhiễu)
unsigned int g_mau_la = 0;   // mẫu 10 bit gần nhất bị trả 99 (ss1 là bit cao nhất) - dùng để debug
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
  //  CASE 14: VẠCH TRẠM/DỪNG
  else if((ss1 == 1 && ss2 == 1 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
        //(ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)||
          (ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0)){return 14;}

else if((ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)||
        (ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1)){return 15;}

  //else if(ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 0     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 0){return 16;}
  else if(ss1 == 1 && ss2 == 1 && ss3 == 1 && ss4 == 1     && ss5 == 1 && ss6 == 1     && ss7 == 1 && ss8 == 1 && ss9 == 1 && ss10 == 1){return 13;}
  else if(ss1 == 0 && ss2 == 0 && ss3 == 0 && ss4 == 0     && ss5 == 0 && ss6 == 0     && ss7 == 0 && ss8 == 0 && ss9 == 0 && ss10 == 0){return 12;}


  // ===== MẪU KHÔNG CÓ TRONG BẢNG Ở TRÊN (trước đây: không return => giá trị trả về không xác định) =====
  // Bảng chỉ có 32 mẫu; line bản rộng (3 cảm biến liền nhau) hoặc 1 cảm biến nhiễu bên cạnh line sẽ rơi vào đây.
  // Cách xử lý: tìm các đoạn cảm biến "thấy line" liên tiếp, chọn đoạn dài nhất (hòa thì chọn đoạn gần mã trước nhất),
  // lấy tâm đoạn làm mã lệch (cùng thang -9..9 với bảng). Chỉ chấp nhận khi tổng <= 4 cảm biến và đoạn <= 3 cảm biến.
  int v[10] = {ss1, ss2, ss3, ss4, ss5, ss6, ss7, ss8, ss9, ss10};
  int tong0 = 0, tot_len = 0, tot_dau = -1, tot_cuoi = -1;
  int i = 0;
  while (i < 10) {
    if (v[i] == 0) {
      int j = i;
      while (j + 1 < 10 && v[j + 1] == 0) j++;
      int len = j - i + 1;
      tong0 += len;
      int tam = -9 + i + j;
      bool tot_hon = (len > tot_len);
      if (!tot_hon && len == tot_len) {
        int d_moi = tam - g_ma_truoc;                           if (d_moi < 0) d_moi = -d_moi;
        int d_cu  = (-9 + tot_dau + tot_cuoi) - g_ma_truoc;     if (d_cu < 0)  d_cu  = -d_cu;
        tot_hon = (d_moi < d_cu);
      }
      if (tot_hon) { tot_len = len; tot_dau = i; tot_cuoi = j; }
      i = j + 1;
    } else {
      i++;
    }
  }
  if (tong0 <= 4 && tot_len >= 1 && tot_len <= 3) return -9 + tot_dau + tot_cuoi;

  // Còn lại (quá nhiều cảm biến cùng sáng/rời rạc, đoạn quá dài không phải vạch trạm) -> 99 = "không phải line hợp lệ"
  g_mau_la = 0;
  for (int k = 0; k < 10; k++) g_mau_la = (g_mau_la << 1) | v[k];
  return 99;
}

int In_SenSor()
{
  int ma = DocMauCamBien();
  if (ma >= -9 && ma <= 9) g_ma_truoc = ma;
  return ma;
}
#endif