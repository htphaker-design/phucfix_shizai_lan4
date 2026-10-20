// AGV SHIZAI - BAN HOAN CHINH KICH PIN 10, 11, 12 (UPDATE LOGIC LÙI)
#include "control_step.h"
#include "sensor.h"

// Đặt 1 để in ra Serial (115200) mẫu 10 cảm biến mỗi khi xe đọc ra "không phải line" (mã 99). Dùng khi xe dừng bất ngờ.
#ifndef DEBUG_MAU_LA
#define DEBUG_MAU_LA 0
#endif

// --- KHAI BÁO CÁC CHÂN TÍN HIỆU ---
const int pin_bao_mat_line = 45;
const int pin_dung = 46;
const int pin_tin_hieu_RF = 44;
const int pin_tin_hieu_10 = 10;
const int pin_tin_hieu_11 = 11;
const int pin_tin_hieu_12 = 12;

const unsigned long time_bao_mat_line_lau = 60000; // Mất line liên tục 60s -> kích chân 45 (LOW) để tắt AGV
const unsigned long thoi_gian_bo_qua_mat_line = 500;
const unsigned long thoi_gian_xac_nhan_khi_chay = 50;  // Đang chạy: phải thấy line hợp lệ liên tục >= 50ms mới coi là 'đã thấy line' (lọc nhiễu)
const unsigned long thoi_gian_xac_nhan_co_line = 300;  // Đã dừng vì mất line: thấy line liên tục >= 300ms mới chạy tiếp
const unsigned long time_hu_line = 50;
const int time_tang_toc_case14_lan_1 = 8000;
const unsigned long time_dung_lay_hang = 20000;
const unsigned long time_chay_thoat_case_14 = 2000; // Thời gian đề-ba chạy thoát khỏi vạch Case 14 sau khi đếm xong 20s
// --- XẢ HÀNG (sau 8s kể từ vạch 14): dừng + kích chân 10, rồi đứng thêm trước khi chạy ---
const unsigned long thoi_gian_kich_pin_10 = 1000;     // Chân 10 giữ LOW bao lâu kể từ lúc xe dừng
const unsigned long thoi_gian_dung_them_sau_tha = 2000; // Sau khi nhả chân 10 (+200ms) xe đứng thêm bấy lâu mới đề-pa
const int speed = 1200;
const int slowSpeed = 12000;
const int startSpeed = 12000;

const int tg_gui_tinh_hieu_rf = 3000;

const unsigned long thoi_gian_tang_toc = 9000;
const unsigned long thoi_gian_giam_toc = 6000;
// --- BẢNG PID THEO TỪNG MÃ LỆCH |error| = 0..9 (thay cho 3 vùng 123/456/789) ---
// Kp lớn -> sửa mạnh hơn. BU (µs) = làm chậm cả xe khi lệch, tạo chỗ cho bánh trong tăng tốc.
// Mỗi cột tăng dần, không nhảy vọt giữa các mã -> tránh "chạy thẳng rồi giật".
//                             e:   0    1    2    3  |   4    5    6  |   7    8    9
const float KP_TBL[10] = {         50,  80,  90, 100,  110, 120, 130,  320, 410, 500 };
const int   BU_TBL[10] = {          0,  10,  10,  20,  100, 170, 240,  320, 410, 500 };
const float Kd_PID = 500.0;
const float Ki = 0.01;
const float TIME_VAO_CUA = 200.0;
const float TIME_RA_CUA  = 300.0;

// --- CHẾ ĐỘ LÙI CỦA SHIZAI --- 
const int toc_do_lui_15 = 6000;
const int BU_TRAI_LUI = 0; // Bù tốc độ bánh trái khi lùi (µs cộng thêm vào chu kỳ xung) - chỉnh khi test
const unsigned long thoi_gian_xac_nhan_15 = 0; // Mã 15 phải liên tục >= N ms mới lùi (0 = lùi ngay khi thấy)
const unsigned long time_lui_ve = 10000; // Thời gian lùi 10s
const unsigned long time_chay_mu_sau_lui = 23000; // 23s bỏ qua trạm
unsigned long t_bat_dau_lui_15 = 0;
unsigned long t_bat_dau_thay_15 = 0;
bool dang_thay_15 = false;
unsigned long t_bat_dau_dung_sau_lui = 0;
unsigned long t_ket_thuc_lui = 0;
bool dang_chay_mu_sau_lui = false;

// Biến quản lý tự tắt chân 11 và 12 sau 1s
unsigned long t_bat_dau_pin_11 = 0;
bool dang_bat_pin_11 = false;
unsigned long t_bat_dau_pin_12 = 0;
bool dang_bat_pin_12 = false;

enum TrangThaiTram { 
    CHAY_BINH_THUONG=0, BAT_DAU_GIAM_TOC=1, CHO_DUNG=2, DANG_DUNG_LAY_HANG=3, 
    ROI_KHOI_TRAM=4, QUAY_DAU=5, TAM_DUNG_DE_THA=6, 
    LUI_CASE_15=7, TAM_DUNG_SAU_LUI=8 
};
TrangThaiTram trang_thai_14 = CHAY_BINH_THUONG;

unsigned long thoi_gian_cap_nguon = 0, thoi_gian_bat_dau = 0, thoi_gian_bat_dau_giam = 0, t_dung = 0;
unsigned long t_chong_doi_tram = 0, last_ramp_time = 0;
unsigned long thoi_gian_mat_line = 0, t_bat_dau_tam_dung = 0, thoi_gian_roi_tram = 0;
unsigned long t_bat_dau_thay_line = 0;
bool dang_thay_line = false;
bool da_kich_pin_45 = false;
bool dang_mat_line = false, dang_dung_vi_vat_can = false, dang_dung_vi_mat_line = false;
bool dang_tang_toc = true, dang_giam_toc = false, is_special_code = false;
int raw_sensor = 0, steering_error = 0, last_valid_sensor = 0;
float toc_do_khi_bat_dau_giam = 0, toc_do_khi_bat_dau_tang = 0, currentSpeedVal = startSpeed, smoothBaseSpeed = 0;
float smoothedError = 0, lastError = 0, integral = 0, lastPidOutput = 0;
float current_Kp = 50.0, current_Kd = 500.0, current_offset = 0.0; // global, được reset trong KhoiTaoThongSoDeBa()

void setup() {
#if DEBUG_MAU_LA
  //  Serial.begin(115200);
#endif
    pinMode(pin_dung, OUTPUT); digitalWrite(pin_dung, HIGH);
    pinMode(pin_bao_mat_line, OUTPUT); digitalWrite(pin_bao_mat_line, HIGH);
    pinMode(pin_tin_hieu_RF, OUTPUT); digitalWrite(pin_tin_hieu_RF, HIGH);
    
    // Cài đặt các chân tín hiệu mới
    pinMode(pin_tin_hieu_10, OUTPUT); digitalWrite(pin_tin_hieu_10, HIGH);
    pinMode(pin_tin_hieu_11, OUTPUT); digitalWrite(pin_tin_hieu_11, HIGH);
    pinMode(pin_tin_hieu_12, OUTPUT); digitalWrite(pin_tin_hieu_12, HIGH);

    pinMode(PUL_TRAI, OUTPUT); pinMode(DIR_TRAI, OUTPUT); pinMode(ENA_TRAI, OUTPUT);
    pinMode(PUL_PHAI, OUTPUT); pinMode(DIR_PHAI, OUTPUT); pinMode(ENA_PHAI, OUTPUT);
    pinMode(T4, INPUT); pinMode(T3, INPUT); pinMode(T2, INPUT); pinMode(T1, INPUT);
    pinMode(TG, INPUT); pinMode(PG, INPUT);
    pinMode(P1, INPUT); pinMode(P2, INPUT); pinMode(P3, INPUT); pinMode(P4, INPUT);
    pinMode(cb1, INPUT_PULLUP);
    digitalWrite(ENA_TRAI, LOW); digitalWrite(ENA_PHAI, LOW);
    
    thoi_gian_cap_nguon = millis();
    KhoiTaoThongSoDeBa();
    if (In_SenSor() == 14 || In_SenSor() == 15) trang_thai_14 = ROI_KHOI_TRAM;
}

void loop() {
    raw_sensor = In_SenSor();
    unsigned long now = millis();
#if DEBUG_MAU_LA
    static unsigned long t_in_debug = 0;
    if (raw_sensor == 99 && now - t_in_debug >= 200) {
        t_in_debug = now;
        //Serial.print("MAU LA (ss1..ss10, 0=thay line): ");
       // for (int i = 9; i >= 0; i--) Serial.print((g_mau_la >> i) & 1);
       // Serial.println();
    }
#endif

    // Tự động tắt chân 11 (trả về HIGH) sau 1s
    if (dang_bat_pin_11 && (now - t_bat_dau_pin_11 >= 1000)) {
        digitalWrite(pin_tin_hieu_11, HIGH);
        dang_bat_pin_11 = false;
    }

    // Tự động tắt chân 12 (trả về HIGH) sau 1s
    if (dang_bat_pin_12 && (now - t_bat_dau_pin_12 >= 1000)) {
        digitalWrite(pin_tin_hieu_12, HIGH);
        dang_bat_pin_12 = false;
    }

    // Tự động tắt khiên mù sau 23s
    if (dang_chay_mu_sau_lui && (now - t_ket_thuc_lui >= time_chay_mu_sau_lui)) {
        dang_chay_mu_sau_lui = false;
    }

    bool dang_lui = (trang_thai_14 == LUI_CASE_15 || trang_thai_14 == TAM_DUNG_SAU_LUI);
    bool dang_dung_tram = (trang_thai_14 == DANG_DUNG_LAY_HANG || trang_thai_14 == TAM_DUNG_DE_THA);
    // Ân hạn khi vừa lùi xong (2s) HOẶC khi đang đề-ba chạy thoát khỏi vạch Case 14 (3s) để không bị bắt nhầm mất line trên vạch
    bool an_han_tim_line = (dang_chay_mu_sau_lui && (now - t_ket_thuc_lui < 2000)) ||
                           (trang_thai_14 == ROI_KHOI_TRAM && (now - thoi_gian_roi_tram < 3000));
    
    // Luôn kiểm tra vật cản; chỉ kiểm tra mất line khi không lùi, không dừng trạm và không trong thời gian ân hạn thoát trạm
    if (!dang_lui) {
        if (KiemTraAnToanGap()) return;
        if (!an_han_tim_line && !dang_dung_tram) {
            if (XuLyMatLine()) return;
        }
    }
    
    DocVaLocCamBien();
    XuLyMayTrangThai();
    QuyetDinhXuatXung();
}

void KhoiTaoThongSoDeBa() {
    currentSpeedVal = smoothBaseSpeed = toc_do_khi_bat_dau_tang = (float)startSpeed;
    thoi_gian_bat_dau = last_ramp_time = thoi_gian_roi_tram = millis();
    dang_tang_toc = true; dang_giam_toc = false;
    integral = lastError = smoothedError = lastPidOutput = 0;
    current_Kp = KP_TBL[0]; current_Kd = Kd_PID; current_offset = 0.0;
    dang_thay_15 = false;
    dang_mat_line = false; dang_thay_line = false; dang_dung_vi_mat_line = false; // xóa sạch đồng hồ và cờ dừng mất line cũ
}

bool KiemTraAnToanGap() {
    if (digitalRead(cb1) == LOW) {
        step_dc(false, false, HIGH, LOW, 0, 0);
        dang_dung_vi_vat_can = true;
        return true;
    } else if (dang_dung_vi_vat_can) {
        KhoiTaoThongSoDeBa();
        trang_thai_14 = CHAY_BINH_THUONG;
        dang_dung_vi_vat_can = false;
    }
    return false;
}

// "Đang thấy line" = mã lệch -9..9 hoặc vạch trạm 14/15.
// 13 (toàn 1), 12 (toàn 0), 16, 99 (mẫu lạ)... KHÔNG phải line hợp lệ.
bool la_line_hop_le(int ma) {
    return (ma >= -9 && ma <= 9) || ma == 14 || ma == 15;
}

bool XuLyMatLine() {
    unsigned long now = millis();

    // Đã kích chân 45 (mất line 60s): giữ xe đứng, không tự chạy lại (muốn chạy phải reset/cấp nguồn lại)
    if (da_kich_pin_45) {
        step_dc(false, false, HIGH, LOW, 0, 0);
        return true;
    }

    if (!la_line_hop_le(raw_sensor)) {
        // ---------- KHÔNG THẤY LINE ----------
        dang_thay_line = false;
        if (!dang_mat_line) { dang_mat_line = true; thoi_gian_mat_line = now; }
        unsigned long t_mat = now - thoi_gian_mat_line;

        if (t_mat >= time_bao_mat_line_lau) {          // 60s -> tắt AGV
            digitalWrite(pin_bao_mat_line, LOW);
            da_kich_pin_45 = true;
        }
        if (t_mat >= thoi_gian_bo_qua_mat_line) {      // quá 0,5s -> DỪNG NGAY
            step_dc(false, false, HIGH, LOW, 0, 0);
            dang_dung_vi_mat_line = true;
            return true;
        }
        return false; // < 0,5s: vẫn vượt (băng keo rách), PID dùng sai số cuối
    }

    // ---------- THẤY LINE HỢP LỆ ----------
    if (!dang_thay_line) { dang_thay_line = true; t_bat_dau_thay_line = now; }
    unsigned long can_xac_nhan = dang_dung_vi_mat_line ? thoi_gian_xac_nhan_co_line : thoi_gian_xac_nhan_khi_chay;

    if (now - t_bat_dau_thay_line >= can_xac_nhan) {
        // Đã thấy line đủ lâu -> xóa trạng thái mất line
        dang_mat_line = false;
        if (dang_dung_vi_mat_line) { KhoiTaoThongSoDeBa(); dang_dung_vi_mat_line = false; }
        return false;
    }
    // Chưa đủ lâu: đang dừng thì giữ đứng; đang chạy thì chạy tiếp, đồng hồ mất line KHÔNG bị xóa
    if (dang_dung_vi_mat_line) {
        step_dc(false, false, HIGH, LOW, 0, 0);
        return true;
    }
    return false;
}

void DocVaLocCamBien() {
    if ((raw_sensor == 14 || raw_sensor == 15) && trang_thai_14 == CHAY_BINH_THUONG) {
        if (abs(last_valid_sensor) >= 6 || abs(smoothedError) > 5.5) raw_sensor = (last_valid_sensor > 0) ? 9 : -9;
    }

    bool la_ma_lech = (raw_sensor >= -9 && raw_sensor <= 9);   // mã dò line bình thường
    is_special_code = !la_ma_lech;                              // 13, 14, 15, 12, 16, 99...
    if (la_ma_lech) {
        steering_error = last_valid_sensor = raw_sensor;
    } else if (raw_sensor == 14 || raw_sensor == 15 || trang_thai_14 == ROI_KHOI_TRAM) {
        steering_error = 0;                  // vạch trạm hoặc đang đề-ba thoát trạm: giữ thẳng lái để vượt qua vạch
    } else {
        steering_error = last_valid_sensor;  // mất line / mã lạ: giữ sai số cuối (KHÔNG nạp 12, 16, 99 vào PID)
    }
}

void XuLyMayTrangThai() {
    unsigned long now = millis();
    
    // =======================================================
    // KỊCH BẢN 15: KÍCH CHÂN 11 VÀ LÙI 10s -> KÍCH CHÂN 12 VÀ DỪNG 1s -> ĐỀ BA 23s
    // =======================================================
    bool du_dieu_kien_lui = (!dang_chay_mu_sau_lui && (trang_thai_14 == BAT_DAU_GIAM_TOC || trang_thai_14 == CHO_DUNG));
    if (raw_sensor == 15 && du_dieu_kien_lui) {
        if (!dang_thay_15) { dang_thay_15 = true; t_bat_dau_thay_15 = now; }
    } else {
        dang_thay_15 = false; // mất mã 15 hoặc không đủ điều kiện -> hủy đếm
    }
    if (raw_sensor == 15 && du_dieu_kien_lui && dang_thay_15 && (now - t_bat_dau_thay_15 >= thoi_gian_xac_nhan_15)) {
        dang_thay_15 = false;
        trang_thai_14 = LUI_CASE_15;
        t_bat_dau_lui_15 = now;
        
        // Kích chân 11 ngay lúc vừa chạm 15 và bắt đầu lùi
        digitalWrite(pin_tin_hieu_11, LOW); 
        dang_bat_pin_11 = true;
        t_bat_dau_pin_11 = now;
        
        return; // Thoát ra ngay để chạy lùi
    }

    if (trang_thai_14 == LUI_CASE_15) {
        if (now - t_bat_dau_lui_15 >= time_lui_ve) { // Hết thời gian lùi (10s)
            trang_thai_14 = TAM_DUNG_SAU_LUI;
            t_bat_dau_dung_sau_lui = now;
            
            // Lùi xong -> kích chân 12 và bắt đầu nghỉ 1s
            digitalWrite(pin_tin_hieu_12, LOW); 
            dang_bat_pin_12 = true;
            t_bat_dau_pin_12 = now;
        }
        return;
    }

    if (trang_thai_14 == TAM_DUNG_SAU_LUI) {
        if (now - t_bat_dau_dung_sau_lui >= 1000) { // Hết 1s dừng
            trang_thai_14 = CHAY_BINH_THUONG;
            KhoiTaoThongSoDeBa(); // Đề ba vọt đi
            dang_chay_mu_sau_lui = true; // Bật khiên mù 23s
            t_ket_thuc_lui = now;
        }
        return;
    }

    // =======================================================
    // LOGIC CHẠY BÌNH THƯỜNG VÀ XẢ HÀNG CASE 14 (KÍCH CHÂN 10)
    // =======================================================
    if (now - thoi_gian_cap_nguon < thoi_gian_tang_toc && (raw_sensor == 14 || raw_sensor == 15)) return;
    
    if (raw_sensor == 14 && (now - thoi_gian_roi_tram >= 4000) && !dang_chay_mu_sau_lui) {
        if (trang_thai_14 == CHAY_BINH_THUONG) {
            trang_thai_14 = BAT_DAU_GIAM_TOC; t_chong_doi_tram = now;
            dang_tang_toc = false; dang_giam_toc = true;
            thoi_gian_bat_dau_giam = now; toc_do_khi_bat_dau_giam = smoothBaseSpeed;
        } else if (trang_thai_14 == CHO_DUNG && (now - t_chong_doi_tram > 500)) {
            trang_thai_14 = DANG_DUNG_LAY_HANG; t_dung = now;
            digitalWrite(pin_tin_hieu_RF, LOW);
        }
    }
    
    if (trang_thai_14 == BAT_DAU_GIAM_TOC && !is_special_code && (now - t_chong_doi_tram > 150)) {
        trang_thai_14 = CHO_DUNG;
    }
    
    if ((trang_thai_14 == BAT_DAU_GIAM_TOC || trang_thai_14 == CHO_DUNG) && (now - thoi_gian_bat_dau_giam > time_tang_toc_case14_lan_1)) {
        trang_thai_14 = TAM_DUNG_DE_THA; 
        t_bat_dau_tam_dung = now;
        digitalWrite(pin_tin_hieu_10, LOW); // Kích chân 10 khi xe dừng
    }
    
    if (trang_thai_14 == TAM_DUNG_DE_THA) {
        if (now - t_bat_dau_tam_dung >= thoi_gian_kich_pin_10) { 
            digitalWrite(pin_tin_hieu_10, HIGH); // Hết thời gian kích thì tắt chân 10
        }
        if (now - t_bat_dau_tam_dung >= thoi_gian_kich_pin_10 + 200 + thoi_gian_dung_them_sau_tha) {  // Chờ thêm 200ms để nhả xong tín hiệu rồi đề ba
            trang_thai_14 = ROI_KHOI_TRAM;
            KhoiTaoThongSoDeBa();
        }
    }
    
    if (trang_thai_14 == DANG_DUNG_LAY_HANG) {
        if (now - t_dung >= tg_gui_tinh_hieu_rf) digitalWrite(pin_tin_hieu_RF, HIGH);
        if (now - t_dung >= time_dung_lay_hang) {
            trang_thai_14 = ROI_KHOI_TRAM; KhoiTaoThongSoDeBa();
        }
    }
    
    // Đề-ba chạy tối thiểu 2s (time_chay_thoat_case_14) VÀ phải thoát hẳn khỏi mã đặc biệt (Case 14) mới về lại CHAY_BINH_THUONG
    if (trang_thai_14 == ROI_KHOI_TRAM && (now - thoi_gian_roi_tram >= time_chay_thoat_case_14) && !is_special_code) {
        trang_thai_14 = CHAY_BINH_THUONG;
        thoi_gian_roi_tram = now;
    }
}

void QuyetDinhXuatXung() {
    // NGẮT ĐỘNG CƠ CHO CÁC TRẠNG THÁI DỪNG
    if (trang_thai_14 == DANG_DUNG_LAY_HANG || trang_thai_14 == TAM_DUNG_DE_THA || trang_thai_14 == TAM_DUNG_SAU_LUI) {
        step_dc(false, false, HIGH, LOW, 0, 0); last_ramp_time = millis(); return;
    }
    
    // XUẤT XUNG LÙI CỦA CASE 15
    if (trang_thai_14 == LUI_CASE_15) {
        step_dc(true, true, LOW, HIGH, toc_do_lui_15, toc_do_lui_15 + BU_TRAI_LUI); last_ramp_time = millis(); return;
    }
    
    TinhToanBiendangTocDo();
    TinhToanVaXuatPID();
}

void TinhToanBiendangTocDo() {
    if (dang_tang_toc) {
        unsigned long t_chay = millis() - thoi_gian_bat_dau;
        if (t_chay < thoi_gian_tang_toc) {
            currentSpeedVal = toc_do_khi_bat_dau_tang - ((toc_do_khi_bat_dau_tang - speed) * ((float)t_chay / thoi_gian_tang_toc));
        } else {
            currentSpeedVal = (float)speed; dang_tang_toc = false;
        }
    } else if (dang_giam_toc) {
        unsigned long t_giam = millis() - thoi_gian_bat_dau_giam;
        if (t_giam < thoi_gian_giam_toc) {
            currentSpeedVal = toc_do_khi_bat_dau_giam + ((slowSpeed - toc_do_khi_bat_dau_giam) * ((float)t_giam / thoi_gian_giam_toc));
        } else {
            currentSpeedVal = (float)slowSpeed; dang_giam_toc = false;
        }
    }
    smoothBaseSpeed = (currentSpeedVal * 0.1) + (smoothBaseSpeed * 0.9);
}

void TinhToanVaXuatPID() {
    int baseSpeed = (int)smoothBaseSpeed;
    smoothedError = ((float)steering_error * 0.35) + (smoothedError * 0.65);
    int absError = abs(steering_error);
    int idx = (absError > 9) ? 9 : absError;   // mã lạ (12, 16...) kẹp về 9, tránh tràn bảng
    float target_Kp = KP_TBL[idx];
    int target_offset_raw = BU_TBL[idx];
    
    float speed_ratio = constrain((float)(startSpeed - baseSpeed) / (startSpeed - speed), 0.0, 1.0);
    int target_offset = (int)(target_offset_raw * speed_ratio);
    int UPDATE_RATE = 5;
    
    if (millis() - last_ramp_time >= UPDATE_RATE) {
        last_ramp_time = millis();
        float time_vao = max(TIME_VAO_CUA, 1.0f), time_ra = max(TIME_RA_CUA, 1.0f);
        float step_offset_vao = ((float)BU_TBL[9] / time_vao) * UPDATE_RATE;
        float step_offset_ra  = ((float)BU_TBL[9] / time_ra) * UPDATE_RATE;
        float step_Kp_vao = ((KP_TBL[9] - KP_TBL[0]) / time_vao) * UPDATE_RATE, step_Kp_ra = ((KP_TBL[9] - KP_TBL[0]) / time_ra) * UPDATE_RATE;
        
        if (current_offset < target_offset) current_offset = min(current_offset + step_offset_vao, (float)target_offset);
        else if (current_offset > target_offset) current_offset = max(current_offset - step_offset_ra, (float)target_offset);
        if (current_Kp < target_Kp) current_Kp = min(current_Kp + step_Kp_vao, target_Kp);
        else if (current_Kp > target_Kp) current_Kp = max(current_Kp - step_Kp_ra, target_Kp);
    }
    
    int finalBaseSpeed = constrain(baseSpeed + (int)current_offset, speed, startSpeed);
    integral = constrain(integral + smoothedError, -100, 100);
    float derivative = constrain(smoothedError - lastError, -2.0, 2.0);
    float rawPid = (current_Kp * smoothedError) + (Ki * integral) + (current_Kd * derivative);
    lastError = smoothedError;
    
    float filteredPid = (rawPid * 0.05) + (lastPidOutput * 0.95);
    if (filteredPid > lastPidOutput + 100) filteredPid = lastPidOutput + 100;
    else if (filteredPid < lastPidOutput - 100) filteredPid = lastPidOutput - 100;
    lastPidOutput = filteredPid;
    
    int PHUC_DEPTRAI = constrain(finalBaseSpeed - (int)filteredPid, speed, startSpeed);
    int PHUC_KDEP = constrain(finalBaseSpeed + (int)filteredPid, speed, startSpeed);
    step_dc(true, true, HIGH, LOW, PHUC_KDEP, PHUC_DEPTRAI + 0);
}