/**
 * @file fire_alarm_final.cpp
 * @author Doan Van Ngoc - 104317 - DTD64CL
 * @brief Hệ thống cảnh báo cháy sử dụng Arduino Uno, DHT22, MQ2, LCD I2C và ngắt ngoài.
 * @version 1.0
 * @date 2025-11-08
 */

// =============================================================================
// THƯ VIỆN
// =============================================================================
#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// =============================================================================
// CẤU HÌNH PHẦN CỨNG (DEFINES)
// =============================================================================

// ==== LCD I2C ====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== LED ====
#define ledGreen 10  // LED báo trạng thái An Toàn
#define ledRed 11    // LED báo trạng thái Nguy Hiểm
#define ledYellow 12 // LED báo trạng thái Cảnh Báo

// ==== NÚT NHẤN & CÒI ====
#define btnActive 2    // Nút kích hoạt khẩn cấp (Phải là chân ngắt 2 hoặc 3)
#define btnReset 8     // Nút tắt báo động (dùng polling)
#define buzzerPin 9    // Chân còi báo động
#define DEBOUNCE_MS 50 // Thời gian chống dội cho nút nhấn (50ms)

// ==== Cảm biến DHT (Nhiệt độ/Độ ẩm) ====
#define DHT_PIN 4      // Chân data của DHT22 (Đã đổi sang 4 để tránh xung đột với ngắt)
#define DHT_TYPE DHT22 // Loại cảm biến
DHT dht(DHT_PIN, DHT_TYPE);
#define TEMP_OFFSET -2.0 // Giá trị hiệu chỉnh (offset) cho cảm biến nhiệt độ

// ==== Ngưỡng nhiệt độ ====
#define safety 35.0  // Ngưỡng an toàn (<= 35°C)
#define warning 38.0 // Ngưỡng cảnh báo (35°C < T <= 38°C)
#define danger 42.0  // Ngưỡng nguy hiểm (> 38°C, hoặc > 42°C tùy logic)

// ==== Cảm biến MQ2 (Khí Gas) ====
#define MQ2_PIN A0                // Chân Analog
#define MQ2_THRESHOLD_NORMAL 300  // Ngưỡng an toàn
#define MQ2_THRESHOLD_WARNING 500 // Ngưỡng cảnh báo
#define MQ2_THRESHOLD_DANGER 700  // Ngưỡng nguy hiểm

// ==== Thông số logic hệ thống ====
#define MQ2_CALIBRATION_TIME 20000 // Thời gian làm nóng MQ2 (20 giây)
#define temp_tb 0.2                // Ngưỡng thay đổi nhiệt độ để cập nhật LCD
constexpr int MQ2_tb = 10;         // Ngưỡng thay đổi MQ2 để cập nhật LCD

// =============================================================================
// BIẾN TOÀN CỤC (GLOBAL VARIABLES)
// =============================================================================

// ---- Trạng thái cảm biến ----
float tempLast = 0.0;        // Lưu giá trị nhiệt độ cuối cùng
int valueMQ2Last = 0;        // Lưu giá trị MQ2 cuối cùng
bool dhtErrorActive = false; // Cờ báo lỗi cảm biến DHT

// ---- Ngắt ngoài (Nút Active) ----
volatile bool activeButtonPressedFlag = false; // Cờ (flag) được ISR giương lên
unsigned long lastActiveButtonPress = 0;       // Thời gian cuối cùng nhấn (cho debounce)

// ---- Timer chính (cho service_business) ----
unsigned long lastTime = 0; // Mốc thời gian cho timer 2 giây

// ---- Chống dội (Nút Reset - Polling) ----
int btnReset_lastReading = HIGH;         // Trạng thái đọc cuối cùng
int btnReset_stableState = HIGH;         // Trạng thái ổn định (đã chống dội)
unsigned long btnReset_lastDebounce = 0; // Mốc thời gian chống dội

// ---- Trạng thái còi báo động (State Machine) ----
enum AlarmMode
{
    ALARM_NONE = 0,
    ALARM_WARNING = 2,
    ALARM_DANGER = 3,
};
AlarmMode alarmMode = ALARM_NONE; // Trạng thái còi hiện tại
unsigned long alarmLast = 0;      // Mốc thời gian cuối cùng còi đổi trạng thái
bool alarmStateOn = false;        // Còi đang bật (true) hay tắt (false) trong chu kỳ
unsigned long alarmOnMs = 0;      // Thời gian bật (ms)
unsigned long alarmOffMs = 0;     // Thời gian tắt (ms)
int alarmFreq = 1000;             // Tần số còi

// =============================================================================
// CÁC HÀM TIỆN ÍCH (UTILITY FUNCTIONS)
// =============================================================================

/**
 * @brief Kiểm tra xem một khoảng thời gian (non-blocking) đã trôi qua chưa.
 * @param lastTime Mốc thời gian cuối cùng (truyền bằng tham chiếu &).
 * @param timeDelay Khoảng thời gian (ms) cần kiểm tra.
 * @return true nếu đã đủ thời gian, false nếu chưa.
 */
bool checkTimer(unsigned long &lastTime, const unsigned long timeDelay)
{
    unsigned long now = millis();

    // Khởi tạo mốc thời gian nếu đây là lần gọi đầu
    if (lastTime == 0)
    {
        lastTime = now;
        return false;
    }

    // Kiểm tra (an toàn với tràn số)
    if (now - lastTime >= timeDelay)
    {
        lastTime = now; // Cập nhật mốc thời gian
        return true;
    }

    return false;
}

/**
 * @brief Khởi tạo giao tiếp Serial.
 * @param baudRate Tốc độ baud (ví dụ: 9600, 115200).
 */
void serialSetUp(int const baudRate)
{
    Serial.begin(baudRate);
    Serial.println("--- He thong khoi dong ---");
}

/**
 * @brief Khởi tạo màn hình LCD.
 */
void lcdSetUp()
{
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);
    lcd.print("He thong bao chay");
    lcd.setCursor(0, 1);
    lcd.print("GROUP 7");
    delay(2000);
    lcd.clear();
}

/**
 * @brief Thực hiện chờ (blocking) để làm nóng cảm biến MQ2.
 * Chỉ được gọi bên trong setup().
 * @param time_calibrate Thời gian chờ (ms).
 */
void MQ2Setup(unsigned long const time_calibrate)
{
    Serial.println("Dang lam nong cam bien MQ2...");
    lcd.clear();
    lcd.print("Hieu chuan MQ2...");
    lcd.setCursor(0, 1);
    lcd.print("Vui long cho...");

    // Dùng delay() trong setup() là an toàn
    delay(time_calibrate);

    Serial.println("MQ2 san sang.");
    lcd.clear();
}

/**
 * @brief Đọc giá trị thô (raw) từ cảm biến MQ2.
 * @param MQ2_pin Chân Analog để đọc.
 * @return int Giá trị analog (0-1023).
 */
int getValueMQ2(int const MQ2_pin)
{
    // Chỉ đọc và trả về, không in Serial ở đây để tránh spam
    return analogRead(MQ2_pin);
}

// =============================================================================
// CÁC HÀM XỬ LÝ TRẠNG THÁI (STATE HANDLERS)
// =============================================================================

/**
 * @brief Điều khiển 3 đèn LED trạng thái.
 * @param led_Green_State Trạng thái (HIGH/LOW) cho LED Xanh.
 * @param led_Red_State Trạng thái (HIGH/LOW) cho LED Đỏ.
 * @param led_Yellow_State Trạng thái (HIGH/LOW) cho LED Vàng.
 */
void ledServices(bool const led_Green_State, const bool led_Red_State, const bool led_Yellow_State)
{
    digitalWrite(ledGreen, led_Green_State);
    digitalWrite(ledRed, led_Red_State);
    digitalWrite(ledYellow, led_Yellow_State);
}

/**
 * @brief Kích hoạt trạng thái CẢNH BÁO (LED vàng, còi ngắt quãng, LCD).
 */
void showWarn()
{
    ledServices(LOW, LOW, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("CANH BAO       ");
}

/**
 * @brief Kích hoạt trạng thái NGUY HIỂM (LED đỏ, còi hú liên tục, LCD).
 */
void showDanger()
{
    ledServices(LOW, HIGH, LOW);
    lcd.setCursor(0, 1);
    lcd.print("!!!BAO DONG!!! ");
}

/**
 * @brief Khởi tạo chân còi báo động.
 */
void buzzerSetUp()
{
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
}

/**
 * @brief Bắt đầu một chế độ báo động (non-blocking).
 * Cập nhật các tham số cho handleAlarm().
 * @param mode Chế độ báo động (ALARM_WARNING, ALARM_DANGER).
 */
void startAlarm(AlarmMode const mode)
{
    // Chỉ cập nhật nếu chế độ thay đổi
    if (alarmMode == mode)
        return;

    alarmMode = mode;
    alarmLast = millis();
    alarmStateOn = false; // Bắt đầu bằng trạng thái TẮT

    if (mode == ALARM_WARNING)
    {
        alarmFreq = 1000; // 1 kHz
        alarmOnMs = 400;  // Bật 400ms
        alarmOffMs = 600; // Tắt 600ms
    }
    else if (mode == ALARM_DANGER)
    {
        alarmFreq = 2000; // 2 kHz
        alarmOnMs = 150;  // Bật 150ms
        alarmOffMs = 150; // Tắt 150ms
    }
}

/**
 * @brief Tắt còi báo động ngay lập tức và reset trạng thái.
 */
void stopAlarm()
{
    noTone(buzzerPin);
    alarmMode = ALARM_NONE;
    alarmStateOn = false;
    digitalWrite(buzzerPin, LOW);
}

/**
 * @brief Kích hoạt trạng thái AN TOÀN (LED xanh, còi tắt, LCD).
 */
void showSafety()
{
    stopAlarm();
    ledServices(HIGH, LOW, LOW);
    lcd.setCursor(0, 1);
    lcd.print("AN TOAN        ");
}

/**
 * @brief Xử lý còi báo động (State Machine, non-blocking).
 * Phải được gọi liên tục trong loop() để tạo âm thanh ngắt quãng.
 * @param now Thời gian hiện tại (millis()).
 */
void handleAlarm(const unsigned long now)
{
    if (alarmMode == ALARM_NONE)
        return;

    const unsigned long interval = alarmStateOn ? alarmOnMs : alarmOffMs;

    if (now - alarmLast >= interval)
    {
        alarmLast = now;
        alarmStateOn = !alarmStateOn; // Đảo trạng thái
        if (alarmStateOn)
        {
            tone(buzzerPin, alarmFreq); // Bật còi
        }
        else
        {
            noTone(buzzerPin); // Tắt còi
        }
    }
}

/**
 * @brief Kiểm tra lỗi đọc DHT (NaN) (Non-Blocking).
 * Sẽ hiển thị lỗi lên LCD (1 lần) nếu phát hiện.
 * Sẽ tự động xóa lỗi nếu đọc thành công trở lại.
 * @param temp Giá trị nhiệt độ vừa đọc được.
 * @return true nếu CÓ lỗi (NaN), false nếu không có lỗi.
 */
bool checkNAN_nonBlocking(float const temp)
{
    if (isnan(temp))
    {
        // Nếu lỗi MỚI xảy ra (trước đó không lỗi)
        if (!dhtErrorActive)
        {
            Serial.println("Loi: Khong doc duoc cam bien DHT!");
            stopAlarm(); // Dừng còi vì không có dữ liệu

            lcd.clear();
            lcd.setCursor(0, 0);
            lcd.print("LOI: CAM BIEN");
            lcd.setCursor(0, 1);
            lcd.print("KIEM TRA DHT22");

            dhtErrorActive = true; // Đánh dấu là đang có lỗi
        }
        return true; // Báo cho loop() biết là CÓ lỗi
    }

    // Nếu code chạy đến đây, nghĩa là đọc thành công

    // Nếu trước đó đang bị lỗi, bây giờ hết
    if (dhtErrorActive)
    {
        Serial.println("Cam bien DHT hoat dong tro lai.");
        dhtErrorActive = false; // Xóa cờ lỗi
        lcd.clear();            // Xóa màn hình lỗi để chuẩn bị in dữ liệu mới
    }

    return false; // Báo cho loop() biết là KHÔNG có lỗi
}

/**
 * @brief Cập nhật trạng thái (chống dội) cho nút nhấn (dùng polling).
 * Phải được gọi liên tục trong loop().
 * @param pin Chân nút nhấn.
 * @param lastReading Biến lưu trạng thái đọc cuối cùng (tham chiếu &).
 * @param stableState Biến lưu trạng thái ổn định (tham chiếu &).
 * @param lastDebounceTime Mốc thời gian chống dội cuối (tham chiếu &).
 * @param debounceMs Thời gian chống dội (ms).
 */
void updateButtonDebounce(const int pin, int &lastReading, int &stableState, unsigned long &lastDebounceTime, const unsigned long debounceMs)
{
    int reading = digitalRead(pin);
    unsigned long now = millis();

    if (reading != lastReading)
    {
        // Reset mốc thời gian nếu có thay đổi
        lastDebounceTime = now;
    }

    if ((now - lastDebounceTime) >= debounceMs)
    {
        // Nếu trạng thái đã ổn định trong [debounceMs]
        if (reading != stableState)
        {
            stableState = reading; // Chấp nhận trạng thái mới
        }
    }

    lastReading = reading;
}

// =============================================================================
// LOGIC NGẮT (INTERRUPT HANDLERS)
// =============================================================================

/**
 * @brief HÀM PHỤC VỤ NGẮT (ISR) cho nút Active (khẩn cấp).
 * HÀM NÀY PHẢI SIÊU NHANH. Chỉ giương cờ (flag).
 */
void ISR_ACTIVE_BUTTON()
{
    activeButtonPressedFlag = true;
}

/**
 * @brief Xử lý logic cho nút Active (chống dội và kích hoạt).
 * Được gọi liên tục từ loop() để kiểm tra cờ (flag).
 */
void handleActiveButtonLogic()
{
    // Kiểm tra cờ ngắt
    if (activeButtonPressedFlag == true)
    {
        activeButtonPressedFlag = false; // Hạ cờ ngay lập tức

        // Chống dội (debounce) cho nút ngắt
        if (millis() - lastActiveButtonPress >= DEBOUNCE_MS)
        {
            lastActiveButtonPress = millis(); // Cập nhật mốc thời gian

            // Thực thi logic báo động khẩn cấp
            Serial.println("!!! KICH HOAT BAO DONG KHAN CAP (NGAT) !!!");
            startAlarm(ALARM_DANGER);
            showDanger(); // Cập nhật LED và LCD
        }
        // Nếu chưa đủ DEBOUNCE_MS, đó là tín hiệu "dội" (bounce), bỏ qua.
    }
}

// =============================================================================
// HÀM NGHIỆP VỤ CHÍNH (MAIN BUSINESS LOGIC)
// =============================================================================

/**
 * @brief Xử lý logic nghiệp vụ chính (quyết định trạng thái báo động).
 * Chỉ chạy khi không có lỗi DHT và không bị nhấn Reset.
 * @param tempCurrent Nhiệt độ hiện tại.
 * @param valueMQ2 Giá trị MQ2 hiện tại.
 */
void service_business(const float tempCurrent, const int valueMQ2)
{
    // Chỉ cập nhật LCD/Serial nếu giá trị thay đổi đáng kể
    if (fabs(tempCurrent - tempLast) >= temp_tb || abs(valueMQ2 - valueMQ2Last) >= MQ2_tb)
    {
        bool stateChanged = false; // Cờ để kiểm tra logic

        // LOGIC 1: NGUY HIỂM (Ưu tiên cao nhất)
        if (tempCurrent > warning || tempCurrent >= danger || valueMQ2 >= MQ2_THRESHOLD_DANGER)
        {
            startAlarm(ALARM_DANGER);
            showDanger();
            stateChanged = true;
        }
        // LOGIC 2: CẢNH BÁO
        else if ((tempCurrent > safety && tempCurrent <= warning) || (valueMQ2 >= MQ2_THRESHOLD_WARNING && valueMQ2 <= MQ2_THRESHOLD_DANGER))
        {
            startAlarm(ALARM_WARNING);
            showWarn();
            stateChanged = true;
        }
        // LOGIC 3: AN TOÀN
        else if (tempCurrent <= safety && valueMQ2 <= MQ2_THRESHOLD_NORMAL)
        {
            // Chỉ gọi showSafety() nếu trạng thái trước đó KHÔNG PHẢI là an toàn
            if (alarmMode != ALARM_NONE)
            {
                showSafety();
                stateChanged = true;
            }
        }

        // Nếu có bất kỳ thay đổi nào, cập nhật giá trị
        if (stateChanged)
        {
            tempLast = tempCurrent;
            valueMQ2Last = valueMQ2;
        }

        // Cập nhật Serial (chỉ khi có thay đổi)
        Serial.print("Cap nhat: Temp C=");
        Serial.print(tempCurrent, 2);
        Serial.print(" | MQ2 Value=");
        Serial.println(valueMQ2);

        // Cập nhật LCD (phiên bản an toàn, không dùng String)
        lcd.setCursor(0, 0);
        lcd.print("                "); // Xóa dòng 0
        lcd.setCursor(0, 0);
        lcd.print("MQ2:");
        lcd.print(valueMQ2);
        lcd.setCursor(9, 0);
        lcd.print("T:");
        lcd.print(tempCurrent, 1);
        lcd.print((char)223); // Ký tự độ
        lcd.print("C");
    }
}

// =============================================================================
// SETUP - Chạy 1 lần khi khởi động
// =============================================================================
void setup()
{
    // Khởi tạo Serial
    serialSetUp(9600);

    // Khởi tạo cảm biến
    dht.begin();
    lcdSetUp();

    // Làm nóng MQ2 (chặn 20 giây)
    MQ2Setup(MQ2_CALIBRATION_TIME);

    // Khởi tạo LED
    pinMode(ledGreen, OUTPUT);
    pinMode(ledRed, OUTPUT);
    pinMode(ledYellow, OUTPUT);
    ledServices(LOW, LOW, LOW); // Tắt hết LED

    // Khởi tạo Nút nhấn
    pinMode(btnActive, INPUT_PULLUP); // Chân 2 (Ngắt)
    pinMode(btnReset, INPUT_PULLUP);  // Chân 8 (Polling)

    // Khởi tạo Còi
    buzzerSetUp();
    stopAlarm();

    // === ĐĂNG KÝ NGẮT NGOÀI ===
    attachInterrupt(digitalPinToInterrupt(btnActive), ISR_ACTIVE_BUTTON, FALLING);

    Serial.println("--- Khoi dong hoan tat. Bat dau loop() ---");
    showSafety(); // Bắt đầu ở trạng thái An Toàn
}

// =============================================================================
// LOOP - Chạy liên tục
// =============================================================================
void loop()
{
    const unsigned long now = millis();

    // ==============================================
    // === 1. XỬ LÝ NGẮT (ƯU TIÊN CAO NHẤT) ===
    // ==============================================
    // (Kiểm tra cờ 'activeButtonPressedFlag' có được ISR giương lên không)
    handleActiveButtonLogic();

    // ==============================================
    // === 2. XỬ LÝ TÁC VỤ THƯỜNG XUYÊN ===
    // ==============================================

    // Xử lý còi báo động (non-blocking, luôn chạy)
    handleAlarm(now);

    // Đọc và chống dội nút Reset (non-blocking, luôn chạy)
    updateButtonDebounce(btnReset, btnReset_lastReading, btnReset_stableState, btnReset_lastDebounce, DEBOUNCE_MS);
    const bool resetPressed = (btnReset_stableState == LOW);

    // Xử lý logic Reset (phản hồi tức thì)
    if (resetPressed)
    {
        // Tắt còi và reset về trạng thái an toàn ngay
        showSafety();
    }

    // ==============================================
    // === 3. XỬ LÝ TÁC VỤ ĐỊNH KỲ (MỖI 2 GIÂY) ===
    // ==============================================

    // Chỉ chạy logic cảm biến mỗi 2000 ms
    if (checkTimer(lastTime, 2000))
    {
        // Chỉ khi KHÔNG nhấn reset thì mới chạy logic báo động
        if (!resetPressed)
        {
            // 1. Đọc cảm biến DHT
            float const tempCurrent = dht.readTemperature() + TEMP_OFFSET;

            // 2. Kiểm tra lỗi (non-blocking)
            if (checkNAN_nonBlocking(tempCurrent))
            {
                // Nếu CÓ lỗi:
                // Không làm gì cả, màn hình lỗi sẽ được giữ nguyên.
                // Chờ 2 giây sau đọc lại.
            }
            else
            {
                // 3. KHÔNG có lỗi DHT:
                // Tiếp tục đọc MQ2 và chạy nghiệp vụ
                int const valueMQ2Current = getValueMQ2(MQ2_PIN);
                service_business(tempCurrent, valueMQ2Current);
            }
        }
    }
}