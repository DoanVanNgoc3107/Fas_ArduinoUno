// Author: Doan Van Ngoc - 104317 - DTD64CL
#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

/*
 * Ở đây là định nghĩa các chân của linh kiện
 * TODO : Chia tách các phần linh để dễ quản lý và theo dõi
 */

// ==== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== LED Config
#define ledGreen 10
#define ledRed 11
#define ledYellow 12

// ==== Button Config
#define btnActive 2
#define buzzerPin 9
#define btnReset 8
#define DEBOUNCE_MS 50 // 50 milliseconds

// ==== DHT Config
#define DHT_PIN 4
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

#define TEMP_OFFSET -2.0
// ==== Service - TEMP
#define safety 35.0
#define warning 38.0
#define danger 42.0
#define hys 0.5

// ==== MQ2 Config
#define MQ2_PIN A0
#define MQ2_THRESHOLD_NORMAL 300  // ngưỡng bình thường
#define MQ2_THRESHOLD_WARNING 500 // ngưỡng cảnh báo
#define MQ2_THRESHOLD_DANGER 700  // ngưỡng nguy hiểm
constexpr int MQ2_tb = 10;

// ==== Service - MQ2 Logic
#define MQ2_SAMPLE_TIME 1000       // đọc mỗi 1 giây
#define MQ2_CALIBRATION_TIME 20000 // hiệu chuẩn 20 giây đầu

// ==== Services Logic
#define temp_tb 0.2      // Nhiet do nguong
#define bonusCorrect 1.0 // Tang do chinh xac cho cam bien nhiet

float tempLast = 0.0;

int valueMQ2Last = 0;

// Cờ để nhận biết nút ngắt ngoài
volatile bool activeButtonPressedFlag = false;
// Biến lưu thời gian cuối cùng nhấn nút (để chống dội)
unsigned long lastActiveButtonPress = 0;

unsigned long lastTime = 0;

// Debounce state for reset button (polling)
int btnReset_lastReading = HIGH;
int btnReset_stableState = HIGH;
unsigned long btnReset_lastDebounce = 0;

/**
 * @brief Kiểm tra xem đã đủ thời gian trễ hay chưa (non-blocking).
 * @param lastTime Biến lưu mốc thời gian trước đó (truyền bằng tham chiếu).
 * @param timeDelay Khoảng thời gian trễ mong muốn (ms).
 * @return true nếu đã đủ thời gian, false nếu chưa.
 */
bool checkTimer(unsigned long &lastTime, const unsigned long timeDelay)
{
    // Get current time
    unsigned long now = millis();

    // If this is the first call (lastTime == 0), initialize and don't trigger yet.
    if (lastTime == 0)
    {
        lastTime = now;
        return false;
    }

    // Compare with the last time (overflow-safe)
    if (now - lastTime >= timeDelay)
    {
        // if enough time has passed, update lastTime
        lastTime = now;
        return true;
    }

    // If not enough time has passed
    return false;
}

/*
 * Serial set up
 * Hàm serialSetUp() - dùng để cài đặt kết nối arduino uno r3 với Serial của máy tính
 * @param : int - Baud rate (tốc độ baud)
 * Baud rate (tốc độ baud) là tốc độ truyền dữ liệu nối tiếp (serial)
 * — tức là số lượng bit được truyền trong 1 giây qua cổng UART (Universal Asynchronous Receiver/Transmitter).
 */
void serialSetUp(int const baudRate)
{
    Serial.begin(baudRate);
    Serial.println("Serial set up finish");
}

/*
 * AlarmMode (contract):
 * - ALARM_NONE: không có cảnh báo, buzzer tắt, LED tương ứng sẽ báo trạng thái an toàn
 * - ALARM_WARNING: cảnh báo ở ngưỡng giữa (sẽ phát còi tần số và chu kỳ khác)
 * - ALARM_DANGER: báo động (tần số cao hơn, nhịp nhanh)
 *
 * Biến liên quan:
 * - alarmMode: lưu mode hiện tại
 * - alarmLast: timestamp lần thay đổi trạng thái buzz (dùng millis())
 * - alarmStateOn: trạng thái on/off hiện tại của còi trong chu kỳ
 * - alarmOnMs/alarmOffMs: thời gian ON/OFF cho chu kỳ
 * - alarmFreq: tần số (Hz) của tone khi bật
 */
enum AlarmMode
{
    ALARM_NONE = 0,
    ALARM_SAFETY = 1,
    ALARM_WARNING = 2,
    ALARM_DANGER = 3,

    ALARM_GAS_WARNING = 4,
    ALARM_GAS_DANGER = 5,
    ALARM_COMBINED = 6,
};

AlarmMode alarmMode = ALARM_NONE;

unsigned long alarmLast = 0;
bool alarmStateOn = false;
unsigned long alarmOnMs = 0;
unsigned long alarmOffMs = 0;

int alarmFreq = 1000;

// Setup Buzzer
// "buzzerSetUp" chỉ đặt pinMode và trạng thái ban đầu, KHÔNG phát âm.
void buzzerSetUp()
{
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
}

/*
 * startAlarm: khởi tạo tham số chu kỳ âm thanh cho từng mức
- Chỉ thay đổi trạng thái và tham số; không làm blocking hoặc delay
- Gọi handleAlarm() trong loop để thực hiện ON/OFF dựa trên millis()
*/
void startAlarm(AlarmMode const mode)
{
    alarmMode = mode;
    alarmLast = millis();
    alarmStateOn = false;
    if (mode == ALARM_WARNING)
    {
        alarmFreq = 1000; // 1 kHz
        alarmOnMs = 400;
        alarmOffMs = 600;
    }
    else if (mode == ALARM_DANGER)
    {
        alarmFreq = 2000; // 2 kHz
        alarmOnMs = 150;
        alarmOffMs = 150;
    }
}

// stopAlarm: tắt còi ngay lập tức và reset trạng thái alarm
void stopAlarm()
{
    noTone(buzzerPin);
    alarmMode = ALARM_NONE;
    alarmStateOn = false;
    digitalWrite(buzzerPin, LOW);
}

/*
 * handleAlarm: state machine non-blocking cho buzzer
- Phải được gọi thường xuyên trong loop() để cập nhật trạng thái theo millis()
- Nếu AlarmMode == NONE thì hàm trả về, không làm gì
*/
void handleAlarm(const unsigned long now)
{
    if (alarmMode == ALARM_NONE)
        return;

    const unsigned long interval = alarmStateOn ? alarmOnMs : alarmOffMs;

    if (now - alarmLast >= interval)
    {
        alarmLast = now;
        alarmStateOn = !alarmStateOn;
        if (alarmStateOn)
        {
            tone(buzzerPin, alarmFreq);
        }
        else
        {
            noTone(buzzerPin);
        }
    }
}

// ===== Logic business MQ2 ===
/*
 * Hàm này sử đụng dể hiệu chuẩn cho MQ2
 * @param : start - thời gian hiện tại
 *          time_calibrate - thời gian hiệu chuẩn (ms)
 * @return : bool
 */
bool MQ2Setup(const unsigned long start, unsigned long const time_calibrate)
{
    if (millis() - start > time_calibrate)
        return true;
    return false;
}

/*
 * Hàm này trả về giá trị khi đọc của MQ2
 * @param : MQ2 pin
 */
int getValueMQ2(int const MQ2_pin)
{
    return analogRead(MQ2_pin);
}

// ==== LCD
void lcdSetUp()
{
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);

    lcd.print("Hello!");

    lcd.setCursor(0, 1);
    lcd.print("Fire Alarm");

    delay(2000);
    lcd.clear();
}

// Kiểm tra xem dht22 có hoạt động hay không?
bool checkNAN(float const temp)
{
    if (isnan(temp))
    {
        Serial.println("Failed to read from DHT sensor!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.println("ERROR: DHT");
        delay(1000);
        return true;
    }
    return false;
}

/*
 * Services Alarm Logic
 * Những hàm showXXX chỉ chịu trách nhiệm cập nhật LED + LCD (actuators)
 * Quy tắc tách trách nhiệm: quyết định trạng thái (service_business) -> gọi showXXX/startAlarm
 */
void ledServices(bool const led_Green_State, const bool led_Red_State, const bool led_Yellow_State)
{
    digitalWrite(ledGreen, led_Green_State);
    digitalWrite(ledRed, led_Red_State);
    digitalWrite(ledYellow, led_Yellow_State);
}

// ================================Logic Business===================================
void showSafety()
{
    stopAlarm();
    ledServices(HIGH, LOW, LOW);
    lcd.setCursor(0, 1);
    lcd.print("AN TOAN      ");
}

void showWarn()
{
    ledServices(LOW, LOW, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("BAT THUONG   ");
}

void showDanger()
{
    ledServices(LOW, HIGH, LOW);
    lcd.setCursor(0, 1);
    lcd.print("BAO DONG     ");
}

// Update debounce state for a polled button. Call frequently from loop().
void updateButtonDebounce(const int pin, int &lastReading, int &stableState, unsigned long &lastDebounceTime, const unsigned long debounceMs)
{
    int reading = digitalRead(pin);
    unsigned long now = millis();

    if (reading != lastReading)
    {
        // reset debounce timer on any change
        lastDebounceTime = now;
    }

    if ((now - lastDebounceTime) >= debounceMs)
    {
        // if the reading has been stable for the debounce period, take it as the actual state
        if (reading != stableState)
        {
            stableState = reading;
        }
    }

    lastReading = reading;
}

/*
    * Hàm xử lý nghiệp vụ chính
    * @param : tempCurrent - nhiệt độ hiện tại
    *          valueMQ2 - giá trị đọc từ MQ2
    *          resetPressed - trạng thái nút reset
    *          activePressed - trạng thái nút active
    * @return : void
    * Logic:
    * 1. Nếu nút reset được nhấn, dừng báo động và hiển thị trạng thái an toàn
    * 2. Nếu nút active được nhấn, kích hoạt báo động nguy hiểm và hiển thị trạng thái nguy hiểm
    * 3. Cập nhật trạng thái hiển thị và báo động dựa trên nhiệt độ và giá trị MQ2 với ngưỡng đã định nghĩa
    * 4. Chỉ cập nhật hiển thị nếu có sự thay đổi đáng kể để giảm nhấp nháy của LCD
    * 5. Cập nhật biến tempLast và valueMQ2Last sau mỗi lần kiểm tra
    * 6. Cập nhật lastTime mỗi khi có sự kiện nhấn nút

 */
void service_business(const float tempCurrent, const int valueMQ2, const bool resetPressed)
{

    if (resetPressed)
    {
        ledServices(HIGH, HIGH, HIGH);
        stopAlarm();
        return;
    }

    // Update only if significant change to reduce LCD flicker
    if (fabs(tempCurrent - tempLast) >= temp_tb || abs(valueMQ2 - valueMQ2Last) >= MQ2_tb)
    {
        // SAFETY
        if (tempCurrent <= safety && valueMQ2 <= MQ2_THRESHOLD_NORMAL)
        {
            showSafety();
            stopAlarm();
            tempLast = tempCurrent;
            valueMQ2Last = valueMQ2;
        }
        else if ((tempCurrent > safety && tempCurrent <= warning) || (valueMQ2 >= MQ2_THRESHOLD_WARNING && valueMQ2 <= MQ2_THRESHOLD_DANGER))
        {
            startAlarm(ALARM_WARNING);
            // handleAlarm(now);
            showWarn();
            tempLast = tempCurrent;
            valueMQ2Last = valueMQ2;
        }
        else if (tempCurrent > warning || tempCurrent >= danger || valueMQ2 >= MQ2_THRESHOLD_DANGER)
        {
            startAlarm(ALARM_DANGER);
            // handleAlarm(now);
            showDanger();
            tempLast = tempCurrent;
            valueMQ2Last = valueMQ2;
        }

        Serial.print("Temp C=");
        Serial.println(tempCurrent, 2);

        // Build strings for MQ2 (left) and temperature (right) and print without overlap
        String mq2Str = String("MQ2:") + String(valueMQ2);
        String tempStr = String("T:") + String(tempCurrent, 1) + String((char)223) + String("C");

        int tempStart = 16 - tempStr.length();
        if (tempStart < 0)
            tempStart = 0;
        // Ensure at least one column between MQ2 and temp; if collision, adjust tempStart
        if (tempStart <= (int)mq2Str.length())
        {
            tempStart = mq2Str.length() + 1;
            if (tempStart > 15)
                tempStart = 15 - tempStr.length() + 1; // fallback
            if (tempStart < 0)
                tempStart = 0;
        }

        lcd.setCursor(0, 0);
        lcd.print(mq2Str);
        // pad spaces up to tempStart
        int pad = tempStart - mq2Str.length();
        for (int i = 0; i < pad; ++i)
            lcd.print(' ');

        lcd.setCursor(tempStart, 0);
        lcd.print(tempStr);
    }
}

// Hàm ngắt ngoài xử lý báo cháy khẩn cấp khi bấm nút chủ động.
void ISR_ACTIVE_BUTTON()
{
    // Giương cờ cho loop() biết nút đã được nhấn
    activeButtonPressedFlag = true;
}

void handleActiveButtonLogic(volatile bool &dangerFlag)
{
    if (activeButtonPressedFlag == true)
    {
        // 1.1. Hạ cờ xuống ngay lập tức
        activeButtonPressedFlag = false;

        // 1.2. Chống dội (Debounce)
        // Chỉ xử lý nếu đã qua 50ms kể từ lần nhấn hợp lệ cuối
        if (millis() - lastActiveButtonPress >= DEBOUNCE_MS)
        {
            lastActiveButtonPress = millis(); // Cập nhật mốc thời gian

            // 1.3. Thực thi logic báo động khẩn cấp
            // (Đây là code bạn đã cố viết trong ISR cũ)
            Serial.println("!!! KICH HOAT BAO DONG KHAN CAP !!!");
            ledServices(LOW, HIGH, LOW);
            startAlarm(ALARM_DANGER);
            showDanger();
        }
        // Nếu chưa đủ 50ms, đây là tín hiệu "dội" (bounce), ta bỏ qua
    }
}

void setup()
{
    serialSetUp(9600);

    dht.begin();

    lcdSetUp();

    MQ2Setup(millis(), MQ2_CALIBRATION_TIME);

    // GREEN - RED - YELLOW
    pinMode(ledGreen, OUTPUT);
    pinMode(ledRed, OUTPUT);
    pinMode(ledYellow, OUTPUT);

    // Btn - Button Active
    pinMode(btnActive, INPUT_PULLUP); // Chân 2

    // Reset - Button Reset (use internal pull-up, expect button to ground when pressed)
    pinMode(btnReset, INPUT_PULLUP);

    // Set initial LED states off
    ledServices(LOW, LOW, LOW);

    // Hàm ngắt ngoài này đang chạy ở chế độ FALLING nghĩa là : khi chân btnActive (Pin 2) có tín hiệu từ HIGH xuống LOW
    attachInterrupt(digitalPinToInterrupt(btnActive), ISR_ACTIVE_BUTTON, FALLING);

    // Reset - Button Reset
    pinMode(btnReset, INPUT_PULLUP); // Chân 8

    // Buzzer setup
    buzzerSetUp();
    stopAlarm();
}

void loop()
{
    const unsigned long now = millis();

    // ==============================================
    // === 1. XỬ LÝ LOGIC NGẮT (ƯU TIÊN CAO NHẤT) ===
    // ==============================================
    handleActiveButtonLogic(activeButtonPressedFlag);

    // ==============================================
    // === 2. CÁC TÁC VỤ BÌNH THƯỜNG KHÁC ===
    // ==============================================
    // Xử lý còi báo động (non-blocking)
    handleAlarm(now);

    // Đọc cảm biến
    float const tempCurrent = dht.readTemperature() + TEMP_OFFSET;
    int const valueMQ2Current = getValueMQ2(MQ2_PIN);

    // Update debounce state for reset button (polled)
    updateButtonDebounce(btnReset, btnReset_lastReading, btnReset_stableState, btnReset_lastDebounce, DEBOUNCE_MS);
    const bool resetPressed = (btnReset_stableState == LOW);

    // Check if read failed
    if (checkNAN(tempCurrent))
        return;

    // Chỉ chạy service_business mỗi 2000 ms
    if (checkTimer(lastTime, 2000))
    {
        service_business(tempCurrent, valueMQ2Current, resetPressed);
    }
}
