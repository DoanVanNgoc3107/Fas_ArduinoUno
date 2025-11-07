// Author: Doan Van Ngoc - 104317 - DTD64CL
#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

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
#define btnActive 7
#define buzzerPin 9
#define btnReset 8

// ==== DHT Config
#define DHT_PIN 2
#define DHT_TYPE DHT22

DHT dht(DHT_PIN, DHT_TYPE);

// ==== Service - TEMP
#define safety 35.0
#define warning 38.0
#define danger 42.0
#define hys 0.5

// ==== MQ2 Config
#define MQ2_PIN A0
#define MQ2_THRESHOLD_NORMAL 300   // ngưỡng bình thường
#define MQ2_THRESHOLD_WARNING 500  // ngưỡng cảnh báo
#define MQ2_THRESHOLD_DANGER 700   // ngưỡng nguy hiểm
constexpr int MQ2_tb = 10;

// ==== Service - MQ2 Logic
#define MQ2_SAMPLE_TIME 1000      // đọc mỗi 1 giây
#define MQ2_CALIBRATION_TIME 20000 // hiệu chuẩn 20 giây đầu

// ==== Services Logic
#define temp_tb 0.2 // Nhiet do nguong
#define bonusCorrect 1.0 // Tang do chinh xac cho cam bien nhiet


unsigned long lastTime = 0;

float tempLast = 0.0;

int valueMQ2Last = 0;

/*
 * Serial set up
 * Hàm serialSetUp() - dùng để cài đặt kết nối arduino uno r3 với Serial của máy tính
 * @param : int - Baud rate (tốc độ baud)
 * Baud rate (tốc độ baud) là tốc độ truyền dữ liệu nối tiếp (serial)
 * — tức là số lượng bit được truyền trong 1 giây qua cổng UART (Universal Asynchronous Receiver/Transmitter).
 */
void serialSetUp(int const baudRate) {
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
enum AlarmMode {
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
void buzzerSetUp() {
    pinMode(buzzerPin, OUTPUT);
    digitalWrite(buzzerPin, LOW);
}

/*
 * startAlarm: khởi tạo tham số chu kỳ âm thanh cho từng mức
- Chỉ thay đổi trạng thái và tham số; không làm blocking hoặc delay
- Gọi handleAlarm() trong loop để thực hiện ON/OFF dựa trên millis()
*/
void startAlarm(AlarmMode const mode) {
    alarmMode = mode;
    alarmLast = millis();
    alarmStateOn = false;
    if (mode == ALARM_WARNING) {
        alarmFreq = 1000; // 1 kHz
        alarmOnMs = 400;
        alarmOffMs = 600;
    } else if (mode == ALARM_DANGER) {
        alarmFreq = 2000; // 2 kHz
        alarmOnMs = 150;
        alarmOffMs = 150;
    }
}

// stopAlarm: tắt còi ngay lập tức và reset trạng thái alarm
void stopAlarm() {
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
void handleAlarm(const unsigned long now) {
    if (alarmMode == ALARM_NONE) return;

    const unsigned long interval = alarmStateOn ? alarmOnMs : alarmOffMs;

    if (now - alarmLast >= interval) {
        alarmLast = now;
        alarmStateOn = !alarmStateOn;
        if (alarmStateOn) {
            tone(buzzerPin, alarmFreq);
        } else {
            noTone(buzzerPin);
        }
    }
}

// ===== Logic business MQ2 ===
/*
 * Hàm này trả về giá trị khi đọc của MQ2
 *
 */
int getValueMQ2(int const MQ2) {
    return analogRead(MQ2);
}

// ==== LCD
void lcdSetUp() {
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
bool checkNAN(float const temp) {
    if (isnan(temp)) {
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
void ledServices(bool const led_Green_State, const bool led_Red_State, const bool led_Yellow_State) {
    digitalWrite(ledGreen, led_Green_State);
    digitalWrite(ledRed, led_Red_State);
    digitalWrite(ledYellow, led_Yellow_State);
}

// ================================Logic Business===================================
void showSafety() {
    stopAlarm();
    ledServices(HIGH, LOW, LOW);
    lcd.setCursor(0, 1);
    lcd.print("AN TOAN      ");
}

void showWarn() {
    ledServices(LOW, LOW, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("BAT THUONG   ");
}

void showDanger() {
    ledServices(LOW, HIGH, LOW);
    lcd.setCursor(0, 1);
    lcd.print("BAO DONG     ");
}

/*
 * service_business (contract):
 * - Inputs:
 * temp: giá trị nhiệt độ hiện tại (đọc từ DHT22)
 * tempCurrent: (hiện đang trùng với temp) dự phòng cho logic nâng cao
 * resetPressed: true nếu nút Reset đang được nhấn
 * - Behavior:
 * - Nếu resetPressed: tắt còi và bật tắt LED để người dùng kiểm tra phần cứng
 * - Nếu nhiệt thay đổi đủ lớn (temp_tb) hoặc khác mẫu trước: quyết định trạng thái
 * - Side effects: cập nhật LED, LCD, và gọi start/stop alarm.
 *
 * Lưu ý về debounce: hiện dùng delay(500) nhỏ khi reset được nhấn để giảm bouncing.
 * Độ chính xác đọc DHT22: DHT22 có độ phân giải cao hơn DHT11.
 */
void service_business(const float tempCurrent, const int valueMQ2, const bool resetPressed, const bool activePressed) {
    if (resetPressed) {
        ledServices(HIGH, HIGH, HIGH);

        stopAlarm();

        delay(50);

        return;
    }

    if (activePressed) {
        ledServices(LOW, HIGH, LOW);

        startAlarm(ALARM_DANGER);

        showDanger();
    }

    // Cập nhật chỉ khi nhiệt độ thay đổi đủ lớn so với mẫu trước để giảm flicker LCD
    if (tempCurrent != tempLast || tempCurrent - tempLast >= temp_tb ||
        valueMQ2 != valueMQ2Last || valueMQ2 - valueMQ2Last >= temp_tb) {
        // SAFETY
        if (tempCurrent <= safety && valueMQ2 <= MQ2_THRESHOLD_NORMAL) {
            showSafety();
            stopAlarm();
            tempLast = tempCurrent;
            valueMQ2Last = valueMQ2;
        } else if ((tempCurrent > safety && tempCurrent <= warning)
                   || (valueMQ2 >= MQ2_THRESHOLD_WARNING && valueMQ2 <= MQ2_THRESHOLD_DANGER)) {
            startAlarm(ALARM_WARNING);
            // handleAlarm(now);
            showWarn();
            tempLast = tempCurrent;
            valueMQ2Last = valueMQ2;
        } else if (tempCurrent > warning || tempCurrent >= danger
                   || valueMQ2 >= MQ2_THRESHOLD_DANGER) {
            startAlarm(ALARM_DANGER);
            // handleAlarm(now);
            showDanger();
            tempLast = tempCurrent;
            valueMQ2Last = valueMQ2;
        }

        Serial.print("Temp C=");
        Serial.println(tempCurrent, 2);

        lcd.setCursor(0, 0);
        lcd.print("T: ");
        lcd.print(tempCurrent, 1); // one decimal
        lcd.print(static_cast<char>(223));
        lcd.print("C   ");
    }
}

void setup() {
    serialSetUp(9600);

    dht.begin();

    lcdSetUp();

    // Led - Green - An toan
    pinMode(ledGreen, OUTPUT);

    // Led - Red - Danger
    pinMode(ledRed, OUTPUT);

    // Led - Yellow - Warning
    pinMode(ledYellow, OUTPUT);

    // Btn - Button Active (khai báo sẵn để mở rộng - hiện chưa dùng trong logic)
    pinMode(btnActive, INPUT_PULLUP);

    // Reset - Button Reset (use internal pull-up, expect button to ground when pressed)
    pinMode(btnReset, INPUT_PULLUP);

    // Set initial LED states off
    ledServices(LOW, LOW, LOW);

    // Buzzer setup
    buzzerSetUp();
    stopAlarm();
}

void loop() {

    const unsigned long now = millis();

    // continue alarm state machine each loop
    handleAlarm(now);

    float const tempCurrent = dht.readTemperature() - 2.0;
    int const valueMQ2Current = getValueMQ2(MQ2_PIN);

    // Check if read failed
    if (checkNAN(tempCurrent)) return;

    // Khi ma thoi gian nho hon 2s thi se thoat khoi if
    // Su dung if trong khoang thoi gian 2s
    if (now - lastTime < 2000) {
        return; // đọc mỗi 2 giây
    }

    lastTime = now;

    // Reset button uses INPUT_PULLUP: pressed == LOW
    const bool resetPressed = (digitalRead(btnReset) == LOW);
    const bool activePressed = (digitalRead(btnActive) == LOW);

    service_business(tempCurrent, valueMQ2Current, resetPressed, activePressed);
}
