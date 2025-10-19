#include <Arduino.h>
#include <DHT.h>
#include <LiquidCrystal_I2C.h>

/*
 * Ở đây là định nghĩa các chân của linh kiện
 * TODO : Chia tách các phần linh để dễ quản lý và theo dõi
 */

// LED Config
#define ledGreen 10
#define ledRed 11
#define ledYellow 12

// Button Config
#define btnActive 7
#define buzzerPin 9
#define btnReset 8

// DHT11 Config
#define DHT11PIN 2

// Service - Alarm
#define safety 35.0
#define warning 37.0
#define danger 42.0
#define hys 0.5

// Services Logic
#define temp_tb 0.2
#define bonusCorrect 1.0

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
 *  - ALARM_NONE: không có cảnh báo, buzzer tắt, LED tương ứng sẽ báo trạng thái an toàn
 *  - ALARM_WARNING: cảnh báo ở ngưỡng giữa (sẽ phát còi tần số và chu kỳ khác)
 *  - ALARM_DANGER: báo động (tần số cao hơn, nhịp nhanh)
 *
 * Biến liên quan:
 *  - alarmMode: lưu mode hiện tại
 *  - alarmLast: timestamp lần thay đổi trạng thái buzz (dùng millis())
 *  - alarmStateOn: trạng thái on/off hiện tại của còi trong chu kỳ
 *  - alarmOnMs/alarmOffMs: thời gian ON/OFF cho chu kỳ
 *  - alarmFreq: tần số (Hz) của tone khi bật
 */

enum AlarmMode {
    ALARM_NONE = 0,
    ALARM_SAFETY = 1,
    ALARM_WARNING = 2,
    ALARM_DANGER = 3
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
    noTone(buzzerPin); // Tăt còi
    alarmMode = ALARM_NONE;
    alarmStateOn = false;
    digitalWrite(buzzerPin, LOW);
}

// handleAlarm: state machine non-blocking cho buzzer
// - Phải được gọi thường xuyên trong loop() để cập nhật trạng thái theo millis()
// - Nếu AlarmMode == NONE thì hàm trả về, không làm gì
void handleAlarm() {
    if (alarmMode == ALARM_NONE) return;
    unsigned long now = millis();
    unsigned long interval = alarmStateOn ? alarmOnMs : alarmOffMs;
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

LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long lastTime = 0; // mốc lấy mẫu (millis())

float tempLast = 0.0; // lưu mẫu trước để tránh in LCD quá thường xuyên

// LCD
void lcdSetUp() {
    lcd.init();
    lcd.backlight();
    lcd.setCursor(0, 0);

    lcd.print("Hello!");
    // delay(2000);
    // lcd.clear();

    lcd.setCursor(0, 1);
    lcd.print("Fire Alarm");
    delay(2000);
    lcd.clear();
}

/*
 * Cấu hình DHT11
 */
DHT dht11(DHT11PIN, DHT11);

// Kiểm tra xem dht có hoạt động hay không?
void checkNAN(float const temp) {
    if (isnan(temp)) {
        Serial.println("Failed to read from DHT sensor!");
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.println("ERROR: DHT");
        delay(1000);
        return;
    }
}

/*
 * Services Alarm Logic
 * Những hàm showXXX chỉ chịu trách nhiệm cập nhật LED + LCD (actuators)
 * Quy tắc tách trách nhiệm: quyết định trạng thái (service_business) -> gọi showXXX/startAlarm
*/
void showSafety() {
    stopAlarm(); // ensure buzzer off when safe
    digitalWrite(ledGreen, HIGH);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledRed, LOW);
    lcd.setCursor(0, 1);
    lcd.print("AN TOAN      ");
}

void showWarn() {
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledYellow, HIGH);
    digitalWrite(ledRed, LOW);
    lcd.setCursor(0, 1);
    lcd.print("BAT THUONG   ");
}

void showDanger() {
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledRed, HIGH);
    lcd.setCursor(0, 1);
    lcd.print("BAO DONG     ");
}

void setup() {
    serialSetUp(9600);

    dht11.begin();

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
    digitalWrite(ledGreen, LOW);
    digitalWrite(ledYellow, LOW);
    digitalWrite(ledRed, LOW);

    // Buzzer setup
    buzzerSetUp();
    stopAlarm();
}

/*
 * service_business (contract):
 *  - Inputs:
 *      temp: giá trị nhiệt độ hiện tại (đọc từ DHT11)
 *      tempCurrent: (hiện đang trùng với temp) dự phòng cho logic nâng cao
 *      resetPressed: true nếu nút Reset đang được nhấn
 *  - Behavior:
 *      - Nếu resetPressed: tắt còi và bật tắt LED để người dùng kiểm tra phần cứng
 *      - Nếu nhiệt thay đổi đủ lớn (temp_tb) hoặc khác mẫu trước: quyết định trạng thái
 *  - Side effects: cập nhật LED, LCD, và gọi start/stop alarm.
 *
 * Lưu ý về debounce: hiện dùng delay(500) nhỏ khi reset được nhấn để giảm bouncing.
 * Độ chính xác đọc DHT11: DHT11 có độ phân giải ~1°C và chậm; DHT22 sẽ cho kết quả tốt hơn.
 */
void service_business(float temp, float tempCurrent, bool resetPressed) {
    if (resetPressed) {
        // Turn on all three LEDs while reset is pressed
        digitalWrite(ledRed, HIGH);
        digitalWrite(ledYellow, HIGH);
        digitalWrite(ledGreen, HIGH);

        stopAlarm(); // silence buzzer while reset pressed

        // small debounce/delay
        delay(50);
        return; // go to next loop iteration
    }

    // Cập nhật chỉ khi nhiệt độ thay đổi đủ lớn so với mẫu trước để giảm flicker LCD
    if (tempCurrent != tempLast || tempCurrent - tempLast >= temp_tb) {
        if (temp <= safety) {
            showSafety();
            stopAlarm();
            tempLast = tempCurrent;
        } else if (temp > safety && temp <= warning) {
            startAlarm(ALARM_WARNING); // start before handle for immediate effect
            handleAlarm();
            showWarn();
            tempLast = tempCurrent;
        } else if (temp > warning || temp >= danger) {
            startAlarm(ALARM_DANGER);
            handleAlarm();
            showDanger();
            tempLast = tempCurrent;
        }
        Serial.print("Temp C=");
        Serial.println(temp, 2);

        lcd.setCursor(0, 0);
        lcd.print("Nhiet do: ");
        lcd.print(temp, 1); // one decimal
        lcd.print(static_cast<char>(223)); // degree symbol (may vary)
        lcd.print("C   ");
    }
}


/*
 * Main loop (quy tắc):
 *  - Đọc cảm biến (DHT11)
 *  - Kiểm tra lỗi đọc
 *  - Dùng millis() để giới hạn tần số đọc (1 giây)
 *  - Đọc trạng thái nút Reset (INPUT_PULLUP => pressed == LOW)
 *  - Gọi service_business để xử lý quyết định
 *  - Gọi handleAlarm để cập nhật bật/tắt buzzer (non-blocking)
 */
void loop() {
    unsigned long now = millis();
    float const temp = dht11.readTemperature();
    float const tempCurrent = temp;

    // Check if read failed
    checkNAN(temp);

    if (now - lastTime < 1000) {
        return; // đọc mỗi 1 giây
    }
    lastTime = now;

    // Reset button uses INPUT_PULLUP: pressed == LOW
    const bool resetPressed = (digitalRead(btnReset) == LOW);

    service_business(temp, tempCurrent, resetPressed);

    // continue alarm state machine each loop
    handleAlarm();
}