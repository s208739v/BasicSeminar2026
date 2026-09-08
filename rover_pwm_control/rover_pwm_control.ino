#include <HardwareSerial.h>

// ==================== ピン設定 ====================
#define RX_PIN 26
#define TX_PIN 27
#define TR_ctrl 4     // RS485のDE/RE制御ピン

// ==================== モータ設定（4輪分） ====================
const uint8_t MOTOR_ID[4] = {1, 2, 3, 4};
const int NUM_MOTORS = 4;

// ==================== 急発進防止パラメータ ====================
// 指令PWMと現在の追従PWMとの差が大きいときに、1回の更新でどこまで変化させるかを制限する
const int MAX_PWM_STEP_PER_UPDATE = 50;         // 1更新あたりの最大PWM変化量 (-1000〜1000スケール)
const unsigned long UPDATE_INTERVAL_MS = 20;    // 追従PWMの更新周期 [ms]

// ==================== 現在位置送信パラメータ ====================
const unsigned long REPORT_INTERVAL_MS = 100;   // 現在位置をPCへ送信する周期 [ms]

// ==================== STS3215制御クラス ====================
class STS3215 {
private:
    HardwareSerial* _serial;
    int _tr_pin;

    void send_packet(uint8_t* message, int length) {
        while (_serial->available()) _serial->read();

        digitalWrite(_tr_pin, HIGH);
        _serial->write(message, length);
        _serial->flush();

        digitalWrite(_tr_pin, LOW);
        delayMicroseconds(50);

        // 抵抗ミックス回路では自分の送信がRXにも回り込むため、
        // 送った分(length)だけエコーとして読み捨てる
        discard_echo(length);
    }

    bool discard_echo(int expected_length) {
        int count = 0;
        unsigned long startTime = micros();
        // 安全弁として数十us程度だけ見る（待つためではなく、ハング防止のため）
        while (count < expected_length && (micros() - startTime) < 50) {
            if (_serial->available()) {
                _serial->read();
                count++;
            }
        }
        if (count != expected_length) {
            Serial.printf("[ECHO WARN] expected=%d got=%d\n", expected_length, count);
            return false;
        }
        return true;
    }

    long sts_read_register(uint8_t id, uint8_t address, uint8_t length) {
        uint8_t message[8];
        message[0] = 0xFF;
        message[1] = 0xFF;
        message[2] = id;
        message[3] = 4;
        message[4] = 0x02;
        message[5] = address;
        message[6] = length;

        uint8_t checksum = 0;
        for (int i = 2; i < 7; i++) checksum += message[i];
        message[7] = ~checksum;

        uint8_t expected_len = 6 + length;
        const int MAX_RETRY = 3;

        for (int attempt = 0; attempt < MAX_RETRY; attempt++) {
            send_packet(message, 8);

            uint8_t response[16];
            int receivedCount = 0;
            unsigned long startTime = micros();

            while ((micros() - startTime) < 500) {
                if (_serial->available()) {
                    uint8_t b = _serial->read();

                    if (receivedCount == 0) {
                        if (b == 0xFF) response[receivedCount++] = b;
                    } else if (receivedCount == 1) {
                        if (b == 0xFF) response[receivedCount++] = b;
                        else receivedCount = 0;
                    } else if (receivedCount == 2) {
                        if (b == id) response[receivedCount++] = b;
                        else if (b != 0xFF) receivedCount = 0;
                    } else {
                        response[receivedCount++] = b;
                    }

                    if (receivedCount >= expected_len) break;
                }
            }

            if (receivedCount == expected_len) {
                uint8_t sum = 0;
                for (int i = 2; i < expected_len - 1; i++) sum += response[i];
                if ((uint8_t)(~sum) == response[expected_len - 1]) {
                    long result = 0;
                    for (int i = 0; i < length; i++) {
                        result |= ((long)response[5 + i] << (8 * i));
                    }
                    return result;
                }
            }
            while (_serial->available()) _serial->read();
        }
        return -1;
    }

    int to_signed16(unsigned int val) {
        if (val > 32767) return (int)val - 65536;
        return (int)val;
    }

public:
    STS3215(HardwareSerial& serial, int tr_pin) {
        _serial = &serial;
        _tr_pin = tr_pin;
    }

    void begin(unsigned long baudrate = 1000000, int rx_pin = 16, int tx_pin = 17) {
        pinMode(_tr_pin, OUTPUT);
        digitalWrite(_tr_pin, LOW);
        _serial->begin(baudrate, SERIAL_8N1, rx_pin, tx_pin);
    }

    float sts_readPos(uint8_t id) {
        long val = sts_read_register(id, 56, 2);
        if (val == -1) return -1.0;
        return (float)val * 360.0 / 4095.0;
    }

    float sts_readSpeed(uint8_t id) {
        long val = sts_read_register(id, 58, 2);
        if (val == -1) return -1.0;
        return (float)to_signed16(val) * 360.0 / 4095.0 * PI / 180.0;
    }

    int sts_readLoad(uint8_t id) {
        long val = sts_read_register(id, 60, 2);
        if (val == -1) return -1;
        return to_signed16(val);
    }

    float sts_readVoltage(uint8_t id) {
        long val = sts_read_register(id, 62, 1);
        if (val == -1) return -1.0;
        return (float)val * 0.1;
    }

    int sts_readTemp(uint8_t id) {
        return (int)sts_read_register(id, 63, 1);
    }

    float sts_readCurrent(uint8_t id) {
        long val = sts_read_register(id, 69, 2);
        if (val == -1) return -1.0;
        return (float)val * 6.5;
    }

    void sts_set_operation_mode(uint8_t id, uint8_t mode) {
        uint8_t message[8] = { 0xFF, 0xFF, id, 4, 3, 33, mode };
        uint8_t checksum = 0;
        for (int i = 2; i < 7; i++) checksum += message[i];
        message[7] = ~checksum;
        send_packet(message, 8);
    }

    void sts_pwm_target(uint8_t id, int pwm_value) {
        if (pwm_value > 1000) pwm_value = 1000;
        if (pwm_value < -1000) pwm_value = -1000;

        unsigned int value;
        if (pwm_value < 0) value = abs(pwm_value) + 1024;
        else value = pwm_value;

        uint8_t message[9] = {
            0xFF, 0xFF, id, 5, 3, 0x2C,
            (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF), 0
        };

        uint8_t checksum = 0;
        for (int i = 2; i < 8; i++) checksum += message[i];
        message[8] = ~checksum;
        send_packet(message, 9);
    }

    void sts_vel_target(uint8_t id, int speed) {
        unsigned int value;
        if (speed >= 0) value = speed & 0x7FFF;
        else value = ((-speed) & 0x7FFF) | (1 << 15);

        uint8_t message[9] = {
            0xFF, 0xFF, id, 5, 3, 0x2E,
            (uint8_t)(value & 0xFF), (uint8_t)((value >> 8) & 0xFF), 0
        };

        uint8_t checksum = 0;
        for (int i = 2; i < 8; i++) checksum += message[i];
        message[8] = ~checksum;
        send_packet(message, 9);
    }

    int16_t sts_readRawSpeed(uint8_t id) {
        long val = sts_read_register(id, 58, 2);
        if (val == -1) return -1;

        if (val & 0x8000) {
            int16_t speed_abs = val & 0x7FFF;
            return -speed_abs;
        } else {
            return val & 0x7FFF;
        }
    }
};

// ==================== グローバル変数 ====================
STS3215 servo(Serial2, TR_ctrl);

int targetPWM[NUM_MOTORS];    // ユーザーが指令した最終目標PWM (-1000〜1000)
float commandPWM[NUM_MOTORS]; // 実際にサーボへ毎周期送る、なめらかに追従中のPWM

unsigned long lastUpdateTime = 0;
unsigned long lastReportTime = 0;

void setup() {
    Serial.begin(115200);

    pinMode(TR_ctrl, OUTPUT);
    digitalWrite(TR_ctrl, LOW);

    servo.begin(1000000, RX_PIN, TX_PIN);

    // 4輪ともPWM(オープンループ)モードに設定
    for (int i = 0; i < NUM_MOTORS; i++) {
        servo.sts_set_operation_mode(MOTOR_ID[i], 2); // 2 = PWMモード
        delay(50);
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        targetPWM[i] = 0;
        commandPWM[i] = 0;
        servo.sts_pwm_target(MOTOR_ID[i], 0); // 起動時は必ず停止状態から開始
    }

    Serial.println("Ready. Enter 4 target PWMs as: p1,p2,p3,p4 (each -1000 to 1000)");
    Serial.println("Example: 300,300,-300,-300");
}

void loop() {
    handleSerialInput();

    unsigned long now = millis();
    if (now - lastUpdateTime >= UPDATE_INTERVAL_MS) {
        lastUpdateTime = now;
        updateMotorPWMs();
    }

    if (now - lastReportTime >= REPORT_INTERVAL_MS) {
        lastReportTime = now;
        reportCurrentPositions();
    }
}

// "p1,p2,p3,p4" 形式（-1000〜1000）で4輪分の目標PWMを一括受信する
void handleSerialInput() {
    if (Serial.available() <= 0) return;

    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.length() == 0) return;

    int values[NUM_MOTORS];
    int idx = 0;
    int startPos = 0;

    for (int i = 0; i <= (int)line.length(); i++) {
        if (i == (int)line.length() || line.charAt(i) == ',') {
            if (idx < NUM_MOTORS) {
                String token = line.substring(startPos, i);
                values[idx] = token.toInt();
                idx++;
            }
            startPos = i + 1;
        }
    }

    if (idx != NUM_MOTORS) {
        Serial.println("Invalid format! Use: p1,p2,p3,p4 (e.g. 300,300,-300,-300)");
        return;
    }

    for (int i = 0; i < NUM_MOTORS; i++) {
        if (values[i] < -1000 || values[i] > 1000) {
            Serial.print("Motor "); Serial.print(MOTOR_ID[i]);
            Serial.println(": Invalid range! Enter -1000 to 1000. (skipped)");
            continue;
        }
        // 目標値だけを更新。実際にサーボへ送るcommandPWMは
        // updateMotorPWMs()の中で少しずつ近づけていく。
        targetPWM[i] = values[i];
    }
}

// 目標PWM(targetPWM)に向かって、追従PWM(commandPWM)を
// 1周期あたり最大 MAX_PWM_STEP_PER_UPDATE だけ近づけてからサーボへ送信する。
// これにより、指令値が急に大きく変わった場合でも急発進せず、なめらかに追従する。
void updateMotorPWMs() {
    for (int i = 0; i < NUM_MOTORS; i++) {
        float diff = targetPWM[i] - commandPWM[i];

        if (fabs(diff) < 0.5f) continue; // ほぼ到達済みなら何もしない

        float step;
        if (fabs(diff) > MAX_PWM_STEP_PER_UPDATE) {
            step = (diff > 0) ? MAX_PWM_STEP_PER_UPDATE : -MAX_PWM_STEP_PER_UPDATE;
        } else {
            step = diff;
        }

        commandPWM[i] += step;

        int pwmValue = (int)commandPWM[i];
        pwmValue = constrain(pwmValue, -1000, 1000);

        servo.sts_pwm_target(MOTOR_ID[i], pwmValue);
    }
}

// 4輪すべての実際の現在位置をサーボから読み取り、
// "pos:p1,p2,p3,p4" 形式（各0-4095、読み取り失敗時は-1）でPCへ送信する。
void reportCurrentPositions() {
    int rawPos[NUM_MOTORS];

    for (int i = 0; i < NUM_MOTORS; i++) {
        float posDeg = servo.sts_readPos(MOTOR_ID[i]);
        if (posDeg < 0) {
            rawPos[i] = -1; // 読み取り失敗
        } else {
            rawPos[i] = (int)(posDeg * 4095.0f / 360.0f);
            rawPos[i] = constrain(rawPos[i], 0, 4095);
        }
    }

    Serial.print("pos:");
    for (int i = 0; i < NUM_MOTORS; i++) {
        Serial.print(rawPos[i]);
        if (i < NUM_MOTORS - 1) Serial.print(",");
    }
    Serial.println();
}
