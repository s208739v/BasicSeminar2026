#include <HardwareSerial.h>

// --- ピン設定 ---
#define RX_PIN 26
#define TX_PIN 27 
#define TR_ctrl 4

// --- サーボID（4輪分） ---
const uint8_t MOTOR_ID[4] = {1, 2, 3, 4};

// --- STS3215制御クラス ---
class STS3215 {
private:
    HardwareSerial* _serial;
    int _tr_pin;

    void send_packet(uint8_t* message, int length) {
        while (_serial->available()) _serial->read();

        digitalWrite(_tr_pin, HIGH);
        _serial->write(message, length);
        _serial->flush(); 
        
        // 【修正】半二重RS485の送信完了に必要な物理ウェイトを追加
        delayMicroseconds(10);

        digitalWrite(_tr_pin, LOW);
        delayMicroseconds(50);
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

        const int MAX_RETRY = 3; // 最初の1回 + リトライ2回

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

                    if (receivedCount >= expected_len) {
                        break;
                    }
                }
            }

            if (receivedCount == expected_len) {
                uint8_t sum = 0;
                for (int i = 2; i < expected_len - 1; i++) sum += response[i];
                if ((uint8_t)(~sum) == response[expected_len - 1]) {
                    // チェックサム一致
                    long result = 0;
                    for (int i = 0; i < length; i++) {
                        result |= ((long)response[5 + i] << (8 * i));                    
                    }
                    return result;
                }
            }
            // 【重要】バス上に残った半端なデータを掃除してから再送する
            while (_serial->available()) _serial->read();

        }
        return -1;
    }

    int to_signed16(unsigned int val) {
        if (val > 32767) {
            return (int)val - 65536;
        }
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
        if (pwm_value < 0) {
            value = abs(pwm_value) + 1024; 
        } else {
            value = pwm_value;
        }

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
        if (speed >= 0) {
            value = speed & 0x7FFF;
        } else {
            value = ((-speed) & 0x7FFF) | (1 << 15); 
        }

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
    if (val == -1) return -1; // 通信エラー

    // STS3215の仕様に合わせた符号判定
    // 15ビット目（0x8000）が1ならマイナス回転
    if (val & 0x8000) {
        int16_t speed_abs = val & 0x7FFF; // 下位15ビット（速度の大きさ）を抽出
        return -speed_abs;                // C++の正しい負の数にする
    } else {
        return val & 0x7FFF;              // プラスの回転
    }
}
 
 
};

bool is_experiment_running = false;


struct MotorConfig {
    char mode;
    float T;
    float freq0;   // 初期周波数
    float freq1;      //終了周波数
    float amp;
    float base;
    int last_PWM;
    int force_send_count; // 追加：強制送信の残り回数
};

MotorConfig motors[4];
STS3215 servo(Serial2, TR_ctrl);

union Packet {
    struct {
        uint8_t header[2]; 
        float speed[4];    
    } data;
    uint8_t bytes[18]; 
};

Packet packet;

void setup() {
  pinMode(TR_ctrl, OUTPUT);
  digitalWrite(TR_ctrl, LOW); 

  Serial.begin(921600);
  Serial.setTimeout(5); // 【重要】コマンドパースのブロック防止

  packet.data.header[0] = 0xAA;
  packet.data.header[1] = 0xBB;

  servo.begin(1000000, RX_PIN, TX_PIN);
  delay(100);

  for (int i = 0; i < 4; i++) {
    motors[i] = {'C', 0.0, 0.0, 0.0, 0.0, 0.0};
  }

  // サーボを輪番で速度閉ループモード(1)へ移行
  for (int i = 0; i < 4; i++) {
    servo.sts_set_operation_mode(MOTOR_ID[i], 2); 
    delay(50); // 【重要】設定が確実に書き込まれるよう少し長めに待機
  }

  // 【重要】安定通信化により、この初期回転指令が確実に通るようになります
  //servo.sts_vel_target(1, 10);
  servo.sts_pwm_target(1,100);

  delay(5000);
  Serial.println("System Start: Combined Version");
  Serial.printf("sizeof(Packet)=%d\n", sizeof(Packet));
}

void set_motor_speed(float t) {
    for (int i = 0; i < 4; i++) {
        int target = 0;
        float local_t = t;

        float k = 0.0f;
        if (motors[i].T > 0.0f) {
            k = (motors[i].freq1 - motors[i].freq0) / motors[i].T;
        }

        if (motors[i].mode == 'V') {
            if (motors[i].T > 0.0f) {
                local_t = fmod(t, motors[i].T);
            }
            float phase = 2.0 * PI * (motors[i].freq0 * local_t + 0.5 * k * local_t * local_t);
            float my_sin = sin(phase);
            target = (int)(motors[i].base + motors[i].amp * my_sin);
        } else {
            target = (int)motors[i].base;
        }

        // 値が変わった場合、または強制送信期間中なら送信する
        if (target != motors[i].last_PWM || motors[i].force_send_count > 0) {
            servo.sts_pwm_target(MOTOR_ID[i], target);
            motors[i].last_PWM = target;
            if (motors[i].force_send_count > 0) {
                motors[i].force_send_count--;
            }
        }
    }
}


void loop() {
  checkAndParseCommand();

  if (is_experiment_running) {
    static unsigned long last_time = 0;
    unsigned long current_time = micros();

    // 2ms（2000us）に正しく修正
    if (current_time - last_time >= 10000) { 
      last_time = current_time;

      static float t = 0.0;
      t += 0.01;

      set_motor_speed(t);
      loop_pi_send();
    }
  } else {
    for (int i = 0; i < 4; i++) {
      //servo.sts_vel_target(MOTOR_ID[i], 0);
      servo.sts_pwm_target(MOTOR_ID[i], 0);
    }
    delay(100);
  }
}


void loop_pi_send() {
    float pos1 = servo.sts_readPos(MOTOR_ID[0]);
    float pos2 = servo.sts_readPos(MOTOR_ID[1]);
    float pos3 = servo.sts_readPos(MOTOR_ID[2]);
    float pos4 = servo.sts_readPos(MOTOR_ID[3]);


    uint8_t buf[18];
    buf[0] = 0xAA;
    buf[1] = 0xBB;
    memcpy(&buf[2],  &pos1, 4);
    memcpy(&buf[6],  &pos2, 4);
    memcpy(&buf[10], &pos3, 4);
    memcpy(&buf[14], &pos4, 4);

    uint8_t chk = 0;
    for (int i = 2; i < 18; i++) chk ^= buf[i]; // XORチェックサム

    Serial.write(buf, 18);
    Serial.write(chk); // 19バイト目として送信
}

void checkAndParseCommand() {
    if (Serial.available() > 0) {
        String cmd = Serial.readStringUntil('\n');
        cmd.trim(); 

        if (cmd.startsWith("START,")) { 
            String dataStr = cmd.substring(6);
            float params[24];
            int paramCount = 0;
            int pIndex = 0;
            while (pIndex >= 0 && paramCount < 24) {
                int nextIndex = dataStr.indexOf(',', pIndex);
                String token;
                if (nextIndex >= 0) {
                    token = dataStr.substring(pIndex, nextIndex);
                    pIndex = nextIndex + 1;
                } else {
                    token = dataStr.substring(pIndex);
                    pIndex = -1;
                }
                token.trim();
                if (token.length() > 0) {
                    params[paramCount++] = token.toFloat();
                }
            }

            if (paramCount == 24) {
                motors[0] = {((int)params[0] == 1) ? 'V' : 'C', params[1], params[2], params[3], params[4], params[5], 10000, 4};  //mode freq k amp base
                motors[1] = {((int)params[6] == 1) ? 'V' : 'C', params[7], params[8], params[9], params[10], params[11], 10000, 4}; 
                motors[2] = {((int)params[12] == 1) ? 'V' : 'C', params[13], params[14], params[15], params[16], params[17], 10000, 4}; 
                motors[3] = {((int)params[18] == 1) ? 'V' : 'C', params[19], params[20], params[21], params[22], params[23], 10000, 4}; 
                is_experiment_running = true;
                Serial.println("ACK:START_OK");
            } else {
                Serial.printf("ERR:INVALID_PARAM_COUNT(%d/20)\n", paramCount);
            }
        }
        else if (cmd.startsWith("STOP")) {
            is_experiment_running = false;
            Serial.println("ACK:STOP_OK");
        }
    }
}