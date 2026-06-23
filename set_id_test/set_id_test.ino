#include <HardwareSerial.h>

// --- ピン設定 ---
#define RX_PIN 26
#define TX_PIN 27
#define TR_ctrl 4 // RS485のDE/RE制御ピン

const byte servoID = 2;
float radiansval = 0.0;
float radiansIncrement = 0.06;
float maxSpeed = 10000 * 0.732; // RPM相当の計算

void setup() {
  pinMode(TR_ctrl, OUTPUT);
  digitalWrite(TR_ctrl, LOW); // 初期状態は受信モード

  // Serial2をSTSサーボ用に設定
  Serial2.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);

  // ID設定とモード切替 (念のため遅延を入れる)
  sts_set_ID(1, servoID);
  delay(500);
  sts_set_operation_mode(servoID, 1); // 1: 速度制御モード
  delay(500);
}

void loop() {
  radiansval += radiansIncrement;
  if (radiansval > 2 * PI) radiansval = 0;

  // サインカーブによる速度変動
  int targetSpeed = int(sin(radiansval) * maxSpeed);

  sts_vel_target(servoID, targetSpeed);

  delay(20);
}

// --- 通信制御用ヘルパー関数 ---
void send_packet(byte* message, int length) {
  digitalWrite(TR_ctrl, HIGH);   // 送信モード開始
  Serial2.write(message, length);
  Serial2.flush();               // 送信完了待ち
  delayMicroseconds(50);         // RS485の切り替えマージン
  digitalWrite(TR_ctrl, LOW);    // 受信モードへ戻す
}

// --- 各種命令パケット作成関数 ---
void sts_set_operation_mode(byte id, int mode) {
  byte message[8] = {0xFF, 0xFF, id, 4, 3, 33, (byte)mode};
  
  byte checksum = 0;
  for (int i = 2; i < 7; i++) checksum += message[i];
  message[7] = ~checksum;

  send_packet(message, 8);
}

void sts_vel_target(byte id, int speed) {
  byte message[9];
  unsigned int value = (speed >= 0) ? ((unsigned int)speed & 0x7FFF) : (((unsigned int)(-speed) & 0x7FFF) | (1 << 15));

  message[0] = 0xFF;
  message[1] = 0xFF;
  message[2] = id;
  message[3] = 5;
  message[4] = 3;
  message[5] = 0x2E;
  message[6] = value & 0xFF;
  message[7] = (value >> 8) & 0xFF;

  byte checksum = 0;
  for (int i = 2; i < 8; i++) checksum += message[i];
  message[8] = ~checksum;

  send_packet(message, 9);
}

void sts_set_ID(byte id, int target_id) {
  byte message[8] = {0xFF, 0xFF, id, 4, 3, 5, (byte)target_id};
  
  byte checksum = 0;
  for (int i = 2; i < 7; i++) checksum += message[i];
  message[7] = ~checksum;

  send_packet(message, 8);
}