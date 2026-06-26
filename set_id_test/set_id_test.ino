#include <HardwareSerial.h>

// --- ピン設定 ---
#define RX_PIN 26
#define TX_PIN 27
#define TR_ctrl 4 // RS485のDE/RE制御ピン

const byte targetServoID = 4; // 変更後の新しいID
const byte currentID=1 ; //変更前のID
float radiansval = 0.0;
float radiansIncrement = 0.06;
float maxSpeed = 10000 * 0.732; // RPM相当の計算

// 関数のプロトタイプ宣言（C++の仕様上、setup前に宣言しておくと安全です）
void send_packet(byte* message, int length);
void sts_unlock_eprom(byte id);
void sts_lock_eprom(byte id);
void sts_set_ID(byte current_id, byte new_id);
void sts_set_operation_mode(byte id, int mode);
void sts_vel_target(byte id, int speed);

void setup() {
  Serial.begin(115200); // デバッグ用
  
  pinMode(TR_ctrl, OUTPUT);
  digitalWrite(TR_ctrl, LOW); // 初期状態は受信モード

  // Serial2をSTSサーボ用に設定
  Serial2.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);

  Serial.println("--- Setup Start ---");

  // --------------------------------------------------
  // 【重要】EPROMのロック解除とID書き換えプロセス
  // --------------------------------------------------
  //byte currentID = 1; // 工場出荷時や現在のID

  Serial.println("1. Unlocking EPROM...");
  sts_unlock_eprom(currentID);
  delay(500); // EPROMへの書き込みは少し時間がかかるため待つ

  Serial.printf("2. Changing ID from %d to %d...\n", currentID, targetServoID);
  sts_set_ID(currentID, targetServoID);
  delay(500);

  Serial.println("3. Locking EPROM with new ID...");
  //sts_lock_eprom(targetServoID); // 【注意】書き換え後の「新しいID」に対してロック命令を送る
  delay(500);

  // --------------------------------------------------
  // 動作モード設定
  // --------------------------------------------------
  Serial.println("4. Setting Operation Mode (Velocity Control)...");
  sts_set_operation_mode(targetServoID, 1); // 1: 速度制御モード
  delay(100);

  Serial.println("--- Setup Complete ---");
}

void loop() {
  radiansval += radiansIncrement;
  if (radiansval > 2 * PI) radiansval = 0;

  // サインカーブによる速度変動
  int targetSpeed = int(sin(radiansval) * maxSpeed);

  sts_vel_target(targetServoID, targetSpeed);

  delay(20);
}

// --- 通信制御用ヘルパー関数 ---
void send_packet(byte* message, int length) {
  // 受信バッファにゴミが残っていればクリア
  while(Serial2.available()) Serial2.read();

  digitalWrite(TR_ctrl, HIGH);   // 送信モード開始
  Serial2.write(message, length);
  Serial2.flush();               // 送信完了待ち
  delayMicroseconds(50);         // RS485の切り替えマージン (物理抜け待ち)
  digitalWrite(TR_ctrl, LOW);    // 受信モードへ戻す
}

// --- 各種命令パケット作成関数 ---

// EPROMのロック解除 (アドレス 55 / 0x37)
void sts_unlock_eprom(byte id) {
  byte message[8] = {0xFF, 0xFF, id, 4, 3, 55, 0};
  
  byte checksum = 0;
  for (int i = 2; i < 7; i++) checksum += message[i];
  message[7] = ~checksum;

  send_packet(message, 8);
}

// EPROMのロック (アドレス 55 / 0x37)
void sts_lock_eprom(byte id) {
  byte message[8] = {0xFF, 0xFF, id, 4, 3, 55, 1};
  
  byte checksum = 0;
  for (int i = 2; i < 7; i++) checksum += message[i];
  message[7] = ~checksum;

  send_packet(message, 8);
}

// ID変更 (アドレス 5)
void sts_set_ID(byte current_id, byte new_id) {
  byte message[8] = {0xFF, 0xFF, current_id, 4, 3, 5, new_id};
  
  byte checksum = 0;
  for (int i = 2; i < 7; i++) checksum += message[i];
  message[7] = ~checksum;

  send_packet(message, 8);
}

// 動作モード変更 (アドレス 33 / 0x21)
void sts_set_operation_mode(byte id, int mode) {
  byte message[8] = {0xFF, 0xFF, id, 4, 3, 33, (byte)mode};
  
  byte checksum = 0;
  for (int i = 2; i < 7; i++) checksum += message[i];
  message[7] = ~checksum;

  send_packet(message, 8);
}

// 速度指定 (アドレス 46 / 0x2E)
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