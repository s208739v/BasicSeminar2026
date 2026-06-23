#include <HardwareSerial.h>

#define RX_PIN 26 // お使いのボードに合わせて設定してください
#define TX_PIN 27 // お使いのボードに合わせて設定してください
#define TR_ctrl 4 // RS485のDE/RE制御ピン

// サーボIDと角度を設定
const byte servoID = 1; 
int centerPosition = 2048;
int goalPosition = 0;
float radiansval = 0.0; 
float radiansIncrement = 0.06; 
float maxPosition = 1600;

void setup() {
  Serial.begin(115200);
  
  // RS485制御ピンの設定
  pinMode(TR_ctrl, OUTPUT);
  digitalWrite(TR_ctrl, LOW); // 最初は受信モード（LOW）にしておく

  // Serial2の開始
  Serial2.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
  delay(100);
}

void loop() {
  radiansval += radiansIncrement;
  radiansval = (radiansval > 2 * PI) ? 0 : radiansval;

  goalPosition = centerPosition + int(sin(radiansval) * maxPosition);
  goalPosition = constrain(goalPosition, 0, 4095); // 範囲制限

  sts_moveToPos(servoID, goalPosition);
  
  Serial.print("TargetPosition: ");
  Serial.println(goalPosition);

  delay(20);
}

void sts_moveToPos(byte id, int position) {
  byte message[13];
  message[0] = 0xFF;
  message[1] = 0xFF;
  message[2] = id;
  message[3] = 9;
  message[4] = 3;
  message[5] = 42;
  message[6] = position & 0xFF;
  message[7] = (position >> 8) & 0xFF;
  message[8] = 0x00;
  message[9] = 0x00;
  message[10] = 0x00;
  message[11] = 0x00;

  byte checksum = 0;
  for (int i = 2; i < 12; i++) {
    checksum += message[i];
  }
  message[12] = ~checksum;

  // 【重要】RS485送信処理
  digitalWrite(TR_ctrl, HIGH);    // 送信モード開始
  Serial2.write(message, 13);      // 配列を一括送信
  Serial2.flush();                 // 送信完了まで待機
  delayMicroseconds(50);           // 送信後の安定待ち
  digitalWrite(TR_ctrl, LOW);     // 受信モードへ戻す
}