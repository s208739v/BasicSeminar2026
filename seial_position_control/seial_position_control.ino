#include <HardwareSerial.h>

#define RX_PIN 26 
#define TX_PIN 27 
#define TR_ctrl 4 // RS485のDE/RE制御ピン

int goalPosition = 2048; // 初期位置

void setup() {
  Serial.begin(115200);
  
  // RS485制御ピンの設定
  pinMode(TR_ctrl, OUTPUT);
  digitalWrite(TR_ctrl, LOW); 

  // Serial1, 2の初期化
  Serial1.begin(115200, SERIAL_8N1, 18, 19);
  Serial2.begin(1000000, SERIAL_8N1, RX_PIN, TX_PIN);
  
  Serial.println("Ready. Enter a position (0-4095) to move the servo.");
}

void loop() {
  // シリアル通信で値が届いているか確認
  if (Serial.available() > 0) {
    // 文字列として受信し、整数に変換
    int input = Serial.parseInt();
    
    // 0〜4095の範囲内かチェック
    if (input >= 0 && input <= 4095) {
      goalPosition = input;
      Serial.print("Moving to: ");
      Serial.println(goalPosition);
      
      // 送信
      sts_moveToPos(1, goalPosition);
    } else {
      Serial.println("Invalid range! Enter 0-4095.");
    }
  }
    // シリアル通信で値が届いているか確認
  if (Serial1.available() > 0) {
    // 文字列として受信し、整数に変換
    int input = Serial1.parseInt();
    
    // 0〜4095の範囲内かチェック
    if (input >= 0 && input <= 4095) {
      goalPosition = input;
      Serial1.print("Moving to: ");
      Serial1.println(goalPosition);
      
      // 送信
      sts_moveToPos(1, goalPosition);
    } else {
      Serial1.println("Invalid range! Enter 0-4095.");
    }
  }
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
  message[8] = 0x00; // 時間情報（必要に応じて調整）
  message[9] = 0x00;
  message[10] = 0x00; // 速度情報
  message[11] = 0x00;

  byte checksum = 0;
  for (int i = 2; i < 12; i++) {
    checksum += message[i];
  }
  message[12] = ~checksum;

  digitalWrite(TR_ctrl, HIGH);
  Serial2.write(message, 13);
  Serial2.flush();
  delayMicroseconds(50);
  digitalWrite(TR_ctrl, LOW);
}