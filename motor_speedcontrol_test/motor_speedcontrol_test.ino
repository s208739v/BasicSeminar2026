// サーボIDと角度を設定
const byte servoID = 1; // サーボのID
int centerPosition = 2048;//サーボ位置のセンター値
int goalPosition = 0;//サーボの目標位置
float radiansval = 0.0; // サインカーブ算出用のラジアン値
float radiansIncrement = 0.06; // ループ毎のラジアン値の増加量
float maxPosition = 1600;// センター値を2048としたとき、±どこまで振るか(0-2048)
float maxSpeed = 10000*0.732; //stepヲ入力　最初の数字がRPM

void setup() {
  // Serial2の開始（ボーレートを1000000に設定）
  Serial.begin(1000000);
  delay(100); // サーボの初期化待ち
  //sts_set_angle_limit(servoID, 0, 0); //速度指定モードの時は、角度制限をいじる必要あり
  delay(500);
  sts_set_operation_mode(servoID, 1); //速度指定
  delay(1000);
}

void loop() {
  // サインカーブ用の値を算出
  radiansval += radiansIncrement;  //ラジアン値を増加
  radiansval = (radiansval > 2 * PI) ? 0 : radiansval; //ラジアン値が2πを超えたら0にリセット

  // サーボの値をセットする
  goalPosition = centerPosition + int(sin(radiansval) * maxPosition);
  int targetSpeed = int(sin(radiansval) * maxSpeed);
  //targetSpeed = int(maxSpeed);

  // サーボにコマンドを送信
  //sts_moveToPos(servoID, goalPosition);
  sts_vel_target(servoID, targetSpeed) ;
  delay(20);
}


void sts_set_operation_mode(byte id, int mode){

  // コマンドパケットを作成 モード変更
  byte message[8];
  message[0] = 0xFF;  // ヘッダ 0xFF固定
  message[1] = 0xFF;  // ヘッダ 0xFF固定
  message[2] = id;    // サーボID
  message[3] = 4;     // パケットデータ長　(IDからチェックサム手前まで)-1
  message[4] = 3;     // コマンド（3は書き込み命令）
  message[5] = 33;    // レジスタ先頭番号
  message[6] = mode;  // モード番号指定

  // チェックサムの計算
  byte checksum = 0;
  for (int i = 2; i < 7; i++) {
    checksum += message[i];
  }
  message[7] = ~checksum; // チェックサム

  // コマンドパケットを送信
  for (int i = 0; i < 8; i++) {
    Serial.write(message[i]);
  }
}

void sts_vel_target(byte id, int speed) {
  byte message[9];
  unsigned int value;

  if (speed >= 0) {
    value = (unsigned int)speed & 0x7FFF;  // BIT15=0 正転
  } else {
    value = ((unsigned int)(-speed) & 0x7FFF) | (1 << 15); // BIT15=1 逆転
  }

  message[0] = 0xFF;
  message[1] = 0xFF;
  message[2] = id;
  message[3] = 5; // データ長
  message[4] = 3; // WRITE_DATA
  message[5] = 0x2E; // Goal Speed アドレス
  message[6] = value & 0xFF;
  message[7] = (value >> 8) & 0xFF;

  byte checksum = 0;
  for (int i = 2; i < 8; i++) checksum += message[i];
  message[8] = ~checksum;

  for (int i = 0; i < 9; i++) Serial.write(message[i]);
}
void sts_set_angle_limit(byte id, int cwLimit, int ccwLimit) {
  byte message[11];

  message[0] = 0xFF;          // ヘッダ
  message[1] = 0xFF;          // ヘッダ
  message[2] = id;            // サーボID
  message[3] = 7;             // データ長（INSTRUCTION + PARAMETER）= 7
  message[4] = 3;             // WRITE_DATA命令
  message[5] = 0x09;          // 書き込み開始アドレス（CW Angle Limit L）

  // CW Angle Limit（下位・上位）
  message[6] = cwLimit & 0xFF;
  message[7] = (cwLimit >> 8) & 0xFF;

  // CCW Angle Limit（下位・上位）
  message[8] = ccwLimit & 0xFF;
  message[9] = (ccwLimit >> 8) & 0xFF;

  // チェックサム計算
  byte checksum = 0;
  for (int i = 2; i < 10; i++) {
    checksum += message[i];
  }
  message[10] = ~checksum;

  // パケット送信
  for (int i = 0; i < 11; i++) {
    Serial.write(message[i]);
  }
}


void sts_moveToPos(byte id, int position) {
  // コマンドパケットを作成
  byte message[13];
  message[0] = 0xFF;  // ヘッダ 0xFF固定
  message[1] = 0xFF;  // ヘッダ 0xFF固定
  message[2] = id;    // サーボID
  message[3] = 9;     // パケットデータ長　(IDからチェックサム手前まで)-1
  message[4] = 3;     // コマンド（3は書き込み命令）
  message[5] = 42;    // レジスタ先頭番号
  message[6] = position & 0xFF; // 位置情報バイト下位
  message[7] = (position >> 8) & 0xFF; // 位置情報バイト上位
  message[8] = 0x00;  // 時間情報バイト下位
  message[9] = 0x00;  // 時間情報バイト上位
  message[10] = 0x00; // 速度情報バイト下位
  message[11] = 0x00; // 速度情報バイト上位

  // チェックサムの計算
  byte checksum = 0;
  for (int i = 2; i < 12; i++) {
    checksum += message[i];
  }
  message[12] = ~checksum; // チェックサム

  // コマンドパケットを送信
  for (int i = 0; i < 13; i++) {
    Serial.write(message[i]);
  }
}
