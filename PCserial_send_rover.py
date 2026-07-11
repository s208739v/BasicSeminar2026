import serial
import time
import threading
import paho.mqtt.client as mqtt
import json

"""吉川ローバ動作用プログラム (修正版)"""

class RobotCommunicator:
    def __init__(self, port, baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.running = False
        self.mqtt = None  # RoverMqtt インスタンスへの参照用
        
        # 最新の4軸の現在位置を格納するリスト (初期値はNone)
        self.current_positions = [None, None, None, None]
        
        # データのスレッド間安全性を確保するためのロック
        self.data_lock = threading.Lock()

    def start_receive_thread(self):
        """受信用のバックグラウンドスレッドを開始する"""
        self.running = True
        self.thread = threading.Thread(target=self._receive_loop, daemon=True)
        self.thread.start()

    def stop(self):
        """通信とスレッドを安全に終了する"""
        print("\nシステムを安全に停止しています...")
        self.running = False
        if hasattr(self, 'thread'):
            self.thread.join(timeout=1.0)
        if self.ser and self.ser.is_open:
            # 終了時にロボットを停止させるコマンドを送る場合はここに追記
            self.send_motor_positions(0, 0, 0, 0) # 例: 中央値で停止
            time.sleep(0.1)
            self.ser.close()
        print("シリアル通信を終了しました。")

    def _receive_loop(self):
        """【コア受信関数】裏で常にシリアルラインを監視し、パースする"""
        while self.running:
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore')
                    line = line.strip()
                    
                    if not line:
                        continue
                    if line.startswith("pos:"):
                        data_part = line.replace("pos:", "")
                        parts = data_part.split(',')
                        
                        if len(parts) == 4:
                            try:
                                positions = [int(p) for p in parts]
                                
                                # 【追加】いずれかの軸が -1 (エラー) の場合は無効データとして捨てる
                                if any(p == -1 for p in positions):
                                    continue
                                
                                # スレッド安全にメンバ変数を更新
                                with self.data_lock:
                                    self.current_positions = positions
                                
                                if self.mqtt:
                                    self.mqtt.publish_telemetry(*positions)
                            except ValueError:
                                pass # パースエラー対策
                    else:
                        # デバッグメッセージは通常出力
                        print(f"[ESP32 Log]: {line}")
                        
                else:
                    time.sleep(0.01) # CPU負荷を下げるためのウェイト
                    
            except Exception as e:
                print(f"受信エラーが発生しました: {e}")
                time.sleep(0.5)

    def get_latest_positions(self):
        """メイン処理側から安全に最新の現在位置を取得するための関数"""
        with self.data_lock:
            return list(self.current_positions)

    def send_motor_positions(self, p1, p2, p3, p4):
        """モーター位置指定コマンドの送信"""
        if not self.ser or not self.ser.is_open:
            return False
        # バリデーション (0〜4095)
        for pos in [p1, p2, p3, p4]:
            if not (0 <= pos <= 4095):
                print(f"無効な位置データです: {[p1, p2, p3, p4]}")
                return False
        try:
            command = f"{p1},{p2},{p3},{p4}\n"
            self.ser.write(command.encode('utf-8'))
            return True
        except Exception as e:
            print(f"送信エラー: {e}")
            return False
    
    def send_motor_pwms(self, p1, p2, p3, p4):
        """モーター位置指定コマンドの送信"""
        if not self.ser or not self.ser.is_open:
            return False
        # バリデーション (0〜4095)
        for pwm in [p1, p2, p3, p4]:
            if not (-1000 <= pwm <= 1000):
                print(f"無効な位置データです: {[p1, p2, p3, p4]}")
                return False
        try:
            command = f"{p1},{p2},{p3},{p4}\n"
            self.ser.write(command.encode('utf-8'))
            return True
        except Exception as e:
            print(f"送信エラー: {e}")
            return False

class RoverMqtt:
    def __init__(self, broker_host="localhost", port=1883):
        # paho-mqtt v2.x 系への互換性対応 (CallbackAPIを指定)
        try:
            self.client = mqtt.Client(callback_api_version=mqtt.CallbackAPIVersion.VERSION2)
        except AttributeError:
            self.client = mqtt.Client() # 旧バージョン(v1.x)用フォールバック
            
        self.broker_host = broker_host
        self.port = port
        self.rover = None 
        
        # コールバックの登録
        self.client.on_connect = self._on_connect
        self.client.on_message = self._on_message

    def connect_rover(self, rover_instance):
        """制御インスタンスと紐付け、通信を開始する"""
        self.rover = rover_instance
        self.client.connect(self.broker_host, self.port, 60)
        self.client.loop_start()

    def _on_connect(self, client, userdata, flags, rc, properties=None):
        print("MQTT Broker Connected.")
        self.client.subscribe("rover/command")

    def _on_message(self, client, userdata, msg):
        """PCからのコマンド受信時の処理"""
        if self.rover is None:
            return
            
        try:
            data = json.loads(msg.payload.decode())
            action = data.get("action")
            
            if action == "setspeed":
                self.rover.send_motor_positions(p1=data["m1"], p2=data["m2"], p3=data["m3"], p4=data["m4"])

            elif action == "setpwm":
                self.rover.send_motor_pwms(p1=data["m1"], p2=data["m2"], p3=data["m3"], p4=data["m4"])

            elif action == "stop":
                # 安全停止コマンドを送り、受信スレッドを止める
                self.rover.stop()

        except Exception as e:
            print(f"MQTT Command Error: {e}")

    def publish_telemetry(self, pos1, pos2, pos3, pos4):
        """PC（可視化側）へデータを送信するメソッド"""
        payload = {
            "pos1": pos1,
            "pos2": pos2,
            "pos3": pos3,
            "pos4": pos4
        }
        self.client.publish("rover/telemetry", json.dumps(payload))

    def disconnect(self):
        self.client.loop_stop()
        self.client.disconnect()
        print("MQTT通信を終了しました。")


if __name__ == "__main__":
    PORT = "COM3" 
    PORT="/dev/ttyUSB0"
    
    rover_mqtt = RoverMqtt()
    robot = RobotCommunicator(PORT, 115200)
    
    # 相互参照の紐付け
    robot.mqtt = rover_mqtt
    rover_mqtt.connect_rover(robot)
    
    print("ESP32と接続中... 起動を待ちます。")
    time.sleep(2) 
    
    # 受信スレッドを起動
    robot.start_receive_thread()
    print("受信スレッドを開始しました。")
    print("System Booted. Waiting for commands...")
    
    try:
        while True:
            # メインループで定期的に最新位置をログ出力したい場合はここを有効化
            # print(f"Current: {robot.get_latest_positions()}", end="\r")
            time.sleep(1)
            
    except KeyboardInterrupt:
        # 安全な終了シークエンスの呼び出し
        robot.stop()
        rover_mqtt.disconnect()
        print("Program safely closed.")