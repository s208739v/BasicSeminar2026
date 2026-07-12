import paho.mqtt.client as mqtt
import json
import time
import threading
from inputs import get_gamepad
 
BROKER_HOST = "raspberrypi.local"
 
# ==================== PWMパラメータ ====================
PWM_MAX = 250                  # Arduino側のPWM最大値（-1000〜1000）
STICK_MAX = 32767               # ゲームパッドのスティック生値の最大値
DEADZONE = 2000                 # 不感帯（この値未満は0とみなす）
PWM_SCALE = PWM_MAX / STICK_MAX # スティック生値 → PWM値への変換係数
 
# グローバル状態管理（テレメトリはPWM制御では必須ではないが、監視表示用に維持）
current_pos = [None, None, None, None]
 
# スティックの現在の生値を保持
stick_inputs = {
    "ABS_Y": 0,
    "ABS_X": 0,
    "ABS_RY": 0,
    "ABS_RX": 0
}
 

# 【重要】ゲームパッドの最終受信時刻（初期値は現在時刻）
last_gamepad_recv_time = time.time()
 
 
def on_connect(client, userdata, flags, rc, properties=None):
    print(f"MQTT Broker に接続しました: {BROKER_HOST}")
    client.subscribe("rover/telemetry")
 
 
def on_message(client, userdata, msg):
    global current_pos
    try:
        data = json.loads(msg.payload.decode())
        print(f"\r[Telemetry] M1:{data['pos1']}, M2:{data['pos2']}, M3:{data['pos3']}, M4:{data['pos4']}", end="")
        current_pos = [data['pos1'], data['pos2'], data['pos3'], data['pos4']]
    except Exception as e:
        print(f"\nデータ受信エラー: {e}")
 
 
def send_command(client, m1, m2, m3, m4):
    # PWM値なので範囲は-1000〜1000にクランプする
    m1 = max(-PWM_MAX, min(PWM_MAX, int(m1)))
    m2 = max(-PWM_MAX, min(PWM_MAX, int(m2)))
    m3 = max(-PWM_MAX, min(PWM_MAX, int(m3)))
    m4 = max(-PWM_MAX, min(PWM_MAX, int(m4)))
 
    payload = {
        "action": "setpwm",
        "m1": m1,
        "m2": m2,
        "m3": m3,
        "m4": m4
    }
    client.publish("rover/command", json.dumps(payload))
 
 
def gamepad_listener_loop():
    """【別スレッド】ゲームパッドのイベントを監視"""
    global stick_inputs, last_gamepad_recv_time
    try:
        while True:
            events = get_gamepad()
            # イベントが届いたら、最終受信時刻を更新
            last_gamepad_recv_time = time.time()
 
            for event in events:
                if event.code in stick_inputs:
                    # 不感帯（デッドゾーン）処理
                    if abs(event.state) < DEADZONE:
                        stick_inputs[event.code] = 0
                    else:
                        stick_inputs[event.code] = event.state
    except Exception as e:
        print(f"\n[CRITICAL] ゲームパッドの通信が物理的に切断されました: {e}")
        # 例外を検知したら即座に入力を全リセット
        for k in stick_inputs:
            stick_inputs[k] = 0
 
 
def control_loop(client):
    global stick_inputs, last_gamepad_recv_time
 
    print("制御開始。")
 
    INTERVAL = 0.05  # 送信周期 [秒]（Arduino側のSERIAL_TIMEOUT_MS=300msより十分短く保つこと）
 
    try:
        while True:
            start_time = time.time()
 
            # ゲームパッドのタイムアウト監視（物理切断・スレッド停止対策）
            if time.time() - last_gamepad_recv_time > 1.5:
                for k in stick_inputs:
                    stick_inputs[k] = 0
 
            # 【PWM化】スティックの傾き量をそのままPWM値に変換して即座に送信する
            # 位置の積分やなめらか追従処理は行わない（速度制御ではないため不要）
            m1_pwm = stick_inputs["ABS_Y"] * PWM_SCALE
            m2_pwm = stick_inputs["ABS_X"] * PWM_SCALE
            m3_pwm = stick_inputs["ABS_RY"] * PWM_SCALE
            m4_pwm = stick_inputs["ABS_RX"] * PWM_SCALE
 
            # 指令送信
            send_command(client, m1_pwm, m2_pwm, m3_pwm, m4_pwm)
 
            # 周期維持ウェイト
            elapsed = time.time() - start_time
            sleep_time = max(0, INTERVAL - elapsed)
            time.sleep(sleep_time)
 
    except KeyboardInterrupt:
        print("\n制御ループを停止します。停止コマンドを送信します。")
        # 終了時は必ず全モータPWM=0を送っておく（Arduino側タイムアウトに任せきりにしない）
        send_command(client, 0, 0, 0, 0)
 
 
if __name__ == "__main__":
    client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
    client.on_connect = on_connect
    client.on_message = on_message
    client.connect(BROKER_HOST, 1883)
    client.loop_start()
 
    listener_thread = threading.Thread(target=gamepad_listener_loop, daemon=True)
    listener_thread.start()
 
    control_loop(client)
 
    client.loop_stop()
    client.disconnect()