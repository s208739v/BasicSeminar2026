import serial
import time

# ポート設定（Windowsなら 'COM3' など、Mac/Linuxなら '/dev/ttyUSB0' など）
SERIAL_PORT = 'COM3' 
BAUD_RATE = 115200

try:
    # シリアル接続の開始
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
    time.sleep(2)  # 接続安定待ち
    print(f"Connected to {SERIAL_PORT}")

    while True:
        # ユーザーから入力を受け取る
        user_input = input("Target Position (0-4095) or 'q' to quit: ")

        if user_input.lower() == 'q':
            break

        # 入力が数字か確認してから送信
        if user_input.isdigit():
            # 改行コードを付けて送信
            command = f"{user_input}"
            ser.write(command.encode('utf-8'))
            print(f"Sent: {user_input}")
        else:
            print("Please enter a valid number.")

except Exception as e:
    print(f"Error: {e}")

finally:
    if 'ser' in locals() and ser.is_open:
        ser.close()
        print("Connection closed.")