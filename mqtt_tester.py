import time
import paho.mqtt.client as mqtt

# ================= 配置参数 =================
BROKER_HOST = "210.0.159.242"
BROKER_PORT = 1883
USERNAME = "hkcrctest"
PASSWORD = "crcHK3130"

# 主题配置
HB_TOPIC = "ai_safety/ais001/heartbeat/device_manager/hook"
CMD_TOPIC = "ai_safety/ais001/command/device_manager/hook"
INFORM_TOPIC = "ai_safety/ais001/inform/hook/device_manager"  # 新增：订阅的通知主题

# 发送间隔 (秒)
HB_INTERVAL = 1.0       # 心跳: 1s
MODE_CMD_INTERVAL = 5.0 # 工作模式问询: 5s
BATT_CMD_INTERVAL = 10.0 # 电池电量问询: 10s

# ================= CRC16 计算 =================
def crc16_ccitt(data: bytes) -> int:
    crc = 0x0000
    for byte in data:
        crc ^= (byte << 8)
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc

# ================= MQTT 回调 =================
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("\n[INFO] 成功连接到 MQTT 服务器！")
        # 连接成功后立刻订阅指定的上报主题
        client.subscribe(INFORM_TOPIC)
        print(f"[INFO] 已成功订阅主题: {INFORM_TOPIC}")
        print("[INFO] 多线程调度测试序列已启动...\n")
    else:
        print(f"[ERROR] 连接失败，返回码: {rc}")

def on_disconnect(client, userdata, rc):
    print("[WARN] 与服务器断开连接，正在尝试重连...")

# 新增：处理接收到的消息
def on_message(client, userdata, msg):
    # 将接收到的 payload（字节流）转换为易读的 HEX 格式
    payload_bytes = msg.payload
    hex_str = " ".join([f"{b:02X}" for b in payload_bytes])
    
    # 打印醒目的接收提示
    print(f"\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>")
    print(f"[📥 RX INFORM] 收到设备上报消息！")
    print(f"  主题: {msg.topic}")
    print(f"  内容 (HEX): {hex_str}")
    print(f"<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n")

# ================= 主程序 =================
def main():
    client = mqtt.Client(client_id="Python_Tester_004")
    client.username_pw_set(USERNAME, PASSWORD)
    
    # 绑定回调函数
    client.on_connect = on_connect
    client.on_disconnect = on_disconnect
    client.on_message = on_message  # 绑定接收消息处理函数

    print(f"[INFO] 正在连接 {BROKER_HOST}:{BROKER_PORT} ...")
    try:
        client.connect(BROKER_HOST, BROKER_PORT, 60)
    except Exception as e:
        print(f"[ERROR] 无法连接到服务器: {e}")
        return

    # 启动网络循环后台线程 (处理心跳、重连以及【接收消息】)
    client.loop_start()

    # 状态与序号变量
    seq_hb = 0       
    seq_batt = 0     
    seq_mode = 0
    data_val = 0     
    count_hb = 0     

    # 时间戳记录
    current_time = time.time()
    last_hb_time = current_time
    last_mode_time = current_time + 1.5  
    last_batt_time = current_time + 3.0  

    try:
        while True:
            current_time = time.time()

            # ---------------------------------------------------------
            # 任务 1：每 10 秒发送一次【BMS 电量】问询帧 (CMD_ID: 0x02)
            # ---------------------------------------------------------
            if current_time - last_batt_time >= BATT_CMD_INTERVAL:
                frame_head = bytearray([0xA5, seq_batt, 0x02, 0x00])
                crc = crc16_ccitt(frame_head)
                full_frame = frame_head + bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
                
                client.publish(CMD_TOPIC, full_frame, qos=0)
                
                hex_str = " ".join([f"{b:02X}" for b in full_frame])
                print(f"[🔋 TX CMD-BATT] 发送电量问询包 -> {hex_str}")
                
                seq_batt = (seq_batt + 1) % 256
                last_batt_time = current_time

            # ---------------------------------------------------------
            # 任务 2：每 5 秒发送一次【工作模式】问询帧 (CMD_ID: 0x06)
            # ---------------------------------------------------------
            if current_time - last_mode_time >= MODE_CMD_INTERVAL:
                frame_head = bytearray([0xA5, seq_mode, 0x06, 0x00])
                crc = crc16_ccitt(frame_head)
                full_frame = frame_head + bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
                
                client.publish(CMD_TOPIC, full_frame, qos=0)
                
                hex_str = " ".join([f"{b:02X}" for b in full_frame])
                print(f"[⚙️ TX CMD-MODE] 发送工作模式问询包 -> {hex_str}")
                
                seq_mode = (seq_mode + 1) % 256
                last_mode_time = current_time

            # ---------------------------------------------------------
            # 任务 3：每 1 秒发送一次【心跳帧】 (CMD_ID: 0x05)
            # ---------------------------------------------------------
            if current_time - last_hb_time >= HB_INTERVAL:
                frame_head = bytearray([0xA5, seq_hb, 0x05, 0x01, data_val])
                crc = crc16_ccitt(frame_head)
                full_frame = frame_head + bytearray([crc & 0xFF, (crc >> 8) & 0xFF])
                
                client.publish(HB_TOPIC, full_frame, qos=0)
                
                count_hb += 1
                hex_str = " ".join([f"{b:02X}" for b in full_frame])
                print(f"[❤️ TX HB] 发送第 {count_hb} 个心跳包 -> {hex_str}")

                seq_hb = (seq_hb + 1) % 256
                data_val = (data_val + 1) % 256
                last_hb_time = current_time

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n[INFO] 测试被用户手动终止。")
    finally:
        client.loop_stop()
        client.disconnect()

if __name__ == "__main__":
    main()