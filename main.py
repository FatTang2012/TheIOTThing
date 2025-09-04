#!/usr/bin/env python3
import time
import statistics
import paho.mqtt.client as mqtt
from seeed_dht import DHT
from datetime import datetime
import Adafruit_DHT
import Adafruit_MCP3008
import RPi.GPIO as GPIO
import I2C_LCD_driver

# ====== Cấu hình ThingSpeak MQTT ======
CHANNEL_ID   = "3041584"   # Channel ID của bạn
MQTT_SERVER  = "mqtt3.thingspeak.com"
MQTT_PORT    = 1883
#CLIENT_ID    = "Hide"
#USERNAME     = "Hide"
#PASSWORD     = "HIDE"
TOPIC_PUB    = f"channels/{CHANNEL_ID}/publish"
TOPIC_SUB    = f"channels/{CHANNEL_ID}/subscribe"   # Để đọc trạng thái nút nhấn

# ====== File log ======
LOG_FILE = "mqtt_log.txt"

# ====== Khởi tạo cảm biến ======
dht = DHT("11", 5)    # DHT11 tại GPIO5

# ADC MCP3008 (SPI)
mcp = Adafruit_MCP3008.MCP3008(clk=18, cs=25, miso=23, mosi=24)

# HC-SR04 (GPIO)
TRIG = 20
ECHO = 21
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# LED, Buzzer, Relay
LED_PIN = 12
BUZZER_PIN = 16
RELAY_PIN = 19
for pin in [LED_PIN, BUZZER_PIN, RELAY_PIN]:
    GPIO.setup(pin, GPIO.OUT)
    GPIO.output(pin, GPIO.LOW)

# LCD I2C
lcd = I2C_LCD_driver.lcd()

# ====== Biến toàn cục ======
mode_manual = True  # True = Manual, False = Auto
remote_cmd = {"led": 0, "buzzer": 0, "relay": 0}

# ====== MQTT Client ======
client = mqtt.Client(client_id=CLIENT_ID)
client.username_pw_set(USERNAME, PASSWORD)

def on_message(client, userdata, msg):
    global mode_manual, remote_cmd
    try:
        payload = msg.payload.decode()
        print(f"[MQTT SUB] {payload}")
        # Giả sử server gửi JSON: {"mode":"manual","led":1,"buzzer":0,"relay":1}
        import json
        data = json.loads(payload)
        if data["mode"] == "manual":
            mode_manual = True
            remote_cmd = {"led": data["led"], "buzzer": data["buzzer"], "relay": data["relay"]}
        else:
            mode_manual = False
    except Exception as e:
        print("Lỗi parse:", e)

client.on_message = on_message
client.connect(MQTT_SERVER, MQTT_PORT, 60)
client.subscribe(TOPIC_SUB)
client.loop_start()

# ====== Hàm đo cảm biến ======
def read_distance():
    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)
    start, stop = time.time(), time.time()
    while GPIO.input(ECHO) == 0:
        start = time.time()
    while GPIO.input(ECHO) == 1:
        stop = time.time()
    elapsed = stop - start
    distance = (elapsed * 34300) / 2
    return distance

def read_voltage(channel=0):
    value = mcp.read_adc(channel)
    voltage = (value / 1023.0) * 3.3
    return voltage

def gui_mqtt(temp, hum, volt, dist):
    try:
        payload = f"field1={temp:.1f}&field2={hum:.1f}&field3={volt:.2f}&field4={dist:.1f}"
        result = client.publish(TOPIC_PUB, payload, qos=0)
        msg = f"[MQTT] {'✅' if result.rc == 0 else '❌'} {payload}"
        print(msg)
        with open(LOG_FILE, "a") as f:
            f.write(f"{datetime.now()} | {msg}\n")
    except Exception as e:
        print(f"Lỗi MQTT: {e}")

# ====== Vòng lặp chính ======
print("=== Bắt đầu chạy chương trình ===")
start_time = time.time()
temps, hums, volts, dists = [], [], [], []

while True:
    # Đọc dữ liệu
    try:
        hum, temp = dht.read()
        volt = read_voltage()
        dist = read_distance()
        if hum is not None and temp is not None:
            temps.append(temp)
            hums.append(hum)
            volts.append(volt)
            dists.append(dist)
    except Exception as e:
        print("Lỗi đọc cảm biến:", e)

    # Hiển thị thời gian lên LCD
    now = datetime.now().strftime("%H:%M:%S")
    lcd.lcd_display_string(f"Time: {now}", 1)

    # Mỗi 20 giây gửi dữ liệu trung bình
    if len(temps) >= 20:
        avg_t = statistics.mean(temps)
        avg_h = statistics.mean(hums)
        avg_v = statistics.mean(volts)
        avg_d = statistics.mean(dists)
        gui_mqtt(avg_t, avg_h, avg_v, avg_d)
        temps, hums, volts, dists = [], [], [], []

    # Manual / Auto mode xử lý điều khiển
    if mode_manual:
        GPIO.output(LED_PIN, remote_cmd["led"])
        GPIO.output(BUZZER_PIN, remote_cmd["buzzer"])
        GPIO.output(RELAY_PIN, remote_cmd["relay"])
    else:
        hour = datetime.now().hour
        # LED: 18h–22h
        GPIO.output(LED_PIN, GPIO.HIGH if 18 <= hour < 22 else GPIO.LOW)
        # Buzzer: >40°C hoặc <30°C
        if len(temps) > 0:
            if temps[-1] > 40:
                GPIO.output(BUZZER_PIN, GPIO.HIGH)
            elif temps[-1] < 30:
                GPIO.output(BUZZER_PIN, GPIO.LOW)
        # Relay: >70% hoặc <40%
        if len(hums) > 0:
            if hums[-1] > 70:
                GPIO.output(RELAY_PIN, GPIO.HIGH)
            elif hums[-1] < 40:
                GPIO.output(RELAY_PIN, GPIO.LOW)

    time.sleep(1)
