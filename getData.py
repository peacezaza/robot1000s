# import paho.mqtt.client as mqtt
# import paho.mqtt
# print(paho.mqtt.__version__)
#
# # The callback for when the client receives a CONNACK response from the server.
# def on_connect(client, userdata, flags, rc, properties=None):  # 'properties' for version 5
#     print(f"Connected with result code {rc}")
#     client.subscribe("/ESP32/ultraSonicFront")
#
# # The callback for when a PUBLISH message is received from the server.
# def on_message(client, userdata, msg):
#     topic = msg.topic
#     message = msg.payload.decode("utf-8")
#     print(f"{topic}: {message}")
#
# # Create a new MQTT client instance, specifying MQTT version 5
# client = mqtt.Client(protocol=mqtt.MQTTv5)
#
# # Assign the callbacks
# client.on_connect = on_connect
# client.on_message = on_message
#
# # Connect to the MQTT broker
# client.connect("20.2.250.248", 1883, 60)
#
# # Start the network loop (non-blocking)
# client.loop_start()
#
# # Example of publishing a message
# your_data = "data"
# client.publish("/ESP32/ultraSonic", your_data)
#
# # Keep the script running (since loop_start() is non-blocking)
# import time
# while True:
#     time.sleep(1)
import math

import paho.mqtt.client as mqtt
import matplotlib.pyplot as plt
import time


x_data, y_data = [], []
x, y = 0, 0

current_direction = "North"

direction_map = {
    "North": 0,
    "East": 90,
    "South": 180,
    "West": 270
}


def update_position(distance, direction):
    global x, y
    angle = direction_map[direction] * (3.14159 / 180)
    x += distance * math.cos(angle)
    y += distance * math.sin(angle)
    return x, y



def on_connect(client, userdata, flags, rc, properties=None):
    print(f"Connected with result code {rc}")
    client.subscribe("/ESP32/ultraSonicFront")
    client.subscribe("/ESP32/ultraSonicRight")
    client.subscribe("/ESP32/direction")

def on_message(client, userdata, msg):
    global current_direction
    message = msg.payload.decode("utf-8")
    print(f"{msg.topic}: {message}")



# Set up MQTT client
client = mqtt.Client(protocol=mqtt.MQTTv5)
client.on_connect = on_connect
client.on_message = on_message
client.connect("20.2.250.248", 1883, 60)
client.loop_start()
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    print("Plotting stopped.")
