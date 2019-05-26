from umqtt.simple import MQTTClient
import ubinascii
import time
import config

mac_addr = ubinascii.hexlify(wlan.config('mac')).decode("utf-8")
clientname = "client" + mac_addr[-6:]

mqtt = MQTTClient(clientname, config.mqtt_server)

topic = "node/" + mac_addr[-4:] + "/sensors"

mqtt.connect()

while True:
    mqtt.publish(topic, b"Hello")
    time.sleep(5)
