import paho.mqtt.client as mqtt
import json
import sys

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60 

mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
mqtt_client.loop_start()

def send_mission_from_file(farm_id, json_file):
    topic = f"farm/{farm_id}"
    
    with open(json_file, "r") as f:
        payload = json.load(f)

    mqtt_client.publish(topic, json.dumps(payload))
    print(f"Published to {topic}: {json.dumps(payload, indent=2)}")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python send_mission.py <farm_id> <json_file>")
        sys.exit(1)

    farm_id = sys.argv[1]
    json_file = sys.argv[2]
    send_mission_from_file(farm_id, json_file)
