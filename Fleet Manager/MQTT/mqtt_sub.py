import json
import paho.mqtt.client as mqtt
import threading

def start_mqtt_sub(farm_id, ros2_client, fleet_logger, mqtt_broker="localhost", mqtt_port=1883):
    topic = f"farm/{farm_id}"

    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            fleet_logger.info(f"[MQTT] - Connected to broker at {mqtt_broker}:{mqtt_port}")
            client.subscribe(topic)
            fleet_logger.info(f"[MQTT] - Subscribed to topic: {topic}")
        else:
            fleet_logger.error(f"[MQTT] - Connection failed with code {rc}")

    def on_message(client, userdata, msg):
        try:
            payload = json.loads(msg.payload.decode())
            fleet_logger.info(f"[MQTT] - Received message on {msg.topic}: {json.dumps(payload)}")
            
            drone_id = payload.get("vehicleId")
            commands_list = payload.get("commands", [])

            if drone_id is not None and commands_list:
                ros2_client.send_goal(drone_id, commands_list)
            else:
                fleet_logger.warning("[MQTT] - Invalid message format. 'vehicleId' or 'commands' missing.")
        except Exception as e:
            fleet_logger.error(f"[MQTT] - Failed to process message: {e}")

    mqtt_client = mqtt.Client()
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    mqtt_client.connect(mqtt_broker, mqtt_port, keepalive=60)

    mqtt_thread = threading.Thread(target=mqtt_client.loop_forever, daemon=True)
    mqtt_thread.start()
    
    fleet_logger.info("[MQTT] - MQTT subscriber started.")

    return mqtt_client
