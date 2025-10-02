import paho.mqtt.client as mqtt
import json

MQTT_BROKER = "localhost"
MQTT_PORT = 1883
MQTT_KEEPALIVE = 60 

mqtt_client = mqtt.Client()

mqtt_client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
mqtt_client.loop_start()

def mqtt_goal(farm_id, drone_id, message, fleet_logger):
    topic = f"farm/{farm_id}/{drone_id}"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published goal -> {topic}: {payload}")


def mqtt_feedback(farm_id, drone_id, message, fleet_logger):
    topic = f"farm/{farm_id}/{drone_id}"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published feedback -> {topic}: {payload}")


def mqtt_result(farm_id, drone_id, message, fleet_logger):
    """
    Publica resultado final de la misión
    """
    topic = f"farm/{farm_id}/{drone_id}"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published result -> {topic}: {payload}")


def mqtt_warning(farm_id, message, fleet_logger):
    """
    Publica advertencias a nivel granja (no asociadas a un dron específico)
    """
    topic = f"farm/{farm_id}/warning"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published warning -> {topic}: {payload}")
