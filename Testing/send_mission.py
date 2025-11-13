"""
This module allows testing of the Fleet Manager system without using the Node-RED dashboard.

It reads a mission definition from a JSON file and publishes it to the appropriate
MQTT topic corresponding to a specific farm. This simulates the behavior of Node-RED
when sending missions to the Fleet Manager, making it useful for standalone testing
and debugging.

Typical usage example:
    python send_mission.py <farm_id> <json_file>
"""

import paho.mqtt.client as mqtt
import json
import sys

#: MQTT broker hostname or IP address.
MQTT_BROKER = "localhost"

#: Port number used for the MQTT connection.
MQTT_PORT = 1885

#: Keepalive interval (in seconds) for the MQTT connection.
MQTT_KEEPALIVE = 60 

#: Global MQTT client instance used to publish missions.
mqtt_client = mqtt.Client()
mqtt_client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
mqtt_client.loop_start()


def send_mission_from_file(farm_id, json_file):
    """
    Sends a mission file as a JSON payload to the Fleet Manager via MQTT.

    This function reads a JSON mission file, serializes it, and publishes it
    to the MQTT topic corresponding to the specified farm. It is typically used
    for testing the system without the Node-RED interface.

    :param farm_id: Identifier of the target farm where the mission will be sent.
    :type farm_id: str
    :param json_file: Path to the JSON file containing the mission data.
    :type json_file: str
    :return: None
    :rtype: None
    """
    topic = f"farms/farm_{farm_id}/mission"
    
    with open(json_file, "r") as f:
        payload = json.load(f)

    mqtt_client.publish(topic, json.dumps(payload))
    print(f"Published to {topic}: {json.dumps(payload, indent=2)}")


if __name__ == "__main__":
    """
    Command-line entry point for sending missions to a Fleet Manager.

    The script expects two arguments:
        1. The farm identifier (farm_id)
        2. The path to a JSON mission file

    Example:
        python send_mission.py 1 mission.json
    """
    if len(sys.argv) != 3:
        print("Usage: python send_mission.py <farm_id> <json_file>")
        sys.exit(1)

    farm_id = sys.argv[1]
    json_file = sys.argv[2]
    send_mission_from_file(farm_id, json_file)
