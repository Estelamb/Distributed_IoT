"""
This module provides utility functions for publishing messages to the MQTT broker
from the Fleet Manager system. It acts as the outbound communication interface,
sending updates to Node-RED or other subscribers.

It handles four main types of messages:
1. **Goal messages** – Sent when a drone accepts a mission.
2. **Feedback messages** – Sent to report intermediate progress from the drone.
3. **Result messages** – Sent when a mission has been completed.
4. **Warning messages** – Sent to notify about system-level issues or unavailable drones.

All functions share the same structure:
- Build a JSON payload with a textual message.
- Publish it to a topic associated with a specific farm or drone.
- Log the publication through the Fleet Manager's logger.
"""

import paho.mqtt.client as mqtt
import json

#: MQTT broker hostname or IP address (Docker bridge host).
MQTT_BROKER = "host.docker.internal"

#: Port number for the MQTT connection.
MQTT_PORT = 1883

#: Keepalive interval in seconds for the MQTT connection.
MQTT_KEEPALIVE = 60 

#: Global MQTT client instance used for publishing all messages.
mqtt_client = mqtt.Client()

mqtt_client.connect(MQTT_BROKER, MQTT_PORT, MQTT_KEEPALIVE)
mqtt_client.loop_start()


def mqtt_goal(farm_id, drone_id, message, fleet_logger):
    """
    Publishes a "goal" message to the MQTT broker.

    This function is typically called when a drone accepts a mission request.
    The message is sent to the topic corresponding to the specific farm and drone.

    :param farm_id: Unique identifier of the farm.
    :type farm_id: int
    :param drone_id: Unique identifier of the drone.
    :type drone_id: int
    :param message: Message content to publish (usually mission confirmation).
    :type message: str
    :param fleet_logger: Logger instance for recording system activity.
    :type fleet_logger: logging.Logger
    :return: None
    :rtype: None
    """
    topic = f"farm/{farm_id}/{drone_id}"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published goal -> {topic}: {payload}")


def mqtt_feedback(farm_id, drone_id, message, fleet_logger):
    """
    Publishes a "feedback" message to the MQTT broker.

    This function reports intermediate mission updates from the drone to the Fleet Manager.
    Feedback messages indicate command progress or partial mission states.

    :param farm_id: Unique identifier of the farm.
    :type farm_id: int
    :param drone_id: Unique identifier of the drone.
    :type drone_id: int
    :param message: Message content to publish (usually feedback information).
    :type message: str
    :param fleet_logger: Logger instance for recording system activity.
    :type fleet_logger: logging.Logger
    :return: None
    :rtype: None
    """
    topic = f"farm/{farm_id}/{drone_id}"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published feedback -> {topic}: {payload}")


def mqtt_result(farm_id, drone_id, message, fleet_logger):
    """
    Publishes the final mission result to the MQTT broker.

    This message indicates that a mission has been completed by the drone and
    includes the outcome or status of the plan execution.

    :param farm_id: Unique identifier of the farm.
    :type farm_id: int
    :param drone_id: Unique identifier of the drone.
    :type drone_id: int
    :param message: Message describing the mission result.
    :type message: str
    :param fleet_logger: Logger instance for recording system activity.
    :type fleet_logger: logging.Logger
    :return: None
    :rtype: None
    """
    topic = f"farm/{farm_id}/{drone_id}"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published result -> {topic}: {payload}")


def mqtt_warning(farm_id, message, fleet_logger):
    """
    Publishes a warning message to the MQTT broker at the farm level.

    Warnings are not associated with a specific drone. They are used to
    communicate system issues such as unavailable drones, communication
    errors, or initialization problems.

    :param farm_id: Unique identifier of the farm.
    :type farm_id: int
    :param message: Warning text to publish.
    :type message: str
    :param fleet_logger: Logger instance for recording system activity.
    :type fleet_logger: logging.Logger
    :return: None
    :rtype: None
    """
    topic = f"farm/{farm_id}/warning"
    payload = json.dumps({
        "message": message
    })
    mqtt_client.publish(topic, payload)
    fleet_logger.info(f"[MQTT] - Published warning -> {topic}: {payload}")
