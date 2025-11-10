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

#: Keepalive interval in seconds for the MQTT connection.
MQTT_KEEPALIVE = 60 

#: Global MQTT client instance used for publishing all messages. Initialized in initialize_publisher_client.
mqtt_client = None

# --- Last Will Configuration ---
LWT_PAYLOAD = json.dumps({"status": "offline", "service": "FleetManagerPublisher"})
LWT_QOS = 1
LWT_RETAIN = True
CLIENT_ID = "fleet_manager_publisher_client"


def initialize_publisher_client(farm_id, fleet_logger, mqtt_broker: str, mqtt_port: int) -> None:
    """
    Initializes the global MQTT publishing client, configures the Last Will and 
    Testament (LWT), connects to the broker, and starts the network loop.

    This function must be called once before any publishing operations.
    
    :param fleet_logger: Logger instance for recording system activity.
    :type fleet_logger: logging.Logger
    :param mqtt_broker: Hostname or IP address of the MQTT broker.
    :type mqtt_broker: str
    :param mqtt_port: Port number of the MQTT broker.
    :type mqtt_port: int
    :return: None
    :rtype: None
    """
    global mqtt_client
    
    if mqtt_client:
        fleet_logger.warning("[MQTT] - Publisher client already initialized.")
        return

    mqtt_client = mqtt.Client(client_id=CLIENT_ID)

    # --- Set Last Will and Testament (LWT) before connecting ---
    mqtt_client.will_set(
        f"farm/farm_{farm_id}/pub_status", 
        LWT_PAYLOAD, 
        qos=LWT_QOS, 
        retain=LWT_RETAIN
    )
    fleet_logger.info(f"[MQTT] - Last Will configured on topic farm/farm_{farm_id}/pub_status. Status: offline")
    
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            fleet_logger.info(f"[MQTT] - Publisher connected to broker at {mqtt_broker}:{mqtt_port}")
            
            client.publish(f"farm/farm_{farm_id}/pub_status", json.dumps({"status": "online", "service": "FleetManagerPublisher"}), qos=1, retain=True)
        else:
            fleet_logger.error(f"[MQTT] - Publisher connection failed with code {rc}")

    mqtt_client.on_connect = on_connect
    
    try:
        # --- Use dynamic parameters for connect ---
        mqtt_client.connect(mqtt_broker, mqtt_port, MQTT_KEEPALIVE)
        mqtt_client.loop_start()
        fleet_logger.info("[MQTT] - Publisher network loop started.")
    except Exception as e:
        fleet_logger.error(f"[MQTT] - Failed to connect or start loop: {e}")


def mqtt_goal(farm_id, drone_id, message, fleet_logger):
    """
    Publishes a "goal" message to the MQTT broker with **QoS 2**.

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
    if mqtt_client is None:
        fleet_logger.error("[MQTT] - Publisher client is not initialized.")
        return
        
    topic = f"farms/farm_{farm_id}/drone_{drone_id}/goal"
    payload = json.dumps({
        "message": message
    })
    
    # --- Publish with QoS 2 (Exactly Once) ---
    mqtt_client.publish(topic, payload, qos=2)
    fleet_logger.info(f"[MQTT] - Published goal -> {topic}: {payload} (QoS 2)")


def mqtt_feedback(farm_id, drone_id, message, fleet_logger):
    """
    Publishes a "feedback" message to the MQTT broker with **QoS 2**.

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
    if mqtt_client is None:
        fleet_logger.error("[MQTT] - Publisher client is not initialized.")
        return

    topic = f"farms/farm_{farm_id}/drone_{drone_id}/feedback"
    payload = json.dumps({
        "message": message
    })
    
    # --- Publish with QoS 2 (Exactly Once) ---
    mqtt_client.publish(topic, payload, qos=2)
    fleet_logger.info(f"[MQTT] - Published feedback -> {topic}: {payload} (QoS 2)")


def mqtt_result(farm_id, drone_id, message, fleet_logger):
    """
    Publishes the final mission result to the MQTT broker with **QoS 2**.

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
    if mqtt_client is None:
        fleet_logger.error("[MQTT] - Publisher client is not initialized.")
        return

    topic = f"farms/farm_{farm_id}/drone_{drone_id}/result"
    payload = json.dumps({
        "message": message
    })
    
    # --- Publish with QoS 2 (Exactly Once) ---
    mqtt_client.publish(topic, payload, qos=2)
    fleet_logger.info(f"[MQTT] - Published result -> {topic}: {payload} (QoS 2)")


def mqtt_warning(farm_id, message, fleet_logger):
    """
    Publishes a warning message to the MQTT broker at the farm level with **QoS 2**.

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
    if mqtt_client is None:
        fleet_logger.error("[MQTT] - Publisher client is not initialized.")
        return

    topic = f"farms/farm_{farm_id}/warning"
    payload = json.dumps({
        "message": message
    })
    
    # --- Publish with QoS 2 (Exactly Once) ---
    mqtt_client.publish(topic, payload, qos=2)
    fleet_logger.info(f"[MQTT] - Published warning -> {topic}: {payload} (QoS 2)")