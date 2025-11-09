"""
This module manages the MQTT subscription component of the Fleet Manager system.
It listens for incoming mission commands from Node-RED (or test tools) and forwards
them to the appropriate drones via the ROS2 Action Client.

Main responsibilities:
1. **Subscribe** to farm-specific MQTT topics (e.g., `farm/<farm_id>`) with QoS 2.
2. **Receive** mission messages in JSON format.
3. **Parse** mission data to extract `vehicleId` and the list of commands.
4. **Forward** parsed commands to the ROS2 Action Client for drone execution.
5. **Set Last Will** for unexpected disconnections.

Expected message format:
-------------------------
The MQTT subscriber expects payloads with the following JSON structure:

.. code-block:: json

    {
        "vehicleId": 1,
        "commands": [
            {
                "missionId": 101,
                "commandId": 1,
                "commandType": "TAKEOFF",
                "params": {"altitude": 10}
            },
            {
                "missionId": 101,
                "commandId": 2,
                "commandType": "MOVE",
                "params": {"x": 20, "y": 30}
            }
        ]
    }

Once received, the subscriber automatically converts each command into a stringified
JSON element and passes them to the ROS2 layer for dispatch to the corresponding drone.
"""

import json
import paho.mqtt.client as mqtt
import threading


def start_mqtt_sub(farm_id, ros2_client, fleet_logger, mqtt_broker: str, mqtt_port: int):
    """
    Starts an MQTT subscriber for the specified farm.

    This function connects to the MQTT broker, subscribes to the topic corresponding
    to the given farm ID with **Quality of Service 2 (QoS 2)**, and sets a
    **Last Will and Testament (LWT)** message for unexpected disconnections.

    :param farm_id: Unique identifier of the farm instance.
    :type farm_id: int
    :param ros2_client: Instance of the ROS2 Action Client used to send goals to drones.
    :type ros2_client: CommandsActionClient
    :param fleet_logger: Logger instance used for recording system activity.
    :type fleet_logger: logging.Logger
    :param mqtt_broker: Hostname or IP address of the MQTT broker.
    :type mqtt_broker: str
    :param mqtt_port: Port number of the MQTT broker.
    :type mqtt_port: int
    :return: Configured MQTT client instance.
    :rtype: paho.mqtt.client.Client
    """
    # Topic for mission commands (subscription target)
    mission_topic = f"farms/farm_{farm_id}/mission"
    
    # Configuration for Last Will and Testament (LWT)
    lwt_topic = f"farms/farm_{farm_id}/status"
    lwt_payload = json.dumps({"status": "offline", "reason": "Client disconnected unexpectedly"})
    
    # Assign a unique Client ID for proper session handling (important for LWT)
    client_id = f"fleet_manager_farm_{farm_id}"


    def on_connect(client, userdata, flags, rc):
        """
        Callback triggered when the MQTT client connects to the broker.

        Subscribes to the mission topic using **QoS 2** if the connection is successful.
        QoS 2 ensures the message is received exactly once.

        :param client: The MQTT client instance.
        :type client: paho.mqtt.client.Client
        :param userdata: User-defined data of any type.
        :type userdata: Any
        :param flags: Response flags sent by the broker.
        :type flags: dict
        :param rc: Connection result code (0 indicates success).
        :type rc: int
        """
        if rc == 0:
            fleet_logger.info(f"[MQTT] - Connected to broker at {mqtt_broker}:{mqtt_port}")
            
            # --- MODIFICATION 1: Subscribe with QoS 2 ---
            # Use tuple (topic, qos) to specify QoS level for subscription
            client.subscribe((mission_topic, 2))
            
            fleet_logger.info(f"[MQTT] - Subscribed to topic: {mission_topic} with QoS 2")
        else:
            fleet_logger.error(f"[MQTT] - Connection failed with code {rc}")

    def on_message(client, userdata, msg):
        """
        Callback executed when a message is received from the MQTT broker.

        Parses the message payload, extracts the drone ID and command list,
        converts each command into a JSON string, and forwards the result
        to the ROS2 Action Client for mission dispatch.

        :param client: The MQTT client instance.
        :type client: paho.mqtt.client.Client
        :param userdata: User-defined data of any type.
        :type userdata: Any
        :param msg: The received MQTT message object.
        :type msg: paho.mqtt.client.MQTTMessage
        """
        try:
            payload = json.loads(msg.payload.decode())
            fleet_logger.info(f"[MQTT] - Received message on {msg.topic} (QoS {msg.qos}): {json.dumps(payload)}")
            
            drone_id = payload.get("vehicleId")
            commands_list = payload.get("commands", [])

            if drone_id is not None and commands_list:
                # --- Convert each command to a JSON string ---
                string_commands = []
                for cmd in commands_list:
                    string_cmd = json.dumps({
                        "missionId": cmd.get("missionId", 0),
                        "commandId": cmd.get("commandId", 0),
                        "commandType": str(cmd.get("commandType", "")),
                        "params": cmd.get("params", {})
                    })
                    string_commands.append(string_cmd)
                # --- End conversion ---

                ros2_client.send_goal(drone_id, string_commands)
            else:
                fleet_logger.warning("[MQTT] - Invalid message format. 'vehicleId' or 'commands' missing.")
        except Exception as e:
            fleet_logger.error(f"[MQTT] - Failed to process message: {e}")

    # Initialize MQTT Client with a specific ID
    mqtt_client = mqtt.Client(client_id=client_id)
    mqtt_client.on_connect = on_connect
    mqtt_client.on_message = on_message

    # --- MODIFICATION 2: Configure Last Will and Testament (LWT) ---
    # The LWT message will be published by the broker if the client disconnects uncleanly.
    # QoS=1 ensures delivery; Retain=True ensures the status remains for new subscribers.
    mqtt_client.will_set(
        lwt_topic, 
        lwt_payload, 
        qos=1, 
        retain=True
    )
    fleet_logger.info(f"[MQTT] - Last Will configured on topic {lwt_topic} with payload: {lwt_payload}")

    # --- MODIFICATION: Use dynamic parameters for connect ---
    mqtt_client.connect(mqtt_broker, mqtt_port, keepalive=60)

    mqtt_thread = threading.Thread(target=mqtt_client.loop_forever, daemon=True)
    mqtt_thread.start()
    
    fleet_logger.info("[MQTT] - MQTT subscriber started.")

    return mqtt_client