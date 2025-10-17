"""
commands_action_client.py
=========================

This module manages the communication between the Fleet Manager and the drones
using ROS2 Actions. It supports automatic drone discovery, concurrent communication
with multiple drones, and integrates with MQTT for mission status reporting.

Main responsibilities:
1. Automatically discovers and connects to drones via ROS2 action topics.
2. Sends mission goals (command sequences) to specific drones.
3. Handles feedback and results asynchronously using callbacks.
4. Reports mission progress and outcomes to the Fleet Manager via MQTT.

It serves as the core bridge between the ROS2-based drone layer and the MQTT-based
Fleet Manager system.
"""

import json
import logging
import rclpy
import queue
from rclpy.action import ActionClient
from rclpy.node import Node
from functools import partial
from rclpy.executors import MultiThreadedExecutor
import threading

from commands_action_interface.action import Commands
from MQTT.mqtt_pub import mqtt_goal, mqtt_feedback, mqtt_result, mqtt_warning


class CommandsActionClient(Node):
    """
    ROS2 Action Client node for managing communication with multiple drones.

    This class enables the Fleet Manager to send missions to drones, receive feedback,
    and handle mission results. It dynamically creates and manages multiple ROS2
    Action Clients (one per connected drone).

    :param farm_id: Unique identifier for the farm instance associated with this Fleet Manager.
    :type farm_id: int
    :param fleet_logger: Logger instance used for recording Fleet Manager events.
    :type fleet_logger: logging.Logger
    """

    def __init__(self, farm_id, fleet_logger):
        super().__init__('commands_action_client')
        #: Dictionary mapping drone IDs to their respective ActionClient instances.
        self.action_clients = {}  

        #: Dictionary tracking whether each drone is currently executing a mission.
        self.drone_busy = {}

        #: Farm ID corresponding to this Fleet Manager instance.
        self.farm_id = farm_id

        #: Logger for system activity.
        self.fleet_logger = fleet_logger

        #: Queue used for internal ROS2 action event management.
        self.ros2_action_queue = queue.Queue()

    def send_goal(self, drone_id, commands_list):
        """
        Sends a mission (list of commands) to the specified drone via ROS2.

        If the drone is already executing another mission, the request is rejected.
        Otherwise, a new ROS2 Action Client is created if necessary, and the mission
        is transmitted asynchronously.

        :param drone_id: Identifier of the target drone.
        :type drone_id: int
        :param commands_list: List of mission commands to be executed by the drone.
        :type commands_list: list[str]
        :return: None
        :rtype: None
        """
        if self.drone_busy.get(drone_id, False):
            self.fleet_logger.warning(f"[ROS2] - Drone {drone_id} is busy, rejecting new mission.")
            mqtt_warning(self.farm_id, f"Drone {drone_id} is currently busy.", self.fleet_logger)
            return

        if drone_id not in self.action_clients:
            action_client = ActionClient(
                self,
                Commands,
                f'drone{drone_id}/commands'
            )
            self.action_clients[drone_id] = action_client
        else:
            action_client = self.action_clients[drone_id]

        if not action_client.wait_for_server(timeout_sec=2.0):
            self.fleet_logger.error(f"[ROS2] - Drone {drone_id} server unavailable.")
            mqtt_warning(self.farm_id, f"Drone {drone_id} server unavailable.", self.fleet_logger)
            return

        goal_msg = Commands.Goal()
        goal_msg.commands_list = commands_list

        self.fleet_logger.info(f'[ROS2] - Sending commands to drone {drone_id}')

        self.drone_busy[drone_id] = True

        send_goal_future = action_client.send_goal_async(
            goal_msg,
            feedback_callback=partial(self.feedback_callback, drone_id=drone_id)
        )
        send_goal_future.add_done_callback(
            partial(self.goal_response_callback, drone_id=drone_id)
        )

    def goal_response_callback(self, future, drone_id):
        """
        Callback executed when the drone responds to a mission goal request.

        Handles both acceptance and rejection of the mission by the drone.
        If accepted, it attaches a callback to handle mission result retrieval.

        :param future: Future object representing the asynchronous goal response.
        :type future: rclpy.task.Future
        :param drone_id: Identifier of the drone that responded to the mission.
        :type drone_id: int
        :return: None
        :rtype: None
        """
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.fleet_logger.info(f"[ROS2] - Drone {drone_id}: Goal rejected.")
            mqtt_warning(self.farm_id, f"Drone {drone_id} rejected the mission.", self.fleet_logger)
            self.ros2_action_queue.put(json.dumps([drone_id, 1]))
            self.drone_busy[drone_id] = False
            return

        self.fleet_logger.info(f"[ROS2] - Drone {drone_id}: Goal accepted.")

        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(
            partial(self.get_result_callback, drone_id=drone_id)
        )

        mqtt_goal(self.farm_id, drone_id, "Goal accepted", self.fleet_logger)

    def get_result_callback(self, future, drone_id):
        """
        Callback executed when the drone finishes executing the mission.

        Publishes the mission result to MQTT and marks the drone as available again.

        :param future: Future object containing the mission result.
        :type future: rclpy.task.Future
        :param drone_id: Identifier of the drone that completed the mission.
        :type drone_id: int
        :return: None
        :rtype: None
        """
        result = future.result().result

        self.fleet_logger.info(f"[ROS2] - Drone {drone_id}: Result: {result.plan_result}")
        mqtt_result(self.farm_id, drone_id, f"Drone {drone_id} mission result: {result.plan_result}", self.fleet_logger)

        self.drone_busy[drone_id] = False

    def feedback_callback(self, feedback_msg, drone_id):
        """
        Callback executed when the drone provides feedback during mission execution.

        Each feedback message is logged and published to the MQTT topic corresponding
        to the farm.

        :param feedback_msg: ROS2 feedback message from the drone.
        :type feedback_msg: commands_action_interface.action.Commands.FeedbackMessage
        :param drone_id: Identifier of the drone sending the feedback.
        :type drone_id: int
        :return: None
        :rtype: None
        """
        feedback = feedback_msg.feedback
        self.fleet_logger.info(f"[ROS2] - Drone {drone_id}: Feedback: {feedback.command_status}")
        mqtt_feedback(self.farm_id, drone_id, f"Drone {drone_id} feedback: {feedback.command_status}", self.fleet_logger)


def create_ros2_action_client(farm_id, fleet_logger):
    """
    Initializes the ROS2 environment and creates a multi-threaded Action Client
    for managing drone communication.

    The Action Client runs in a separate thread, allowing concurrent handling
    of ROS2 callbacks without blocking the main Fleet Manager process.

    :param farm_id: Unique identifier for the farm instance.
    :type farm_id: int
    :param fleet_logger: Logger instance used for recording Fleet Manager events.
    :type fleet_logger: logging.Logger
    :return: Instance of :class:`CommandsActionClient` ready to communicate with drones.
    :rtype: CommandsActionClient
    """
    rclpy.init()
    action_client = CommandsActionClient(farm_id, fleet_logger)

    executor = MultiThreadedExecutor()
    executor.add_node(action_client)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    return action_client
