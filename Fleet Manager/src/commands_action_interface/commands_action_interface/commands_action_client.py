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

    def __init__(self, farm_id, fleet_logger):
        super().__init__('commands_action_client')
        self.action_clients = {}  
        self.drone_busy = {}
        self.farm_id = farm_id
        self.fleet_logger = fleet_logger
        self.ros2_action_queue = queue.Queue()

    def send_goal(self, drone_id, commands_list):
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
        result = future.result().result

        self.fleet_logger.info(f"[ROS2] - Drone {drone_id}: Result: {result.plan_result}")
        mqtt_result(self.farm_id, drone_id, f"Drone {drone_id} mission result: {result.plan_result}", self.fleet_logger)

        self.drone_busy[drone_id] = False

    def feedback_callback(self, feedback_msg, drone_id):
        feedback = feedback_msg.feedback
        self.fleet_logger.info(f"[ROS2] - Drone {drone_id}: Feedback: {feedback.command_status}")
        mqtt_feedback(self.farm_id, drone_id, f"Drone {drone_id} feedback: {feedback.command_status}", self.fleet_logger)


def create_ros2_action_client(farm_id, fleet_logger):
    rclpy.init()
    action_client = CommandsActionClient(farm_id, fleet_logger)

    executor = MultiThreadedExecutor()
    executor.add_node(action_client)

    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()

    return action_client
