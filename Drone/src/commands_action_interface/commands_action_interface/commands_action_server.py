import time
import threading

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from commands_action_interface.action import Commands
from Commands.commands_processor import process_command


class CommandsActionServer(Node):

    def __init__(self, drone_id: int, drone_logger):
        super().__init__(
            'commands_action_server',
            namespace=f'drone{drone_id}',
            allow_undeclared_parameters=True
        )
        self.vehicle_id = drone_id
        self.drone_logger = drone_logger

        self._action_server = ActionServer(
            self,
            Commands,
            'commands',
            self.execute_callback
        )

        self.drone_logger.info(f"[ROS2] - Commands action server initialized for drone {drone_id}")
        self.drone_logger.info(f"[ROS2] - Action service available at: /drone{drone_id}/commands")

    def execute_callback(self, goal_handle):
        self.drone_logger.info(f"[ROS2] - Executing command sequence goal with {len(goal_handle.request.commands_list)} commands")
        feedback_msg = Commands.Feedback()

        try:
            for idx, command in enumerate(goal_handle.request.commands_list):
                if not rclpy.ok():
                    self.drone_logger.warning("[ROS2] - ROS2 shutdown requested during execution")
                    raise Exception("Shutdown requested during command execution")

                self.drone_logger.info(f"[ROS2] - Processing command {idx + 1}/{len(goal_handle.request.commands_list)}: {command}")
                command_id, command_data = process_command(command, self.drone_logger)

                feedback_msg.command_status = [str(self.vehicle_id), str("Feedback"), str(command_data)]
                goal_handle.publish_feedback(feedback_msg)

                self.drone_logger.info(f"[ROS2] - Feedback published: {feedback_msg.command_status}")
                time.sleep(1)

            goal_handle.succeed()
            result = Commands.Result()
            result.plan_result = [str(self.vehicle_id), str("Result")]

            self.drone_logger.info(f"[ROS2] - Command sequence completed successfully: {result.plan_result}")
            return result

        except Exception as e:
            self.drone_logger.error(f"[ROS2] - Error during command execution: {str(e)}", exc_info=True)
            goal_handle.abort()
            raise


def run_action_server(drone_id: int, drone_logger):
    rclpy.init()
    server = CommandsActionServer(drone_id, drone_logger)
    executor = MultiThreadedExecutor()
    executor.add_node(server)

    executor_thread = threading.Thread(target=executor.spin, daemon=True)
    executor_thread.start()

    try:
        while rclpy.ok():
            time.sleep(0.1)
    except KeyboardInterrupt:
        drone_logger.info("[ROS2] - Ctrl+C received, shutting down gracefully")
    finally:
        drone_logger.info("[ROS2] - Cleaning up executor and server")
        executor.shutdown()
        server.destroy_node()
        rclpy.shutdown()
