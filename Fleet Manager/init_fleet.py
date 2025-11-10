"""
This module initializes and runs the Fleet Manager system for a specific farm.

It performs the following main tasks:
1. Creates a thread-safe logger for recording system activity.
2. Initializes a ROS2 Action Client used for communicating with drones.
3. Initializes the MQTT publisher client.
4. Starts the MQTT subscriber to receive mission commands from Node-RED (or other sources).
5. Keeps the Fleet Manager running continuously until manually terminated.

This module serves as the entry point for the Fleet Manager container in the
distributed UAV control architecture, coordinating MQTT and ROS2 communication layers.
"""

import argparse
import logging
import queue
import sys
import threading

from concurrent_log_handler import ConcurrentRotatingFileHandler

from MQTT.mqtt_sub import start_mqtt_sub
from MQTT.mqtt_pub import initialize_publisher_client
from src.commands_action_interface.commands_action_interface.commands_action_client import create_ros2_action_client


def create_logger(logger_name: str, log_file: str) -> logging.Logger:
    """
    Creates and configures a thread-safe logger for the Fleet Manager system.

    The logger writes logs both to the console and to a rotating log file.
    It uses `ConcurrentRotatingFileHandler` to ensure safe access from multiple threads.

    :param logger_name: The name to assign to the logger instance.
    :type logger_name: str
    :param log_file: Path to the log file where entries will be stored.
    :type log_file: str
    :return: A configured logger instance.
    :rtype: logging.Logger
    """
    logger = logging.getLogger(logger_name)
    logger.setLevel(logging.INFO)

    file_handler = ConcurrentRotatingFileHandler(log_file, maxBytes=1_000_000, backupCount=5)
    console_handler = logging.StreamHandler()

    formatter = logging.Formatter('[%(asctime)s][%(name)s][%(levelname)s]%(message)s')
    file_handler.setFormatter(formatter)
    console_handler.setFormatter(formatter)

    logger.addHandler(file_handler)
    logger.addHandler(console_handler)
    return logger


def main(farm_id: int, mqtt_broker: str, mqtt_port: int) -> None:
    """
    Initializes and runs the Fleet Manager system for a specific farm.

    This function sets up the logger, initializes the ROS2 action client, and
    starts the MQTT subscriber. Once initialized, it keeps the system running
    until the user stops it manually (e.g., with Ctrl+C).

    :param farm_id: Unique identifier of the farm instance.
    :type farm_id: int
    :param mqtt_broker: Hostname or IP address of the MQTT broker.
    :type mqtt_broker: str
    :param mqtt_port: Port number of the MQTT broker.
    :type mqtt_port: int
    :return: None
    :rtype: None
    """
    fleet_logger = create_logger(f'Farm {farm_id}', f'Logs/Farm_{farm_id}.log')
    fleet_logger.info(f"[Init Fleet] - Initializing Fleet Manager system for Farm ID: {farm_id}")
    fleet_logger.info(f"[Init Fleet] - Using MQTT Broker: {mqtt_broker}:{mqtt_port}")

    ros2_client = create_ros2_action_client(farm_id, fleet_logger)

    # --- Initialize the publisher client with dynamic broker/port ---
    initialize_publisher_client(farm_id, fleet_logger, mqtt_broker, mqtt_port)
    
    # --- Start the subscriber with dynamic broker/port ---
    start_mqtt_sub(farm_id, ros2_client, fleet_logger, mqtt_broker, mqtt_port)
    
    fleet_logger.info("[Init Fleet] - Fleet Manager system is running. Press Ctrl+C to exit.")

    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        fleet_logger.info("[Init Fleet] - Fleet Manager system shutting down gracefully")


if __name__ == "__main__":
    """
    Command-line entry point for initializing the Fleet Manager system.

    This script starts a Fleet Manager instance for a specific farm,
    identified by its unique ID, and configures the MQTT broker connection.

    Example:
        python init_fleet.py --farm_id 1 --mqtt_broker 192.168.1.10 --mqtt_port 1883

    :raises SystemExit: If the farm ID is invalid or a critical error occurs.
    """
    parser = argparse.ArgumentParser(
        description='Fleet Manager System - Agricultural Drone Management Platform'
    )
    parser.add_argument('--farm_id', type=int, required=True, help='Unique ID of the farm instance')
    
    # --- New MQTT Broker arguments ---
    parser.add_argument(
        '--mqtt_broker', 
        type=str, 
        default="10.138.137.150", 
        help='Hostname or IP address of the MQTT broker'
    )
    parser.add_argument(
        '--mqtt_port', 
        type=int, 
        default=1884, 
        help='Port number of the MQTT broker'
    )
    # --- End New Arguments ---
    
    args = parser.parse_args()

    if args.farm_id <= 0:
        print("Error: Farm ID must be a positive integer")
        sys.exit(1)

    try:
        main(args.farm_id, args.mqtt_broker, args.mqtt_port)
    except Exception as e:
        print(f"Critical error in fleet system: {str(e)}")
        sys.exit(1)