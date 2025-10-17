"""
init_drone.py
==============

This module initializes the ROS2 drone system and starts the drone's Action Server.
It is the entry point for the **Drone Container** in the distributed UAV Fleet Management
architecture.

Main responsibilities:
1. **Initialize logging** for each drone instance (both console and rotating file logs).
2. **Start the ROS2 Action Server**, enabling communication with the Fleet Manager.
3. **Graceful shutdown** when the process is stopped.

The drone instance listens for mission goals sent via ROS2 Actions and simulates their
execution step-by-step, reporting feedback and results back to the Fleet Manager.

Usage example:
--------------
.. code-block:: bash

    python init_drone.py --drone_id 1

This will initialize a drone node with ID 1, creating a dedicated log file in
``Logs/Drone_1.log`` and registering its ROS2 Action Server.
"""

import argparse
import logging
import sys
from concurrent_log_handler import ConcurrentRotatingFileHandler

from src.commands_action_interface.commands_action_interface.commands_action_server import run_action_server


def create_logger(logger_name: str, log_file: str) -> logging.Logger:
    """
    Creates and configures a logger instance for the drone.

    The logger outputs to both console and file (with rotation). Each drone
    maintains its own log file for easier debugging and traceability.

    :param logger_name: Name of the logger instance (typically the drone ID).
    :type logger_name: str
    :param log_file: Path to the log file.
    :type log_file: str
    :return: Configured logger instance.
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


def main(drone_id: int) -> None:
    """
    Main entry point for the drone system.

    Initializes the ROS2 Action Server for the given drone ID and begins listening
    for mission commands from the Fleet Manager.

    :param drone_id: Unique identifier of the drone instance.
    :type drone_id: int
    :return: None
    """
    logger = create_logger(f'Drone {drone_id}', f'Logs/Drone_{drone_id}.log')
    logger.info(f"[Init ROS2] - Initializing ROS2 drone system for drone ID: {drone_id}")

    run_action_server(drone_id, logger)

    logger.info("[Init ROS2] - Drone system shut down gracefully")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='ROS2 Drone Control System - Agricultural Drone Management Platform'
    )
    parser.add_argument('--drone_id', type=int, required=True, help='Unique ID of the drone instance')
    args = parser.parse_args()

    if args.drone_id <= 0:
        print("Error: Drone ID must be a positive integer")
        sys.exit(1)

    try:
        main(args.drone_id)
    except Exception as e:
        print(f"Critical error in drone system: {str(e)}")
        sys.exit(1)
