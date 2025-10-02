import argparse
import logging
import queue
import sys
import threading

from concurrent_log_handler import ConcurrentRotatingFileHandler

from MQTT.mqtt_sub import start_mqtt_sub
from src.commands_action_interface.commands_action_interface.commands_action_client import create_ros2_action_client

def create_logger(logger_name: str, log_file: str) -> logging.Logger:
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

def main(farm_id: int) -> None:
    fleet_logger = create_logger(f'Farm {farm_id}', f'Logs/Farm_{farm_id}.log')
    fleet_logger.info(f"[Init Fleet] - Initializing Fleet Manager system for Farm ID: {farm_id}")

    ros2_client = create_ros2_action_client(farm_id, fleet_logger)

    start_mqtt_sub(farm_id, ros2_client, fleet_logger)
    
    fleet_logger.info("[Init Fleet] - Fleet Manager system is running. Press Ctrl+C to exit.")

    try:
        threading.Event().wait()
    except KeyboardInterrupt:
        fleet_logger.info("[Init Fleet] - Fleet Manager system shutting down gracefully")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description='Fleet Manager System - Agricultural Drone Management Platform'
    )
    parser.add_argument('--farm_id', type=int, required=True, help='Unique ID of the farm instance')
    args = parser.parse_args()

    if args.farm_id <= 0:
        print("Error: Farm ID must be a positive integer")
        sys.exit(1)

    try:
        main(args.farm_id)
    except Exception as e:
        print(f"Critical error in fleet system: {str(e)}")
        sys.exit(1)
