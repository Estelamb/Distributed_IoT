import argparse
import logging
import sys
from concurrent_log_handler import ConcurrentRotatingFileHandler

from src.commands_action_interface.commands_action_interface.commands_action_server import run_action_server


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


def main(drone_id: int) -> None:
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
