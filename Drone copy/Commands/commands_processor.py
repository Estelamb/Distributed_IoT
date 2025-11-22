"""
This module is responsible for parsing, interpreting, and executing mission commands
received by the drone's ROS2 Action Server. It serves as the bridge between the
high-level mission definitions (sent by the Fleet Manager) and the specific simulated
actions implemented in the `Commands.commands_simulator` module.

Main responsibilities:
-----------------------
1. **Parse** incoming command JSON strings and normalize their keys.
2. **Select** the appropriate command class (e.g., `TakeOff`, `Land`, `Go_Waypoint`, etc.).
3. **Execute** the command and capture its result or error.
4. **Return** structured feedback to the calling ROS2 Action Server.

Expected command format:
------------------------
Each command is expected to be a JSON-formatted string with the following structure:

.. code-block:: json

    {
        "missionId": 101,
        "commandId": 1,
        "commandType": "TAKEOFF",
        "params": {
            "altitude": 10
        }
    }

The parser accepts both *camelCase* and *snake_case* naming conventions for compatibility.

"""

import json
import logging
from typing import Tuple, Dict
from Commands.commands_simulator import (
    Command,
    TakeOff,
    Land,
    Go_Waypoint,
    Go_Home,
    Take_Picture,
    Analyse_Image,
    Treatment
)


def process_command(command_str: str, logger: logging.Logger) -> Tuple[int, Dict]:
    """
    Process a command received as a JSON string, execute it, and return the result.

    This function orchestrates the full command lifecycle:
    - Parses the input JSON string into a Python dictionary.
    - Selects the corresponding command class based on its type.
    - Executes the command using its `execute()` method.
    - Collects and returns the resulting data or error details.

    :param command_str: Command in JSON string format (supports both camelCase and snake_case).
    :type command_str: str
    :param logger: Logger instance used to record command execution progress and results.
    :type logger: logging.Logger
    :return: A tuple containing the execution status and the resulting data.
    :rtype: Tuple[int, Dict]
    :raises ValueError: If the input command string cannot be parsed as valid JSON.
    """
    command_dict = parse_command(command_str, logger)
    command = select_command(command_dict, logger)

    try:
        command.execute()
        command_data = command.result()
        status_code = 0  # success
        logger.info("[COMMANDS] - Command executed successfully")
    except Exception as e:
        command_data = {"error": str(e)}
        status_code = 1  # failure
        logger.error(f"[COMMANDS] - Command execution failed: {e}")

    return status_code, command_data


def parse_command(command_str: str, logger: logging.Logger) -> Dict:
    """
    Parse a JSON-formatted string and normalize its keys to `snake_case`.

    This function allows flexible input by supporting both `camelCase` and `snake_case`
    naming conventions in the incoming JSON. It ensures that the resulting dictionary
    always contains consistent field names.

    :param command_str: JSON string containing the command definition.
    :type command_str: str
    :param logger: Logger instance used for debug or error messages.
    :type logger: logging.Logger
    :return: Normalized command dictionary with the following keys:
             - `mission_id`
             - `command_id`
             - `command_type`
             - `params`
    :rtype: Dict
    :raises ValueError: If the JSON string is invalid or cannot be parsed.
    """
    try:
        command_dict = json.loads(command_str)
    except json.JSONDecodeError:
        if logger:
            logger.error("[COMMANDS] - Invalid JSON format in command string")
        raise ValueError("Invalid command string format")

    normalized_dict = {
        'mission_id': command_dict.get('missionId') or command_dict.get('mission_id'),
        'command_id': command_dict.get('commandId') or command_dict.get('command_id'),
        'command_type': command_dict.get('commandType') or command_dict.get('command_type'),
        'params': command_dict.get('params', {})
    }

    if logger:
        logger.debug(f"[COMMANDS] - Parsed command: {normalized_dict}")

    return normalized_dict


def select_command(command_dict: Dict, logger: logging.Logger) -> Command:
    """
    Select the appropriate command class based on the `command_type` field.

    This function maps each command type to its corresponding command class from
    the `Commands.commands_simulator` module. If the command type is unrecognized,
    it defaults to the base `Command` class.

    :param command_dict: Dictionary containing normalized command data.
    :type command_dict: Dict
    :param logger: Logger instance used for logging the command type being processed.
    :type logger: logging.Logger
    :return: Instance of a command class ready to be executed.
    :rtype: Command
    """
    mission_id = command_dict['mission_id']
    command_id = command_dict['command_id']
    command_type = command_dict['command_type']
    params = command_dict['params']

    command_map = {
        'TAKEOFF': (TakeOff, "TAKEOFF"),
        'LAND': (Land, "LAND"),
        'GO_WAYPOINT': (Go_Waypoint, "GO_WAYPOINT"),
        'GO_HOME': (Go_Home, "GO_HOME"),
        'TAKE_PICTURE': (Take_Picture, "TAKE_PICTURE"),
        'ANALYSE_IMAGE': (Analyse_Image, "ANALYSE_IMAGE"),
        'TREATMENT': (Treatment, "TREATMENT"),
    }

    command_class, log_name = command_map.get(command_type.upper(), (Command, "UNKNOWN"))

    if logger:
        logger.info(f"[COMMANDS] - Processing {log_name} command")

    return command_class(mission_id, command_id, command_type, params, logger)
