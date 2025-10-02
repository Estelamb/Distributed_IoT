import random
import json
import logging
from typing import Tuple, Dict
from Commands.commands_simulator import Command, TakeOff, Land, Go_Waypoint, Go_Home, Take_Picture, Analyse_Image, Treatment

def process_command(command_str: str, logger: logging.Logger) -> Tuple[int, Dict]:
    command_dict = parse_command(command_str, logger)
    
    command = select_command(command_dict, logger)
    
    command.execute()
    
    command_data = command.result()
    
    logger.info("[COMMANDS] - Command executed successfully")
    
    return command_data

def parse_command(command_str: str, logger: logging.Logger) -> Dict:
    try:
        command_dict = json.loads(command_str)
    except json.JSONDecodeError:
        if logger:
            logger.error("[COMMANDS] - Invalid JSON format in command string")
        raise ValueError("Invalid command string format")
    return command_dict

def select_command(command_dict: Dict, logger: logging.Logger) -> Command:
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

    command_class, log_name = command_map.get(command_type, (Command, "UNKNOWN"))

    if logger:
        logger.info(f"[COMMANDS] - Processing {log_name} command")

    return command_class(mission_id, command_id, command_type, params, logger)

