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
    Procesa un comando en formato JSON, lo ejecuta y devuelve el resultado.
    
    :param command_str: Comando en formato JSON (puede usar camelCase o snake_case)
    :param logger: Logger para registrar la ejecución
    :return: Tuple[int, Dict] -> (status_code, command_data)
    """
    command_dict = parse_command(command_str, logger)
    
    command = select_command(command_dict, logger)
    
    try:
        command.execute()
        command_data = command.result()
        status_code = 0  # éxito
        logger.info("[COMMANDS] - Command executed successfully")
    except Exception as e:
        command_data = {"error": str(e)}
        status_code = 1  # fallo
        logger.error(f"[COMMANDS] - Command execution failed: {e}")
    
    return status_code, command_data


def parse_command(command_str: str, logger: logging.Logger) -> Dict:
    """
    Parsea un string JSON y normaliza las claves a snake_case.
    Acepta tanto camelCase como snake_case en la entrada.
    """
    try:
        command_dict = json.loads(command_str)
    except json.JSONDecodeError:
        if logger:
            logger.error("[COMMANDS] - Invalid JSON format in command string")
        raise ValueError("Invalid command string format")
    
    # Normalizar claves
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
    Selecciona la clase de comando adecuada según command_type.
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
