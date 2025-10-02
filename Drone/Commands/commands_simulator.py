import time
import random
import logging
from abc import ABC, abstractmethod

class Command(ABC):
    def __init__(self, mission_id: int, command_id: int, command_type: str, 
                 logger: logging.Logger):
        self.mission_id = mission_id
        self.command_id = command_id
        self.command_type = command_type
        self.logger = logger

    @abstractmethod
    def execute(self):
        pass
    
    @abstractmethod
    def result(self):
        pass


class TakeOff(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params[0]
        self.longitude = params[1]
        
    def execute(self):
        self.logger.info("[TAKEOFF] - Taking off...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[TAKEOFF] - Takeoff successful")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "Takeoff successful", "latitude": self.latitude, "longitude": self.longitude}


class Land(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params[0]
        self.longitude = params[1]
        
    def execute(self):
        self.logger.info("[LAND] - Landing...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[LAND] - Landing successful")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "Landing successful", "latitude": self.latitude, "longitude": self.longitude}


class Go_Waypoint(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params[0]
        self.longitude = params[1]
        
    def execute(self):
        self.logger.info("[GO_WAYPOINT] - Going to WP...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[GO_WAYPOINT] - Waypoint reached")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "WP reached", "latitude": self.latitude, "longitude": self.longitude}


class Go_Home(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params[0]
        self.longitude = params[1]
        
    def execute(self):
        self.logger.info("[GO_HOME] - Returning home...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[GO_HOME] - Return home successful")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "Returned home", "latitude": self.latitude, "longitude": self.longitude}


class Take_Picture(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params[0]
        self.longitude = params[1]
        self.camera_device = params[2]
        
    def execute(self):
        self.logger.info("[TAKE_PICTURE] - Taking picture...")
        time.sleep(1)
        
        self.logger.info(f"[TAKE_PICTURE] - Picture taken successfully as {self.latitude}_{self.longitude}.jpg")

    def result(self):

        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": f"Picture taken with {self.camera_device}", "latitude": self.latitude, "longitude": self.longitude}


class Analyse_Image(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params[0]
        self.longitude = params[1]
        self.analyse_model = params[2]
        self.amount = None
        
    def execute(self):
        self.logger.info("[ANALYSE_IMAGE] - Analysing image...")
        time.sleep(1)
        
    def result(self):
        self.logger.info("[ANALYSE_IMAGE] - Analysis completed")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": f"Picture analysed: {self.amount}", "latitude": self.latitude, "longitude": self.longitude}


class Treatment(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params[0]
        self.longitude = params[1]
        self.treatment = params[2]
        self.amount = params[3]
        
    def execute(self):
        self.logger.info("[TREATMENT] - Applying treatment...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[TREATMENT] - Treatment applied successfully")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": f"Treatment applied: {self.treatment}-{self.amount}", "latitude": self.latitude, "longitude": self.longitude}