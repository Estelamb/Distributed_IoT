import base64
import json
import os
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
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        self.logger.info("[TAKEOFF] - Taking off...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[TAKEOFF] - Takeoff successful")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "Takeoff successful", "latitude": self.latitude, "longitude": self.longitude}


class Land(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        self.logger.info("[LAND] - Landing...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[LAND] - Landing successful")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "Landing successful", "latitude": self.latitude, "longitude": self.longitude}


class Go_Waypoint(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        self.logger.info("[GO_WAYPOINT] - Going to WP...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[GO_WAYPOINT] - Waypoint reached")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "WP reached", "latitude": self.latitude, "longitude": self.longitude}


class Go_Home(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        self.logger.info("[GO_HOME] - Returning home...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[GO_HOME] - Return home successful")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": "Returned home", "latitude": self.latitude, "longitude": self.longitude}



class Take_Picture(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        self.camera_device = params["cameraDevice"]
        self.image_b64 = None
        
    def execute(self):
        self.logger.info("[TAKE_PICTURE] - Taking picture...")

        # Simula la captura de una imagen
        image_path = os.path.join("Data", f"{self.latitude:.6f}_{self.longitude:.6f}.png")

        try:
            with open(image_path, "rb") as img_file:
                image_bytes = img_file.read()
                self.image_b64 = "data:image/png;base64," + base64.b64encode(image_bytes).decode("utf-8")
                self.logger.info(f"[TAKE_PICTURE] - Image loaded from {image_path}")
        except FileNotFoundError:
            self.logger.warning(f"[TAKE_PICTURE] - Image not found at {image_path}")
            self.image_b64 = None

        time.sleep(1)

    def result(self):

        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": [f"Picture taken with {self.camera_device}", self.image_b64], "latitude": self.latitude, "longitude": self.longitude}


class Analyse_Image(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        self.analyse_model = params["analyseModel"]
        self.amount = None
        self.image_b64 = None
        
    def execute(self):
        self.logger.info("[ANALYSE_IMAGE] - Analysing image...")

        key = f"{self.latitude:.6f}_{self.longitude:.6f}"
        amount_path = os.path.join("Data", "amounts.json")
        analysed_img_path = os.path.join("Data", f"{key}_analysed.png")

        # Leer amount desde JSON
        try:
            with open(amount_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.amount = data.get(key, 0)
        except Exception as e:
            self.logger.error(f"[ANALYSE_IMAGE] - Error reading {amount_path}: {e}")
            self.amount = 0

        # Leer imagen analizada
        try:
            with open(analysed_img_path, "rb") as img_file:
                image_bytes = img_file.read()
                self.image_b64 = "data:image/png;base64," + base64.b64encode(image_bytes).decode("utf-8")
                self.logger.info(f"[ANALYSE_IMAGE] - Loaded analysed image {analysed_img_path}")
        except FileNotFoundError:
            self.logger.warning(f"[ANALYSE_IMAGE] - Analysed image not found at {analysed_img_path}")
            self.image_b64 = None

        time.sleep(1)
        
    def result(self):
        self.logger.info("[ANALYSE_IMAGE] - Analysis completed")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": [f"Picture analysed: {self.amount}", self.image_b64], "latitude": self.latitude, "longitude": self.longitude}


class Treatment(Command):
    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        self.treatment = params["treatment"]
        self.amount = params["amount"]
        
    def execute(self):
        self.logger.info("[TREATMENT] - Applying treatment...")
        time.sleep(10)
        
    def result(self):
        self.logger.info("[TREATMENT] - Treatment applied successfully")
        return {"mission_id": self.mission_id, "command_id": self.command_id, "message": f"Treatment applied: {self.treatment}-{self.amount}", "latitude": self.latitude, "longitude": self.longitude}