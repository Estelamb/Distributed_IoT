"""
This module simulates the execution of drone mission commands in a controlled
environment. It defines a hierarchy of command classes representing possible
actions a drone can perform, such as takeoff, landing, moving to a waypoint,
taking pictures, analyzing images, and applying treatments.

Each command follows a consistent interface defined by the abstract base class
`Command`, which enforces the implementation of two key methods:
- :meth:`execute`: Simulates the command's operation.
- :meth:`result`: Returns structured data summarizing the outcome.

This simulation layer allows for testing the drone mission workflow
without requiring physical drones or real sensors.

Typical command parameters
--------------------------
All commands expect a set of parameters passed as a dictionary, usually
containing:

- ``latitude`` (*float*): Target latitude coordinate.
- ``longitude`` (*float*): Target longitude coordinate.
- Additional fields depending on the command type (e.g., ``cameraDevice``, ``analyseModel``, etc.).

"""

import base64
import json
import os
import time
import random
import logging
from abc import ABC, abstractmethod


class Command(ABC):
    """
    Abstract base class for all simulated drone commands.

    Defines the common interface used by all command types
    and provides access to basic metadata and logging.

    :param mission_id: Mission identifier associated with this command.
    :type mission_id: int
    :param command_id: Unique identifier of this command within the mission.
    :type command_id: int
    :param command_type: Command type string (e.g., "TAKEOFF", "LAND").
    :type command_type: str
    :param logger: Logger instance for structured message output.
    :type logger: logging.Logger
    """

    def __init__(self, mission_id: int, command_id: int, command_type: str, 
                 logger: logging.Logger):
        self.mission_id = mission_id
        self.command_id = command_id
        self.command_type = command_type
        self.logger = logger

    @abstractmethod
    def execute(self):
        """Execute the command simulation."""
        pass
    
    @abstractmethod
    def result(self):
        """Return a structured dictionary with the command results."""
        pass


class TakeOff(Command):
    """
    Simulates a drone takeoff operation.

    :param params: Dictionary containing 'latitude' and 'longitude' keys.
    :type params: dict
    """

    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        """Simulates the drone ascending into flight."""
        self.logger.info("[TAKEOFF] - Taking off...")
        time.sleep(10)
        
    def result(self):
        """Returns confirmation of a successful takeoff."""
        self.logger.info("[TAKEOFF] - Takeoff successful")
        return {
            "mission_id": self.mission_id,
            "command_id": self.command_id,
            "message": "Takeoff successful",
            "latitude": self.latitude,
            "longitude": self.longitude
        }


class Land(Command):
    """
    Simulates a drone landing operation.

    :param params: Dictionary containing 'latitude' and 'longitude' keys.
    :type params: dict
    """

    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        """Simulates the drone descending and landing."""
        self.logger.info("[LAND] - Landing...")
        time.sleep(10)
        
    def result(self):
        """Returns confirmation of a successful landing."""
        self.logger.info("[LAND] - Landing successful")
        return {
            "mission_id": self.mission_id,
            "command_id": self.command_id,
            "message": "Landing successful",
            "latitude": self.latitude,
            "longitude": self.longitude
        }


class Go_Waypoint(Command):
    """
    Simulates movement to a specific waypoint.

    :param params: Dictionary containing 'latitude' and 'longitude' keys.
    :type params: dict
    """

    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        """Simulates travel to the target waypoint."""
        self.logger.info("[GO_WAYPOINT] - Going to waypoint...")
        time.sleep(10)
        
    def result(self):
        """Returns confirmation that the waypoint has been reached."""
        self.logger.info("[GO_WAYPOINT] - Waypoint reached")
        return {
            "mission_id": self.mission_id,
            "command_id": self.command_id,
            "message": "Waypoint reached",
            "latitude": self.latitude,
            "longitude": self.longitude
        }


class Go_Home(Command):
    """
    Simulates the drone returning to its home base.

    :param params: Dictionary containing 'latitude' and 'longitude' keys.
    :type params: dict
    """

    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        
    def execute(self):
        """Simulates the drone flying back to its home location."""
        self.logger.info("[GO_HOME] - Returning home...")
        time.sleep(10)
        
    def result(self):
        """Returns confirmation of a successful return."""
        self.logger.info("[GO_HOME] - Return home successful")
        return {
            "mission_id": self.mission_id,
            "command_id": self.command_id,
            "message": "Returned home",
            "latitude": self.latitude,
            "longitude": self.longitude
        }


class Take_Picture(Command):
    """
    Simulates taking a picture using the drone's camera.

    :param params: Dictionary containing 'latitude', 'longitude', and 'cameraDevice' keys.
    :type params: dict
    """

    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        self.camera_device = params["cameraDevice"]
        self.image_b64 = None
        
    def execute(self):
        """
        Simulates image capture from the drone camera.

        Attempts to load a predefined image from the `Data/` directory
        based on the current coordinates.
        """
        self.logger.info("[TAKE_PICTURE] - Taking picture...")

        image_path = os.path.join("Data", f"{self.latitude:.6f}_{self.longitude:.6f}.png")

        try:
            with open(image_path, "rb") as img_file:
                image_bytes = img_file.read()
                self.image_b64 = "data:image/png;base64," + base64.b64encode(image_bytes).decode("utf-8")
                self.logger.info(f"[TAKE_PICTURE] - Image loaded from {image_path}")
        except FileNotFoundError:
            self.logger.warning(f"[TAKE_PICTURE] - Image not found at {image_path}")
            self.image_b64 = None

        time.sleep(10)

    def result(self):
        """Returns a base64-encoded image string along with metadata."""
        return {
            "mission_id": self.mission_id,
            "command_id": self.command_id,
            "message": [f"Picture taken with {self.camera_device}", self.image_b64],
            "latitude": self.latitude,
            "longitude": self.longitude
        }


class Analyse_Image(Command):
    """
    Simulates image analysis using a preloaded model or mock dataset.

    :param params: Dictionary containing 'latitude', 'longitude', and 'analyseModel' keys.
    :type params: dict
    """

    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        self.analyse_model = params["analyseModel"]
        self.amount = None
        self.image_b64 = None
        
    def execute(self):
        """
        Simulates image processing and analysis.

        Loads a mock result from `Data/amounts.json` and an analyzed image
        from `Data/<lat>_<lon>_analysed.png` if available.
        """
        self.logger.info("[ANALYSE_IMAGE] - Analysing image...")

        key = f"{self.latitude:.6f}_{self.longitude:.6f}"
        amount_path = os.path.join("Data", "amounts.json")
        analysed_img_path = os.path.join("Data", f"{key}_analysed.png")

        try:
            with open(amount_path, "r", encoding="utf-8") as f:
                data = json.load(f)
            self.amount = data.get(key, 0)
        except Exception as e:
            self.logger.error(f"[ANALYSE_IMAGE] - Error reading {amount_path}: {e}")
            self.amount = 0

        try:
            with open(analysed_img_path, "rb") as img_file:
                image_bytes = img_file.read()
                self.image_b64 = "data:image/png;base64," + base64.b64encode(image_bytes).decode("utf-8")
                self.logger.info(f"[ANALYSE_IMAGE] - Loaded analysed image {analysed_img_path}")
        except FileNotFoundError:
            self.logger.warning(f"[ANALYSE_IMAGE] - Analysed image not found at {analysed_img_path}")
            self.image_b64 = None

        time.sleep(10)
        
    def result(self):
        """Returns analysis results and the processed image (if available)."""
        self.logger.info("[ANALYSE_IMAGE] - Analysis completed")
        return {
            "mission_id": self.mission_id,
            "command_id": self.command_id,
            "message": [f"Picture analysed: {self.amount}", self.image_b64],
            "latitude": self.latitude,
            "longitude": self.longitude
        }


class Treatment(Command):
    """
    Simulates the application of an agricultural treatment.

    :param params: Dictionary containing 'latitude', 'longitude', 'treatment', and 'amount' keys.
    :type params: dict
    """

    def __init__(self, mission_id, command_id, command_type, params, logger):
        super().__init__(mission_id, command_id, command_type, logger)
        self.latitude = params["latitude"]
        self.longitude = params["longitude"]
        self.treatment = params["treatment"]
        self.amount = params["amount"]
        
    def execute(self):
        """Simulates spraying or applying a treatment at a target location."""
        self.logger.info("[TREATMENT] - Applying treatment...")
        time.sleep(10)
        
    def result(self):
        """Returns confirmation of treatment application."""
        self.logger.info("[TREATMENT] - Treatment applied successfully")
        return {
            "mission_id": self.mission_id,
            "command_id": self.command_id,
            "message": f"Treatment applied: {self.treatment}-{self.amount}",
            "latitude": self.latitude,
            "longitude": self.longitude
        }
