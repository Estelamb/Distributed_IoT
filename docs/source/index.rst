Welcome to Farms Manager's Documentation
========================================

Welcome to the documentation of the Farms Manager project. This document provides a comprehensive guide to understanding, using, and developing a smart management system for unmanned autonomous vehicles in precision agriculture on edge.

In this documentation, you will find detailed explanations of the system's architecture, core functionalities, and deployment processes. Additionally, the document covers the usage instructions to facilitate smooth integration and further development.

Whether you are a developer, researcher, or end-user, this documentation aims to serve as a valuable resource for understanding and extending the capabilities of the Farms Manager system.

What is Farms Manager?
----------------------

Farms Manager is an innovative project designed to create a smart management system for unmanned autonomous vehicles used in precision agriculture. The system leverages edge computing to enhance the efficiency and effectiveness of agricultural operations, enabling real-time data processing and decision-making directly on the field.

In each farm, there is a central node that coordinates multiple autonomous vehicles (Fleet Manager), such as drones and ground robots. These vehicles are equipped with various sensors and tools to perform tasks like monitoring crop health, soil analysis, and automated planting or harvesting. 

The fleet manager comunicates with the vehicles via ROS2 actions, collects data, and executes commands to optimize farming operations. All the fleet managers are connected to Node Red via MQTT. 

Node Red provides a user interface for farmers and agricultural experts. This interface allows users to monitor the status of their missions and its results.

Missions and Commands
---------------------

A mission is a high-level task assigned to a drone of a specific farm, which consists of a series of commands that the autonomous vehicle must execute. The commands are the specific actions that the vehicle performs to accomplish the mission. 

Examples of commands include:

- Takeoff: The vehicle takes off from the ground.
- Go to Waypoint: The vehicle flies to a specified GPS coordinate.
- Capture Image: The vehicle captures an image of the crops.
- Analyse Image: The vehicle processes the captured image.
- Spray Treatment: The vehicle sprays the specific treatment on the crops.
- Return Home: The vehicle returns to its starting point.
- Land: The vehicle lands safely on the ground.

.. toctree::
   :maxdepth: 2
   
   getting_started
   modules