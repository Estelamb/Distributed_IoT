How to run
==========

Requirements
------------
- Raspberry Pi 5 with Raspberry Pi OS (64-bit).
- AI HAT + with Hailo8/Hailo8L.
- Pi Camera Module.

Installation
------------

Begin by following the installation instructions in the following tutorials:

- :doc:`ai_hat`
- :doc:`hailo_rpi5`
- :doc:`docker`

Fleet Manager
-------------

First, allow Docker to run without `sudo` by executing the following commands:

.. code-block:: bash
   sudo groupadd docker
   sudo usermod -aG docker $USER
   newgrp docker

In the ``FleetManager`` directory, run the following commands:

.. code-block:: bash

   docker network create --driver=bridge --subnet=192.168.100.0/24 --gateway=192.168.100.1 fleet_net

To start the ``Fleet Manager``, execute:

.. code-block:: bash

   docker compose build
   docker compose up

Drone
-----

To execute the ``ROS2-Drone`` part, in the ``Drone/ROS2`` directory, execute:

.. code-block:: bash

   docker compose build
   docker compose up

In a **separate terminal**, navigate to the ``Drone`` directory and run:

.. code-block:: bash

   sudo apt update
   sudo apt full-upgrade
   sudo apt install libssl-dev
   sudo apt install -y libcap-dev libcamera-dev python3-libcamera python3-picamera2
   sudo apt install libraspberrypi-bin
   sudo reboot

After rebooting, set up the Python environment:

.. code-block:: bash

   python3 -m venv --system-site-packages .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   sudo usermod -aG video $USER

Then, activate the environment and run the ``RPI-Drone``:

.. code-block:: bash

   source .venv/bin/activate
   python -m init_drone --drone_id 1

.. warning::

   It is very important to execute first the ``ROS2-Drone`` part and then, the ``RPI-Drone`` part, in that specific order.

Send a Mission
--------------

To send a mission, go to the ``Missions`` directory and execute:

.. code-block:: bash

   curl -X POST -H "Content-Type: application/json" --data-binary @<json file>.json http://127.0.0.1:8082/mission

Replace ``<json file>`` with the name of your mission file (without the ``.json`` extension).

ThingsBoard
-----------

You can access the ThingsBoard dashboard at: https://srv-iot.diatel.upm.es/