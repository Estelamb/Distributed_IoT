How to run
==========

Requirements
------------
- Laptop/PC with Docker installed (Follow the :doc:`docker` tutorial to install Docker on your Windows machine).
- Laptop/PC with Node-Red installed (Follow the :doc:`nodered` tutorial to install Node-Red on your Windows machine).
- Raspberry Pi 5 with Raspberry Pi OS (64-bit). (Optional)


Fleet Manager
-------------

If it is going to run on the Laptop/PC with Windows 11, start ``Docker Desktop``.

In the ``Fleet Manager`` directory, run the following command:

.. code-block:: bash

   docker network create --driver=bridge --subnet=192.168.100.0/24 --gateway=192.168.100.1 fleet_net

Before executing the Fleet Manager, make sure to change the environment variables in the ``.env`` file to avoid conflicts if you are running multiple fleet managers.

.. code-block:: bash

   FARM_ID=1
   MQTT_BROKER_ADDR=10.138.137.150
   MQTT_PORT_NUM=1885

To start the ``Fleet Manager``, execute:

.. code-block:: bash

   docker compose build
   docker compose up

Drone
-----

If it is going to run on the Laptop/PC with Windows 11, start ``Docker Desktop``.

In the ``Drone`` directory, run the following command (If it's the same machine as the Fleet Manager, skip this step):

.. code-block:: bash

   docker network create --driver=bridge --subnet=192.168.100.0/24 --gateway=192.168.100.1 fleet_net

Before executing the drone, make sure to change the ``DRONE_ID`` environment variables in the ``.env`` file to avoid conflicts if you are running multiple drones.

.. code-block:: bash

   DRONE_ID=1

To execute the ``Drone``, execute:

.. code-block:: bash

   docker compose build
   docker compose up

Start Node-Red
--------------

To start Node-Red, open a terminal and execute:

.. code-block:: bash

   node-red

You can open your browser and navigate to:

.. code-block:: bash

    http://localhost:1880

Install the palette ``node-red-dashboard`` and ``node-red-contrib-aedes`` in Node-Red.

Import the Node-Red json flow that is in the ``Node-Red`` directory and deploy it.

To see the Node-Red UI, open your browser and navigate to:

.. code-block:: bash

    http://localhost:1880/dashboard


