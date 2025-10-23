How to run
==========

Requirements
------------
- Laptop/PC (Windows 11) with Docker installed (Follow the :doc:`docker` tutorial to install Docker on your machine).
- Raspberry Pi 5 with Raspberry Pi OS (64-bit). (Optional)


Fleet Manager
-------------

If it is going to run on the Laptop/PC with Windows 11, start ``Docker Desktop``.

In the ``Fleet Manager`` directory, run the following command:

.. code-block:: bash

   docker network create --driver=bridge --subnet=192.168.100.0/24 --gateway=192.168.100.1 fleet_net

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

To execute the ``Drone``, execute:

.. code-block:: bash

   docker compose build
   docker compose up

Send a Mission
--------------

To send a mission, go to the ``User`` directory and execute:

.. code-block:: bash

   python send_mission.py <Drone ID> <Mission file>

Replace ``<Drone ID>`` with the ID of the drone and ``<Mission file>`` with the name of your mission json file.

