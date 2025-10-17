Docker Engine on the RPI5
==========================

Set up Docker’s APT repository
------------------------------

Add Docker's official GPG key:

.. code-block:: bash

   sudo apt-get update
   sudo apt-get install ca-certificates curl
   sudo install -m 0755 -d /etc/apt/keyrings
   sudo curl -fsSL https://download.docker.com/linux/raspbian/gpg -o /etc/apt/keyrings/docker.asc
   sudo chmod a+r /etc/apt/keyrings/docker.asc

Add Docker’s Repository to APT Sources:

.. code-block:: bash

   echo \
     "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/raspbian \
     $(. /etc/os-release && echo \"$VERSION_CODENAME\") stable" | \
     sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
   sudo apt-get update

Install the Docker packages
---------------------------

To install the latest versions of Docker components, run:

.. code-block:: bash

   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

Verify the installation
-----------------------

To confirm Docker is installed and working correctly, run:

.. code-block:: bash

   sudo docker run hello-world

You should see a message confirming successful installation.

References
----------

1. Docker Documentation – Install Docker Engine on Raspberry Pi OS: https://docs.docker.com/engine/install/raspberry-pi-os/