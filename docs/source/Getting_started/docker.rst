Docker Installation
===================

This guide covers the installation of Docker on two different platforms:
Docker Desktop for Windows and Docker Engine for Raspberry Pi 5 (RPI5).

.. contents::
   :local:
   :depth: 2

Docker Desktop for Windows
--------------------------

Download and Install
~~~~~~~~~~~~~~~~~~~~

1. Visit the official Docker Desktop download page:

   https://www.docker.com/products/docker-desktop/

2. Download the **Docker Desktop for Windows** installer.

3. Run the installer and follow the on-screen instructions.

4. After installation, start Docker Desktop from the Start menu.

Verify Installation
~~~~~~~~~~~~~~~~~~~

To verify Docker is working, open **PowerShell** and run:

.. code-block:: powershell

   docker run hello-world

You should see a message confirming that Docker is installed and running correctly.



Docker Engine for RPI5
----------------------

Set up Docker’s APT repository
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

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

Reboot the RPI5.

Install the Docker packages
~~~~~~~~~~~~~~~~~~~~~~~~~~~

To install the latest versions of Docker components, run:

.. code-block:: bash

   sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

Verify the installation
~~~~~~~~~~~~~~~~~~~~~~~

To confirm Docker is installed and working correctly, run:

.. code-block:: bash

   sudo docker run hello-world

You should see a message confirming successful installation.

Add Docker sudo privileges
~~~~~~~~~~~~~~~~~~~~~~~~~~~

To manage Docker as a non-root user, add your user to the `docker` group:

.. code-block:: bash

   sudo usermod -aG docker $USER

Log out and log back in for the changes to take effect.

References
----------

- Docker Desktop for Windows documentation:
  https://docs.docker.com/desktop/install/windows-install/

- Docker Documentation – Install Docker Engine on Raspberry Pi OS:
  https://docs.docker.com/engine/install/raspberry-pi-os/
