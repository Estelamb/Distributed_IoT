Node-RED Installation
=====================

This guide explains how to install **Node.js** and **Node-RED** on Windows using **Chocolatey** and **npm**.

Install Node.js
---------------

Download and Install
~~~~~~~~~~~~~~~~~~~~

1. Open **PowerShell** as Administrator.

2. Install **Chocolatey** (a Windows package manager):

   .. code-block:: powershell

      powershell -c "irm https://community.chocolatey.org/install.ps1|iex"

3. Use Chocolatey to install **Node.js** version 24.11.1:

   .. code-block:: powershell

      choco install nodejs --version="24.11.1"

4. Verify the installation:

   .. code-block:: powershell

      node -v   # Should print "v24.11.1"
      npm -v    # Should print "11.6.2"

Reboot the laptop after installation.

Install Node-RED
----------------

After Node.js is installed, use **npm** to install Node-RED globally:

.. code-block:: powershell

   npm install -g --unsafe-perm node-red

Reboot the laptop once the installation completes.

Run Node-RED
------------

To start Node-RED, open **PowerShell** and run:

.. code-block:: powershell

   node-red

This will start the Node-RED server. You can then open your browser and navigate to:

.. code-block:: powershell
    http://localhost:1880


to access the Node-RED editor.

References
~~~~~~~~~~~

- Node-RED Official Documentation (On Windows): https://nodered.org/docs/getting-started/windows
- Node.js Official Website (Download): https://nodejs.org/en/download
