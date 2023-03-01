![isaac_demo](.docs/isaac_demo.gif)

# isaac_demo

A combined set of demo working with Isaac SIM on a workstation and Isaac ROS on a NVIDIA Jetson AGX Orin

## Hardware required

Workstation:

1. x86/64 machine
2. Install Ubuntu 20.04
3. NVIDIA Graphic card with RTX
4. Display
5. Keyboard and Mouse

NVIDIA Jetson:

1. NVIDIA Jetson AGX Orin
2. Jetpack 5.0.2

Tools:

1. Router
2. eth cables

# Setup hardware

Before to start check you have all requirements and connect the driver following this image

![hardware-setup](.docs/hardware-setup.drawio.png)

It is preferable to connect workstation and the NVIDIA Jetson AGX Orin with a lan cable and not use WiFi.

# Install

There are two steps to follow, Install FoxGlove and Install Isaac ROS

Follow:

* Install on Jetson
* Install on workstation

## Install on NVIDIA Jetson Orin

Install on your NVIDIA Jetson Orin [Jetpack 5+](https://developer.nvidia.com/embedded/jetpack)

After installation save IP address and hostname
### Connect remotely

In this section you connect to your NVIDIA Jetson with a ssh connection, open a terminal an write

```console
ssh <IP or hostname.local>
```

where **IP** is the of NVIDIA Jetson or **hostname** is the hostname of your board.

If you are connected the output from the terminal is:

![ssh-terminal-orin.png](.docs/ssh-terminal-orin.png)

### Install Isaac Demo

Clone this repository and move to repository folder

```console
git clone https://github.com/rbonghi/isaac_demo.git
cd isaac_demo
```

Add docker group to your user

```console
sudo usermod -aG docker $USER && newgrp docker
```

Run the installer

```console
./isaac_demo.sh
```

If everything is going well (need time before to be done) the terminal will show this output

![Isaac Demo installed on Orin](.docs/isaac_demo_install_orin.png)

## Install on workstation

In this first part, you install different software on your workstation.

* NVIDIA Isaac SIM
* Foxglove
* This repository

### NVIDIA Isaac SIM

Follow the documentation on NVIDIA Isaac SIM [Workstation install](https://docs.omniverse.nvidia.com/app_isaacsim/app_isaacsim/install_workstation.html)

1. Download the [Omniverse Launcher](https://www.nvidia.com/en-us/omniverse/)
2. [Install Omniverse Launcher](https://docs.omniverse.nvidia.com/prod_launcher/prod_launcher/installing_launcher.html)
3. Install [Cache](https://docs.omniverse.nvidia.com/prod_nucleus/prod_utilities/cache/installation/workstation.html) from the Omniverse Launcher
4. Install [Nucleus](https://docs.omniverse.nvidia.com/prod_nucleus/prod_nucleus/workstation/installation.html) from the Omniverse Launcher

Open Omniverse Launcher

![Omniverse launcher](https://docs.omniverse.nvidia.com/app_isaacsim/_images/isaac_main_launcher_exchange.png)

Move to Library and choice "Omniverse Isaac SIM" and download the latest 2022.2 version

![Omniverse library](https://docs.omniverse.nvidia.com/app_isaacsim/_images/isaac_main_launcher_library.png)

### Foxglove on Desktop

Download the latest [foxglove](https://foxglove.dev/download) version for your desktop

```console
sudo apt install ./foxglove-studio-*.deb
sudo apt update
sudo apt install -y foxglove-studio
```

### Isaac SIM and Isaac DEMO

Clone this repository and move to repository folder

```console
git clone https://github.com/rbonghi/isaac_demo.git
cd isaac_demo
```

Now follow the Run demo to start the simulation

# Run demo

From your workstation now you need to do two extra steps

## Setup foxglove

1. Open foxglove
2. Set up **Open connection**

![Foxglove setup connection](.docs/01-foxglove-setup-connection.png)

3. Select **ROS2** and **Open**

![Foxglove ROS2](.docs/02-foxglove-connection.png)

## Run simulation on Jetson

If you are on NVIDIA Jetson AGX Orin you will see start a new terminal inside a docker image, run the command below


```console
bash src/isaac_demo/scripts/run_in_docker.sh
```

Well done! Now all demo is running!
