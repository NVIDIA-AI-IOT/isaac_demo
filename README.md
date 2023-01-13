# isaac_demo

A combined set of demo working with Isaac SIM on a workstation and Isaac ROS on a NVIDIA Jetson AGX Orin

## Hardware required

Workstation:

1. x86/64 machine
2. NVIDIA Graphic card with RTX
3. Display
4. Keyboard and Mouse

NVIDIA Jetson:

1. NVIDIA Jetson AGX Orin
2. Jetpack 5.0.2

Tools:

1. Router
2. eth cables

## Install

Clone this repository and move

```console
git clone ...
cd ...
```

Run the installer

```console
./isaac_demo.sh
```

If you are on NVIDIA Jetson AGX Orin you will see start a new terminal inside a docker image, run the command below

```console
bash src/isaac_demo/scripts/run_in_docker.sh
```
