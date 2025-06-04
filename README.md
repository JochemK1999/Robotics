# ESP32 Line-Following Robot with Dijkstra Pathfinding

[View this project on GitHub](https://github.com/JochemK1999/Robotics)

This repository contains two main components:

1. **ESP32 MicroPython Code**: Hosts a server on the ESP32 that:
   - Accepts JSON pathfinding requests (`{"start": "...", "goal": "..."}`), runs Dijkstra’s algorithm on a predefined graph, and returns the shortest path.

2. **Webots Controller (Python)**: Runs in the Webots simulation environment:
   - Connects to the ESP32’s server over Wi-Fi.
   - Requests a path from a given start node to a goal node.
   - Drives the puck robot along black lines using three ground sensors and a PID controller along the route provided by the ESP.

---

## Table of Contents

- [Hardware Requirements](#hardware-requirements)
- [Software Requirements](#software-requirements)
- [Installation and Setup](#installation-and-setup)
  - [ESP32 MicroPython Firmware](#esp32-micropython-firmware)
  - [ESP32 Code Configuration](#esp32-code-configuration)
  - [Webots Simulation Setup](#webots-simulation-setup)
- [Usage](#usage)
  - [Configure Start/Goal Nodes](#configure-startgoal-nodes)
  - [Run the ESP32 Script](#run-the-esp32-script)
  - [Launch Webots Controller](#launch-webots-controller)

---

## Hardware Requirements

- ESP32 Development Board (e.g., DevKit v1)
- USB cable for flashing MicroPython and serial debugging
- Wi-Fi network accessible by both PC and ESP32

## Software Requirements

1. **ESP32 Side (MicroPython)**
   - MicroPython firmware (≥ 1.19.1) for ESP32
   - Serial terminal (Thonny)
   - Python 3.x (optional for testing)
   - All libraries used are packaged with Python 3.x or MicroPython

2. **PC Side (Webots Simulation)**
   - Webots R2024b or later
   - Python 3.x (Webots uses 3.9+)
   - All libraries used are packaged with Python 3.x or WeBots

## Installation and Setup

### ESP32 MicroPython Firmware

1. Download the latest ESP32 MicroPython `.bin` from https://micropython.org/download/ESP32_GENERIC/
2. Follow the tutorial on https://randomnerdtutorials.com/getting-started-thonny-micropython-python-ide-esp32-esp8266/ to flash MicroPython using Thonny IDE

### ESP32 Code Configuration

1. Edit `main.py` and set your Wi-Fi credentials:

```python
SSID = "YOUR_WIFI_SSID"
PASSWORD = "YOUR_WIFI_PASSWORD"
```

2. Upload `main.py` to ESP32:

3. Reboot ESP32 and note its IP address (printed on serial).

### Webots Simulation Setup

1. Install Webots R2024b (https://cyberbotics.com).
2. In Webots, open world.wbt
3. Create a python controller `Controller.py` in webots and copy the code in `WiFiController.py` to the controller
4. In the controller, set `ESP32_IP` to the IP printed by ESP32
5. Set the robot’s controller to `Controller.py`.

## Usage

### Configure Start/Goal Nodes

1. In `Controller.py`, set the start node to the node the robot is currently on (N in the current world file) and the start direction to the direction the robot is currently facing:

```python
START_NODE = 'X'
START_DIRECTION = 0.0 # 0.0 = right, np.pi = left, 0.5*np.pi = up, -0.5*np.pi = down 
```

2. In `Controller.py`, set the goal node to the node where you want the robot to go:

```python
GOAL_NODE = 'I'
```

Change as needed for your map.

### Run the ESP32 Script

1. Power on ESP32.
2. Ensure serial output shows:

```
Connected to Wi-Fi. IP: <<IP_ADDRESS>>
Waiting for PC connection...
```

### Launch Webots Controller

1. In Webots, press “Play”.
2. The controller connects to ESP32 and requests a path; you’ll see:

```
Connected to ESP32 over Wi-Fi.
I am going to take this path: <<START_NODE>> -> … -> <<GOAL_NODE>>
```

3. Watch the robot follow the black line, make turns at junctions, and stop at the goal.