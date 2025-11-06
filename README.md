# PRECISE Truck

The PRECISE Truck is an adaptation of the autonomous [RCCar](https://github.com/RISE-Dependable-Transport-Systems/RCCar) tailored for PRECISE project to run Security Chaos Engineering experiments and validation work.

## Installing Prerequisites (on Ubuntu 22.04/24.04) & Building

> **Note:** Latest **MAVSDK 2.x** is required (support for version 3.x is not yet available).  
> Pre-built releases can be found [here](https://github.com/mavlink/MAVSDK/releases).  
> To instead build MAVSDK from source, simple [scripts](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/tools/build_MAVSDK) can be found in the **WayWise** repository.

    # Installing MAVSDK
    sudo dpkg -i libmavsdk-dev*.deb

    sudo apt install git build-essential cmake libqt5serialport5-dev libgpiod-dev
    mkdir src && cd src
    git clone git@github.com:RISE-Dependable-Transport-Systems/precise-truck.git precise-truck
    cd precise-truck
    git submodule update --init
    cd ../../
    mkdir build && cd build
    cmake ../src/precise-truck
    make -j4

## Truck Configuration and Launch Guide

The `RCTruck` executable can be configured using either a **JSON configuration file** or **command-line arguments**. Command-line arguments override JSON values, and JSON values override the built-in defaults.

### 1. JSON Configuration (Recommended)

Create a JSON file specifying parameters such as truck ID, trailer ID, geometry, control, and communication settings. Example:

```json
{
  "truck_id": 1,
  "truck_length": 0.5,
  "truck_width": 0.21,
  "axis_distance": 0.3,
  "turn_radius": 0.67,
  "angle_sensor_offset": 90.0,
  "update_period_ms": 25,
  "control_tower_ip": "127.0.0.1",
  "control_tower_port": 14540
}
```

### 2. Command-Line Arguments (Limited Support)

You can configure only limited parameters at runtime using command-line arguments.

| Option                  | Description                                                          |
| ----------------------- | -------------------------------------------------------------------- |
| `-c, --config <file>`   | Path to a JSON configuration file containing truck parameters.       |
| `-i, --truck-id <id>`   | Set the truck ID (overrides JSON or default).                        |
| `-l, --trailer-id <id>` | Set the trailer ID (overrides JSON or default).                      |
| `-a, --attach-trailer`  | Flag to attach the trailer to the truck (overrides JSON or default). |

#### Example Usage

```bash
# Use a JSON configuration file
./RCTruck --config ../src/precise-truck/config/truck_config.json

# Override truck ID and attach trailer
./RCTruck -c ../src/precise-truck/config/truck_config.json -i 2 -l 26 -a
```
