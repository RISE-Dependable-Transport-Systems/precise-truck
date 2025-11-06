# PRECISE Truck

The PRECISE Truck is an adaptation of the autonomous [RCCar](https://github.com/RISE-Dependable-Transport-Systems/RCCar) tailored for PRECISE project to run Security Chaos Engineering experiments and validation work.

## Installing Prerequisites (on Ubuntu 22.04/24.04) & Building

Note: Latest MAVSDK 2.x is required (support for version 3.x is not yet available) and pre-built releases can be found [here](https://github.com/mavlink/MAVSDK/releases). To instead build MAVSDK from source, simple [scripts](https://github.com/RISE-Dependable-Transport-Systems/WayWise/tree/main/tools/build_MAVSDK) can be found in the WayWise repository.

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
