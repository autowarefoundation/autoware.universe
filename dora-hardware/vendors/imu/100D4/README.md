# 100D4

A Python module for accessing the 100D4 digital accelerometer and gyroscope developed by Dora.

## Table of Contents

- [Background](#background)
- [Install](#install)
- [Usage](#usage)
- [API](#api)
- [Contributing](#contributing)
- [License](#license)

## Background

### Overview:
100D4 is an advanced Microelectromechanical Systems (MEMS) Inertial Measurement Unit (IMU) chip developed by InvenSense (now part of TDK). It is a sensor that integrates a three-axis accelerometer and a three-axis gyroscope, capable of accurately measuring acceleration and angular velocity in three-

## Install

The installation of environment dependencies is realized through the requirements.txt file of this file：
```
pip install -r requirements.txt
```

## Usage
```
PATH=$PATH:$(pwd)
cd abc_project(create folder without-> dora new abc_project --lang python) 
#cd /home/crp/dora_project/dora-rs/dora-hardware/vendors/imu/100D4
dora up
sudo chmod 777 /dev/ttyUSB0
dora start dataflow.yml --name first-dataflow
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
dora logs first-dataflow custom_node_1
```
### After that, the content accepted by dora will appear   

**if you need to print the context to test the output:(Use the command line below)**  

- **try:RUST_LOG=debug dora start ./your path.yml --attach --hot-reload**  
- **dora logs (number) (node_name)** 
