# mpu6050

A Python module for accessing the MPU-6050 digital accelerometer and gyroscope developed by Dora.

## Table of Contents

- [Background](#background)
- [Install](#install)
- [Usage](#usage)
- [API](#api)
- [Contributing](#contributing)
- [License](#license)

## Background

### Overview:
MPU6050 is an advanced Microelectromechanical Systems (MEMS) Inertial Measurement Unit (IMU) chip developed by InvenSense (now part of TDK). It is a sensor that integrates a three-axis accelerometer and a three-axis gyroscope, capable of accurately measuring acceleration and angular velocity in three-dimensional space. MPU6050 is one of the widely used low-cost and high-performance IMU solutions in electronic products.

### Key Features:

- Three-Axis MEMS Accelerometer and Gyroscope: The MPU6050 incorporates three-axis accelerometers and three-axis gyroscopes, providing real-time monitoring of the target object's acceleration and angular velocity.

- Digital Output Interface: MPU6050 communicates with the main controller through standard I2C (Inter-Integrated Circuit) or SPI (Serial Peripheral Interface) digital interfaces, simplifying hardware interface design.

- Low Power Consumption: The MPU6050 is designed with multiple low-power modes, making it suitable for battery-powered mobile devices and portable applications.

- High-Precision Measurement: MPU6050 can measure acceleration and angular velocity with a resolution of up to 16 bits, providing accurate attitude and motion information.

## Install

The installation of environment dependencies is realized through the requirements.txt file of this file：
```
pip install -r requirements.txt
```

## Usage
```
PATH=$PATH:$(pwd)
cd abc_project(create folder without-> dora new abc_project --lang python)
dora up
dora start dataflow.yml --name first-dataflow
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
dora logs first-dataflow custom_node_1
```
### After that, the content accepted by dora will appear   

**if you need to print the context to test the output:(Use the command line below)**  
- **try:RUST_LOG=debug dora start ./your path.yml --attach --hot-reload**  
- **dora logs (number) (node_name)** 

## API

Either the `python-smbus` or `python3-smbus` package with`Dora`, according to your Python version.

## Contributing

Thanks for dora's developer in Shenxin(Xi'an) technology company and Shandong University.

## License

[SDU © Chicheng.]