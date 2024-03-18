# GNSS

A Python module for accessing the GNSS  by Dora.

## Table of Contents

- [Usage](#usage)

## Usage

python ==3.11

```
PATH=$PATH:$(pwd)
cd abc_project(create folder without-> dora new abc_project --lang python) 
#cd /home/crp/dora_project/dora-rs/dora-hardware/vendors/gnss/GNSS_D300
dora up
sudo chmod 777 /dev/ttyUSB0
dora start gnss_D300_driver_dora.yaml --name first-dataflow
# Output: c95d118b-cded-4531-a0e4-cd85b7c3916c
dora logs first-dataflow nmea_subscribeTopic
```
