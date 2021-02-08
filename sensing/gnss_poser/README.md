# gnss_poser

## Overview

This is package to use GNSS with ROS messages.

## Install

### GeographicLib

This package use GeographicLib to calculate coordinates.

1. Download files from <https://sourceforge.net/projects/geographiclib/files/distrib/>
2. Build and install Geographiclib

        tar xfpz GeographicLib-1.50.1.
        cd GeographicLib-1.50.1
        mkdir BUILD
        cd BUILD
        ../configure --prefix=/usr
        make
        sudo make install
        sudo cp /usr/share/cmake/GeographicLib/FindGeographicLib.cmake /usr/share/cmake-3.5/Modules
3. install geoid datasets

        geographiclib-get-geoids best

### Ublox

If you use ublox GNSS receiver, install below package.
This allows to obtain heading while vehicle is stopping.

1. Download package <https://github.com/KumarRobotics/ublox.git>
2. build package

### gnss_poser package

This package use gnss package in autoware.

1. download and build Autoware
2. download and build this package

## Usage

    roslaunch gnss_poser gnss_poser.launch
For ublox GNSS receiver

    roslaunch gnss_poser ubloxfix2mgrs.launch

## Configuration

This package use egm2008-1 for geoid datasets.

Parameters can be set in Launch file.
