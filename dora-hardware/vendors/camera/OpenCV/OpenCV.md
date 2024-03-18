# OpenCV Camera

## Description

use OpenCV to read the camera data. This is based on OpenCV Python and dora Operators.

### Installation

To install dependencies:
```
pip install -r requirements.txt
```

### Launch Step

To use, add the operator python file into your dataflow configuration and launch it as usual.

For example:
- yaml config

```yaml
  - id: webcam
    operator:
      python: webcam.py
      inputs:
        tick: dora/timer/millis/50
      outputs:
        - image
    env:
      DEVICE_INDEX: 0
```
- cli launch
```
dora start dataflow.yaml
```

### Development Guidelines

Make sure that your camera is well connected. That it is reachabel at the given index. You can check available camera with `ls /dev/video*`

### Output

The output is a pyarrow array with the shape `(webcam_height, webcam_width, 3)` of type u8.

### Input

The input is a dora tick.
