## Train
* Train your model with [darknet](https://github.com/AlexeyAB/darknet).


## Convert cfg and weight files into onnx file
```bash
    cd scripts
	pip install -r requirements.txt
    python yolov3_to_onnx.py [cfg_file_path] [weights_file_path] [label_flle_path] --ouput_file_path [ouput_file_path] --input_size [input_size]
```
## Build engine
* If yolov3_tlr.engine does not exist in data directory, this node will build engine file from onnx file in data directory.
* If you want to use int8 mode, please put about 100 images of different scene in calib_image directory.
```bash
	roslaunch traffic_light_fine_ditector traffic_light_fine_detector.launch mode:=int8
```
* This node automatically creates data and calib_image directory.

## Note
The trained files are automatically downloaded when you build.