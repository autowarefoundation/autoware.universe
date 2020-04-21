Some pre-trained models provided by other repository are used in some packages.

 - tensorrt_yolo3 <br>
The pre-trained models are provided in the following repository. The trained file is automatically downloaded when you build. <br>
https://github.com/lewes6369/TensorRT-Yolov3 <br>
\[Original URL] <br>
Tranined file (416) : https://drive.google.com/drive/folders/18OxNcRrDrCUmoAMgngJlhEglQ1Hqk_NJ

- traffic_light_fine_detector <br>
A trained model in this package is based on the following .weights file and was fine-tuned with darknet by Tier IV. <br>
\[Original URL] <br>
https://pjreddie.com/media/files/yolov3.weights <br>
After fine-tuning, the trained model is converted to ONNX file with the following script. <br>
https://github.com/tier4/AutowareArchitectureProposal/blob/master/src/perception/traffic_light_recognition/traffic_light_fine_detector/scripts/yolov3_to_onnx.py <br>

- lidar_apollo_instance_segmentation <br>
This package makes use of three pre-trained models provided by apollo. These files are automatically downloaded when you build. <br>
\[Original URL] <br>
VLP-16 : https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne16/deploy.caffemodel <br>
HDL-64 : https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.caffemodel <br>
VLS-128 : https://github.com/ApolloAuto/apollo/raw/91844c80ee4bd0cc838b4de4c625852363c258b5/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne128/deploy.caffemodel <br>
