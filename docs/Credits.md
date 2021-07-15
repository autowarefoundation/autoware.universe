Certain AutowareArchitectureProposal packages rely on pre-trained CNN models provided by other open source repositories.

- tensorrt_yolo3
    - The pre-trained models originate from [TRTForYolov3](https://github.com/lewes6369/TensorRT-Yolov3).
    - [Weights for the trained model](https://drive.google.com/drive/folders/18OxNcRrDrCUmoAMgngJlhEglQ1Hqk_NJ) (416 folder) are automatically downloaded during the build process.


- traffic_light_fine_detector
    - The trained model in this package is based on the [pjreddie's YOLO .weights file](https://pjreddie.com/media/files/yolov3.weights), with additional fine-tuning by Tier IV using [Darknet](https://github.com/pjreddie/darknet).
    - After fine-tuning, the new weights for the trained model are converted into an ONNX file using [Python](https://github.com/tier4/AutowareArchitectureProposal.iv/blob/master/src/perception/traffic_light_recognition/traffic_light_fine_detector/scripts/yolov3_to_onnx.py).


- lidar_apollo_instance_segmentation
    - This package makes use of three pre-trained models provided by [Apollo Auto](https://github.com/ApolloAuto).
    - The following files are automatically downloaded during the build process:
        - [VLP-16](https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne16/deploy.caffemodel)
        - [HDL-64](https://github.com/ApolloAuto/apollo/raw/88bfa5a1acbd20092963d6057f3a922f3939a183/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne64/deploy.caffemodel)
        - [VLS-128](https://github.com/ApolloAuto/apollo/raw/91844c80ee4bd0cc838b4de4c625852363c258b5/modules/perception/production/data/perception/lidar/models/cnnseg/velodyne128/deploy.caffemodel)
