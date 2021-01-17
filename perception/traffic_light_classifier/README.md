# traffic_light_classifier

There's an implementation of CNN and HSV filter versions.

## label support

By default, this package only support typical type of traffic light in Japan. so only basic type of lamp is supported. please refer [LampState](https://github.com/tier4/autoware.iv/blob/master/common/msgs/autoware_perception_msgs/msg/traffic_light_recognition/LampState.msg)

label names are assumed to be comma-separated to represent each lamp.\
For example, the traffic light with red and right arrow lit are represented by label such as `"red,right"`, which are converted to the string array `["red", "right"]` in cnn_classifier, and finally array is converted to [LampState](https://github.com/tier4/autoware.iv/blob/master/common/msgs/autoware_perception_msgs/msg/traffic_light_recognition/LampState.msg) array.

## HSV filter version

## CNN version 

If you use CNN, the trained file are automatically downloaded when you build.

### other country traffic light model

Please download from below list and replace onnx and label file in the data directory.

  | | pretrained onnx model | corresponding label file |
  |:--- | :--- | --- |
  | United States| [LISA](https://drive.google.com/uc?id=1Q-pZNTUBDNWddcURARrvtz2jmTiqV5T7) | [go-stop-warning-label](https://drive.google.com/uc?id=15fxhS2zDAU0aa_cLkrhMKo_1ZY-JnWTE) |
  | Japan |[nishishinjuku](https://drive.google.com/uc?id=19M64ZAo0XNv-Ep2RDynrRipLg3YAm65e)  | [nishishinjuku-label](https://drive.google.com/uc?id=1C4XkFe-G58LcDJSVMp5xlwQniGVozgwW) |


## Reference

M. Sandler, A. Howard, M. Zhu, A. Zhmoginov and L. Chen, "MobileNetV2: Inverted Residuals and Linear Bottlenecks," 2018 IEEE/CVF Conference on Computer Vision and Pattern Recognition, Salt Lake City, UT, 2018, pp. 4510-4520, doi: 10.1109/CVPR.2018.00474.
