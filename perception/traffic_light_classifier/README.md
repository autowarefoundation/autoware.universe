### traffic_light_classifier
There's an implementation of CNN and HSV filter versions.

If you use CNN, the trained file are automatically downloaded when you build.

## Training
The model  used in this package was trained by [AutowareMLPlatform/classification_2d](https://github.com/tier4/AutowareMLPlatform/tree/master/tasks/classification_2d)

## model detail
**TODO1: atach AWS URL here for quick reference when, where and how the model was trained.**\
**TODO2: label file should be link to trained model.**

## label support
By default, this package only support typical type of traffic light in Japan. so only basic type of lamp is supported. please reffer [LampState](https://github.com/tier4/autoware.iv/blob/master/common/msgs/autoware_perception_msgs/msg/traffic_light_recognition/LampState.msg)

label names are assumed to be comma-separated to represent each lamp.\
For example, the traffic light with red and right arrow lit are represented by label such as `"red,right"`, which are converted to the string array `["red", "right"]` in cnn_classifier, and finally array is converted to [LampState](https://github.com/tier4/autoware.iv/blob/master/common/msgs/autoware_perception_msgs/msg/traffic_light_recognition/LampState.msg) array.

## other country traffic light model
please download from below list and replace onnx and label file in the data directory.

  | | pretrained onnx model | corresponding label file |
  |:--- | :--- | --- |
  | United States| [LISA](https://drive.google.com/uc?id=1Q-pZNTUBDNWddcURARrvtz2jmTiqV5T7) | [go-stop-warning-label](https://drive.google.com/uc?id=15fxhS2zDAU0aa_cLkrhMKo_1ZY-JnWTE) |
  | Japan |[nishishinjuku](https://drive.google.com/uc?id=19M64ZAo0XNv-Ep2RDynrRipLg3YAm65e)  | [nishishinjuku-label](https://drive.google.com/uc?id=1C4XkFe-G58LcDJSVMp5xlwQniGVozgwW) |
