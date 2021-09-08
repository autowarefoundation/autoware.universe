# /api/external/set/initialize_pose_auto

## Classification

- Category: Mandatory
- Behavior: Service
- DataType: [autoware_external_api_msgs/srv/InitializePoseAuto](https://github.com/tier4/autoware_api_msgs/blob/develop/autoware_external_api_msgs/srv/InitializePoseAuto.srv)

## Description

GNSS による位置姿勢をもとに、車両の位置姿勢を初期化・再設定する。

## Requirement

位置姿勢に関する事前状態を考慮せず、GNSS による位置姿勢のみを用いて車両の位置姿勢を初期化・再設定する。
