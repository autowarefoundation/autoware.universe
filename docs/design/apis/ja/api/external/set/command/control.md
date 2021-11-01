# /api/external/set/command/control

## Classification

- Category: Optional
- Behavior: Topic
- DataType: [autoware_external_api_msgs/msg/ControlCommandStamped](https://github.com/tier4/autoware_api_msgs/blob/main/autoware_external_api_msgs/msg/ControlCommandStamped.msg)

## Description

車両のアクセル・ブレーキ・ステアリングを制御するコマンドを送信する。

## Requirement

現在の車両状態を考慮し、指定されたコマンドを可能な限り反映した制御を行うこと。
