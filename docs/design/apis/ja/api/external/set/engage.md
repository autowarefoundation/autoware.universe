# /api/external/set/engage

## Classification

- Category: Mandatory
- Behavior: Service
- DataType: [autoware_external_api_msgs/srv/Engage](https://github.com/tier4/autoware_api_msgs/blob/main/autoware_external_api_msgs/srv/Engage.srv)

## Description

車両の停止保持状態を設定・解除する。自律制御開始時、または、停留所や荷物の積み下ろし中など、何らかのユーザー操作が行われるまで車両の停止を継続させる。

## Requirement

エンゲージ状態が true に設定されていない場合、車両が停止保持するように制御信号を出力すること。
