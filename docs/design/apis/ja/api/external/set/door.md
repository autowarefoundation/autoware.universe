# /api/external/set/door

## Classification

- Category: Mandatory
- Behavior: Service
- DataType: [autoware_external_api_msgs/srv/SetDoor](https://github.com/tier4/autoware_api_msgs/blob/develop/autoware_external_api_msgs/srv/SetDoor.srv)

## Description

車両のドア開閉を行う。

## Requirement

車両のドア操作を行うこと。車両にドアがない場合は常に失敗として応答すること。
