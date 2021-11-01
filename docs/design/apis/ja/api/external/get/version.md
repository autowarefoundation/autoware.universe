# /api/external/get/version

## Classification

- Category: Mandatory
- Behavior: Service
- DataType: [autoware_external_api_msgs/srv/GetVersion](https://github.com/tier4/autoware_api_msgs/blob/main/autoware_external_api_msgs/srv/GetVersion.srv)

## Description

Autoware External API のバージョン情報を取得する。

## Requirement

以下の規則に従ったバージョン文字列`major.minor.patch`が取得できること。

- major: 全体に関わる大規模な変更があったとき。
- minor: 互換性の失われる変更があったとき。
- patch: 互換性の維持される変更があったとき。
