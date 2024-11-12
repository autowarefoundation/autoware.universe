# ad_api_adaptors

## initial_pose_adaptor

このノードを使用すると、RVizからローカライゼーションAD APIを簡単に使用できます。
初期姿勢トピックを受信すると、ローカライズの初期化APIを呼び出します。
このノードはマップ高度適合ライブラリに依存します。
[詳細はここを参照してください。](../../../map/autoware_map_height_fitter/README.md)

| インターフェイス    | ローカル名  | グローバル名                  | 説明                                   |
| ------------ | ----------- | ---------------------------- | ----------------------------------------- |
| サブスクリプション | initialpose | /initialpose                 | ローカリゼーション初期化の姿勢           |
| クライアント       | -           | /api/localization/initialize | ローカリゼーション初期化 API          |

## routing_adaptor

このノードを使用すると、RVizからrouting AD APIを簡単に利用できます。
ゴールの姿勢トピックを受信すると、ウェイポイントをリセットしてAPIを呼び出します。
ウェイポイントの姿勢トピックを受信すると、APIを呼び出すためのウェイポイントの末尾に追加されます。
ルートを設定する前に、clear APIが自動的に呼び出されます。

| Interface        | Local Name         | Global Name                                 | Description                                                  |
| ---------------- | ------------------ | ------------------------------------------- | ----------------------------------------------------------- |
| 購読            | -                  | /api/routing/state                          | ルーティング API の状態                                      |
| 購読            | ~/input/fixed_goal | /planning/mission_planning/goal             | 経路のゴール目標値（目標の変更は無効）                    |
| 購読            | ~/input/rough_goal | /rviz/routing/rough_goal                    | 経路のゴール目標値（目標の変更を有効）                      |
| 購読            | ~/input/reroute    | /rviz/routing/reroute                       | 再経路の目標位置                                           |
| 購読            | ~/input/waypoint   | /planning/mission_planning/checkpoint       | 経路のウェイポイント位置                                  |
| クライアント      | -                  | /api/routing/clear_route                   | ルートクリア API                                             |
| クライアント      | -                  | /api/routing/set_route_points              | ルートポイント設定 API                                      |
| クライアント      | -                  | /api/routing/change_route_points           | ルートポイント変更 API                                      |

## パラメーター

{{ json_to_markdown("/system/default_ad_api_helpers/ad_api_adaptors/schema/ad_api_adaptors.schema.json") }}

