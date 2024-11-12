# ルーハンドラ

`route_handler` は、レイレットマップで走行経路を計算するためのライブラリです。

## ユニットテスト

ユニットテストは `autoware_test_utils` パッケージに依存します。
`autoware_test_utils` は、ユニットテストの作成を簡素化するいくつかの共通関数を提供するライブラリです。

![route_handler_test](./images/route_handler_test.svg)

既定では、テストを作成するために経路ファイルが必要です。以下は、ユニットテストで使用される経路を示しています。

### レーンチェンジテスト経路

![lane_change_test_route](./images/lane_change_test_route.svg)

- この経路は `autoware_test_utils\test_map` から取得できるマップに基づいています。

