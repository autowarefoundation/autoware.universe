# autoware_default_adapi

## メモ

サービスを中継するコンポーネントは、マルチスレッドエグゼキューターによって実行される必要があります。

## 機能

このパッケージは既定実装の AD API です。

- [Autoware 状態 (後方互換性)](document/autoware-state.md)
- [フェイルセーフ](document/fail-safe.md)
- [インターフェイス](document/interface.md)
- [局在](document/localization.md)
- [運動](document/motion.md)
- [運転モード](document/operation-mode.md)
- [経路](document/routing.md)

## Web サーバースクリプト

これは、HTTP を使用して API を呼び出すためのサンプルです。

## ガイドメッセージスクリプト

これは、自律モードへの遷移条件を確認するためのデバッグスクリプトです。


```bash
$ ros2 run autoware_default_adapi guide.py

The vehicle pose is not estimated. Please set an initial pose or check GNSS.
The route is not set. Please set a goal pose.
The topic rate error is detected. Please check [control,planning] components.
The vehicle is ready. Please change the operation mode to autonomous.
The vehicle is driving autonomously.
The vehicle has reached the goal of the route. Please reset a route.
```

