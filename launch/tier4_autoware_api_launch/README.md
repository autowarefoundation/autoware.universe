# tier4_autoware_api_launch

## 概要

このパッケージには、Autoware内部トピックを外部ソフトウェア（例：フリート管理システム、シミュレーター）で使用される一貫したAPIに変換するノードを実行する起動ファイルが含まれています。

## パッケージの依存関係

`package.xml`の`<exec_depend>`を参照してください。

## 使用方法

`autoware_api.launch.xml`を使用するには、以下のように`*.launch.xml`に含めることができます。


```xml
  <include file="$(find-pkg-share tier4_autoware_api_launch)/launch/autoware_api.launch.xml"/>
```

## 注意

処理負荷を軽減するため、ROS 2 の [Component](https://docs.ros.org/en/galactic/Concepts/About-Composition.html) 機能（ROS 1 の Nodelet に類似）を使用しています。

