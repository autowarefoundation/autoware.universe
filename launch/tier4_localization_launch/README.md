# tier4_localization_launch

## 構造

![tier4_localization_launch](./localization_launch.drawio.svg)

## パッケージ依存関係

`package.xml`の`<exec_depend>`を参照してください。

## 使用方法

`localization.launch.xml`を他の`launch`ファイルに次のように含めます。

`pose_source`と`twist_source`を指定して、位置推定または速度推定のロカライゼーション方式を選択できます。

さらに、`PACKAGE_param_path`としてパラメータのパスを提供する必要があります。提供する必要のあるパラメータパスのリストは、`localization.launch.xml`の先頭に記載されています。


```xml
  <include file="$(find-pkg-share tier4_localization_launch)/launch/localization.launch.xml">
    <!-- Localization methods -->
    <arg name="pose_source" value="..."/>
    <arg name="twist_source" value="..."/>

    <!-- Parameter files -->
    <arg name="FOO_param_path" value="..."/>
    <arg name="BAR_param_path" value="..."/>
    ...
  </include>
```

