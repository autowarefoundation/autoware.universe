# glog_component

このパッケージは、glog（Google ロギングライブラリ）機能を ros2 コンポーネントライブラリとして提供します。これを使用して、コンテナで glog 機能を動的に読み込みます。

詳細な機能については [glog github](https://github.com/google/glog) を参照してください。

## 例

コンテナで `glog_component` を読み込む場合、起動ファイルは以下のようになります。


```py
glog_component = ComposableNode(
    package="glog_component",
    plugin="GlogComponent",
    name="glog_component",
)

container = ComposableNodeContainer(
    name="my_container",
    namespace="",
    package="rclcpp_components",
    executable=LaunchConfiguration("container_executable"),
    composable_node_descriptions=[
        component1,
        component2,
        glog_component,
    ],
)
```

