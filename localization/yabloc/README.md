# Vector Map Visual Localization

## Build

```shell
mkdir vmvl_ws/src -p
cd vmvl_ws
git clone git@github.com:tier4/VectorMapVisualLocalizer.git src/vector_map_visual_localizer --recursive
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_SOPHUS_TESTS=OFF
```

- (optional) ccache `(--cmake-args) -DCMAKE_C_COMPILER_LAUNCHER=ccache -DCMAKE_CXX_COMPILER_LAUNCHER=ccache`

- (optional) clang-tidy `(--cmake-args) -DCMAKE_EXPORT_COMPILE_COMMANDS=ON`

## Execute
