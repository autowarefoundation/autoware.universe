# Vector Map Visual Localization

## Build

```shell
mkdir vmvl_ws/src -p
cd vmvl_ws
git clone git@github.com:tier4/VectorMapVisualLocalizer.git -b feature/mpf_exporting  src/vector_map_visual_localizer --recursive
colcon build --symlink-install --cmake-args -DCMAKE_EXPORT_COMPILE_COMMANDS=ON -DCMAKE_BUILD_TYPE=Release
```

## Execute
