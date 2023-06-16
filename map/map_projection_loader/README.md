# map_projection_loader

## Feature

`map_projection_loader` is responsible for publishing `map_projector_info` that defines in which kind of coordinate the Autoware is operating on.
This is a necessary information especially when you want to convert from global (geoid) to local coordinate or the other way around.

## Published Topics

- ~/map_projector_info (tier4_map_msgs/MapProjectorInfo) : Topic for defining map projector information

## Parameters

| Name                    | Type        | Description                                                                       | Default value |
| :---------------------- | :---------- | :-------------------------------------------------------------------------------- | :------------ |
| use_local_projector     | bool        | A flag for defining whether to use local coordinate                               | false         |
| map_projector_info_path | std::string | A path to map_projector_info.yaml (used only when `use_local_projector` is false) | N/A           |
