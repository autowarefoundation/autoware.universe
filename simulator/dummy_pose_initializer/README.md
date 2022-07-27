# dummy_pose_initializer

## Purpose

`dummy_pose_initializer` is the dummy package of `pose_initializer`. This packages relays initial pose to the simulator.

## Input / Output

### Input topics

| Name                       | Type                                              | Description           |
| -------------------------- | ------------------------------------------------- | --------------------- |
| `/localization/initialize` | autoware_ad_api_msgs::srv::InitializeLocalization | initial pose from api |

### Output topics

| Name                                 | Type                                                       | Description               |
| ------------------------------------ | ---------------------------------------------------------- | ------------------------- |
| `/localization/initialization_state` | autoware_ad_api_msgs::msg::LocalizationInitializationState | pose initialization state |
| `/initialpose3d`                     | geometry_msgs::msg::PoseWithCovarianceStamped              | initial pose relayed      |
