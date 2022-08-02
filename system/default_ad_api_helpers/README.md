# default_ad_api_helpers

## initial_pose_adaptor

This node makes it easy to use the localization AD API from RViz.
When a initial pose topic is received, call the localization initialize API.
This node depends on fitting to map height service.

| Interface    | Local Name     | Global Name                       | Description                               |
| ------------ | -------------- | --------------------------------- | ----------------------------------------- |
| Subscription | initialpose    | /initialpose                      | The pose for localization initialization. |
| Client       | fit_map_height | /localization/util/fit_map_height | To fix initial pose to map height         |
| Client       | -              | /api/localization/initialize      | The localization initialize API.          |

## automatic_pose_initializer

This node calls localization initialize API when the localization initialization state is uninitialized.
Since the API uses GNSS pose when no pose is specified, initialization using GNSS can be performed automatically.

| Interface    | Local Name | Global Name                            | Description                                |
| ------------ | ---------- | -------------------------------------- | ------------------------------------------ |
| Subscription | -          | /api/localization/initialization_state | The localization initialization state API. |
| Client       | -          | /api/localization/initialize           | The localization initialize API.           |
