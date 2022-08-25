# ad_api_adaptors

## initial_pose_adaptor

This node makes it easy to use the localization AD API from RViz.
When a initial pose topic is received, call the localization initialize API.
This node depends on fitting to map height service.

| Interface    | Local Name     | Global Name                       | Description                               |
| ------------ | -------------- | --------------------------------- | ----------------------------------------- |
| Subscription | initialpose    | /initialpose                      | The pose for localization initialization. |
| Client       | fit_map_height | /localization/util/fit_map_height | To fix initial pose to map height         |
| Client       | -              | /api/localization/initialize      | The localization initialize API.          |
