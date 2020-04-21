# TF tree in Autoware
Autoware uses TF library for transforming coordinates, and TF tree can be accessed from any modules in Autoware.

The TF tree of Autoware is illustrated in the image below.
![TF](/design/img/TF.svg)

## Frames
Here is the description of each frame
* earth: the origin of ECEF coordinate(i.e. center of the earth). Currently, this frame is not used by any modules, but this is set for future support of larger maps and multiple vehicles.
* map: Local ENU coordinate. This keeps xy-plane to be relatively parallel to the ground surface. All map information should be provided in this frame, or it should be provided in a frame that can be statically transformed into this frame. Therefore, most of the planning calculations will be done on this frame as well.
* base_link: Frame rigidly attached to the vehicle. Currently, it is the midpoint of rear wheels projected to ground.
* sensor_frames: This represents the position of sensors. The actual name of this frame will be the name of the sensor, such as `camera`, `gnss`, `lidar`. Note that a camera should provide both camera frame and camera optical frame as suggested in [REP-103](https://www.ros.org/reps/rep-0103.html).

## Transforms
Transforms between each frame are explained in the following:

|TF|Type|Providing Module|Description|
|-|-|-|-|
|earth->map|static|Map|Map modules will provide this TF according to origin information parameter file.|
|map->base_link|dynamic|Localization|Localization module calculates vehicles position relative to maps specified in `map` frame.|
|base_link->sensor|static|Vehicle|Vehicle module provide sensor position relative to base_link using URDF. There may be multiple static transforms between base_link and a sensor frame. For example, if a camera is calibrated against a lidar, then the camera's TF can be expressed by base_link->lidar->camera.|

Remarks:
* Static frame does not mean it does not change. It may be remapped at times, but no interpolation will be done between the change. (i.e. only newest information is used)
* base_link->sensor is assumed to be static. This is not true due to suspension, but we assume that this displacement is small enough that it shouldn't affect control of the vehicle. There is a [discussion](https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/-/issues/292) about nav_base which resolves this issue, which may be integrated.
* Above specification is not meant to restrict the addition of other frames. Developers may add any additional frames but are not allowed to change the meaning of the above frames.

## Regarding REP105
ROS set [REP-105](https://www.ros.org/reps/rep-0105.html
) regarding TF, and the above explanation follows REP-105, but with the significant change that we removed the odom frame.

### What is Odom frame?
Odom frame is defined as following in the REP: 
```
The coordinate frame called odom is a world-fixed frame. The pose of a mobile platform in the odom frame can drift over time, without any bounds. This drift makes the odom frame useless as a long-term global reference. However, the pose of a robot in the odom frame is guaranteed to be continuous, meaning that the pose of a mobile platform in the odom frame always evolves in a smooth way, without discrete jumps.

In a typical setup the odom frame is computed based on an odometry source, such as wheel odometry, visual odometry or an inertial measurement unit.

The odom frame is useful as an accurate, short-term local reference, but drift makes it a poor frame for long-term reference.
```
There are also discussions regarding the existance of odom frame in the following discussions:
* https://discourse.ros.org/t/localization-architecture/8602/28
+ https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/issues/292

Within the above discussions, the main reason for using odom frame is that:
* odom->base_link is high-frequency and therefore suitable for control
* odom->base_link is continuous and keeps control from "jerks"
* odom->base_link is independent of localization algorithm and therefore it is safe at the failure of localization as long as control is done in odom frame.

### Why we removed the odom frame
However, our view is that it doesn't mean that control is not affected by localization as long as trajectory following is done in the odom frame. For example, if a trajectory is calculated from the shape of the lane specified in an HD map, the localization result will be indirectly used when projecting trajectory into odom frame, and thus will be "blown off" when localization fails. Also, if any other map information is before trajectory calculation, such as shape optimization using map drivable area and velocity optimization using predicted trajectory of other vehicles which is derived from lane shape information, then localization failure will still affect the trajectory. Therefore, to make control independent of localization failure, we have to require all preceding calculation to not use map->odom transform. However, since trajectory following comes after planning in Autoware, it is almost impossible that map->odom isn't involved in trajectory calculation. Although it might be possible if Autoware plans like a human, who uses only route graph information from the map and obtain geometry information from perception, it is very unlikely that autonomous driving stack is capable of not using geometry information with safety ensured. 

Therefore, regardless of any frame that control is done, it will still have effects of localization results. To make control not to jerk or do sudden steering at localization failure, we set the following requirements to Localization module:
* localizatoin result must be continuous
* localization must detect localization failure and should not use the result to update vehicle pose.

Additionally, Localization architecture assumes sequential Bayesian Filter, such as EKF and particle filter, to be used to integrate twist and pose. Since it can also integrate odometry information, it can update TF at high frequency. 
As a result, all the merits of odom->base_link stated in REP-105 can be satisfied by map->base_link, and thus there is no need of setting odom frame. 

As a conclusion, we removed odom frame from this architecture proposal due to the following reasons:
1. It is almost impossible to not use map information in Planning module, and trajectory will have a dependency on map->odom(or map->base_link)
2. Therefore, to keep control safe even at localization failure, the following conditions must be satisfied:
   1. It must be able to detect localization failure
   2. When the failure is detected, map->odom should not be updated
3. If Localization module can satisfy the above conditions, there is no merit of using odom->base_link, and all modules should use map->base_link whenever they need a world-fixed frame.

### Possible Concerns
* Above argument focusses on replacing map->odom->base_link with map->base_link, but doesn't prove that map->base_link is better. If we set odom->base_link, wouldn't we have more options of frames?
  * Once we split map->base_link into map->odom and odom->base_link, we loose velocity information and uncertainty(covariance) information between them. We can expect more robustness if we integrate all information(odometry and output of multiple localization) at once.
  * If it is split into map->odom and odom->base_link, we have to wait for both transforms to obtain map->base_link transform. It is easier to estimate delay time if it is combined into one TF.
  * We think that creating odom frame using current architecture is possible. However, we should first discuss if we have any component that wants to use odom->base_link. We anticipate that it might be needed when we design safety architecture, which is out-of-scope in this proposal, but it must be added after all safety analysis is done. As stated above, using odom frame in Control module is not enough for safety.
* Most of ROS localization nodes assumes odom->base_link to calculate map->odom. Wouldn't it be impossible to utilize such nodes without odom frame?
  * It is very unlikely that a single algorithm supports all use cases in different environments. We need a module that integrates output from different localization algorithms, but most of 3rd party localization nodes are not made with consideration of integration modules. Therefore, we still need changes on 3rd party software anyways. Technically, REP-105 is integrating the odometry and localization results by sequentially connecting TFs, but this is not realistic when we add more algorithms. We should be thinking of a way to integrate localization methods in parallel, and current architecture made to support such use cases.

## Reference
* REP105: https://www.ros.org/reps/rep-0105.html
* REP103: https://www.ros.org/reps/rep-0103.html
* Discourse Discussion: https://discourse.ros.org/t/localization-architecture/8602/28
* TF Discussion in Autoware.Auto: https://gitlab.com/autowarefoundation/autoware.auto/AutowareAuto/issues/292
