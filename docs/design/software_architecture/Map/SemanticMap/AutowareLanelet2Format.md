# Modifying Lanelet2 format for Autoware
## Overview
About the basics of the default format, please refer to main [Lanelet2 repository](https://github.com/fzi-forschungszentrum-informatik/Lanelet2). (see [here](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/blob/master/lanelet2_core/doc/LaneletPrimitives.md) about primitives)

In addition to default Lanelet2 Format, users should add following mandatory/optional tags to their osm lanelet files as explained in reset of this document. 

## Additional Conventions
### Lane length
Original Lanelet2 format does not specify the minimum or maximum length of lane. However, without the limit, it is very difficult to estimate memory size and computation time using lane information. Therefore, we set general convention that a lanelet should have length between 1m ~ 50m.

### No Self Crossing Lanes
In order to make geometrical calculation of lane easier (e.g. getting drivable area, operating triangulation, etc) we set a convention that no lanes should overlap itself. Whenever, it overlaps itself, the lane should be separated into two different lanes with same attributes.

## Additional Mandatory Tags
### Elevation Tags
Elevation("ele") information for points(`node`) is optional in default Lanelet2 format.
However, some of Autoware packages(e.g. trafficlight_recognizer) need elevation to be included in HD map. Therefore, users must make sure that all points in their osm maps contain elevation tags. 

Here is an example osm syntax for node object.
```
<node id='1' visible='true' version='1' lat='49.00501435943' lon='8.41687458512'>
  <tag k='ele' v='3.0'/> <!-- this tag is mandatory for Autoware!! --> 
</node>
```

### SpeedLimit Tags
Speed limit("speed_limit") information for lanelet(`relation`) is optional in default Lanelet2 format, and there are several ways of setting the information (e.g. using tag or using regulatory element or using default value in traffic rules). To avoid confusion, we will make this information mandatory for all road lanelets, and should not expect Autoware to use default value. 
Therefore, users must make sure that all lanelets in their osm maps contain speed_limit tags. 

Here is an example osm syntax for a lanelet object.
```
  <relation id="34408" visible="true" version="1">
    <member type="way" ref="34406" role="left" />
    <member type="way" ref="34407" role="right" />
    <tag k="speed_limit" v="30m/h" />
    <tag k="subtype" v="road" />
    <tag k="type" v="lanelet" />
  </relation>
```

### TrafficLights
Default Lanelet2 format uses LineString(`way`) or Polygon class to represent the shape of a traffic light. For Autoware, traffic light objects must be represented only by LineString to avoid confusion, where start point is at bottom left edge and end point is at bottom right edge. Also, "height" tag must be added in order to represent the size in vertical direction(not the position). 

The Following image illustrates how LineString is used to represent shape of Traffic Light in Autoware.
<img src="image/traffic_light.png" width="600">


Here is an example osm syntax for traffic light object.  
```
<way id='13' visible='true' version='1'>
  <nd ref='6' />
  <nd ref='5' />
  <tag k='type' v='traffic_light' />
  <tag k='subtype' v='red_yellow_green' />
  <tag k='height' v='0.5'/> <!-- this tag is mandatory for Autoware!! --> 
</way>
```

### Turn Directions
Users must add "turn_direction" tags to lanelets within intersections to indicate vehicle's turning direction. You do not need this tags for lanelets that are not in intersections. If you do not have this tag, Autoware will not be able to light up turning indicators. 
This tags only take following values:
* left
* right
* straight

Following image illustrates how lanelets should be tagged.

<img src="image/turn_direction.png" width="600">

Here is an example of osm syntax for lanelets in intersections. 
```
<relation id='1' visible='true' version='1'>
  <member type='way' ref='2' role='left' />
  <member type='way' ref='3' role='right' />
  <member type='relation' ref='4' role='regulatory_element' />
  <tag k='location' v='urban' />
  <tag k='one_way' v='yes' />
  <tag k='subtype' v='road' />
  <tag k='type' v='lanelet' />
  <tag k='turn_direction' v='left' /> <!-- this tag is mandatory for lanelets at intersections!! --> 
</relation>
```

## Optional Tags
Following tags are optional tags that you may want to add depending on how you want to use your map in Autoware. 

### Meta Info
Users may add the `MetaInfo` element to their OSM file to indicate format version and map version of their OSM file. This information is not meant to influence Autoware vehicle's behavior, but is published as ROS message so that developers could know which map was used from ROSBAG log files. MetaInfo elements exists in the same hierarchy with `node`, `way`, and `relation` elements, otherwise JOSM wouldn't be able to load the file correctly.

Here is an example of MetaInfo in osm file:
```
<?xml version='1.0' encoding='UTF-8'?>
<osm version='0.6' generator='JOSM'>
  <MetaInfo format_version='1.0' map_version='1.0'/>
  <node>...</node>
  <way>...</way>
  <relation>...</relation>
</osm>
``` 

### Local Coordinate Expression
Sometimes users might want to create Lanelet2 maps that are not georeferenced. 
In such a case, users may use "local_x", "local_y" taggings to express local positions instead of latitude and longitude. 
Autoware Osm Parser will overwrite x,y positions with these tags when they are present.
For z values, use "ele" tags as default Lanelet2 Format. 
You would still need to fill in lat and lon attributes so that parser does not crush, but their values could be anything. 

Here is example `node` element in osm with "local_x", "local_y" taggings:
```
<!-- lat/lon attributes are required, but their values can be anything --> 
<node id='40648' visible='true' version='1' lat='0' lon='0'>
  <tag k='local_x' v=2.54'/>
  <tag k='local_y' v=4.38'/>
  <tag k='ele' v='3.0'/>
</node>
```
