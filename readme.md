# Map merge

This package implements a ROS node that merges 2D maps coming from multiple robots.

Knowledge about the initial relative positions of robots is required. 
These positions are also the map origins which are used to overlay maps on top of each other.

A map is an occupancy grid with many small grid cells. 
Each cell has a value indicating the probability of whether there is an obstacle is occupying the space represented by the cell. 
Once we know how to overlap each grid cell from each map, we can merge the cell probability values together. 
This is performed by using a series of mathematical equations.

## Example

Since `map_merge.py` requires a number of parameters to work properly, it is useful to create it within a ROS launch file.

```xml
<launch>
    <param name="robot_1/map_merge/init_pose_x" value="1.0" />
    <param name="robot_1/map_merge/init_pose_y" value="2.0" />
    <param name="robot_1/map_merge/init_pose_yaw" value="0.5" />
    
    <param name="robot_2/map_merge/init_pose_x" value="-1.0" />
    <param name="robot_2/map_merge/init_pose_y" value="-2.0" />
    <param name="robot_2/map_merge/init_pose_yaw" value="-0.5" />
    
    <node pkg="map_merge" type="map_merge.py" name="map_merge" output="screen">
        <param name="rate" value="1" />
        <param name="robot_count" value="2" />
    </node>
</launch>
```

## Nodes

### map_merge.py

The `map_merge.py` node subscribes to topics that publish 2D maps. It then merges the maps together and publishes the merged map to a topic.
It also broadcasts static transforms to relate frames between individual robot maps and the merged global map.

#### Subscribed topics

`<robot_namespace>/map` [(nav_msgs/OccupancyGrid)](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html)

The map of a robot. The topic is configurable as a parameter.
`<robot_namespace>` is actually composed of two parts: `<robot_common_name><robot_index>` where `robot_common_name` is a parameter, and `robot_index` is an integer starting from 1 and ending at `robot_count`.

#### Published topics

`tf_static` [(tf/tf_message)](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html)

Static transforms to relate frames between individual robot maps and the merged global map.

`map` [(nav_msgs/OccupancyGrid)](http://docs.ros.org/en/api/nav_msgs/html/msg/OccupancyGrid.html)

The merged map. The topic is configurable as a parameter.


#### Robot parameters

Parameters that should be defined in the namespace of each robot.

`<robot_namespace>/map_merge/init_pose_x` (`double`, default: `<no_default>`)

x coordinate of robot initial position in `world_map_frame`. Should be in meters. It does not matter which frame you will consider global (preferably it should be different from all robots frames), but relative positions of robots in this frame must be correct.

`<robot_namespace>/map_merge/init_pose_y` (`double`, default: `<no_default>`)

y coordinate of robot initial position in `world_map_frame`.

`<robot_namespace>/map_merge/init_pose_yaw` (double, default: `<no_default>`)
yaw component of robot initial position in `world_map_frame`. Represents robot rotation in radians.

If any of these parameters are not defined, the node will throw an error and exit.

#### Node parameters

Parameters that should be defined in the namespace of this node.

`~robot_common_name` (`string`, default: `"robot_"`)

The common name of all robots. This is used to compose the `<robot_namespace>`.
The `<robot_namespace>` is used to find the map topic of each robot, and also its initial pose.
It is expected that the map topic of each robot is `<robot_namespace>/map`, and the `<robot_namespace>/map_merge/init_pose_*` parameters are also defined.

`~robot_count` (`int`, default: `2`)

The number of robots.

`~world_map_topic` (`string`, default: `"map"`)

The topic of the merged map for publishing.

`~world_map_frame` (`string`, default: `"world"`)

The frame of the merged map.

`~rate` (`double`, default: `0.25`)

The rate at which the node publishes the merged map.

`~merge_subprocess_count` (`int`, default: `1`)

The number of subprocesses to use for merging maps. 
This is useful if you have a lot of robots and the merging process is taking too long.

#### Transforms

The node broadcasts static transforms to relate frames between individual robot maps and the merged global map.

`~world_map_frame` -> `<robot_namespace>/map` for every `robot_namespace`

## Acknowledgements

This package is inspired by the [multirobot_map_merge](http://wiki.ros.org/multirobot_map_merge) package.
But this only implements the merge method that involves using initial relative positions of robots.
