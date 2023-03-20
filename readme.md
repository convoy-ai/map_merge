# Map merge

This package implements a ROS node that merge 2D maps coming from multiple robots through ROS topics. 

Knowledge about the initial relative positions of robots is required. These positions are also the map origins which are used to overlay maps on top of each other.

A map is an occupancy grid with many small grid cells. Each cell has a value indicating the probability of whether there is an obstacle is occupying the space represented by the cell. Once we know how to overlap each grid cell from each map, we can merge the cell probability values together. This is performed by using a series of mathematical equations.

## Example

To create a `map_merge` ROS node within a ROS launch file.

```xml
<param name="robot_1/map_merge/init_pose_x" value="0.0" />
<param name="robot_1/map_merge/init_pose_y" value="0.0" />
<param name="robot_1/map_merge/init_pose_yaw" value="0.0" />

<param name="robot_2/map_merge/init_pose_x" value="0.0" />
<param name="robot_2/map_merge/init_pose_y" value="0.0" />
<param name="robot_2/map_merge/init_pose_yaw" value="0.0" />

<node pkg="map_merge" type="map_merge.py" name="map_merge" output="screen">
    <param name="rate" value="1" />
    <param name="robot_count" value="2" />
</node>
```

## Nodes

### map_merge.py

#### Subscribed topics

`tf` [(tf/tf_message)](http://docs.ros.org/en/api/tf/html/msg/tfMessage.html)

    Transforms to relate frames between individual robot maps and the merged global map.

`tf