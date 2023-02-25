import rospy
import nav_msgs
import geometry_msgs

import numpy as np
import math
import typing



def calc_world_map_metadata(world_map, robot_maps, robot_initial_poses):

    resolution = world_map.info.resolution

    # calculate width and height
    for robot_namespace, robot_map in robot_maps.items():

        world_map_corners = find_corners(world_map)
        
        robot_map_corners = find_corners(robot_map)

        pose = robot_initial_poses[robot_namespace]
        origin_wrt_world = np.array([pose['x'], pose['y']])
        theta = pose['yaw']
        rotation_matrix = np.array([[np.cos(theta), -1 * np.sin(theta)], [np.sin(theta), np.cos(theta)]])

        for corner in robot_map_corners:
            ab = origin_wrt_world
            bc = rotation_matrix @ corner

            corner_wrt_world = ab + bc
            inside = check_if_inside_corners(world_map_corners, corner_wrt_world)

            rospy.logdebug(f"corner: {corner}, corner_wrt_world: {corner_wrt_world}, inside: {inside}")

            if not inside:

                x, y = corner_wrt_world[0], corner_wrt_world[1]
                world_x, world_y = world_map.info.origin.position.x, world_map.info.origin.position.y
                width_increase, height_increase = 0, 0

                if x < world_x:
                    world_map.info.origin.position.x = x
                    width_increase = abs(x - world_map_corners[0][0]) 
                elif x > world_map_corners[2][0]:
                    width_increase = abs(x - world_map_corners[2][0])

                if y < world_y:
                    world_map.info.origin.position.y = y
                    height_increase = abs(y - world_map_corners[0][1])
                if y > world_map_corners[2][1]:
                    height_increase = abs(y - world_map_corners[2][1])

                world_map.info.width += math.ceil(width_increase / resolution)
                world_map.info.height += math.ceil(height_increase / resolution)


                # update world map corners so that the next robot map corner can be compared with new information
                world_map_corners = find_corners(world_map)


    return world_map





def find_corners(map_: nav_msgs.msg.OccupancyGrid):
    x = map_.info.origin.position.x
    y = map_.info.origin.position.y
    res = map_.info.resolution
    width = map_.info.width
    height = map_.info.height
    
    corners = [
        [x, y],
        [x + width * res, y],
        [x + width * res, y + height * res],
        [x, y + height * res],
    ]

    return corners


def check_if_inside_corners(corners, point):
    x, y = point[0], point[1]

    corner_0, corner_2 = corners[0], corners[2]

    return not (x < corner_0[0] or x > corner_2[0] or y < corner_0[1] or y > corner_2[1])


def map_to_2d(mapData):
    return np.resize(np.array(mapData.data), (mapData.info.height, mapData.info.width))


def point_to_index(mapData, point):
    origin_x, origin_y = mapData.info.origin.position.x, mapData.info.origin.position.y
    resolution = mapData.info.resolution

    x, y = point[0], point[1]

    index_x = int(round((x - origin_x) / resolution))
    index_y = int(round((y - origin_y) / resolution))

    return [index_x, index_y]


def index_to_point(mapData, index):
    origin_x, origin_y = mapData.info.origin.position.x, mapData.info.origin.position.y
    resolution = mapData.info.resolution

    index_x, index_y = index[0], index[1]

    x = origin_x + index_x * resolution
    y = origin_y + index_y * resolution

    return array([x, y])