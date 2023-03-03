import rospy
import nav_msgs
import geometry_msgs

import numpy as np
import math
import typing
import concurrent.futures



def calc_world_map_metadata(world_map, robot_maps, robot_initial_poses):

    resolution = world_map.info.resolution

    # calculate width and height
    for robot_namespace, robot_map in robot_maps.items():

        world_map_corners = find_corners(world_map.info)
        
        robot_map_corners = find_corners(robot_map.info)

        pose = robot_initial_poses[robot_namespace]
        origin_wrt_world = np.array([pose['x'], pose['y']])
        theta = pose['yaw']

        for corner in robot_map_corners:
            ab = origin_wrt_world
            bc = rotate_point(corner, theta)

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
                world_map_corners = find_corners(world_map.info)


    return world_map


def calc_world_map_data(world_map, robot_maps, robot_initial_poses, process_count = 1):

    # initialize world map data
    world_map_2d = None

    # setup intermediate values for robot maps to reduce redundant computation
    robots_dict = {}

    for robot_namespace, robot_map in robot_maps.items():
        pose = robot_initial_poses[robot_namespace]

        robots_dict[robot_namespace] = {
            'robot_map_info': robot_map.info,
            'data_2d': map_to_2d(robot_map),
            'corners': find_corners(robot_map.info),
            'ba': -1 * np.array([pose['x'], pose['y']]),
            'theta': -1 * pose['yaw']
        }
    
    subprocess_args = []

    for i in range(0, process_count):
        width_count = math.floor(world_map.info.width / process_count)
        width_start_index = i * width_count
        if i == process_count - 1:
            width_count += world_map.info.width % process_count

        args = (world_map.info, robots_dict, width_start_index, width_count)
        subprocess_args.append(args)
        rospy.logdebug(f"width_start_index: {width_start_index}; width_count: {width_count}")

    with concurrent.futures.ProcessPoolExecutor() as executor:
        results = executor.map(calc_sub_world_map_data, subprocess_args)
        
        for result in results:
            if world_map_2d is None:
                world_map_2d = result
            else:
                world_map_2d = np.hstack((world_map_2d, result))

    world_map.data = np.resize(world_map_2d, (world_map.info.width * world_map.info.height)).tolist()

    return world_map


def calc_sub_world_map_data(args):
    world_map_info, robots_dict, width_start_index, width_count = args
    
    world_map_2d = np.full((world_map_info.height, width_count), -1, dtype=np.int8)

    # iterate over every world cell in the map for the particular width
    for x in range(0, width_count):

        for y in range(0, world_map_info.height):

            world_map_2d_index = [x, y]

            world_index = [x + width_start_index, y]
            world_point = index_to_point(world_map_info, world_index)

            all_cell_values = []

            for robot_namespace, intermediate_value in robots_dict.items():
                robot_map_info = intermediate_value['robot_map_info']
                
                # a is world (0, 0), b is robot map (0, 0), c is world_point
                bc = intermediate_value['ba'] + world_point
                point_wrt_robot = rotate_point(bc, intermediate_value['theta'])

                if check_if_inside_corners(intermediate_value['corners'], point_wrt_robot):

                    index_wrt_robot = point_to_index(robot_map_info, point_wrt_robot)

                    if index_wrt_robot[1] >= robot_map_info.height or index_wrt_robot[0] >= robot_map_info.width:
                        rospy.logwarn(f"Bad index: {index_wrt_robot}; point_wrt_robot: {point_wrt_robot}; world_point: {world_point}; robot: {robot_namespace}; shape: {intermediate_value['data_2d'].shape}")

                    cell_value = get_grid_value(intermediate_value['data_2d'], index_wrt_robot)

                    if cell_value != -1:
                        all_cell_values.append(cell_value)
            
            if len(all_cell_values) == 0:
                # all robot maps don't contain this world point
                continue

            elif len(all_cell_values) == 1:
                # only 1 robot map contains this value, so just inherit it

                world_map_2d = set_grid_value(world_map_2d, world_map_2d_index, all_cell_values[0])

            else:
                # calculate cell value with probabilistic formula
                all_odds = [cell_value_to_odds(v) for v in all_cell_values]
                world_odds = np.prod(all_odds)
                world_probability = world_odds / (1 + world_odds)
                world_cell_value = int(round(world_probability * 100))

                world_map_2d = set_grid_value(world_map_2d, world_map_2d_index, world_cell_value)
    
    return world_map_2d



def cell_value_to_odds(v):
    p = v / 100

    if p < 0.01:
        p = 0.01
    elif p > 0.996:
        p = 0.996

    return p / (1 - p)


def rotate_point(point, theta):
    rotation_matrix = np.array([[np.cos(theta), -1 * np.sin(theta)], [np.sin(theta), np.cos(theta)]])
    return rotation_matrix @ point


def find_corners(map_info):
    x = map_info.origin.position.x
    y = map_info.origin.position.y
    res = map_info.resolution
    width = map_info.width
    height = map_info.height
    
    corners = [
        np.array([x, y]),
        np.array([x + width * res, y]),
        np.array([x + width * res, y + height * res]),
        np.array([x, y + height * res]),
    ]

    return corners


def check_if_inside_corners(corners, point):
    x, y = point[0], point[1]

    corner_0, corner_2 = corners[0], corners[2]

    return not (x < corner_0[0] or x > corner_2[0] or y < corner_0[1] or y > corner_2[1])


def map_to_2d(mapData):
    return np.resize(np.array(mapData.data), (mapData.info.height, mapData.info.width))


def point_to_index(map_info, point):
    origin_x, origin_y = map_info.origin.position.x, map_info.origin.position.y
    resolution = map_info.resolution
    width, height = map_info.width, map_info.height

    x, y = point[0], point[1]

    index_x = int(round((x - origin_x) / resolution))
    index_y = int(round((y - origin_y) / resolution))

    index_x = index_x if index_x < width else width - 1
    index_y = index_y if index_y < height else height - 1

    return [index_x, index_y]


def index_to_point(map_info, index):
    origin_x, origin_y = map_info.origin.position.x, map_info.origin.position.y
    resolution = map_info.resolution

    index_x, index_y = index[0], index[1]

    x = origin_x + index_x * resolution
    y = origin_y + index_y * resolution

    return np.array([x, y])

def get_grid_value(data_2d, index):
    return data_2d[index[1], index[0]]


def set_grid_value(data_2d, index, value):
    data_2d[index[1], index[0]] = value
    return data_2d
