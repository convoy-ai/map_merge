#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid, MapMetaData
from std_msgs.msg import Header
from geometry_msgs.msg import Pose, Point, Quaternion, TransformStamped, Vector3
import tf2_ros
import tf_conversions

import numpy as np
import math

import functions as fn


robot_maps = {} # dict of OccupancyGrid


def create_map_callback(namespace):
    def map_callback(data):
        global robot_maps
        robot_maps[namespace] = data

    return map_callback


def node():
    global robot_maps

    rospy.init_node('map_merge', anonymous=False)

    world_map_topic = rospy.get_param('~world_map_topic', '/map')
    world_map_frame = rospy.get_param('~world_map_frame', 'world')
    rateHz = rospy.get_param('~rate', 1)


    robot_namespaces = ['robot_1', 'robot_2']
    robot_initial_poses = {}


    # ------------------------------------------------

    for robot_namespace in robot_namespaces:
        map_topic = f'{robot_namespace}/map'
        rospy.Subscriber(map_topic, OccupancyGrid, create_map_callback(robot_namespace))

        init_x = rospy.get_param(f'{robot_namespace}/map_merge/init_pose_x', '0.0')
        init_y = rospy.get_param(f'{robot_namespace}/map_merge/init_pose_y', '0.0')
        init_yaw = rospy.get_param(f'{robot_namespace}/map_merge/init_pose_yaw', '0.0')

        robot_initial_poses[robot_namespace] = {'x': init_x, 'y': init_y, 'yaw': init_yaw}

    world_map_publisher = rospy.Publisher(world_map_topic, OccupancyGrid, queue_size=math.ceil(rateHz * 5))

    rate = rospy.Rate(rateHz)

    world_map = OccupancyGrid()
    world_map.info.origin = Pose(position=Point(x=-5, y=-5, z=0.0), orientation=Quaternion(0, 0, 0, 1))
    world_map.info.resolution = 0.05
    world_map.info.width = 100
    world_map.info.height = 150

    tf_broadcaster = tf2_ros.TransformBroadcaster()

    
    # ------------------------------------------------

    rospy.loginfo("Waiting for robot maps")
    
    # wait for all robot maps
    while len(robot_maps.keys()) != len(robot_namespaces):
        rospy.spin()
        rospy.sleep(0.1)



    # ------------------------------------------------

    rospy.loginfo("Started main loop")


    while not rospy.is_shutdown():
        


        # calculate world map metadata
        world_map = fn.calc_world_map_metadata(world_map, robot_maps, robot_initial_poses)

        # todo: calculate world map data

        world_map_2d = np.full((world_map.info.height, world_map.info.width), -1, dtype=np.int8)
        world_map.data = np.resize(world_map_2d, (world_map.info.height * world_map.info.width)).tolist()

        # setup world map time
        world_map.header = Header(stamp=rospy.Time.now(), frame_id=world_map_frame)
        world_map.info.map_load_time = rospy.Time.now()

        # publish world map
        rospy.loginfo(world_map.info)
        world_map_publisher.publish(world_map)


        # publish transforms
        transform = TransformStamped()

        # shared header
        transform.header.stamp = rospy.Time.now()
        transform.header.frame_id = world_map_frame

        for robot_namespace in robot_namespaces:
            initial_pose = robot_initial_poses[robot_namespace]
            
            # child frame
            transform.child_frame_id = f'{robot_namespace}/map'

            # todo: calculate based on world map origin

            # translation and rotation
            transform.transform.translation = Vector3(x=initial_pose['x'], y=initial_pose['y'], z=0)
            q = tf_conversions.transformations.quaternion_from_euler(0, 0, initial_pose['yaw'])
            transform.transform.rotation = Quaternion(q[0], q[1], q[2], q[3])
            
            tf_broadcaster.sendTransform(transform)





        # --------------------------------------------

        rate.sleep()





if __name__ == '__main__':
    try:
        node()
    except rospy.ROSInterruptException:
        pass