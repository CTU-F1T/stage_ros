#!/usr/bin/env python
# stage_with_ros_map.py
"""Prepare and run a Stage World that uses current ROS map.
"""

import rospy
import numpy, matplotlib.image

from nav_msgs.msg import OccupancyGrid
occupied_thresh = 0.65
free_thresh = 0.1

import subprocess


class StageNode(object):

    def __init__(self):
        super(StageNode, self).__init__()

        rospy.init_node("stage_node")

        self.map_received = False


    def callback_map(self, data):
        if self.map_received:
            return
        else:
            self.map_received = True

        rospy.loginfo("Static map received" if rospy.resolve_name("/map_static") == "/map_static" else "Map received")
        rospy.loginfo("%d X %d map @ %.3f m/cell" % (data.info.width, data.info.height, data.info.resolution))

        # Convert map to numpy.ndarray
        _map = numpy.asarray(data.data).reshape(data.info.height, data.info.width)
        _map = _map * 2.55

        _map[_map > occupied_thresh*255] = 255
        _map[_map < 0] = 127
        _map[_map < free_thresh*255] = 0

        # Add borders so the map does not shrink in Stage
        _map[0, :] = 255
        _map[-1, :] = 255
        _map[:, 0] = 255
        _map[:, -1] = 255

        # Save map to temporary file
        matplotlib.image.imsave("/tmp/ros_map.png", _map, cmap = 'binary', vmin = 0, vmax = 255, origin = 'lower')
        rospy.loginfo("Map saved to '%s'." % "/tmp/ros_map.png")

        # Create World file
        with open("/tmp/ros.world", "w") as f:
            f.write("""
resolution 0.01

interval_sim 100

define floorplan model
(
    # sombre, sensible, artistic
    color "gray30"

    # most maps will need a bounding box
    boundary 0

    gui_nose 0
    gui_grid 0
    gui_move 0 # Do not move the map

    gui_outline 0
    gripper_return 0
    fiducial_return 0
    ranger_return 1
)

define ust10lx ranger
(
    sensor( 			
        range [ 0.0  10.0 ]
        fov 270.0
        samples 3243
    )

    # generic model properties
    color "orange"
)

define car position
(
    # velikost
    size [0.565 0.29 0.175]
    origin [-0.165 0 0 0]
    gui_nose 1

    # model (tx2-auto-3)
    block
    (
        points 12
        point[0] [16.43 7.25]
        point[1] [16.43 9.71]
        point[2] [14 12.76]
        point[3] [14 16.25]
        point[4] [16.43 19.29]
        point[5] [16.43 21.75]
        point[6] [36.63 21.75]
        point[7] [39.35 17.86]
        point[8] [45 16.59]
        point[9] [45 12.41]
        point[10] [39.35 11.14]
        point[11] [36.63 7.25]
        z [1 8.5]
    )

    # Kola LZ
    block
    (
        points 4
        point[0] [7.5 0]
        point[1] [7.5 6]
        point[2] [18 6]
        point[3] [18 0]
        z [0 10.5]
        color "black"
    )

    # Kola PZ
    block
    (
        points 4
        point[0] [7.5 23]
        point[1] [7.5 29]
        point[2] [18 29]
        point[3] [18 23]
        z [0 10.5]
        color "black"
    )

    # Kola PP
    block
    (
        points 4
        point[0] [39.5 23]
        point[1] [39.5 29]
        point[2] [50 29]
        point[3] [50 23]
        z [0 10.5]
        color "black"
    )

    # Kola LP
    block
    (
        points 4
        point[0] [39.5 0]
        point[1] [39.5 6]
        point[2] [50 6]
        point[3] [50 0]
        z [0 10.5]
        color "black"
    )

    # Predni naraznik
    block
    (
        points 4
        point[0] [54.5 4.75]
        point[1] [54.5 24.25]
        point[2] [56.5 24.25]
        point[3] [56.5 4.75]
        z [9 12]
        color "black"
    )

    # Zadni naraznik
    block
    (
        points 4
        point[0] [0 6]
        point[1] [0 23]
        point[2] [0.5 23]
        point[3] [0.5 6]
        z [9 12]
        color "black"
    )

    # Podstavec lidaru (pod nim)
    block
    (
        points 4
        point[0] [28.5 12]
        point[1] [28.5 17]
        point[2] [33.5 17]
        point[3] [33.5 12]
        z [8.5 10.5]
        color "blue"
    )

    # Podstavec lidaru (cast lidaru)
    block
    (
        points 4
        point[0] [28.5 12]
        point[1] [28.5 17]
        point[2] [33.5 17]
        point[3] [33.5 12]
        z [10.5 14]
        color "black"
    )

    # lidar
    block
    (
        points 4
        point[0] [28.5 12]
        point[1] [28.5 17]
        point[2] [33.5 17]
        point[3] [33.5 12]
        z [14 17.5]
        color "orange"
    )

    # typ pohybu
    drive "car"

    # typ lokalizace
    localization "gps"
    # gps = absolutni
    # odom = relativni

    # vzdalenost kol
    wheelbase 0.32

    # LiDAR
    # pozice v metrech, musi se pripocitat i posun stredu otaceni
    # a pocatecni poloha
    ust10lx ( pose [ -0.12 0 0.1575 0 ] size [ 0.05 0.05 0.035 ] )

    # Camera
    #simcam ( pose [ -0.1375 0 0.175 0 ] size [ 0.025 0.09 0.025 ] )

    # Apriltags (octa)
    #atag0_octa( pose [ -0.32 0 0 0 ] )
    #atag0_cube( pose [ -0.26 0 0 0 ] )
    # Vztahuje se ke stredu tohoto objektu, a Z zacina na maximu Z z blokoveho popisu!
    #atag_cube( pose [ -0.285 0 -0.025 0 ] )

    stack_children 0

    # Omezeni na rychlost
    # Pokud neni uvedeno, je to +-1 na xyz, +-90 na a.
    # min, max
    # x - dopredna rychlost
    # y - lateralni rychlost (pro auto muze byt 0)
    # z - taky 0
    # a - zataceni (ve stupnich)
    velocity_bounds [ -33 33 0 0 0 0 -90 90 ]
)

floorplan
( 
    name "ros_map"
    bitmap "/tmp/ros_map.png"
    size [%f %f 0.5]
    pose [%f %f 0 0]
)

car( pose [ 0 0 0 0 ] name "car" color "blue")
car( pose [ 0 0 0 0 ] name "car2" color "orange")
            """ % (data.info.width * data.info.resolution, data.info.height * data.info.resolution, ((data.info.width * data.info.resolution)/2)+data.info.origin.position.x, ((data.info.height * data.info.resolution)/2)+data.info.origin.position.y) )

        rospy.loginfo("World saved to '%s'." % "/tmp/ros.world")


if __name__ == '__main__':
    sn = StageNode()

    rospy.loginfo("Waiting for static map on topic '%s'..." % rospy.resolve_name("/map_static"))

    #rospy.spin()
    sn.callback_map(
        rospy.wait_for_message("/map_static", OccupancyGrid, timeout = None)
    )

    rospy.loginfo("Running Stage...")

    subprocess.call(["rosrun", "stage_ros", "stageros", "/tmp/ros.world"])
