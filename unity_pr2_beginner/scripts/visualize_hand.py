#!/usr/bin/env python
from __future__ import print_function
import rospy
import visualization_msgs.msg
import random
import std_msgs.msg
import geometry_msgs.msg
from IPython import embed
rospy.init_node("visualize_hand")

random.seed(0)

publisher = rospy.Publisher("/hand_markers", visualization_msgs.msg.MarkerArray, queue_size=100)

chains = [
    [1, 3, 4, 5, 6],
    [1, 7, 8, 9, 10, 11],
    [1, 12, 13, 14, 15, 16],
    [1, 17, 18, 19, 20, 21],
    [1, 22, 23, 24, 25, 26],
]

xcolors = {
    1: (1, 1, 1),

    3: (1, 0, 0),
    4: (1, 0, 0),
    5: (1, 0, 0),
    6: (1, 0, 0),

    7: (0, 1, 0),
    8: (0, 1, 0),
    9: (0, 1, 0),
    10: (0, 1, 0),
    11: (0, 1, 0),

    12: (0, 0, 1),
    13: (0, 0, 1),
    14: (0, 0, 1),
    15: (0, 0, 1),
    16: (0, 0, 1),

    17: (0, 1, 1),
    18: (0, 1, 1),
    19: (0, 1, 1),
    20: (0, 1, 1),
    21: (0, 1, 1),

    22: (1, 1, 0),
    23: (1, 1, 0),
    24: (1, 1, 0),
    25: (1, 1, 0),
    26: (1, 1, 0),
}

links = []
for chain in chains:
    for i in range(1, len(chain)):
        links.append([chain[i - 1], chain[i]])

colors = []
marker_array = visualization_msgs.msg.MarkerArray()

def callback(msg):

    marker = visualization_msgs.msg.Marker()
    marker.type = marker.SPHERE_LIST
    marker.action = marker.ADD
    marker.header.frame_id = "map"
    marker.pose.orientation.w = 1.0
    marker.ns = "markers"
    marker.scale.x = 0.015
    marker.scale.y = 1
    marker.scale.z = 1
    for id, p in enumerate(msg.poses):
        if id == 0 or id == 2:
            continue
        print(id, p.position)
        marker.id = id
        marker.points.append(p.position)
        if id in xcolors:
                c = xcolors[id]
                color = std_msgs.msg.ColorRGBA()
                color.r = c[0]
                color.g = c[1]
                color.b = c[2]
                color.a = 1.0
                marker.colors.append(color)
                print(color)
        marker_array.markers.append(marker)

    publisher.publish(marker_array)

    marker = visualization_msgs.msg.Marker()
    marker.type = marker.LINE_LIST
    marker.action = marker.ADD
    marker.header.frame_id = "/ps"
    marker.pose.orientation.w = 1.0
    marker.ns = "links"
    marker.scale.x = 0.01
    # for link in links:
    #     for id, p in enumerate(msg.poses):
    #         if id == link[0]:
    #             a= p
    #             a_id = id
    #         if id == link[1]:
    #             b= p
    #             b_id = id
    #     # a, a_id = [p, id for id, p in enumerate(msg.poses) if id == link[0]]
    #     # b, b_id = [p, id for id, p in enumerate(msg.poses) if id == link[1]]
    #     if len(a) == 0 or len(b) == 0: continue
    #     a = a[0]
    #     b = b[0]
    #     marker.points.append(a.position)
    #     marker.points.append(b.position)
    #     marker.colors.append(colors[a_id])
    #     marker.colors.append(colors[b_id])
    # marker_array = visualization_msgs.msg.MarkerArray()
    # marker_array.markers.append(marker)
    # publisher.publish(marker_array)
    rospy.sleep(0.01)

sub = rospy.Subscriber("/left_hand", geometry_msgs.msg.PoseArray, callback)

rospy.spin()
