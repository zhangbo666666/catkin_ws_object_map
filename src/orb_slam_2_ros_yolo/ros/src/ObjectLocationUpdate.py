# -*- coding:utf-8 -*-
# !/usr/bin/env python2

import numpy as np
import math as math
import rospy
import time
from nav_msgs.msg import MapMetaData
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

rawMap = []
map_size = [7, 7]  # pixel level
origin = [1, 1]  # from top-right
resolution = 0.05

object_pos = [50, 15]  # from top-right
object_size = [1, 1.5]
object_angle = 60

robot_pose = [0, 0]

robot_diameter = 0.35  # meter

# new_map = np.reshape(rawMap, (map_size[0], map_size[1]))
new_map = []

def poseCallback(pose_data):
    global robot_pose, resolution, origin
    robot_pose = [pose_data.pose.pose.position.y, pose_data.pose.pose.position.x]
    robot_pose[0] = int(math.floor(robot_pose[0] / resolution)) + origin[0]
    robot_pose[1] = int(math.floor(robot_pose[1] / resolution)) + origin[1]
    print 'robot_pose = ', robot_pose

    update()

def objectCallback(object_data):
    global object_pos, object_size, object_angle, resolution, origin
    object_pos = [object_data.data[1], object_data.data[0]]
    object_size[0] = int(math.floor(object_data.data[2] / resolution))
    object_size[1] = int(math.floor(object_data.data[3] / resolution))

    object_angle = object_data.data[4]

    object_pos[0] = origin[0] + int(math.floor(object_pos[0] / resolution))
    object_pos[1] = origin[1] + int(math.floor(object_pos[1] / resolution))

    print 'object_pos = ', object_pos
    print 'object_size = ', object_size
    print 'object_angle = ', object_angle

    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.spin()


def mapMetaDataCallback(meta_data):
    global resolution
    resolution = meta_data.resolution
    global map_size
    map_size = [meta_data.height, meta_data.width]
    print 'map_size = ', map_size
    print 'resolution = ', resolution

    global origin
    origin = [meta_data.origin.position.x, meta_data.origin.position.y]
    origin[0] = int(math.floor(origin[0] / resolution))
    origin[1] = int(math.floor(origin[1] / resolution))
    origin = [-origin[1], -origin[0]]

    rospy.Subscriber("/nav_goal/object", Float64MultiArray, objectCallback)
    #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, poseCallback)
    rospy.spin()
    time.sleep(1)

def mapCallback(map_data):
    global rawMap
    rawMap = map_data.data

#def updateGoalObjectPos():
def main():
    rospy.init_node('update_nav_goal_object_pos', anonymous=True)
    rospy.Subscriber("/map_metadata", MapMetaData, mapMetaDataCallback)
    rospy.Subscriber("/map", OccupancyGrid, mapCallback)
    #rospy.Subscriber("/odom", Odometry, odomCallback)
    #rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, poseCallback)
    #rospy.Subscriber("/nav_goal/object", Float64MultiArray, objectCallback)

    rospy.spin()


def update():
    global new_map
    new_map = np.reshape(rawMap, (map_size[0], map_size[1]))

    # array to store new locations of four middle points
    free_location = [[-1, -1, 0], [-1, -1, 0], [-1, -1, 0], [-1, -1, 0]]
    safe_location = [[-1, -1], [-1, -1], [-1, -1], [-1, -1]]

    calculate_middle_points(object_angle, object_pos, object_size, free_location, 1, safe_location)

    print 'safe_location = ', safe_location

    distance = math.sqrt(map_size[0] ** 2 + map_size[1] ** 2)
    final_location = [-1, -1]
    for location in safe_location:
        if location[0] >= 0 and location[1] >= 0:
            dist = calculate_robot_dis(location)
            if dist < distance:
                final_location = [location[0], location[1]]
                distance = dist

    print("final location: ", final_location)

    global resolution, origin
    temp0 = (final_location[0] - origin[0]) * resolution
    temp1 = (final_location[1] - origin[1]) * resolution

    update_object_pos = [temp1, temp0]
    print 'update_object_pos = ', update_object_pos

    # publish the new object position
    pub = rospy.Publisher('/nav_goal/object/update', Float64MultiArray, queue_size=10)
    pub_data = Float64MultiArray(data=update_object_pos)

    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        rospy.loginfo(str(update_object_pos))
        pub.publish(pub_data)
        rate.sleep()

    print 'publish the object position'

    if final_location == [-1, -1]:
        print("didn't find a final location")

def calculate_middle_points(object_angle, object_pos, stride, free_location, count, safe_location):

    allset = True

    if free_location[0][0] == -1:
        # x0 = x + d*cosA, y0 = y + d*sinA
        point0 = [int(math.floor(object_pos[0] + np.cos(object_angle)*stride[0])), int(math.floor(object_pos[1] + np.sin(object_angle)*stride[1]))]
        # the point is free
        is_point_free(0, point0, free_location, count)
        allset = False

    if free_location[1][0] == -1:
        # x0 = x - d*cosA, y0 = y - d*sinA
        point1 = [int(math.floor(object_pos[0] - np.cos(object_angle)*stride[0])), int(math.floor(object_pos[1] - np.sin(object_angle)*stride[1]))]
        is_point_free(1, point1, free_location, count)
        allset = False

    if free_location[2][0] == -1:
        # x0 = x + d*cos(90 - A), y0 = y + d*sin(90 - A)
        point2 = [int(math.floor(object_pos[0] + np.sin(object_angle)*stride[0])), int(math.floor(object_pos[1] - np.cos(object_angle)*stride[1]))]
        is_point_free(2, point2, free_location, count)
        allset = False

    if free_location[3][0] == -1:
        # x0 = x + d*cos(90 - A), y0 = y + d*sin(90 - A)
        point3 = [int(math.floor(object_pos[0] - np.sin(object_angle)*stride[0])), int(math.floor(object_pos[1] + np.cos(object_angle)*stride[1]))]
        is_point_free(3, point3, free_location, count)
        allset = False

    if allset:
        safe_distance = 2 * robot_diameter / resolution
        detect_safe_location(object_angle, object_pos, stride, free_location, safe_distance, safe_location)
        print "safe: ", safe_location
    else:
        stride = np.add(stride, [1, 1])
        count += 1
        print free_location
        calculate_middle_points(object_angle, object_pos, stride, free_location, count, safe_location)

def detect_safe_location(object_angle, object_pos, stride, free_location, safe_distance, safe_location):


    if free_location[0][0] >= 0 and free_location[0][1] >= 0:
        # x0 = x + d*cosA, y0 = y + d*sinA
        point = [int(math.floor(object_pos[0] + np.cos(object_angle) * (stride[0] + safe_distance))),
                  int(math.floor(object_pos[1] + np.sin(object_angle) * (stride[1] + safe_distance)))]
        # the point is free
        if is_point_safe(point):
            safe_location[0] = [(free_location[0][0] + point[0]) / 2, (free_location[0][1] + point[1]) / 2]

    if free_location[1][0] >= 0 and free_location[1][1] >= 0:
        # x0 = x - d*cosA, y0 = y - d*sinA
        point = [int(math.floor(object_pos[0] - np.cos(object_angle) * (stride[0] + safe_distance))),
                  int(math.floor(object_pos[1] - np.sin(object_angle) * (stride[1] + safe_distance)))]
        # the point is free
        if is_point_safe(point):
            safe_location[1] = [(free_location[1][0] + point[0]) / 2, (free_location[1][1] + point[1]) / 2]

    if free_location[2][0] >= 0 and free_location[2][1] >= 0:
        # x0 = x + d*cos(90 - A), y0 = y + d*sin(90 - A)
        point = [int(math.floor(object_pos[0] + np.sin(object_angle) * (stride[0] + safe_distance))),
                  int(math.floor(object_pos[1] - np.cos(object_angle) * (stride[1] + safe_distance)))]

        # the point is free
        if is_point_safe(point):
            safe_location[2] = [(free_location[2][0] + point[0]) / 2, (free_location[2][1] + point[1]) / 2]

    if free_location[3][0] >= 0 and free_location[3][1] >= 0:
        # x0 = x + d*cos(90 - A), y0 = y + d*sin(90 - A)
        point = [int(math.floor(object_pos[0] - np.sin(object_angle) * (stride[0] + safe_distance))),
                 int(math.floor(object_pos[1] + np.cos(object_angle) * (stride[1] + safe_distance)))]
        # the point is free
        if is_point_safe(point):
            safe_location[3] = [(free_location[3][0] + point[0]) / 2, (free_location[3][1] + point[1]) / 2]


def is_point_free(index, point, free_location, count):
    if((point[0] < 0) or (point[0] > map_size[0] - 1) or (point[1] < 0) or (point[1] > map_size[1] - 1)):
        free_location[index] = [-2, -2, count]
    elif new_map[point[0], point[1]] == 0:
        free_location[index] = [point[0], point[1], count]
    elif new_map[point[0], point[1]] == -1:
        free_location[index] = [-2, -2, count]

def is_point_safe(point):
    if ((point[0] < 0) or (point[0] > map_size[0] - 1) or (point[1] < 0) or (point[1] > map_size[1] - 1)):
        return False
    elif new_map[point[0], point[1]] == 0:
        return True
    elif new_map[point[0], point[1]] == -1:
        return False

def calculate_robot_dis(point):

    dist = math.sqrt((point[0] - robot_pose[0]) ** 2 + (point[1] - robot_pose[1]) ** 2)
    return dist

if __name__ == "__main__":
    main()
