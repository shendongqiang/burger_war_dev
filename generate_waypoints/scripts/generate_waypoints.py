#!/usr/bin/env python

import os
import rospy
import csv
from visualization_msgs.msg import Marker
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseStamped

#path_w = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/patrol_waypoints.csv'
path_createWayPointsFromRvizNavGoal = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/rvizNavGoalList.txt'
#path_displayWayPoints = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/allWaypointsForDisplay.csv'
#path_displayAllWayPoints = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/allWaypointsForDisplay.csv'
path_displayPatrolWayPoints = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsForPatrol.csv'
path_displaySingleGoalWaypointsToEmptyCan = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToEmptyCan.csv'
path_displaySingleGoalWaypointsToGarbageBox = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToGarbageBox.csv'
path_displaySingleGoalWaypointsToDockStation = '/home/shen/catkin_ws/src/burger_war_dev/generate_waypoints/scripts/waypointsToDockStation.csv'

def topic_cb(data):
    with open(path_createWayPointsFromRvizNavGoal) as f_createWayPoints:
       #count rows
       num=len(f_createWayPoints.readlines()) 

    pos = data.pose
    #print "{0},{1},0.0,0.0,0.0,{2},{3},".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w)
    print "[{0},({1},{2},0.0),(0.0,0.0,{3},{4})],".format(num-1,pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w)

    with open(path_createWayPointsFromRvizNavGoal, mode='a') as f_createWayPoints:
       #f.writelines(pos.position.x,",",pos.position.y,",",pos.orientation.z,",",pos.orientation.w,os.linesep)
       #f.write(pos.position.x,",",pos.position.y,",",pos.orientation.z,",",pos.orientation.w,os.linesep)
       #f.write("[({0},{1},0.0),(0.0,0.0,{2},{3})],".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))
       #f.write("{0},{1},0.0,0.0,0.0,{2},{3}".format(pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))
       f_createWayPoints.write("{0},{1},{2},0.0,0.0,0.0,{3},{4}".format(num-1,pos.position.x,pos.position.y,pos.orientation.z,pos.orientation.w))
       f_createWayPoints.write(os.linesep)

rospy.init_node("waypoint_manager")

pub = rospy.Publisher("waypoint", Marker, queue_size = 100)

rospy.Subscriber("/move_base_simple/goal", PoseStamped, topic_cb)
#rospy.Subscriber("/generate_waypoints/goal", PoseStamped, topic_cb)

#rate = rospy.Rate(25)
rate = rospy.Rate(10)


while not rospy.is_shutdown():

    #display patrol waypoints
    #with open(path_displayWayPoints, 'r') as f_display:
    with open(path_displayPatrolWayPoints, 'r') as f_display:
        counter = 0
        reader = csv.reader(f_display)
        header = next(reader)

        for row in reader:
            # Mark arrow
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes_Patrol"
            marker_data.id = counter

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = map(float,row)[1]
            marker_data.pose.position.y = map(float,row)[2]
            marker_data.pose.position.z = map(float,row)[3]

            marker_data.pose.orientation.x=map(float,row)[4]
            marker_data.pose.orientation.y=map(float,row)[5]
            marker_data.pose.orientation.z=map(float,row)[6]
            marker_data.pose.orientation.w=map(float,row)[7]

            marker_data.color.r = 1.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 0.1
            marker_data.scale.y = 0.05
            marker_data.scale.z = 0.01

            marker_data.lifetime = rospy.Duration()

            marker_data.type = 0

            pub.publish(marker_data)
            counter +=1


            # Mark num
            marker_data = Marker()
            marker_data.header.frame_id = "map"
            marker_data.header.stamp = rospy.Time.now()

            marker_data.ns = "basic_shapes_Patrol"
            marker_data.id = counter

            marker_data.action = Marker.ADD

            marker_data.pose.position.x = map(float,row)[1]
            marker_data.pose.position.y = map(float,row)[2]
            marker_data.pose.position.z = map(float,row)[3]

            marker_data.pose.orientation.x=map(float,row)[4]
            marker_data.pose.orientation.y=map(float,row)[5]
            marker_data.pose.orientation.z=map(float,row)[6]
            marker_data.pose.orientation.w=map(float,row)[7]

            marker_data.color.r = 0.0
            marker_data.color.g = 0.0
            marker_data.color.b = 0.0
            marker_data.color.a = 1.0
            marker_data.scale.x = 0.2
            marker_data.scale.y = 0.2
            marker_data.scale.z = 0.2

            marker_data.lifetime = rospy.Duration()

            marker_data.type = Marker.TEXT_VIEW_FACING
            marker_data.text = str(int(map(float,row)[0]))

            pub.publish(marker_data)
            counter +=1

    #display waypoints move to empty can
    with open(path_displaySingleGoalWaypointsToEmptyCan, 'r') as f_EmptyCan:
        counter_EmptyCan = 0
        reader_EmptyCan = csv.reader(f_EmptyCan)
        header_EmptyCan = next(reader_EmptyCan)

        for row_EmptyCan in reader_EmptyCan:
            # Mark arrow
            marker_data_EmptyCan = Marker()
            marker_data_EmptyCan.header.frame_id = "map"
            marker_data_EmptyCan.header.stamp = rospy.Time.now()

            marker_data_EmptyCan.ns = "basic_shapes_EmptyCan"
            marker_data_EmptyCan.id = counter_EmptyCan

            marker_data_EmptyCan.action = Marker.ADD

            marker_data_EmptyCan.pose.position.x = map(float,row_EmptyCan)[1]
            marker_data_EmptyCan.pose.position.y = map(float,row_EmptyCan)[2]
            marker_data_EmptyCan.pose.position.z = map(float,row_EmptyCan)[3]

            marker_data_EmptyCan.pose.orientation.x=map(float,row_EmptyCan)[4]
            marker_data_EmptyCan.pose.orientation.y=map(float,row_EmptyCan)[5]
            marker_data_EmptyCan.pose.orientation.z=map(float,row_EmptyCan)[6]
            marker_data_EmptyCan.pose.orientation.w=map(float,row_EmptyCan)[7]

            marker_data_EmptyCan.color.r = 0.0
            marker_data_EmptyCan.color.g = 1.0
            marker_data_EmptyCan.color.b = 0.0
            marker_data_EmptyCan.color.a = 1.0
            marker_data_EmptyCan.scale.x = 0.05
            marker_data_EmptyCan.scale.y = 0.05
            marker_data_EmptyCan.scale.z = 0.01

            marker_data_EmptyCan.lifetime = rospy.Duration()

            marker_data_EmptyCan.type = 2

            pub.publish(marker_data_EmptyCan)
            counter_EmptyCan +=1


    #display waypoints move to GarbageBox
    with open(path_displaySingleGoalWaypointsToGarbageBox, 'r') as f_GarbageBox:
        counter_GarbageBox = 0
        reader_GarbageBox = csv.reader(f_GarbageBox)
        header_GarbageBox = next(reader_GarbageBox)

        for row_GarbageBox in reader_GarbageBox:
            # Mark arrow
            marker_data_GarbageBox = Marker()
            marker_data_GarbageBox.header.frame_id = "map"
            marker_data_GarbageBox.header.stamp = rospy.Time.now()

            marker_data_GarbageBox.ns = "basic_shapes_GarbageBox"
            marker_data_GarbageBox.id = counter_GarbageBox

            marker_data_GarbageBox.action = Marker.ADD

            marker_data_GarbageBox.pose.position.x = map(float,row_GarbageBox)[1]
            marker_data_GarbageBox.pose.position.y = map(float,row_GarbageBox)[2]
            marker_data_GarbageBox.pose.position.z = map(float,row_GarbageBox)[3]

            marker_data_GarbageBox.pose.orientation.x=map(float,row_GarbageBox)[4]
            marker_data_GarbageBox.pose.orientation.y=map(float,row_GarbageBox)[5]
            marker_data_GarbageBox.pose.orientation.z=map(float,row_GarbageBox)[6]
            marker_data_GarbageBox.pose.orientation.w=map(float,row_GarbageBox)[7]

            marker_data_GarbageBox.color.r = 0.0
            marker_data_GarbageBox.color.g = 0.0
            marker_data_GarbageBox.color.b = 1.0
            marker_data_GarbageBox.color.a = 1.0
            marker_data_GarbageBox.scale.x = 0.05
            marker_data_GarbageBox.scale.y = 0.1
            marker_data_GarbageBox.scale.z = 0.01

            marker_data_GarbageBox.lifetime = rospy.Duration()

            marker_data_GarbageBox.type = 1
            #marker_data_GarbageBox.type = shape

            pub.publish(marker_data_GarbageBox)
            counter_GarbageBox +=1


    #display waypoints move to DockStation
    with open(path_displaySingleGoalWaypointsToDockStation, 'r') as f_DockStation:
        counter_DockStation = 0
        reader_DockStation = csv.reader(f_DockStation)
        header = next(reader_DockStation)

        for row_DockStation in reader_DockStation:
            # Mark arrow
            marker_data_DockStation = Marker()
            marker_data_DockStation.header.frame_id = "map"
            marker_data_DockStation.header.stamp = rospy.Time.now()

            marker_data_DockStation.ns = "basic_shapes_DockStation"
            marker_data_DockStation.id = counter_DockStation

            marker_data_DockStation.action = Marker.ADD

            marker_data_DockStation.pose.position.x = map(float,row_DockStation)[1]
            marker_data_DockStation.pose.position.y = map(float,row_DockStation)[2]
            marker_data_DockStation.pose.position.z = map(float,row_DockStation)[3]

            marker_data_DockStation.pose.orientation.x=map(float,row_DockStation)[4]
            marker_data_DockStation.pose.orientation.y=map(float,row_DockStation)[5]
            marker_data_DockStation.pose.orientation.z=map(float,row_DockStation)[6]
            marker_data_DockStation.pose.orientation.w=map(float,row_DockStation)[7]

            marker_data_DockStation.color.r = 0.0
            marker_data_DockStation.color.g = 1.0
            marker_data_DockStation.color.b = 1.0
            marker_data_DockStation.color.a = 1.0
            marker_data_DockStation.scale.x = 0.05
            marker_data_DockStation.scale.y = 0.05
            marker_data_DockStation.scale.z = 0.01

            marker_data_DockStation.lifetime = rospy.Duration()

            marker_data_DockStation.type = 1

            pub.publish(marker_data_DockStation)
            counter_DockStation +=1


    rate.sleep()

rospy.spin()

