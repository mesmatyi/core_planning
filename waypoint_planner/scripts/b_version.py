#!/usr/bin/env python



import numpy as np
import rospy
from autoware_msgs.msg import Lane, Waypoint
import visualization_msgs.msg as vismsg
from geometry_msgs.msg import PoseStamped,TwistStamped
import math
from autoware_msgs.msg import Lane
from std_msgs.msg import Int32
#from shapely.geometry import LineString

import std_msgs.msg as std

lookahead = 100
presence_threshold = 10

current_pose = None
collect_intersected_waypoints=[]
midle_index_list=[]
path_replanned=False
count=0
center=0
uj_szakasz_index_array=[]
original_szakasz = None
detected_object = None
pub_obstacle_wp = None

counter_array=None

elkerules = []


def closest_point(data,x,y):
    min_distance, min_index = 10000000, 0
    for i,w in enumerate(data): 
        dx2 = w[0] - x                                  
        dy2 = w[1] - y                                      
        distance = math.sqrt(dx2**2+dy2**2)
        if distance < min_distance:
            min_distance, min_index = distance,i
    return min_index
    

def line_orientation(x1, x2, y1, y2):
    # https://en.wikipedia.org/wiki/Atan2
    return np.arctan2((y2-y1), (x2-x1))

def line_length(x1, x2, y1, y2):
    return ((x1-x2)**2 + (y1-y2)**2)**0.5 




waypoint_list = []

def waypoints_callback(msg):
    global waypoint_list,elkerules

    del waypoint_list[:]

    for waypoint in msg.waypoints:
        waypoint_list.append([waypoint.pose.pose.position.x,waypoint.pose.pose.position.y,0,0,0])

    elkerules=np.array(waypoint_list)

def callback_current_pose(pose):
    global current_pose
    current_pose = pose

def callback_detectedobjects(data):
    global collect_intersected_waypoints,midle_index_list,path_replanned,elkerules,count,center,counter_array,uj_szakasz_index_array,original_szakasz,detected_objec,pub_obstacle_wp

    waypoints_size = len(elkerules)
    centroids=np.empty((len(data.markers),2))
    polygon_list=[]
    obs_wp = Int32()


    for i in range (len(data.markers)):
        centroids[i,0]=data.markers[i].pose.position.x
        centroids[i,1]=data.markers[i].pose.position.y
        polygon_data=[]
        for j in range(len(data.markers[i].points)):
            polygon_data.append([data.markers[i].points[j].x,data.markers[i].points[j].y])    
        polygon_list.append(polygon_data)


    if current_pose is not None:
        if path_replanned==False:
            
            #rospy.loginfo("Obstacle avoidance started,No valid points")
            closest_waypoint = closest_point(elkerules,current_pose.pose.position.x,current_pose.pose.position.y) 
            
            la = lookahead
            if waypoints_size - closest_waypoint < la:
                la = waypoints_size - closest_waypoint

            min_dist=10000
            min_index=0
            min_j_index=0

            for i in range(closest_waypoint,closest_waypoint+la):
                for j in range(len(centroids)):
                    dist = math.sqrt(math.pow(elkerules[i][0] - centroids[j,0],2) + math.pow(elkerules[i][1] - centroids[j,1],2))
                    if dist < min_dist:
                        min_dist=dist
                        min_index=i
                        min_j_index=j
            if min_dist < 1.5:
                collect_intersected_waypoints.append(min_index)

            # print(min_index)

            # print(collect_intersected_waypoints)
            if len(collect_intersected_waypoints) > 0 :
               
                
                counter_array = np.bincount(np.array(collect_intersected_waypoints),minlength=waypoints_size) 
                valid_point = np.asarray(np.where(counter_array > presence_threshold))

                if(valid_point.size > 0):
                    obs_wp.data = min_index

                # print(valid_point)


            else:
                obs_wp.data = -1

            if pub_obstacle_wp is not None:
                    pub_obstacle_wp.publish(obs_wp)
                    
            
        # else:
        #     rospy.loginfo('path replanned')

def pub():
    
    global pub_obstacle_wp #,time2,time3,time4
    rospy.init_node('points')
    rospy.Subscriber("/converted_euclidean_objects", vismsg.MarkerArray, callback_detectedobjects)
    rospy.Subscriber("/current_pose", PoseStamped,callback_current_pose)
    rospy.Subscriber("/safety_waypoints",Lane,waypoints_callback)
    pub_obstacle_wp = rospy.Publisher("obstacle_waypoint_from_eu",Int32,queue_size=1)

    rospy.spin()

if __name__ == '__main__':
    try:
        pub()
    except rospy.ROSInterruptException:
        pass