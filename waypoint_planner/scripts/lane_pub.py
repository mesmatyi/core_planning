#!/usr/bin/env python


from geometry_msgs import msg
import rospy
import math

from autoware_msgs.msg import LaneArray
from autoware_msgs.msg import Lane
from autoware_msgs.msg import Waypoint
from autoware_msgs.msg import DTLane
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import TwistStamped
from autoware_msgs.msg import WaypointState
from std_msgs.msg import Int32


global_trajectory = None
base_waypoints_pub = None
avoid_waypoints = None
pub = None
cl_waypoint_pub = None
final_wp_arr = None
final_wp_MPC = None
sf_pub = None
closeset_waypoint = Int32()
base_waypoint_store = Lane()
current_pose = PoseStamped()

newLane = Lane()

def closestWaypoint(pose_msg):
    global newLane,cl_waypoint_pub,closeset_waypoint,current_pose

    current_pose = pose_msg

    rate = rospy.Rate(20)
    x = pose_msg.pose.position.x
    y = pose_msg.pose.position.y

    min_dist = 10000
    i_max = 0

    for i in range(len(newLane.waypoints)):
        wp_x = newLane.waypoints[i].pose.pose.position.x
        wp_y = newLane.waypoints[i].pose.pose.position.y
        dist = math.sqrt(math.pow(x - wp_x,2) + math.pow(y - wp_y,2))
        if dist < min_dist:
            min_dist = dist
            i_max = i

    closeset_waypoint.data = i_max

    if cl_waypoint_pub is not None:
        cl_waypoint_pub.publish(closeset_waypoint)
        rate.sleep()

def safety_Lane(LaneMsg):
    global sf_pub

    rate = rospy.Rate(10)

    LaneArr = LaneArray()
    LaneArr.id = 0
    LaneArr.lanes.append(LaneMsg)

    if sf_pub is not None:
        sf_pub.publish(LaneArr)
        rate.sleep()



def Lanearraycb(LaneMSG):
    global pub,newLane,avoid_waypoints,base_waypoints_pub,global_trajectory
    rate = rospy.Rate(5)

    newLane = LaneMSG.lanes[0]

    for i in range(1,len(LaneMSG.lanes)):
        for i in range(len(LaneMSG.lanes[i].waypoints)):
            newLane.waypoints.append(LaneMSG.lanes[i].waypoints[i])

    for waypoint in newLane.waypoints:
        waypoint.pose.header.frame_id = 'map'
        waypoint.twist.header.frame_id = 'map'

    global_trajectory = newLane

    # newLane.header = LaneMSG[0].header
    # newLane.increment = LaneMSG[0].increment
    # newLane.lane_id = LaneMSG[0].lane_id
    # newLane.lane_index = LaneMSG[0].lane_index
    # newLane.cost = LaneMSG[0].cost
    # newLane.closest_object_distance = LaneMSG[0].closest_object_distance
    # newLane.closest_object_velocity = LaneMSG[0].closest_object_velocity
    # newLane.is_blocked = LaneMSG[0].is_blocked

    # for i in range(len(LaneMSG[0].waypoints)):
    #     new_wayp = Waypoint()
    #     new_wayp.gid = LaneMSG[0].waypoints[i].gid
    #     new_wayp.lid = LaneMSG[0].waypoints[i].lid

    if pub is not None:
        pub.publish(newLane)
        rate.sleep()

            
def LanetoLaneArr(msg):
    global final_wp_arr,avoid_waypoints

    avoid_waypoints = msg
      
    LaneArr = LaneArray()
    LaneArr.id = 0
    LaneArr.lanes.append(msg)
    
    if final_wp_arr is not None:
        final_wp_arr.publish(LaneArr)
        
        
def sq_dist(wp_one,wp_two):
    
    return math.hypot(wp_two.pose.pose.position.x - wp_one.pose.pose.position.x,wp_two.pose.pose.position.y - wp_one.pose.pose.position.y)
        
        
def LaneArrtoLane(msg):
    global final_wp_MPC,global_trajectory,closeset_waypoint,base_waypoint_store,current_pose

    base_wp_lane = Lane()

    base_wp_lane.header = newLane.header
    base_wp_lane.header.stamp = rospy.Time.now()
    
    print("Final wp length:" + str(len(msg.lanes[0].waypoints)))

    # if global_trajectory is not None:
    #     for i in range(len(global_trajectory.waypoints)):
    #         if i < closeset_waypoint.data:
    #             base_wp_lane.waypoints.append(global_trajectory.waypoints[i])
                #base_wp_lane.waypoints[i].twist.twist.linear.x = (base_wp_lane.waypoints[i].twist.twist.linear.x * 3.6)

    for i in range(len(msg.lanes[0].waypoints)):
        msg.lanes[0].waypoints[i].twist.twist.linear.x = (msg.lanes[0].waypoints[i].twist.twist.linear.x * 3.6)
        # base_wp_lane.waypoints.append(msg.lanes[0].waypoints[i])
        
    # for i in range(len(base_wp_lane.waypoints)):
    #     base_wp_lane.waypoints[i].twist.twist.linear.x *= 3.6
    
    
    
    if len(base_waypoint_store.waypoints) == 0:
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y

        min_dist = 10000
        i_max = 0

        for i in range(len(msg.lanes[0].waypoints)):
            wp_x = msg.lanes[0].waypoints[i].pose.pose.position.x
            wp_y = msg.lanes[0].waypoints[i].pose.pose.position.y
            dist = math.sqrt(math.pow(x - wp_x,2) + math.pow(y - wp_y,2))
            if dist < min_dist:
                min_dist = dist
                i_max = i
                
        start_index = i_max - 100
        if start_index < 0:
            start_index = 0
            
        if global_trajectory is not None:
            for i in range(start_index,len(global_trajectory.waypoints)):
                    base_waypoint_store.waypoints.append(global_trajectory.waypoints[i])                   
        
        
        
    else:
        x = current_pose.pose.position.x
        y = current_pose.pose.position.y

        min_dist = 10000
        i_max = 0
        closest_pose = PoseStamped()

        
        for base_wp in range(0,len(base_waypoint_store.waypoints)):
            dist = sq_dist(base_waypoint_store.waypoints[base_wp],msg.lanes[0].waypoints[0])
            if dist < min_dist:
                min_dist = dist
                i_max = base_wp

           
        base_index = i_max - 100
        if(base_index < 0):
            base_index = 0
        

        for i in range(base_index,i_max):
                base_wp_lane.waypoints.append(base_waypoint_store.waypoints[i])
                
        for final_waypoint in msg.lanes[0].waypoints:
            base_wp_lane.waypoints.append(final_waypoint)

        base_waypoint_store = base_wp_lane
        

    if base_waypoints_pub is not None:
        base_waypoints_pub.publish(base_wp_lane)
    
    lane = Lane()
    lane = msg.lanes[0]
    
    if final_wp_MPC is not None:
        final_wp_MPC.publish(lane)
    
    

def talker():
    global pub,cl_waypoint_pub,sf_pub,final_wp_arr,final_wp_MPC,base_waypoints_pub
    rospy.init_node('lane_pub_node', anonymous=True)
    pub = rospy.Publisher('base_waypoints_global_traj', Lane, queue_size=10)
    cl_waypoint_pub = rospy.Publisher('astar_closest_wp',Int32,queue_size=10)
    final_wp_arr = rospy.Publisher('final_waypoints_arr',LaneArray,queue_size=10)
    final_wp_MPC = rospy.Publisher('final_waypoints',Lane,queue_size=10)
    base_waypoints_pub = rospy.Publisher('base_waypoints',Lane,queue_size=10)
    rospy.Subscriber('based/lane_waypoints_raw',LaneArray,Lanearraycb)
    rospy.Subscriber('/current_pose',PoseStamped,closestWaypoint)
    rospy.Subscriber('final_waypoints_from_veloc',Lane,LanetoLaneArr)
    rospy.Subscriber('final_waypoints_rp',LaneArray,LaneArrtoLane)
    #rospy.Subscriber('/safety_waypoints',Lane,safety_Lane)
    #sf_pub = rospy.Publisher('safety_waypoints_array',LaneArray,queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass