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

newLane = Lane()
base_wp_lane = Lane()

def closestWaypoint(pose_msg):
    global newLane,cl_waypoint_pub,closeset_waypoint,base_wp_lane


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
        while True:
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
        
        
def LaneArrtoLane(msg):
    global final_wp_MPC,global_trajectory,closeset_waypoint,base_wp_lane

    base_wp_lane.header = newLane.header
    base_wp_lane.header.stamp = rospy.Time.now()

    if global_trajectory is not None:
        for i in range(len(global_trajectory.waypoints)):
            if i < closeset_waypoint.data:
                base_wp_lane.waypoints.append(global_trajectory.waypoints[i])

    for i in range(len(msg.lanes[0].waypoints)):
        msg.lanes[0].waypoints[i].twist.twist.linear.x = (msg.lanes[0].waypoints[i].twist.twist.linear.x * 3.6)
        base_wp_lane.waypoints.append(msg.lanes[0].waypoints[i])


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
    cl_waypoint_pub = rospy.Publisher('closest_waypoint',Int32,queue_size=10)
    final_wp_arr = rospy.Publisher('final_waypoints_arr',LaneArray,queue_size=10)
    final_wp_MPC = rospy.Publisher('final_waypoints_for_MPC',Lane,queue_size=10)
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