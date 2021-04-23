import rospy

from autoware_msgs.msg import Lane
import visualization_msgs.msg as viz

pub = None


def Lanecb(msg):
    global pub


    rate = rospy.Rate(10)
    MarkerArr = viz.MarkerArray()

    for i in range(len(msg.waypoints)):
      
        Marker = viz.Marker()

        Marker.header.frame_id = "map"
        Marker.header.stamp = rospy.Time.now()

        Marker.type = 0
        Marker.action = 0
        Marker.scale.x = 0.8
        Marker.scale.y = 0.15
        Marker.scale.z = 0.15
        Marker.color.r = 0.1
        Marker.color.g = 1.0
        Marker.color.b = 0.0
        Marker.color.a = 1.0
        Marker.pose = msg.waypoints[i].pose.pose
        Marker.id = i
        MarkerArr.markers.append(Marker)
    
    if pub is not None:
        pub.publish(MarkerArr)
        rate.sleep()

def talker():
    global pub

    rospy.init_node('talker', anonymous=True)
    rospy.Subscriber('final_waypoints',Lane,Lanecb)
    pub = rospy.Publisher('final_wp_local', viz.MarkerArray, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass