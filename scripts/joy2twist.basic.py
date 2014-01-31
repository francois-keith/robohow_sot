#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Joy


#def callback(data):
#    rospy.loginfo(rospy.get_name() + ": I heard %f" % data.axes[0])

#
#def listener():
#    rospy.init_node('listener', anonymous=True)
#    rospy.Subscriber("joy", Joy, callback)
#    rospy.spin()
#

#if __name__ == '__main__':
#    listener()




## publisher
from geometry_msgs.msg import Twist, Vector3

#if __name__ == '__main__':
#    try:
#        pub = rospy.Publisher('twist', Twist)
#        rospy.init_node('twister')
#    except rospy.ROSInterruptException:
#        pass


def talker():
    pub = rospy.Publisher('twist', Twist)
    rospy.init_node('twister')
    while not rospy.is_shutdown():
        rospy.loginfo("youhou")
        pub.publish(Twist(linear=Vector3(0,1,2), angular=Vector3(3,4,5)))
        rospy.sleep(1.0)


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
