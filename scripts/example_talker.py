#!/usr/bin/env python
# source: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29#rospy_tutorials.2FTutorials.2FWritingPublisherSubscriber.Writing_the_Publisher_Node

import rospy
from std_msgs.msg import String

counter = 0

def talker():
    global counter
    pub = rospy.Publisher('/chatter', String, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        hello_str = "hello world: " + str(counter)
        counter += 1
        rospy.loginfo(hello_str)
        pub.publish(hello_str)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass