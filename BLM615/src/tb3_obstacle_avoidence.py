#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class ObstacleAvoidance:
    def __init__(self):
        try:
            rospy.init_node('obstacle_avoidance', anonymous=True)
            rospy.Subscriber('/scan', LaserScan, self.laser_callback)
            self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
            self.twist = Twist()

        except rospy.ROSException as e:
            rospy.logerr('Error initializing node: %s', str(e))
            return

    def laser_callback(self, data):
        try:
            front_distance = min(min(data.ranges[0:15]), min(data.ranges[-15:]))
            right_distance = min(data.ranges[300:359])
            left_distance = min(data.ranges[0:59])

            if front_distance < 0.3:
                self.twist.linear.x = 0
                self.twist.angular.z = 0
                self.pub.publish(self.twist)

                rospy.loginfo('Front distance: %.2f, Right distance: %.2f, Left distance: %.2f',
                              front_distance, right_distance, left_distance)

                if right_distance > left_distance:
                    self.twist.angular.z = -0.5
                else:
                    self.twist.angular.z = 0.5

            else:
                self.twist.linear.x = 0.2
                self.twist.angular.z = 0

            self.pub.publish(self.twist)

        except rospy.ROSException as e:
            rospy.logerr('Error in laser callback: %s', str(e))

if __name__ == '__main__':
    try:
        ObstacleAvoidance()
        rospy.spin()

    except rospy.ROSInterruptException as e:
        rospy.logerr('Error in main: %s', str(e))