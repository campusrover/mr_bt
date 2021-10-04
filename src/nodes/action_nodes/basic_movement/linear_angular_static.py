#!/usr/bin/env python3


import rospy

from geometry_msgs.msg import Twist

from ...nodes import Action



class LinearAngularStatic(Action):


    def __init__(self, lin_vel, ang_vel):

        self.twist = Twist()

        self.twist.linear.x = lin_vel

        self.twist.angular.z = ang_vel

        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


    def tick(self, blackboard):

        self.pub.publish(self.twist)

        return "success"




