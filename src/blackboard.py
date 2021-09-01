import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, CompressedImage

class SensorPublisher:

    def __init__(self, blackboard, rate=0.034):

        subscribers = []

        self.rate = rate

        self.time_last_pub = rospy.get_time()

        for var in self.blackboard:

            if var[0] == "/":
                msg_type = self.blackboard[var]
                self.blackboard[var] = None
                subscribers.append(rospy.Subscriber(var, msg_type, self.callback, var))
                


    def callback(self, msg, var_name):

        self.blackboard[var_name] = msg

