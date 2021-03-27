#!/usr/bin/python
# created by buenos at 2021.3.26

import rospy
import tf
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Range
from nav_msgs.msg import Odometry
import serial
import struct
import time

# const
DEBUG = True
DEVICE = "/dev/ttyACM0"  
SONAR_MAXDIS = 0.2          # m
LOOP_RATE = 4              # hz

class Controller:
    def __init__(self):
        # serial init
        self.ser_ = serial.Serial(DEVICE, 2000000, timeout=0.5)

        # msg init
        self.destBytes = ""
        self.odomBytes = ""
        self.frontSensorBytes = ""
        self.leftSensorBytes = ""
        self.rightSensorBytes = ""
        self.initMsg_()

        # ROS init
        rospy.init_node("controller_node", anonymous=True, log_level=rospy.DEBUG if DEBUG else rospy.INFO)
        self.rate = rospy.Rate(10)
        self.pub = rospy.Publisher("/rrc_2wheel_robot/cmd_vel", Twist, queue_size=1)
        rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.destCb_)
        rospy.Subscriber("/rrc_2wheel_robot/odom", Odometry, self.odomCb_)
        rospy.Subscriber("/sonar_f_sensor", Range, self.frontSensorCb_)
        rospy.Subscriber("/sonar_l_sensor", Range, self.leftSensorCb_)
        rospy.Subscriber("/sonar_r_sensor", Range, self.rightSensorCb_)

    def run(self):
        while not rospy.is_shutdown():
            self.sendRobotMsg_()
            ret, v, w = self.recvControlMsg_()
            if ret:
                rospy.logdebug("v: %.2f, w: %.2f" % (v, w))
                self.pub.publish(self.genControlMsg_(v, w))
            self.rate.sleep()

    def initMsg_(self):
        self.destBytes = struct.pack("<ff", 4.0, 5.0)
        self.odomBytes = struct.pack("<fff", 0.0, 0.0, 0.0)
        self.frontSensorBytes = struct.pack("<f", SONAR_MAXDIS)
        self.leftSensorBytes = struct.pack("<f", SONAR_MAXDIS)
        self.rightSensorBytes = struct.pack("<f", SONAR_MAXDIS)

    def recvControlMsg_(self):
        """
        # msg: v(float)   w(float) /n /r
        # len: 4          4        1  1
        # val: 0.1        0.0      /n /r
        """
        pattern = "<ffcc"
        byteSize = 4 + 4 + 1 + 1
        byteData = self.ser_.read(byteSize)
        if len(byteData) != byteSize:
            # drop wrong bytes
            return False, 0.0, 0.0
        v, w, _, _ = struct.unpack(pattern, byteData)
        return True, v, w


    def sendRobotMsg_(self):
        bytesData = self.destBytes + self.odomBytes + self.frontSensorBytes + self.leftSensorBytes + self.rightSensorBytes
        self.ser_.write(bytesData)

    def destCb_(self, msg):
        x = msg.pose.position.x
        y = msg.pose.position.y
        self.destBytes = struct.pack("<ff", 10, 10)

    def odomCb_(self, msg):
        x0 = msg.pose.pose.position.x
        y0 = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        _, _, yaw = tf.transformations.euler_from_quaternion([x, y, z, w]) 
	#rospy.logdebug("x: %.2f, y: %.2f" % (x0, y0))
        self.odomBytes = struct.pack("<fff", x0, y0, yaw)

    def frontSensorCb_(self, msg):
        self.frontSensorBytes = struct.pack("<f", msg.range)

    def leftSensorCb_(self, msg):
        self.leftSensorBytes = struct.pack("<f", msg.range)

    def rightSensorCb_(self, msg):
        self.rightSensorBytes = struct.pack("<f", msg.range)

    def genControlMsg_(self, v, w):
        msg = Twist()
        msg.linear.x  = v
        msg.angular.z = w
        return msg

if __name__=="__main__":
    con = Controller()
    con.run()
