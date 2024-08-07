#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan   

class ReactiveNavigation():
    def _init_(self):
        self.cmd_vel = Twist()
        self.laser_msg = LaserScan()
        self.robot_stopped = False
        self.obstacle_distance = 100

         # Params from config file
        self.active_robot = str(rospy.get_param("reactive_controller_py/active_robot"))
        
        # Topics
        self._cmd_topic = self.active_robot+"/cmd_vel"
        self._laser_topic = self.active_robot+"/base_scan"

        self.rospy_sub_laser = rospy.Subscriber(self._laser_topic, LaserScan, self.laser_cb, queue_size=1)
        self.pub_CMD = rospy.Publisher(self._cmd_topic, Twist, queue_size=1)
        self.rate = rospy.Rate(5)

    def laser_cb(self, msg):
        self.laser_msg = msg
    
    def calculate_command(self):
        if hasattr(self.laser_msg, 'ranges') and len(self.laser_msg.ranges) > 0:
            self.obstacle_distance = min(self.laser_msg.ranges)

            if self.obstacle_distance > 1.0:
                self.cmd_vel.linear.x = 2.0
                self.cmd_vel.angular.z = 0.0
            else:
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = -2.0

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            #print(self.cmd_vel)
            self.pub_CMD.publish(self.cmd_vel)
            self.rate.sleep()

if __name__ == "_main_":
    rospy.init_node("reactive_controller_py")
    controller = ReactiveNavigation()
    controller.run()