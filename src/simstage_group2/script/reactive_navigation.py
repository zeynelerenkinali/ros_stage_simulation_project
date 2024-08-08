#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan   

class ReactiveNavigation():
    def __init__(self):

        self.cmd_vel = Twist()
        self.laser_msg = LaserScan()
        self.obstacle_distance = 100
        self.obstacle_threshold = 0.28
        self.turning = False
        self.turn_direction = 1

        # Params from config file
        # self.active_robot = str(rospy.get_param("reactive_controller_py/active_robot"))
        
        # Topics
        self._cmd_topic = "cmd_vel"
        self._laser_topic = "base_scan"

        self.rospy_sub_laser = rospy.Subscriber(self._laser_topic, LaserScan, self.laser_cb, queue_size=1)
        self.pub_CMD = rospy.Publisher(self._cmd_topic, Twist, queue_size=1)
        self.rate = rospy.Rate(5)

    def laser_cb(self, msg):
        self.laser_msg = msg
    
    def calculate_command(self):
        if type(self.laser_msg.ranges) == tuple:
            self.obstacle_distance = min(self.laser_msg.ranges)

            if self.obstacle_distance > self.obstacle_threshold:
                if self.turning:
                    # Finish turning before moving forward
                    self.cmd_vel.linear.x = 0.0
                    self.cmd_vel.angular.z = 2.0 * self.turn_direction
                    if min(self.laser_msg.ranges) > self.obstacle_threshold:
                        self.turning = False
                        self.cmd_vel.angular.z = 0.0
                else:
                    # Move forward
                    self.cmd_vel.linear.x = 2.0
                    self.cmd_vel.angular.z = 0.0
            else:
                # Stop and turn
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = 2.0 * self.turn_direction
                self.turning = True
                # Change direction if stuck turning
                if self.obstacle_distance < self.obstacle_threshold / 2:
                    self.turn_direction *= -1
                
            # self.pub_CMD.publish(self.cmd_vel)

                

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("reactive_controller_py")

    controller = ReactiveNavigation()
    controller.run()