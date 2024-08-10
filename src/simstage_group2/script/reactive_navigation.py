#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan   

class ReactiveNavigation():
    def __init__(self):
        # Initializations for robots speed, laser variables.
        self.cmd_vel = Twist()
        self.laser_msg = LaserScan()
        # Initialization for algorithm
        self.obstacle_distance = 100
        self.obstacle_threshold = 0.28
        self.turning = False
        self.turn_direction = 1
        # Topics
        self._cmd_topic = "cmd_vel"
        self._laser_topic = "base_scan"
        # Correctly subsicribe and publish data by topic(get contact with robot).
        self.rospy_sub_laser = rospy.Subscriber(self._laser_topic, LaserScan, self.laser_cb, queue_size=1)
        self.pub_CMD = rospy.Publisher(self._cmd_topic, Twist, queue_size=1)
        # Determine appropriate hertz(hz)
        self.rate = rospy.Rate(5)

    def laser_cb(self, msg):
        self.laser_msg = msg
    
    def calculate_command(self):
        if type(self.laser_msg.ranges) == tuple:
            # Get minimum distance for all directions
            self.obstacle_distance = min(self.laser_msg.ranges)
            # Get the total range from length of lidar's data
            total_ranges = len(self.obstacle_distance)
            # Divide the FOV to three and seperate indices to each direction in order to focus information correctly for each direciton.
            one_third = total_ranges // 3

            right_indices = range(0, one_third) # Implement first 80 indices as right
            front_indices = range(one_third, 2 * one_third) # Implement second 80 indices as front
            left_indices = range(2 * one_third, total_ranges) # Implement third(last) 80 indices as left

            # Save all distances to array variables, from indices that took before from all the lidar data.
            right_distances = [self.laser_msg.ranges[i] for i in right_indices]
            front_distances = [self.laser_msg.ranges[i] for i in front_indices]
            left_distances = [self.laser_msg.ranges[i] for i in left_indices]

            # Now get the minimum obstacle distance from each direction in order to detect wall
            right_obstacle_distance = min(right_distances)
            front_obstacle_distance = min(front_distances)
            left_obstacle_distance = min(left_distances)
            #------------------------
            # Algorithm to search map
            # -----------------------
            # 1.
                
            #self.pub_CMD.publish(self.cmd_vel)
            
    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("reactive_controller_py")

    controller = ReactiveNavigation()
    controller.run()