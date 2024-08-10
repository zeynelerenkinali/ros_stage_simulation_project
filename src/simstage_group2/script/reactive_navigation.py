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
        self.threshold = 0.8
        self.forward_speed = 1
        self.turn_speed = 1
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
            #self.obstacle_distance = min(self.laser_msg.ranges)
            # Get the total range from length of lidar's data
            total_ranges = len(self.laser_msg.ranges)
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
            # Algorithm to search map *c=close *o=open
            # -----------------------
            #1. If there is a wall on right, front is clear, left is clear: go straight -- coo
            #2. else if there is a wall on right, front, left is clear: turn left -- cco
            #3. else if there is a wall on right, front, left: turn backwards --- ccc
            #4. else if right is clear, front is clear, left is clear: turn place that has more range(r or l) -- ooo
            #6. else if right is clear, there is a wall on front, left is clear: turn the place that has bigger range of view(r or l sensor has more value) -- oco
            #7. else if right is clear, front is clear, there is a wall on left: go straight -- ooc
            #--------code-----------
            
            if right_obstacle_distance < self.threshold and front_obstacle_distance >= self.threshold and left_obstacle_distance >= self.threshold:
                # Case 1: coo - Go straight
                if right_obstacle_distance < self.threshold / 2:
                    self.cmd_vel.linear.x = self.forward_speed
                    self.cmd_vel.angular.z = -self.turn_speed
                else:
                    self.cmd_vel.linear.x = self.forward_speed
                    self.cmd_vel.angular.z = 0.0
            elif right_obstacle_distance < self.threshold and front_obstacle_distance < self.threshold and left_obstacle_distance >= self.threshold:
                # Case 2: cco - Turn left
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = self.turn_speed
            elif right_obstacle_distance < self.threshold and front_obstacle_distance < self.threshold and left_obstacle_distance < self.threshold:
                # Case 3: ccc - Turn backwards
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = self.turn_speed
            elif right_obstacle_distance >= self.threshold and front_obstacle_distance >= self.threshold and left_obstacle_distance >= self.threshold:
                self.cmd_vel.linear.x = self.forward_speed
                self.cmd_vel.angular.z = 0.0
            # elif right_obstacle_distance >= self.threshold and front_obstacle_distance < self.threshold and left_obstacle_distance >= self.threshold:
            #     # Case 5: oco - Turn to the place with more range (right or left sensor has more value)
            #     if right_obstacle_distance > left_obstacle_distance:
            #         # Turn right
            #         self.cmd_vel.linear.x = 0.0
            #         self.cmd_vel.angular.z = -self.turn_speed
            #     else:
            #         # Turn left
            #         self.cmd_vel.linear.x = 0.0
            #         self.cmd_vel.angular.z = self.turn_speed
            elif right_obstacle_distance >= self.threshold and front_obstacle_distance >= self.threshold and left_obstacle_distance < self.threshold:
                # Case 6: ooc - Go straight
                if left_obstacle_distance < self.threshold / 2:
                    self.cmd_vel.linear.x = self.forward_speed
                    self.cmd_vel.angular.z = self.turn_speed
                else:
                    self.cmd_vel.linear.x = self.forward_speed
                    self.cmd_vel.angular.z = 0.0

            # Publish the command
            self.pub_CMD.publish(self.cmd_vel)
            
    def run(self):
        while not rospy.is_shutdown():
            self.calculate_command()
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node("reactive_controller_py")

    controller = ReactiveNavigation()
    controller.run()