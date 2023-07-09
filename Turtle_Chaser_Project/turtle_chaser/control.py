#!/usr/bin/env python3

###########################################################
# @file  : control.py
# @author: Laila Ebrahim Tawfik
# @brief : ROS2 package that spawn turtle at ramdom poses and move defult turtle to this poseto kill the second turtle
###########################################################

##################### import section start ############################
import rclpy
import math
from rclpy.node        import Node
from turtlesim.msg     import Pose
from geometry_msgs.msg import Twist
##################### import section start #############################

##################### class definition section start #######################

class control_node(Node):
    
    def __init__(self):
        super().__init__("controller")
        self.chaser_x     = 0
        self.chaser_y     = 0
        self.chaser_angle = 0
        self.spawned_x    = 0
        self.spawned_y    = 0
        
        self.spawned_position_sub = self.create_subscription(Pose, "turtle2/pose", self.get_spawned_position, 10)        
        self.chaser_position_sub  = self.create_subscription(Pose, "turtle1/pose", self.get_chaser_position, 10)        
        self.velocity_pub         = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        
    def get_spawned_position(self, position):
        self.spawned_x = position.x
        self.spawned_y = position.y
        
    def get_chaser_position(self, position):
        self.chaser_x     = position.x
        self.chaser_y     = position.y
        self.chaser_angle = position.theta
        
        self.chasing()
        
    def chasing(self):

        velocity = Twist()
        
        displacement = math.sqrt(((self.spawned_x-self.chaser_x)**2) + ((self.spawned_y-self.chaser_y)**2))
        desired_angle = math.atan2(self.spawned_y-self.chaser_y, self.spawned_x-self.chaser_x)
        
        erro_theta = desired_angle-self.chaser_angle

        if erro_theta > math.pi:
            erro_theta -= 2*math.pi
        elif erro_theta < -1*math.pi:
            erro_theta += 2*math.pi
            
        velocity.angular.z = 6.0 * (erro_theta)
        velocity.linear.x  = 2.0 * displacement
        
        self.velocity_pub.publish(velocity)

##################### main definition section start ################################

def main(args=None):
    rclpy.init(args=args)
    controller = control_node()
    rclpy.spin(controller)
    rclpy.shutdown()
##################### main definition section end  ################################

if __name__ == '__main__':
    main()

