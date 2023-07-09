#!/usr/bin/env python3

###########################################################
# @file  : spawn.py
# @author: Laila Ebrahim Tawfik
# @brief : ROS2 package that spawn turtle at ramdom poses and move defult turtle to this pose to kill the second turtle
###########################################################

##################### import section start #######################
import rclpy
import random 
from time          import sleep
from math          import pi
from rclpy.node    import Node
from functools     import partial
from turtlesim.srv import Kill
from turtlesim.srv import Spawn
from std_srvs.srv  import Empty
from turtlesim.msg import Pose
##################### import section start #######################

##################### class definition section start #######################


class Spawner(Node):
    
    def __init__(self):
        super().__init__("spawner")
        self.x    = 0
        self.y    = 0
        self.name = "turtle2"
        self.spawn_turtle()
        
        self.chaser_position_subscriber = self.create_subscription(Pose, "turtle1/pose", self.kill_clear_reSpawn , 10)
        
    def kill_clear_reSpawn(self, chaser_position):
        if abs(chaser_position.x - self.x) < 0.1 and abs(chaser_position.y - self.y) < 0.1:
            
            self.kill_turtle()

            clear_path = self.create_client(Empty,"clear")
            clear_screen = Empty.Request()
            clear_path.call_async(clear_screen)

            sleep(0.1)

            self.spawn_turtle()

        else: 
            pass

    
    def kill_turtle(self):

        
        turtle_killer_client = self.create_client(Kill, "kill")
        turtle_to_kill = Kill.Request()
        turtle_to_kill.name = self.name
        
        fut_obj= turtle_killer_client.call_async(turtle_to_kill)
        fut_obj.add_done_callback(self.rsp_killed_turtle)

        
    def rsp_killed_turtle(self,fut_obj):
        
            self.get_logger().info("Done chasing!")

    
    def spawn_turtle(self):

        spawner = self.create_client(Spawn, "spawn")

        while spawner.wait_for_service(1.0) == False :
            self.get_logger().info("Waiting ..")
            
        turtle_to_spawn       = Spawn.Request()
        turtle_to_spawn.x     = random.uniform(0.0, 10.0)
        turtle_to_spawn.y     = random.uniform(0.0, 10.0)
        turtle_to_spawn.theta = random.uniform(-pi, pi)
        turtle_to_spawn.name  = self.name
        
        self.x = turtle_to_spawn.x
        self.y = turtle_to_spawn.y
        
        fut_obj = spawner.call_async(turtle_to_spawn)
        fut_obj.add_done_callback(self.rsp_callback)

    def rsp_callback(self, rsp):
            response = rsp.result()
            self.get_logger().info(f"Spawned {response.name}")
        


##################### class definition section end #######################


##################### main definition section start #######################

def main(args=None):
    rclpy.init(args=args)
    spawner = Spawner()
    rclpy.spin(spawner)
    rclpy.shutdown()

##################### main definition section end #########################


if __name__ == '__main__':
    main()
