from launch_ros.actions import Node
from launch import LaunchDescription

def generate_launch_description():
    ld = LaunchDescription()
    
    turtlesim_node = Node(
        package='turtlesim',
        executable='turtlesim_node'
    )
    
    spawner= Node(
        package='turtle_chaser',
        executable='spawn'
    )
    
    controller = Node(
        package='turtle_chaser',
        executable='control'
    )
    
    ld.add_action(turtlesim_node)
    ld.add_action(spawner)
    ld.add_action(controller)   
    return ld
