from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'robot_ip',
            default_value='192.168.1.1',
            description='IP address of the Robot Kit (Compute Box)'
        ),
        
        DeclareLaunchArgument(
            'robot_port',
            default_value='502',
            description='Modbus TCP port'
        ),
        
        DeclareLaunchArgument(
            'device_address',
            default_value='65',
            description='Modbus device address (65 for Quick Changer, 66/67 for Dual QC)'
        ),
        
        DeclareLaunchArgument(
            'gripper_type',
            default_value='rg2',
            description='Gripper type: rg2, rg6, vg10, vgc10, vgp20, vgp30, mg10, 2fg7, 2fg14, 3fg15, 3fg25'
        ),
        
        DeclareLaunchArgument(
            'update_rate',
            default_value='50',
            description='Status update rate in Hz'
        ),
        
        Node(
            package='onrobot_driver',
            executable='onrobot_node',
            name='onrobot_node',
            output='screen',
            parameters=[{
                'robot_ip': LaunchConfiguration('robot_ip'),
                'robot_port': LaunchConfiguration('robot_port'),
                'device_address': LaunchConfiguration('device_address'),
                'gripper_type': LaunchConfiguration('gripper_type'),
                'update_rate': LaunchConfiguration('update_rate'),
            }]
        )
    ])
