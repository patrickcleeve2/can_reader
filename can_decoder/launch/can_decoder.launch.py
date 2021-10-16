from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='can_decoder',
            executable='can_reader',
            name='can_reader',
            output={'both':'log'},
        ),
        Node(
            package='can_decoder',
            executable='can_publisher',
            name='can_publisher', 
            output={'both': 'log'}
        ),
        Node(
            package='can_decoder',
            executable='can_decoder_ui',
            name='can_decoder_ui', 
            output={'both': 'log'}
        ),

    ])