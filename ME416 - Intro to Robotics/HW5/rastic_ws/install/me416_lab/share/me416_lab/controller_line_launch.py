from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    """
    Launches motor_command, image_segment, and controller_line
    """
    ldesc = LaunchDescription()
    motor_command = Node(package="me416_lab",executable="motor_command")
    image_segment = Node(package="me416_lab",executable="image_segment")
    controller_line = Node(package="me416_lab", executable="controller_line")
    ldesc.add_action(motor_command)
    ldesc.add_action(image_segment)
    ldesc.add_action(controller_line)
    return ldesc
