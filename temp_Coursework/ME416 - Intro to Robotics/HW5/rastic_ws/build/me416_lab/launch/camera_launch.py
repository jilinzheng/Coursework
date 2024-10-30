from launch_ros.actions import Node

from launch import LaunchDescription


def generate_launch_description():
    """
    Launches a camera node
    """
    ldesc = LaunchDescription()
    camera_node = Node(package="v4l2_camera",
                       executable="v4l2_camera_node",
                       name="camera0",
                       parameters=[
                           {
                               "video_device": "/dev/video1"
                           },
                           {
                               "image_size": [320, 240]
                           },
                       ])
    ldesc.add_action(camera_node)
    return ldesc
