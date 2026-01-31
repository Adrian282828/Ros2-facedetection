from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. The USB Camera Node 
        Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            parameters=[
                {'video_device': '/dev/video0'},
                {'pixel_format': 'mjpeg2rgb'},  
                {'image_width': 640},
                {'image_height': 480}
            ]
        ),

        # 2. Face Detection Node
        Node(
            package='my_python_package',       
            executable='my_node',  
            name='face_detect_node',
            remappings=[
                ('/image_raw', '/image_raw'), # Ensuring the topics match
            ]
        ),

        # 3. RQT Image View for Visualization
        Node(
            package='rqt_image_view',
            executable='rqt_image_view',
            name='visualizer'
        )
    ])