from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='hybrid_a_star_ros2',
            executable='hybrid_a_star_node',
            output='screen'
        )
    ])


#三方库
sudo apt install ros-${ROS_DISTRO}-rclcpp \
                 ros-${ROS_DISTRO}-nav-msgs \
                 ros-${ROS_DISTRO}-geometry-msgs \
                 ros-${ROS_DISTRO}-visualization-msgs \
                 ros-${ROS_DISTRO}-tf2 \
                 ros-${ROS_DISTRO}-tf2-geometry-msgs \
                 ros-${ROS_DISTRO}-rviz2
                 
sudo apt install libeigen3-dev

