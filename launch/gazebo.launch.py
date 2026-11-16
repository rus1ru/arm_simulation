from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    share = Path(get_package_share_directory('robo2'))
    gz_share = Path(get_package_share_directory('ros_gz_sim'))
    description = xacro.process_file(str(share / 'urdf/robo2_sim.urdf.xacro')).toxml()
    spawn = Node(package='ros_gz_sim', executable='create', arguments=[
        '-name', 'robo2', '-topic', 'robot_description', '-z', '0.002'], output='screen')
    controllers = [Node(package='controller_manager', executable='spawner', arguments=[
        name, '--controller-manager', '/controller_manager'], output='screen')
        for name in ['joint_state_broadcaster', 'arm_controller', 'gripper_controller']]
    return LaunchDescription([
        DeclareLaunchArgument('gz_args', default_value=f'-r "{share / "worlds/pick_place.sdf"}"'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', str(share.parent)),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(gz_share / 'launch/gz_sim.launch.py')),
                                 launch_arguments={'gz_args': LaunchConfiguration('gz_args')}.items()),
        Node(package='robot_state_publisher', executable='robot_state_publisher',
             parameters=[{'robot_description': description, 'use_sim_time': True}], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                        '/model/box_b/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/gripper/state@std_msgs/msg/String[gz.msgs.StringMsg',
                        '/gripper/attach@std_msgs/msg/Empty]gz.msgs.Empty',
                        '/gripper/detach@std_msgs/msg/Empty]gz.msgs.Empty'], output='screen'),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=controllers)),
        spawn,
    ])
