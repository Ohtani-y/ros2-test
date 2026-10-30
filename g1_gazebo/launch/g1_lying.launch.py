"""
Gazebo（Fortress）を起動し、G1 を寝た姿勢で出現させる。

  仰向け（既定）: ros2 launch g1_gazebo g1_lying.launch.py
  うつ伏せ      : ros2 launch g1_gazebo g1_lying.launch.py pitch:=1.5708 z:=0.35
  画面付き      : gui:=true を足す（既定は画面なし。データ収集を速くするため）

コントローラの有効化は、ここでは行わない（g1_standup/sim.py が 1 つずつ有効化する）。
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context):
    share = get_package_share_directory('g1_gazebo')
    with open(os.path.join(share, 'urdf', 'g1_29dof_gazebo.urdf'), encoding='utf-8') as f:
        urdf = f.read().replace('@CONTROLLERS_YAML@', os.path.join(share, 'config', 'g1_controllers.yaml'))

    world = os.path.join(share, 'worlds', 'ground.sdf')
    gui = LaunchConfiguration('gui').perform(context) == 'true'
    # 画面付きのときは、窓の大きさとカメラの位置を gui.config で決める
    gz_args = f"-r {world} --gui-config {os.path.join(share, 'gui', 'gui.config')}" if gui else f'-r -s {world}'

    # package://g1_gazebo/meshes/... を Gazebo が見つけられるようにする
    resource_path = os.pathsep.join(filter(None, [os.environ.get('IGN_GAZEBO_RESOURCE_PATH'), os.path.dirname(share)]))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': gz_args}.items(),
    )
    robot_state_publisher = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'robot_description': urdf, 'use_sim_time': True}],
    )
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'g1',
                   '-x', '0', '-y', '0', '-z', LaunchConfiguration('z').perform(context),
                   '-R', '0', '-P', LaunchConfiguration('pitch').perform(context), '-Y', '0'],
    )
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                   '/model/g1/odometry@nav_msgs/msg/Odometry[ignition.msgs.Odometry'],
    )
    return [SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', resource_path),
            gazebo, robot_state_publisher, spawn, bridge]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pitch', default_value='-1.5708', description='体の前後の傾き（rad）。−90° で仰向け'),
        DeclareLaunchArgument('z', default_value='0.12', description='出現させる高さ（m）'),
        DeclareLaunchArgument('gui', default_value='false', description='true で Gazebo の画面を出す'),
        OpaqueFunction(function=launch_setup),
    ])
