"""
Start MoveIt for G1 (move_group + RViz MotionPlanning).
Start Gazebo first:
  ros2 launch g1_gazebo g1_lying.launch.py gui:=true
Then:
  ros2 launch g1_moveit_config moveit.launch.py
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    share = get_package_share_directory('g1_moveit_config')
    urdf = os.path.join(get_package_share_directory('g1_gazebo'), 'urdf', 'g1_29dof_gazebo.urdf')
    mc = (MoveItConfigsBuilder('g1_29dof_rev_1_0', package_name='g1_moveit_config')
          .robot_description(file_path=urdf)
          .robot_description_semantic(file_path='config/g1.srdf')
          .robot_description_kinematics(file_path='config/kinematics.yaml')
          .joint_limits(file_path='config/joint_limits.yaml')
          .trajectory_execution(file_path='config/moveit_controllers.yaml')
          .planning_pipelines(pipelines=['ompl'])
          .to_moveit_configs())
    sim = {'use_sim_time': True}
    spawn = LaunchConfiguration('spawn_controllers')
    return LaunchDescription([
        DeclareLaunchArgument('spawn_controllers', default_value='true',
                              description='activate joint_state_broadcaster and g1_controller'),
        Node(package='controller_manager', executable='spawner', condition=IfCondition(spawn),
             arguments=['joint_state_broadcaster', 'g1_controller']),
        Node(package='g1_moveit_config', executable='odom_tf.py', parameters=[sim]),
        Node(package='moveit_ros_move_group', executable='move_group', output='screen',
             parameters=[mc.to_dict(), sim]),
        Node(package='rviz2', executable='rviz2', output='log',
             arguments=['-d', os.path.join(share, 'config', 'moveit.rviz')],
             parameters=[mc.robot_description, mc.robot_description_semantic,
                         mc.robot_description_kinematics, mc.planning_pipelines, mc.joint_limits, sim]),
    ])
