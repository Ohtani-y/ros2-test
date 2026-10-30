import os, sys, yaml, xml.etree.ElementTree as ET
H = os.path.expanduser('~')
PKG = '/workspace/g1_moveit_config'
# True: MoveIt ignores collisions between the robot's own links (plans even if the pose has interference).
# Gazebo does not simulate self-collision for this model either (no <self_collide>). False: check them again.
IGNORE_SELF_COLLISION = True
URDF = H + '/g1_ws/install/g1_gazebo/share/g1_gazebo/urdf/g1_29dof_gazebo.urdf'
CTRL = H + '/g1_ws/install/g1_gazebo/share/g1_gazebo/config/g1_controllers.yaml'
if len(sys.argv) > 1:
    PKG, URDF, CTRL = sys.argv[1:4]
for d in ('config', 'launch', 'scripts'):
    os.makedirs(os.path.join(PKG, d), exist_ok=True)
def w(p, s):
    if p == 'config/moveit.rviz' and os.path.exists(os.path.join(PKG, p)):
        return  # keep the layout saved from RViz (File > Save Config)
    open(os.path.join(PKG, p), 'w', encoding='utf-8').write(s)

r = ET.parse(URDF).getroot()
robot = r.get('name')
J = r.findall('joint')
rev = [j for j in J if j.get('type') == 'revolute']
par = {j.find('child').get('link'): (j.find('parent').get('link'), j.get('type')) for j in J}
links = [l.get('name') for l in r.findall('link') if l.find('collision') is not None]
def anc(l):
    out, d = [(l, 0)], 0
    while l in par:
        p, t = par[l]
        d += 0 if t == 'fixed' else 1
        l = p
        out.append((l, d))
    return out
def dist(a, b):
    A = dict(anc(a))
    for l, d in anc(b):
        if l in A:
            return A[l] + d
    return 99
ctrl = yaml.safe_load(open(CTRL))
cj = ctrl['g1_controller']['ros__parameters']['joints']

# SRDF
s = ['<?xml version="1.0"?>', f'<robot name="{robot}">', '  <group name="whole_body">']
s += [f'    <joint name="{j.get("name")}"/>' for j in rev]
s += ['  </group>', '  <group_state name="zero" group="whole_body">']
s += [f'    <joint name="{j.get("name")}" value="0"/>' for j in rev]
# extra named states taken from scripts/set_pose.py (POSES)
import ast
POSES = {}
for nd in ast.parse(open(os.path.join(PKG, 'scripts', 'set_pose.py')).read()).body:
    if isinstance(nd, ast.Assign) and getattr(nd.targets[0], 'id', '') == 'POSES':
        POSES = ast.literal_eval(nd.value)
s += ['  </group_state>']
for pname, pose in POSES.items():
    if pname == 'zero':
        continue
    s += [f'  <group_state name="{pname}" group="whole_body">']
    s += [f'    <joint name="{j.get("name")}" value="{pose.get(j.get("name"), 0.0)}"/>' for j in rev]
    s += ['  </group_state>']
s += ['  <virtual_joint name="virtual_joint" type="floating" parent_frame="odom" child_link="pelvis"/>']
n = 0
for i, a in enumerate(links):
    for b in links[i + 1:]:
        d = dist(a, b)
        if d <= 2 or IGNORE_SELF_COLLISION:
            reason = 'Adjacent' if d <= 1 else ('Default' if d <= 2 else 'User')
            s.append(f'  <disable_collisions link1="{a}" link2="{b}" reason="{reason}"/>')
            n += 1
s.append('</robot>')
w('config/g1.srdf', '\n'.join(s) + '\n')

lim = {'default_velocity_scaling_factor': 0.3, 'default_acceleration_scaling_factor': 0.3, 'joint_limits': {}}
for j in rev:
    v = float(j.find('limit').get('velocity', 1.0))
    lim['joint_limits'][j.get('name')] = {'has_velocity_limits': True, 'max_velocity': v,
                                          'has_acceleration_limits': True, 'max_acceleration': 5.0}
w('config/joint_limits.yaml', yaml.safe_dump(lim, sort_keys=False))
w('config/kinematics.yaml', '{}\n')
w('config/moveit_controllers.yaml', yaml.safe_dump({
    'moveit_controller_manager': 'moveit_simple_controller_manager/MoveItSimpleControllerManager',
    'moveit_simple_controller_manager': {'controller_names': ['g1_controller'], 'g1_controller': {
        'type': 'FollowJointTrajectory', 'action_ns': 'follow_joint_trajectory', 'default': True, 'joints': cj}},
    'trajectory_execution': {'allowed_execution_duration_scaling': 2.0, 'allowed_start_tolerance': 0.05},
}, sort_keys=False))
w('config/ompl_planning.yaml', '''planning_plugin: ompl_interface/OMPLPlanner
request_adapters: >-
  default_planner_request_adapters/AddTimeOptimalParameterization
  default_planner_request_adapters/FixWorkspaceBounds
  default_planner_request_adapters/FixStartStateBounds
  default_planner_request_adapters/FixStartStateCollision
  default_planner_request_adapters/FixStartStatePathConstraints
start_state_max_bounds_error: 0.1
planner_configs:
  RRTConnectkConfigDefault:
    type: geometric::RRTConnect
    range: 0.0
whole_body:
  planner_configs:
    - RRTConnectkConfigDefault
''')
w('config/moveit.rviz', '''Panels:
  - Class: rviz_common/Displays
    Name: Displays
Visualization Manager:
  Class: ""
  Displays:
    - Class: rviz_default_plugins/Grid
      Name: Grid
      Enabled: true
    - Class: moveit_rviz_plugin/MotionPlanning
      Name: MotionPlanning
      Enabled: true
      Robot Description: robot_description
      Move Group Namespace: ""
      Planning Request:
        Planning Group: whole_body
        Query Goal State: true
        Query Start State: false
      Planned Path:
        Loop Animation: false
        Show Trail: false
  Global Options:
    Fixed Frame: odom
  Tools:
    - Class: rviz_default_plugins/MoveCamera
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 3
''')
w('scripts/odom_tf.py', '''#!/usr/bin/env python3
"""Relay Gazebo /model/g1/odometry to TF (odom -> pelvis)."""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTF(Node):
    def __init__(self):
        super().__init__('odom_tf')
        self.br = TransformBroadcaster(self)
        self.create_subscription(Odometry, '/model/g1/odometry', self.cb, 10)

    def cb(self, m):
        t = TransformStamped()
        t.header = m.header
        t.child_frame_id = m.child_frame_id
        p = m.pose.pose.position
        t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = p.x, p.y, p.z
        t.transform.rotation = m.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    rclpy.spin(OdomTF())


if __name__ == '__main__':
    main()
''')
os.chmod(os.path.join(PKG, 'scripts/odom_tf.py'), 0o755)
w('launch/moveit.launch.py', '''"""
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
    mc = (MoveItConfigsBuilder('%(robot)s', package_name='g1_moveit_config')
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
''' % {'robot': robot})
w('CMakeLists.txt', '''cmake_minimum_required(VERSION 3.8)
project(g1_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/odom_tf.py DESTINATION lib/${PROJECT_NAME})
ament_package()
''')
w('package.xml', '''<?xml version="1.0"?>
<package format="3">
  <name>g1_moveit_config</name>
  <version>0.1.0</version>
  <description>MoveIt config for G1 (whole_body group)</description>
  <maintainer email="user@example.com">OH</maintainer>
  <license>Apache-2.0</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>g1_gazebo</exec_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>moveit_configs_utils</exec_depend>
  <exec_depend>moveit_planners_ompl</exec_depend>
  <exec_depend>moveit_simple_controller_manager</exec_depend>
  <exec_depend>moveit_ros_visualization</exec_depend>
  <exec_depend>controller_manager</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <exec_depend>rclpy</exec_depend>
  <exec_depend>tf2_ros</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
''')
print('robot', robot, 'joints', len(rev), 'ctrl_joints', len(cj), 'disabled_pairs', n, 'collision_links', len(links))
