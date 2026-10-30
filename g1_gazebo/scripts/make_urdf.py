#!/usr/bin/env python3
"""
公式の G1 URDF（unitree_ros の g1_29dof_rev_1_0.urdf）から、Gazebo Fortress で動かす URDF を作る。

  1. メッシュの参照を package://g1_gazebo/meshes/ に書き換え、使う STL だけを写す
  2. 29 関節すべてに ros2_control を付ける（ign_ros2_control/IgnitionSystem、
     指令 = position、状態 = position・velocity）
  3. Gazebo プラグイン ign_ros2_control を付ける（設定ファイルの場所は、起動時に差し替える）
  4. 骨盤の位置と姿勢を出すプラグイン（OdometryPublisher、50 Hz、3 次元）を付ける

公式の URDF は書き換えない。

使い方:
  python3 make_urdf.py <公式 URDF> <公式 meshes フォルダ> <g1_gazebo パッケージのフォルダ>
"""

import copy
import shutil
import struct
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

CONTROLLERS_PLACEHOLDER = '@CONTROLLERS_YAML@'  # 起動時に g1_controllers.yaml の絶対パスへ差し替える


def stl_bbox(path):
    """Bounding box (min xyz, max xyz) of a binary STL."""
    data = path.read_bytes()
    n = struct.unpack_from('<I', data, 80)[0]
    lo, hi = [1e9] * 3, [-1e9] * 3
    for i in range(n):
        v = struct.unpack_from('<12f', data, 84 + 50 * i)[3:]
        for k in range(9):
            lo[k % 3] = min(lo[k % 3], v[k])
            hi[k % 3] = max(hi[k % 3], v[k])
    return lo, hi


def main():
    src_urdf, src_meshes, pkg = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
    tree = ET.parse(src_urdf)  # コメント（MuJoCo 用の浮遊関節）は読み込まれない
    robot = tree.getroot()

    # MuJoCo 用の設定は Gazebo では使わない
    for tag in robot.findall('mujoco'):
        robot.remove(tag)

    # 1.5 official URDF has no collision on the rubber hands: copy the visual mesh as collision (keeps hands out of the floor)
    for link in robot.findall('link'):
        if link.get('name', '').endswith('rubber_hand') and link.find('collision') is None:
            # mesh collision is very slow in Gazebo, so use the mesh bounding box
            mesh = link.find('visual/geometry/mesh').get('filename')
            lo, hi = stl_bbox(src_meshes / Path(mesh).name)
            col = ET.SubElement(link, 'collision')
            ET.SubElement(col, 'origin', {'xyz': ' '.join(f'{(a + b) / 2:.4f}' for a, b in zip(lo, hi)), 'rpy': '0 0 0'})
            geo = ET.SubElement(col, 'geometry')
            ET.SubElement(geo, 'box', {'size': ' '.join(f'{b - a:.4f}' for a, b in zip(lo, hi))})

    # 1.6 extra weights: fixed links (Gazebo merges them into the parent link)
    #     (link name, parent link, position in the parent frame, kg, box size a x b x c in m)
    WEIGHTS = [
        ('right_shoe_weight', 'right_ankle_roll_link', '0.035 0 -0.01', 2.0, (0.16, 0.07, 0.02)),
        ('left_shoe_weight', 'left_ankle_roll_link', '0.035 0 -0.01', 2.0, (0.16, 0.07, 0.02)),
        ('head_weight', 'head_link', '0.005 0 0.43', 2.0, (0.10, 0.12, 0.15)),
    ]
    for wname, parent, xyz, kg, (a, b, c) in WEIGHTS:
        if kg <= 0 or robot.find(f"link[@name='{parent}']") is None:
            print('weight skipped:', wname)
            continue
        w = ET.SubElement(robot, 'link', {'name': wname})
        ine = ET.SubElement(w, 'inertial')
        ET.SubElement(ine, 'origin', {'xyz': '0 0 0', 'rpy': '0 0 0'})
        ET.SubElement(ine, 'mass', {'value': f'{kg}'})
        ET.SubElement(ine, 'inertia', {'ixx': f'{kg * (b * b + c * c) / 12:.6f}', 'ixy': '0', 'ixz': '0',
                                       'iyy': f'{kg * (a * a + c * c) / 12:.6f}', 'iyz': '0',
                                       'izz': f'{kg * (a * a + b * b) / 12:.6f}'})
        j = ET.SubElement(robot, 'joint', {'name': wname + '_joint', 'type': 'fixed'})
        ET.SubElement(j, 'origin', {'xyz': xyz, 'rpy': '0 0 0'})
        ET.SubElement(j, 'parent', {'link': parent})
        ET.SubElement(j, 'child', {'link': wname})

    # 1. メッシュ
    out_meshes = pkg / 'meshes'
    out_meshes.mkdir(parents=True, exist_ok=True)
    used = set()
    for mesh in robot.iter('mesh'):
        name = Path(mesh.get('filename')).name  # 例: meshes/pelvis.STL -> pelvis.STL
        used.add(name)
        mesh.set('filename', f'package://g1_gazebo/meshes/{name}')
    for name in sorted(used):
        # Windows のフォルダをマウントした /workspace では属性（時刻）を写せないので中身だけ写す
        shutil.copyfile(src_meshes / name, out_meshes / name)

    # 2. ros2_control（ファイルに出てくる順 = データの関節の並び）
    joints = [j.get('name') for j in robot.findall('joint') if j.get('type') == 'revolute']
    rc = ET.SubElement(robot, 'ros2_control', {'name': 'IgnitionSystem', 'type': 'system'})
    hardware = ET.SubElement(rc, 'hardware')
    ET.SubElement(hardware, 'plugin').text = 'ign_ros2_control/IgnitionSystem'
    for name in joints:
        joint = ET.SubElement(rc, 'joint', {'name': name})
        ET.SubElement(joint, 'command_interface', {'name': 'position'})
        state = ET.SubElement(joint, 'state_interface', {'name': 'position'})
        ET.SubElement(state, 'param', {'name': 'initial_value'}).text = '0.0'
        ET.SubElement(joint, 'state_interface', {'name': 'velocity'})

    # 3. ign_ros2_control
    gz_control = ET.SubElement(robot, 'gazebo')
    plugin = ET.SubElement(gz_control, 'plugin', {
        'filename': 'ign_ros2_control-system',
        'name': 'ign_ros2_control::IgnitionROS2ControlPlugin',
    })
    ET.SubElement(plugin, 'parameters').text = CONTROLLERS_PLACEHOLDER

    # 4. 骨盤のオドメトリ（/model/g1/odometry）
    gz_odom = ET.SubElement(robot, 'gazebo')
    odom = ET.SubElement(gz_odom, 'plugin', {
        'filename': 'ignition-gazebo-odometry-publisher-system',
        'name': 'ignition::gazebo::systems::OdometryPublisher',
    })
    for key, value in [('odom_frame', 'odom'), ('robot_base_frame', 'pelvis'),
                       ('odom_publish_frequency', '50'), ('dimensions', '3')]:
        ET.SubElement(odom, key).text = value

    out_urdf = pkg / 'urdf' / 'g1_29dof_gazebo.urdf'
    out_urdf.parent.mkdir(parents=True, exist_ok=True)
    ET.indent(tree)
    tree.write(out_urdf, encoding='utf-8', xml_declaration=True)
    print(f'joints={len(joints)} meshes={len(used)} -> {out_urdf}')


if __name__ == '__main__':
    main()
