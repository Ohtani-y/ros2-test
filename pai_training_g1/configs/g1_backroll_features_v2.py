"""
G1 後転の特徴量の定義（第 2 版）。g1_backroll_features.py から変えたところ:
  - observation.environment_state を「関節角の複製（29）」から「骨盤の姿勢と回転の速さ（7）」に変えた
    （お手本は体の回転角で次の動きに進むので、関節角だけでは「いつ進むか」が分からない）
  - そのために ENV_TOPIC と env_from_odom() を足した（変換と推論で同じ関数を使う）
"""

# URDF に出てくる順（コントローラ・データ・方策で共通の並び）
JOINT_NAMES = [
    'left_hip_pitch_joint', 'left_hip_roll_joint', 'left_hip_yaw_joint', 'left_knee_joint',
    'left_ankle_pitch_joint', 'left_ankle_roll_joint',
    'right_hip_pitch_joint', 'right_hip_roll_joint', 'right_hip_yaw_joint', 'right_knee_joint',
    'right_ankle_pitch_joint', 'right_ankle_roll_joint',
    'waist_yaw_joint', 'waist_roll_joint', 'waist_pitch_joint',
    'left_shoulder_pitch_joint', 'left_shoulder_roll_joint', 'left_shoulder_yaw_joint', 'left_elbow_joint',
    'left_wrist_roll_joint', 'left_wrist_pitch_joint', 'left_wrist_yaw_joint',
    'right_shoulder_pitch_joint', 'right_shoulder_roll_joint', 'right_shoulder_yaw_joint', 'right_elbow_joint',
    'right_wrist_roll_joint', 'right_wrist_pitch_joint', 'right_wrist_yaw_joint',
]
STATE_TOPIC = '/joint_states'
ACTION_TOPIC = '/roll/action'
ENV_TOPIC = '/model/g1/odometry'   # 追加: 骨盤の回転センサ（OdometryPublisher）
ENV_NAMES = ['grav_x', 'grav_y', 'grav_z', 'ang_vel_x', 'ang_vel_y', 'ang_vel_z', 'pelvis_z']


def env_from_odom(qx, qy, qz, qw, wx, wy, wz, pz):
    """追加: 骨盤から見た重力の向き（向きの回り方に左右されない 3 つ）＋角速度 3 つ＋骨盤の高さ。"""
    gx = -2 * (qx * qz - qw * qy)
    gy = -2 * (qy * qz + qw * qx)
    gz = -(1 - 2 * (qx * qx + qy * qy))
    return [gx, gy, gz, wx, wy, wz, pz]


features = {
    'observation.state': {'dtype': 'float32', 'shape': (29,), 'names': JOINT_NAMES},
    'observation.environment_state': {'dtype': 'float32', 'shape': (7,), 'names': ENV_NAMES},   # 変更: 29 → 7
    'action': {'dtype': 'float32', 'shape': (29,), 'names': JOINT_NAMES},
}
# Below is used for the dataset conversion
robot_type = 'unitree_g1_29dof'
repo_id = 'g1_backroll'
root = '/home/ubuntu/dataset/lerobot_dataset/g1_backroll_v2'
task_name = 'backroll'
