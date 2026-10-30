"""
G1（29 関節）の特徴量の定義（レッスン2・ステップ1：うつ伏せ → 四つん這い）。
講義の configs/crane_features.py と同じ書き方。G1 用に、関節の並び（JOINT_NAMES）と、
読むトピック（STATE_TOPIC・ACTION_TOPIC）を足した。
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

features = {
    'observation.state': {'dtype': 'float32', 'shape': (29,), 'names': JOINT_NAMES},
    'observation.environment_state': {'dtype': 'float32', 'shape': (29,), 'names': JOINT_NAMES},
    'action': {'dtype': 'float32', 'shape': (29,), 'names': JOINT_NAMES},
}
# Below is used for the dataset conversion
robot_type = 'unitree_g1_29dof'
repo_id = 'g1_backroll'
root = '/home/ubuntu/dataset/lerobot_dataset/g1_backroll'
task_name = 'backroll'
