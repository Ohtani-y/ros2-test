#!/bin/bash
# G1 の公式モデルを取得し、Gazebo 用の URDF を作って g1_gazebo をビルドする（コンテナの中、ubuntu で実行）
set -e
source /opt/ros/humble/setup.bash
mkdir -p ~/g1_ws/src
cd ~/g1_ws/src
if [ ! -d unitree_ros ]; then
  # 履歴は取るが、ファイルの中身は必要なものだけ取る（robots/g1_description だけ）
  git clone --filter=blob:none --sparse https://github.com/unitreerobotics/unitree_ros.git
  cd unitree_ros
  git sparse-checkout set robots/g1_description
  git checkout -q 5994d4f   # レッスン1 と同じ版
  cd ..
fi
cd ~/g1_ws/src/unitree_ros
git log -1 --format='unitree_ros: %h (%cd)' --date=short
G1=~/g1_ws/src/unitree_ros/robots/g1_description
python3 /workspace/g1_gazebo/scripts/make_urdf.py "$G1/g1_29dof_rev_1_0.urdf" "$G1/meshes" /workspace/g1_gazebo
cd ~/g1_ws
colcon build --base-paths /workspace/g1_gazebo --build-base build --install-base install 2>&1 | tail -4
source ~/g1_ws/install/setup.bash
echo "g1_gazebo: $(ros2 pkg prefix g1_gazebo)"
