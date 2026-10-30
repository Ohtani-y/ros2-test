#!/bin/bash
# 第 2 版: environment_state に骨盤の姿勢と角速度（7）、action は 0.02 秒刻み × 50（1 秒先まで）で学び直す -> デプロイ
# コンテナの中、ubuntu で: bash /workspace/tools/after_v2.sh [epochs] [stop_after] > ~/after_v2.log 2>&1
source /opt/ros/humble/setup.bash; source ~/g1_ws/install/setup.bash
export PATH=$HOME/.pixi/bin:$PATH
EP=${1:-6}; OUT=/workspace/checkpoints/g1_backroll_act_v2
STOP=${2:+--stop-after $2}
[ -n "$2" ] && OUT=/tmp/ckpt_v2_test
cd ~/pai_training
echo "== train (GPU, $EP epochs) $STOP"
pixi run python /workspace/pai_training_g1/scripts/train_g1.py \
  --dataset-root ~/dataset/lerobot_dataset/g1_backroll_v2 --repo-id g1_backroll \
  --output-dir $OUT --epochs $EP --chunk-size 50 --action-dt 0.02 --save-every 5000 --device cuda $STOP \
  > /workspace/checkpoints/train_g1_backroll_v2.log 2>&1
grep -E "input_features|frames=|View run|saved|Error" /workspace/checkpoints/train_g1_backroll_v2.log | tail -4
echo "== deploy x5"
cd /workspace/g1_moveit_config/scripts && python3 -u deploy_roll.py --model-path $OUT \
  --config /workspace/pai_training_g1/configs/g1_backroll_features_v2.py \
  --trials ${3:-5} --run-s 6 --out ~/deploy/g1_backroll_v2${2:+_test}
echo AFTER_V2_DONE
