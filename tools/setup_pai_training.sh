#!/bin/bash
# 講義の pai_training を、レッスン1 と同じ版（b4d6b54）で入れる（コンテナの中、ubuntu で実行）
set -e
cd ~
if [ ! -d pai_training ]; then
  git clone --recursive https://github.com/matsuolab/pai_training
fi
cd ~/pai_training
git checkout -q b4d6b54
git submodule update --init --recursive
git log -1 --format='pai_training: %h (%cd)' --date=short
git -C lerobot log -1 --format='lerobot: %h'
if [ ! -x ~/.pixi/bin/pixi ]; then
  curl -fsSL https://pixi.sh/install.sh | bash
fi
export PATH=~/.pixi/bin:$PATH
pixi --version
pixi install 2>&1 | tail -5
pixi run python -c "import torch, lerobot; print('torch', torch.__version__, '| cuda available:', torch.cuda.is_available())"
echo SETUP_PAI_TRAINING_DONE
