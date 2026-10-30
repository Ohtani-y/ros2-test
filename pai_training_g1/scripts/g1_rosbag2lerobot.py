#!/usr/bin/env python3
"""
G1 用の rosbag → LeRobotDataset 変換。

講義の scripts/rosbag2lerobot.py と同じ方法で変換する。
  - 各トピックの共通の時間範囲を取る
  - 指定 fps で等間隔に区切る（各時刻は、その時刻以前で一番近い値を使う＝直前値を保持）
  - 短いエピソードは最終フレームで埋めて、最長のエピソードにそろえる
CRANE+ の 2 トピック（アームとグリッパ）の代わりに、G1 の 29 関節を関節名で並べ替えて読む。
observation.state と observation.environment_state には、講義と同じく同じ関節角を入れる。

使い方（コンテナの中、~/pai_training で）:
  pixi run python /workspace/pai_training_g1/scripts/g1_rosbag2lerobot.py \
      --config /workspace/pai_training_g1/configs/conversion_config_lesson2_fours.yaml [--overwrite]
"""

import argparse
import importlib.util
import shutil
from pathlib import Path

import numpy as np
import torch
import yaml
from rosbags.rosbag2 import Reader
from rosbags.serde import deserialize_cdr

from lerobot.common.datasets.lerobot_dataset import LeRobotDataset


def load_py(path):
    spec = importlib.util.spec_from_file_location('g1_features', path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def read_bag(ep_dir, feat):
    """関節の状態と指令を、関節名の順（JOINT_NAMES）に並べて読む。時刻は bag の時刻（ns）。"""
    names = feat.JOINT_NAMES
    js_t, js_q, ac_t, ac_q = [], [], [], []
    env_topic = getattr(feat, 'ENV_TOPIC', None)   # 追加: 骨盤の回転センサ（features にあるときだけ読む）
    en_t, en_v = [], []
    with Reader(ep_dir) as reader:
        conns = [c for c in reader.connections if c.topic in (feat.STATE_TOPIC, feat.ACTION_TOPIC, env_topic)]
        for conn, ts, raw in reader.messages(connections=conns):
            msg = deserialize_cdr(raw, conn.msgtype)
            if conn.topic == feat.STATE_TOPIC:
                m = dict(zip(msg.name, msg.position))
                if all(n in m for n in names):
                    js_t.append(ts)
                    js_q.append([m[n] for n in names])
            elif conn.topic == env_topic:   # 追加
                o, w, p = msg.pose.pose.orientation, msg.twist.twist.angular, msg.pose.pose.position
                en_t.append(ts)
                en_v.append(feat.env_from_odom(o.x, o.y, o.z, o.w, w.x, w.y, w.z, p.z))
            elif msg.points:
                m = dict(zip(msg.joint_names, msg.points[0].positions))
                if all(n in m for n in names):
                    ac_t.append(ts)
                    ac_q.append([m[n] for n in names])
    return (np.array(js_t, dtype=np.int64), np.array(js_q, dtype=np.float32),
            np.array(ac_t, dtype=np.int64), np.array(ac_q, dtype=np.float32),
            np.array(en_t, dtype=np.int64), np.array(en_v, dtype=np.float32))


def sample_and_hold(times, values, target_times):
    idx = np.clip(np.searchsorted(times, target_times, side='right') - 1, 0, len(times) - 1)
    return values[idx]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--config', required=True)
    ap.add_argument('--overwrite', action='store_true', help='出力先があれば消して作り直す')
    args = ap.parse_args()

    cfg = yaml.safe_load(open(args.config, encoding='utf-8'))
    feat = load_py(cfg['config'])
    fps = cfg.get('target_freq', 20)
    out = Path(cfg['output_dir'])
    episode_dirs = sorted(p.parent for p in Path(cfg['bag_dir']).rglob('*.db3'))
    if not episode_dirs:
        raise SystemExit(f"bag が見つからない: {cfg['bag_dir']}")

    # 1 回目：各エピソードのフレーム数（共通の時間範囲 × fps）
    episodes = []
    for ep in episode_dirs:
        js_t, js_q, ac_t, ac_q, en_t, en_v = read_bag(ep, feat)
        if len(js_t) == 0 or len(ac_t) == 0:
            print(f'skip {ep.name}: 関節の状態か指令が無い')
            continue
        start, end = max(js_t[0], ac_t[0]), min(js_t[-1], ac_t[-1])
        if cfg.get('trim_idle'):   # 追加: 動き出す前の静止部分を切る（同じ状態で「待つ／動く」が混ざると方策が動き出せない）
            moving = np.where(np.abs(ac_q - ac_q[0]).max(axis=1) > 0.02)[0]
            if len(moving):
                start = max(start, ac_t[moving[0]] - int(0.04e9))
            settled = np.where(np.abs(ac_q - ac_q[-1]).max(axis=1) > 0.02)[0]   # 追加: 終わった後の静止部分も切る
            if len(settled):
                end = min(end, ac_t[settled[-1]] + int(0.1e9))
        n = int((end - start) * fps / 1e9)
        if len(en_t):   # 追加: 回転センサの時間範囲にもそろえる
            start, end = max(start, en_t[0]), min(end, en_t[-1])
            n = int((end - start) * fps / 1e9)
        episodes.append((ep, js_t, js_q, ac_t, ac_q, en_t, en_v, start, end, n))
        print(f'{ep.name}: {n} frames')
    max_frames = max(e[-1] for e in episodes)
    pad = cfg.get('pad', True)   # 追加: false なら最終フレームで埋めない（後転では最後の姿勢＝最初の姿勢のため）
    print(f'longest: {max_frames} frames')

    if out.exists():
        if not args.overwrite:
            raise SystemExit(f'出力先がすでにある: {out}（作り直すときは --overwrite）')
        shutil.rmtree(out)
    ds = LeRobotDataset.create(repo_id=feat.repo_id, fps=fps, root=out, robot_type=feat.robot_type,
                               features=feat.features, use_videos=True)
    if feat.task_name not in ds.meta.task_to_task_index:
        ds.meta.add_task(feat.task_name)

    # 2 回目：直前値を保持して区切り、短いエピソードは最終フレームで埋める
    for ep, js_t, js_q, ac_t, ac_q, en_t, en_v, start, end, n in episodes:
        target = np.linspace(start, end, n)
        q = sample_and_hold(js_t, js_q, target)
        a = sample_and_hold(ac_t, ac_q, target)
        env = sample_and_hold(en_t, en_v, target) if len(en_t) else q   # 追加: 無ければ講義と同じく関節角
        for i in range(max_frames if pad else n):
            k = min(i, n - 1)
            ds.add_frame({
                'observation.state': torch.from_numpy(q[k]),
                'observation.environment_state': torch.from_numpy(env[k]),
                'action': torch.from_numpy(a[k]),
                'task': feat.task_name,
            })
        ds.save_episode()
    total = len(episodes) * max_frames if pad else sum(e[-1] for e in episodes)
    print(f'episodes={len(episodes)} longest={max_frames} total_frames={total} -> {out}')


if __name__ == '__main__':
    main()
