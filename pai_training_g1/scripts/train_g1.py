"""
G1 レッスン2 の ACT 学習（Colab / GPU 用）。

講義の pai_training/scripts/train.py と同じ学習（同じ ACT 設定・損失・バッチ・ステップの数え方）で、
次の 3 点だけを足しています。

  1. 学習の長さを「エポック数」で指定し、ステップ数に直す。端数の 1 フレームは捨てる（drop_last=True と同じ）。
  2. SAVE_EVERY ステップごとに途中経過（モデル・オプティマイザ・ステップ数）を保存し、
     切断されても同じ場所から再開できるようにする。講義の train.py は最後に 1 回だけ保存する。
  3. 特徴量は、データセットの meta/info.json から作る。観測 2 つと action の 3 つだけを使う。

使い方（pai_training の直下で実行する）:
  python scripts/train_g1_colab.py --dataset-root /content/data/g1_lesson2_fours \
      --repo-id g1_lesson2_fours --output-dir /content/drive/MyDrive/g1_lesson2/ckpt/g1_lesson2_fours \
      --epochs 8 --save-every 5000
"""

import argparse
import os
from pathlib import Path

import torch
import wandb
from lerobot.common.datasets.lerobot_dataset import LeRobotDataset, LeRobotDatasetMetadata
from lerobot.common.datasets.utils import dataset_to_policy_features
from lerobot.common.policies.act.configuration_act import ACTConfig
from lerobot.common.policies.act.modeling_act import ACTPolicy
from lerobot.configs.types import FeatureType

# 講義の crane_features.py と同じ 3 つ（G1 では各 29 次元）
POLICY_KEYS = ["observation.state", "observation.environment_state", "action"]

# 講義の train_config.yaml と同じ。action は 0.0〜1.5 秒を 0.1 秒刻みで 16 個（講義 p.49）
DELTA_TIMESTAMPS = {
    "observation.environment_state": [0.0],
    "observation.state": [0.0],
    "action": [round(0.1 * i, 1) for i in range(16)],
}


def parse_args():
    p = argparse.ArgumentParser()
    p.add_argument("--dataset-root", required=True, help="LeRobotDataset のフォルダ（meta/ と data/ を含む）")
    p.add_argument("--repo-id", required=True, help="データセットの名前（例: g1_lesson2_fours）")
    p.add_argument("--output-dir", required=True, help="学習結果の保存先。途中経過は output-dir/last/ に入る")
    p.add_argument("--epochs", type=int, default=8)
    p.add_argument("--batch-size", type=int, default=2)
    p.add_argument("--chunk-size", type=int, default=16)
    p.add_argument("--n-action-steps", type=int, default=1)
    p.add_argument("--action-dt", type=float, default=0.1,
                   help="追加: action の刻み（秒）。講義は 0.1。指令を出す間隔（G1 は 0.02）に合わせるときに変える")
    p.add_argument("--learning-rate", type=float, default=1e-4)
    p.add_argument("--num-workers", type=int, default=2)
    p.add_argument("--log-freq", type=int, default=1)
    p.add_argument("--save-every", type=int, default=5000, help="途中経過を保存するステップ間隔")
    p.add_argument("--device", default="cuda")
    p.add_argument("--wandb-project", default="pai_crane_gazebo")
    p.add_argument("--stop-after", type=int, default=None,
                   help="テスト用。このステップ数で止める（切断の再現に使う）")
    return p.parse_args()


def build_policy_features(meta):
    """データセットの meta/info.json から、観測 2 つと action の定義を作る。"""
    missing = [k for k in POLICY_KEYS if k not in meta.features]
    if missing:
        raise SystemExit(f"データセットに {missing} がありません。変換の特徴量の設定を確認してください。")
    features = {}
    for key in POLICY_KEYS:
        ft = meta.features[key]
        features[key] = {"dtype": ft["dtype"], "shape": tuple(ft["shape"]), "names": ft.get("names")}
    return features


def save_last(policy, optimizer, step, run_id, last_dir: Path):
    """途中経過を保存する。一時ファイルに書いてから置き換えるので、書き込みの途中で切れても前の保存は壊れない。"""
    last_dir.mkdir(parents=True, exist_ok=True)
    tmp = last_dir / "policy.pt.tmp"
    torch.save(policy.state_dict(), tmp)
    os.replace(tmp, last_dir / "policy.pt")
    tmp = last_dir / "resume.pt.tmp"
    torch.save({"optimizer": optimizer.state_dict(), "step": step, "wandb_run_id": run_id}, tmp)
    os.replace(tmp, last_dir / "resume.pt")
    print(f"saved checkpoint at step {step} -> {last_dir}", flush=True)


def load_last(policy, optimizer, last_dir: Path, device):
    """途中経過があれば読み込み、(再開するステップ数, wandb の run id) を返す。なければ (0, None)。"""
    resume_path = last_dir / "resume.pt"
    if not resume_path.exists():
        return 0, None
    policy.load_state_dict(torch.load(last_dir / "policy.pt", map_location=device, weights_only=False))
    state = torch.load(resume_path, map_location=device, weights_only=False)
    optimizer.load_state_dict(state["optimizer"])
    print(f"resume from step {state['step']}", flush=True)
    return state["step"], state.get("wandb_run_id")


def main():
    args = parse_args()
    device = torch.device(args.device)
    dataset_root = Path(args.dataset_root)
    output_dir = Path(args.output_dir)
    last_dir = output_dir / "last"
    output_dir.mkdir(parents=True, exist_ok=True)

    meta = LeRobotDatasetMetadata(args.repo_id, root=dataset_root)
    features = dataset_to_policy_features(build_policy_features(meta))
    output_features = {k: ft for k, ft in features.items() if ft.type is FeatureType.ACTION}
    input_features = {k: ft for k, ft in features.items() if k not in output_features}
    print(f"input_features: {input_features}", flush=True)
    print(f"output_features: {output_features}", flush=True)

    delta = dict(DELTA_TIMESTAMPS, action=[round(args.action_dt * i, 4) for i in range(args.chunk_size)])   # 追加
    dataset = LeRobotDataset(args.repo_id, root=dataset_root, delta_timestamps=delta)
    num_frames = len(dataset)
    steps_per_epoch = num_frames // args.batch_size  # drop_last=True と同じ数え方（端数の 1 フレームは捨てる）
    total_steps = args.epochs * steps_per_epoch
    print(f"frames={num_frames} steps_per_epoch={steps_per_epoch} epochs={args.epochs} "
          f"total_steps={total_steps}", flush=True)

    cfg = ACTConfig(
        input_features=input_features,
        output_features=output_features,
        chunk_size=args.chunk_size,
        n_action_steps=args.n_action_steps,
    )
    policy = ACTPolicy(cfg, dataset_stats=meta.stats)
    policy.train()
    policy.to(device)
    optimizer = torch.optim.Adam(policy.parameters(), lr=args.learning_rate)

    step, run_id = load_last(policy, optimizer, last_dir, device)
    wandb.init(
        project=args.wandb_project,
        name=args.repo_id,
        id=run_id,
        resume="allow",
        config={**vars(args), "frames": num_frames, "total_steps": total_steps},
    )

    dataloader = torch.utils.data.DataLoader(
        dataset,
        num_workers=args.num_workers,
        batch_size=args.batch_size,
        shuffle=True,
        pin_memory=False,
        drop_last=True,
    )

    done = step >= total_steps
    while not done:
        for batch in dataloader:
            batch = {k: (v.to(device) if isinstance(v, torch.Tensor) else v) for k, v in batch.items()}
            # 講義の train.py と同じ: 系列の次元を落としてから ACT に渡す
            batch["observation.state"] = batch["observation.state"][:, -1, :]
            batch["observation.environment_state"] = batch["observation.environment_state"][:, -1, :]
            loss, _ = policy.forward(batch)
            loss.backward()
            optimizer.step()
            optimizer.zero_grad()

            if step % args.log_freq == 0:
                print(f"step: {step} loss: {loss.item():.3f}", flush=True)
                wandb.log({"loss": loss.item(), "step": step})
            step += 1

            if step % args.save_every == 0 or step >= total_steps:
                save_last(policy, optimizer, step, wandb.run.id if wandb.run else None, last_dir)
            if args.stop_after is not None and step >= args.stop_after:
                print(f"--stop-after={args.stop_after} で止めます（テスト用）", flush=True)
                wandb.finish()
                return
            if step >= total_steps:
                done = True
                break

    # 講義の train.py と同じ場所・同じ形式で保存する（infer.py の --model-path にそのまま指定できる）
    policy.save_pretrained(output_dir)
    print(f"policy saved to {output_dir}", flush=True)
    wandb.finish()


if __name__ == "__main__":
    main()
