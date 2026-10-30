# Unitree G1 の後転（Gazebo・ACT）

Gazebo の上で仰向けの Unitree G1 に後転させ、そのお手本を ACT で学習し、学習したモデルで 2 回続けて後転させるまでのコードです。
手順は [docs/G1後転_シンプル版.md](docs/G1後転_シンプル版.md) にあります。

## フォルダ

| フォルダ・ファイル | 中身 |
|---|---|
| `run_pai_container.ps1` | コンテナ `pai_ros2` を作る（Windows。このフォルダを `/workspace` につなぐ） |
| `tools/` | セットアップ（`setup_pai_training.sh`・`setup_g1.sh`）と、学習から評価までの一括実行（`after_v2.sh`） |
| `g1_gazebo/` | Gazebo 用の G1。`scripts/make_urdf.py` が公式の URDF に回転センサ・重り・手の当たり判定・ros2_control を足す |
| `g1_moveit_config/` | MoveIt の設定と、お手本・データ集め・推論・2 回後転のスクリプト |
| `pai_training_g1/` | rosbag → LeRobot 形式の変換、ACT の学習・推論 |
| `docs/` | 手順書 |

## このリポジトリに入っていないもの

- 学習済みモデル（`checkpoints/`。1 つ約 600 MB）：手順書の §5 で学習して作ります
- 結果の動画（`export/`）
- `g1_gazebo/meshes/`・`g1_gazebo/urdf/`：`tools/setup_g1.sh` が公式の G1 から作ります

## 使っているもの

- コンテナ：`airobotbook/ros2-desktop-ai-robot-book-humble`（ROS 2 Humble・Gazebo）
- 学習環境：[matsuolab/pai_training](https://github.com/matsuolab/pai_training)（`b4d6b54`）・LeRobot
- ロボット：[unitreerobotics/unitree_ros](https://github.com/unitreerobotics/unitree_ros) の `g1_description`（`5994d4f`）

## ライセンス

[Apache License 2.0](LICENSE) です。ほかのプロジェクトのものから作った部分は [NOTICE](NOTICE) にまとめました。

- unitree_ros（BSD 3-Clause）：G1 のモデルは含めず、`tools/setup_g1.sh` が取得します。関節名・可動範囲など、そこから作った設定だけを含みます
- pai_training：ライセンスの表示がないため、コードは含めていません。`tools/setup_pai_training.sh` が取得します。`pai_training_g1/` のスクリプトは、講義の scripts と同じ方法で G1 用に書いたものです
- LeRobot（Apache-2.0）：pai_training の中で使うだけで、含めていません
