# この PC（Windows）で、講義の演習用コンテナ pai_ros2 を作る。講義の run_pai_container.sh にあたる。
# 講義からの違いは 3 か所: --shm-size を 512m から 4g にした（同じコンテナで学習をするため。
# 学習のデータ読み込みは共有メモリを使う。レッスン1 は学習用コンテナを分けて 8g にしていた）。
# 2 つ目は --gpus all（学習を GPU で行うため）。
# 3 つ目はポートを 127.0.0.1 にだけ開くようにした（noVNC と rosbridge には認証がないので、LAN の他の PC から入れないようにする）。
# このファイルのあるフォルダを /workspace につなぐ。
# 注意: Windows PowerShell 5.1 で実行するので、このファイルは BOM 付きの UTF-8 で保存する。
docker run -d --name pai_ros2 `
  -p 127.0.0.1:6080:80 --shm-size=4g --gpus all `
  -p 127.0.0.1:9090:9090 -p 127.0.0.1:9876:9876 `
  -v "${PSScriptRoot}:/workspace" `
  airobotbook/ros2-desktop-ai-robot-book-humble:latest
