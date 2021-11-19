# phoenix-jetson
[phoenix-pcb](https://github.com/Nkyoku/phoenix-pcb)および[phoenix-firmware](https://github.com/Nkyoku/phoenix-firmware)を制御するROS2パッケージ

## 開発環境
- Jetson Nano Developer Kit SD Card Image R32.6.1
- ROS2 Foxy
- Qt 5.15.2
- VSCode

## インストール
1. Jetson NanoのSDカードにイメージを書き込み初回起動を行う。
1. このリポジトリをクローンする。  
  `cd ~`  
  `git clone --recursive https://github.com/Nkyoku/phoenix-jetson.git`
1. Jetson Nanoの初期設定を行って再起動する。  
  `cd ~/phoenix-jetson`  
  `./initial_setup.sh`  
  `sudo shutdown -r now`
1. ROS2 Foxyをインストールする。(時間が掛かるので途中でパスワードの再入力が要求されるだろう)  
  後で要らなくなったビルド中間生成物を削除する。  
  `cd ~`  
  `git clone https://github.com/Nkyoku/installROS2.git`  
  `cd installROS2`  
  `./installROS2.sh`  
  `cd ~`  
  `sudo rm -rf installROS2`
1. ROS2がインストールできたか確認する。ros2コマンドが起動したら成功している。  
  `source /opt/ros/foxy/setup.bash`  
  `ros2`
1. このリポジトリをビルドする。  
  `cd ~/phoenix-jetson`  
  `colcon build --merge-install --packages-select phoenix_msgs phoenix`  
  `source ./install/local_setup.bash`
1. 開発時の手間を低減するためbash起動時に自動でROS2の環境を読み込むように設定する。  
  `echo "source /opt/ros/foxy/setup.bash" | tee -a ~/.bashrc`  
  `echo "source ~/phoenix-jetson/install/local_setup.bash" | tee -a ~/.bashrc`

## ファイル
- initial_setup.sh  
  Jetson Nanoの初期設定を行うスクリプト。
- phoenix-firmware  
  対応するバージョンのファームウェアのリポジトリ。  
  Jetson Nano上に実体をダウンロードする必要はない。
- src
  - phoenix  
  基板上に存在するデバイスを制御するノード類を収めたパッケージ。
  - phoenix_msgs  
  phoenixパッケージのノードが用いるトピックやサービスのメッセージ。
  - phoenix_gui  
  phoenixパッケージのノードから受信したデータを表示したりコントローラでロボットを操縦するパッケージ。  
  Windowsでしかテストしていない。
