# crane_x7_Hand_Tracking
このパッケージは、mediapipeを使ってハンドトラッキングでロボットアーム（CRANE-X7）を動かすことができるros2パッケージです。

## このパッケージを使う前に
### ROS 2及びCRANE-X7セットアップ
  この資料はUbuntu 22.04 LTSを元に書いています.   
  * ROS 2インストール  
　　上田先生の[動画](https://youtu.be/mBhtD08f5KY)及び[インストールスクリプト](https://github.com/ryuichiueda/ros2_setup_scripts)を参照し, インストールを行ってください.   
  * CRANE-X7及び関連パッケージのインストール  
　　[RT社公式リポジトリ](https://github.com/rt-net/crane_x7_ros/tree/ros2)よりインストールできます. 以下にインストールコマンドを載せます.   
    ```
    # Setup ROS environment
    source /opt/ros/humble/setup.bash

    # MediaPipe（手の認識用）
    pip install mediapipe

    # Download crane_x7 repositories
    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    git clone -b ros2 https://github.com/rt-net/crane_x7_ros.git
    git clone -b ros2 https://github.com/rt-net/crane_x7_description.git

    # Install dependencies
    rosdep install -r -y -i --from-paths .

    # Build & Install
    cd ~/ros2_ws
    colcon build --symlink-install
    source ~/ros2_ws/install/setup.bash
    ```
    （[https://github.com/rt-net/crane_x7_ros/tree/ros2/README.md](https://github.com/rt-net/crane_x7_ros/tree/ros2/README.md)より一部転載）  
    (#の行はコメント)  
    また, インストールが完了したらパッケージに含まれるサンプルコードをシミュレータ（Gazebo）で試すことができます. 詳しくは
    [こちら](https://github.com/rt-net/crane_x7_ros/tree/ros2/crane_x7_examples)を参照してください.

    crane_x7_controlの[README](https://github.com/rt-net/crane_x7_ros/blob/ros2/crane_x7_control/README.md)に詳しく書いてあります.

# このパッケージの使い方
## インストール
```
cd ~/ros2_ws/src
git clone https://github.com/akajaika/crane_x7_Hand_Tracking
```
## ビルド 
```
cd ~/ros2_ws
colcon build
source ~/ros2_ws/install/setup.bash

# ３行目のコマンドは、~/.bashrcに書いておくことを推奨します.   
# 下のコマンドで.bashrcに追記できます.  
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bachrc
# .bashrcに書いてあるとき下のコマンドが代わりにできます.
source ~/.bashrc
```
## 実行  
シミュレータ（Gazebo）あるいは実機で動かす際には、可視化ツール（RViz）とGazeboの両方を起動する必要があります. 詳しくは[こちら](https://github.com/rt-net/crane_x7_ros/tree/ros2/crane_x7_examples#3-move_group%E3%81%A8controller%E3%82%92%E8%B5%B7%E5%8B%95%E3%81%99%E3%82%8B)を確認してください.

```
#骨格解析、MediaPipeの実行
cd ros2_ws/src/crane_x7_Hand_Tracking/
python3 hand_manipulator.py

#launchの実行
ros2 launch crane_x7_Hand_Tracking move_hand.launch.py
#シミュレータ（Gazebo）の場合
ros2 launch crane_x7_Hand_Tracking move_hand.launch.py use_sim_time:='true'
```
## 操作
webカメラの画角内で片手を移動し操作。動作についてはアームを上から見た場合下記に対応。
* 手の上下の動き：アームの奥行き
* 手の左右の動き：アームの横移動

また、手が画角外に出た場合、再度映った箇所に移動し再開

# 引継ぎ事項
## やり残したこと
* realsenseによる深度取得の組み込み。詳しくは[こちら](https://github.com/Mainichi0501/realsense_depth/tree/main)を確認してください.
* アームの開閉

## ライセンス

(C) 2024 Kai Nonaka,Gakuto Kutsuna,Souma Sirai
 
このパッケージはApache License, Version 2.0に基づき公開されています.  
ライセンスの全文は[LICENSE](./LICENSE)または[https://www.apache.org/licenses/LICENSE-2.0](https://www.apache.org/licenses/LICENSE-2.0)から確認できます。

本パッケージが依存する[crane_x7_description](https://github.com/rt-net/crane_x7_description)には株式会社アールティの非商用ライセンスが適用されています。
詳細は[crane_x7_description/LICENSE](https://github.com/rt-net/crane_x7_description/blob/master/LICENSE)を参照してください。

[MediaPipe](https://github.com/google/mediapipe)（Apache License 2.0）

