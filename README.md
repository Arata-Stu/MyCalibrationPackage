
```bash
source set_path.sh
```

# 1. 依存関係のインストール

## ROS2 の依存関係
```bash
# ROS2用ツールのインストール
sudo apt-get install python3-vcstool

# 必要なパッケージの取得
vcs import ros2_ws/src/ < package.repos

cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Python 
```
# 仮想環境の作成
python3.9 -m venv my_env

# 仮想環境の有効化
source my_env/bin/activate

# 必要なPythonパッケージのインストール
pip3 install -r requirements.txt

deactivate

```

# 2. イベントカメラとRGBカメラのデータを取得する
```
# terminal 1
ros2 launch calib_launch calibration.launch.xml 
# terminal 2
ros2 bag record -o ${PROJECT_ROOT}/output/event_and_frame/<record name>
```

# 3. カメラのトリガーのタイミングを抽出する
```
ros2 launch time_manager extract_time.launch.xml  bag_file:=${PROJECT_ROOT}/output/event_and_frame/<record name> topic_name:=/rgb_image project_root:=${PROJECT_ROOT}
```

# 4. RosbagからイベントをH5フォーマットとして保存する
```
ros2 launch event_decoder decoder.launch.xml bag_file:=${PROJECT_ROOT}/output/event_and_frame/<record name>/ project_root:=${PROJECT_ROOT}
 
```

# 5. イベント再構築画像を生成する　
```
## e2valibを利用する
python offline_reconstruction.py  --upsample_rate 2 --h5file data.h5 --output_folder gen3_with_trigger/ --timestamps_file triggers.txt --height 480 --width 640


## test detect checker board
python calibration.py --image_dir /path/to/images --checkerboard_rows 6 --checkerboard_cols 10

```

# 6. RGBカメラのRos2 bagをRos1形式に変換する
```
rosbags-convert ./output/event_and_frame/<record name> --dst ./output/ros1_bag/name.bag --include-topic /image_topic_name

```