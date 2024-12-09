import h5py
import sys
import numpy as np

# コマンドライン引数からファイルパスを取得
if len(sys.argv) < 2:
    print("使い方: python script.py <ファイルパス>")
    sys.exit(1)

file_path = sys.argv[1]

# HDF5ファイルを読み込む
try:
    with h5py.File(file_path, 'r') as f:
        # ファイルのキー（トップレベルのグループ）を表示
        print("ファイル内のキー:", list(f.keys()))
        
        # 各データセットの最大値と最小値を表示
        for key in f.keys():
            data = f[key][:]
            max_value = np.max(data)
            min_value = np.min(data)
            print(f"{key} の最大値: {max_value}")
            print(f"{key} の最小値: {min_value}")
            print('-' * 40)

except Exception as e:
    print(f"エラーが発生しました: {e}")
