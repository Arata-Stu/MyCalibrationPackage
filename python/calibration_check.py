import cv2
import numpy as np
import glob
import argparse
import os

# コマンドライン引数の処理
parser = argparse.ArgumentParser(description="カメラキャリブレーションのためのチェッカーボード画像を処理します。")
parser.add_argument(
    '--image_dir', 
    type=str, 
    required=True, 
    help='キャリブレーション画像が保存されているディレクトリのパス'
)
parser.add_argument(
    '--checkerboard_rows',
    type=int,
    required=True,
    help='チェッカーボードの縦方向（行）の内部コーナー数'
)
parser.add_argument(
    '--checkerboard_cols',
    type=int,
    required=True,
    help='チェッカーボードの横方向（列）の内部コーナー数'
)
args = parser.parse_args()

# チェッカーボードのサイズ
CHECKERBOARD = (args.checkerboard_cols, args.checkerboard_rows)

# 3Dのポイント（現実の座標系）
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)

# 3Dポイントと2Dポイントを格納するリスト
objpoints = []  # 3Dポイント
imgpoints = []  # 2Dポイント

# 画像ディレクトリを読み込む
if not os.path.isdir(args.image_dir):
    print(f"指定されたディレクトリが存在しません: {args.image_dir}")
    exit()

images = glob.glob(os.path.join(args.image_dir, "*.png"))

if not images:
    print("指定されたディレクトリにPNG画像が見つかりませんでした。")
    exit()

for fname in images:
    img = cv2.imread(fname)
    if img is None:
        print(f"画像を読み込めませんでした: {fname}")
        continue

    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # チェッカーボードのコーナーを検出
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)

    if ret:
        objpoints.append(objp)

        # コーナー位置を精密化
        corners2 = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), 
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        )
        imgpoints.append(corners2)

        # 検出結果を画像に描画
        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('Chessboard Detection', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

if not objpoints or not imgpoints:
    print("キャリブレーションに必要なデータが揃っていません。")
    exit()

# カメラキャリブレーション
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("カメラマトリックス:\n", mtx)
print("歪み係数:\n", dist)
print("回転ベクトル:\n", rvecs)
print("平行移動ベクトル:\n", tvecs)

# 歪み補正の例
img = cv2.imread(images[0])  # キャリブレーションしたい画像
h, w = img.shape[:2]
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w, h), 1, (w, h))

# 歪み補正
dst = cv2.undistort(img, mtx, dist, None, newcameramtx)

# 補正後の画像を保存
cv2.imwrite('calibrated_image.png', dst)
print("補正後の画像を 'calibrated_image.png' に保存しました。")
