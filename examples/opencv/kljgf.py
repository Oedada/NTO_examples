import cv2
import numpy as np
import glob

# === Параметры шахматной доски ===
CHECKERBOARD = (8, 5)  # внутренние углы
square_size = 0.022  # размер клетки в метрах (например 2.5 см)

# === Критерий уточнения углов ===
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# === 3D точки шахматной доски в реальном мире ===
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D точки
imgpoints = []  # 2D точки

images = glob.glob('data/calibre/*.png')

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Ищем углы шахматной доски
    ret, corners = cv2.findChessboardCornersSB(gray, CHECKERBOARD)

    if ret:
        objpoints.append(objp)

        # Уточняем координаты углов
        corners2 = cv2.cornerSubPix(gray, corners, (11,11), (-1,-1), criteria)
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(1)

cv2.destroyAllWindows()

# === Калибровка ===
ret, cameraMatrix, distCoeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints,
    imgpoints,
    gray.shape[::-1],
    None,
    None
)
print("Reprojection error:", ret)
print("Camera matrix:\n", cameraMatrix)
print("Distortion coefficients:\n", distCoeffs)

# Сохраняем
np.savez("calibration_data.npz",
         cameraMatrix=cameraMatrix,
         distCoeffs=distCoeffs)
