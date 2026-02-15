import cv2

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
marker = cv2.aruco.generateImageMarker(dictionary, 23, 200)

# Добавим рамку вокруг маркера
marker_with_border = cv2.copyMakeBorder(marker, 50, 50, 50, 50, cv2.BORDER_CONSTANT, value=255)
cv2.imwrite("23.png", marker_with_border)
