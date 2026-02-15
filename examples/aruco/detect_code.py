import cv2
import numpy as np
dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_100)
image = cv2.imread("23.png")

detectorParams = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)

def detect_marker(img):
    corners, ids, rejected = detector.detectMarkers(img)
    return corners, ids

def draw_markers(img, bboxs, ids):
    if ids is None:
        return img
    for bbox, id in zip(bboxs, ids):
        img = cv2.polylines(img, [bbox.astype(np.int32).reshape(-1, 1, 2)], True, (0, 0, 255), 10)
    return img


cap = cv2.VideoCapture(0)
bboxs, ids = None, None
frame_number = 0
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, cap.get(cv2.CAP_PROP_FPS), (640, 480))
while True:
    ret, frame = cap.read()
    frame_number += 1
    if frame_number % 1 == 0:
        bboxs, ids = detect_marker(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))
    frame = draw_markers(frame, bboxs, ids)
        # print(ids)
    cv2.imshow('video feed', frame)
    out.write(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
out.release()
cap.release()
cv2.destroyAllWindows()
# print("Corners:", corners)
# print("IDs:", ids)  # теперь должно быть [[23]]
# print("Rejected:", rejected)
