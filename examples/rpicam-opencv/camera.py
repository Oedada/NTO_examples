import cv2 as cv
from picamera2 import Picamera2

picam2 = Picamera2()
picam2.configure(picam2.create_preview_configuration(main={"size": (640, 480)}))
picam2.start()

while True:
    frame = picam2.capture_array()
    cv.imshow("image", frame)

picam2.stop()
cv.destroyAllWindows()
