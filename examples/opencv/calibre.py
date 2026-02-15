import cv2
from pathlib import Path

root = Path("data/calibre")

cap = cv2.VideoCapture(0)
i = 0
while True:
    _, frame = cap.read()
    pressed_key = cv2.waitKey(1) & 0xFF
    if pressed_key == ord('p'):
        cv2.imwrite(root / f"{i}.png", frame)
        print(i)
        i += 1
    if pressed_key == ord('q'):
        break
    cv2.imshow("video", frame)

cap.release()
cv2.destroyAllWindows()
