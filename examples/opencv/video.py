import cv2
cap = cv2.VideoCapture(0)
fourcc = cv2.VideoWriter_fourcc(*'mp4v')
out = cv2.VideoWriter('output.mp4', fourcc, cap.get(cv2.CAP_PROP_FPS), (640, 480)) # обращаем внимание на разрешение, если будет неправильное, то записываться не будет
while True:
    ret, frame = cap.read()
    cv2.imshow('video feed', frame)
    out.write(frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
        
out.release()
cap.release()
cv2.destroyAllWindows()
