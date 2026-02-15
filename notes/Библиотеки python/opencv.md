### Импорт
```python
import cv2
import numpy as np
```

---

### 1. Чтение/запись изображений
```python
cv2.imread(filename, flags)
# flags: cv2.IMREAD_COLOR, cv2.IMREAD_GRAYSCALE, cv2.IMREAD_UNCHANGED

cv2.imwrite(filename, img)

cv2.imencode(ext, img, params)
cv2.imdecode(buf, flags)
```

---

### 2. Отображение
```python
cv2.namedWindow(winname, flags)
# flags: cv2.WINDOW_NORMAL, cv2.WINDOW_AUTOSIZE, cv2.WINDOW_FULLSCREEN

cv2.imshow(winname, mat)

cv2.waitKey(delay)  # delay в мс, 0 - бесконечно

cv2.destroyWindow(winname)
cv2.destroyAllWindows()

cv2.resizeWindow(winname, width, height)
cv2.moveWindow(winname, x, y)
```

---

### 3. Видео и камера
```python
# Захват
cv2.VideoCapture(index)        # index: 0,1,... для камер
cv2.VideoCapture(filename)     # видеофайл

# Методы VideoCapture
cap.read()                     # -> ret, frame
cap.grab()                     # -> ret
cap.retrieve()                 # -> ret, frame
cap.get(propId)                # получить параметр
cap.set(propId, value)         # установить параметр
cap.release()                  # освободить

# Запись видео
cv2.VideoWriter(filename, fourcc, fps, frameSize)
cv2.VideoWriter_fourcc(c1, c2, c3, c4)  # кодек: ('M','J','P','G'), ('X','V','I','D') и т.д.

# Методы VideoWriter
out.write(frame)
out.release()
```

**Параметры cap.set/get:**
```python
cv2.CAP_PROP_FRAME_WIDTH
cv2.CAP_PROP_FRAME_HEIGHT
cv2.CAP_PROP_FPS
cv2.CAP_PROP_BRIGHTNESS
cv2.CAP_PROP_CONTRAST
cv2.CAP_PROP_SATURATION
cv2.CAP_PROP_HUE
cv2.CAP_PROP_GAIN
cv2.CAP_PROP_EXPOSURE
cv2.CAP_PROP_FOURCC
```

---

### 4. Основные структуры
```python
# Создание изображений
np.zeros((height, width, channels), dtype=np.uint8)
np.ones((height, width, channels), dtype=np.uint8) * 255

# Цвета в BGR
(blue, green, red)           # кортеж для cv2

# Точки и прямоугольники
(x, y)                       # точка
(x, y, w, h)                 # прямоугольник
((x, y), (width, height))    # альтернатива
```

---

### 5. Цветовые пространства
```python
cv2.cvtColor(src, code)
# code: cv2.COLOR_BGR2GRAY, cv2.COLOR_BGR2RGB, cv2.COLOR_BGR2HSV
#       cv2.COLOR_GRAY2BGR, cv2.COLOR_HSV2BGR

cv2.split(mv)                # разделить каналы
cv2.merge(mv)                # объединить каналы
```

---

### 6. Геометрические трансформации
```python
cv2.resize(src, dsize, fx, fy, interpolation)
# interpolation: cv2.INTER_LINEAR, cv2.INTER_NEAREST, cv2.INTER_CUBIC, cv2.INTER_AREA

cv2.warpAffine(src, M, dsize)           # аффинное преобразование
cv2.warpPerspective(src, M, dsize)      # перспективное

cv2.getRotationMatrix2D(center, angle, scale)
cv2.getAffineTransform(src, dst)
cv2.getPerspectiveTransform(src, dst)

cv2.flip(src, flipCode)                  # 0 - по X, 1 - по Y, -1 - по обоим

cv2.transpose(src)
```

---

### 7. Рисование
```python
cv2.line(img, pt1, pt2, color, thickness, lineType)
cv2.circle(img, center, radius, color, thickness, lineType)
cv2.rectangle(img, pt1, pt2, color, thickness, lineType)
cv2.ellipse(img, center, axes, angle, startAngle, endAngle, color, thickness)
cv2.polylines(img, pts, isClosed, color, thickness)
cv2.fillPoly(img, pts, color)
cv2.putText(img, text, org, fontFace, fontScale, color, thickness, lineType)

# Толщина: thickness=-1 заполняет фигуру
# Тип линии: cv2.LINE_8, cv2.LINE_4, cv2.LINE_AA (сглаживание)

# Шрифты
cv2.FONT_HERSHEY_SIMPLEX
cv2.FONT_HERSHEY_PLAIN
cv2.FONT_HERSHEY_DUPLEX
cv2.FONT_HERSHEY_COMPLEX
cv2.FONT_HERSHEY_TRIPLEX
cv2.FONT_HERSHEY_COMPLEX_SMALL
cv2.FONT_HERSHEY_SCRIPT_SIMPLEX
cv2.FONT_HERSHEY_SCRIPT_COMPLEX
```

---

### 8. Фильтрация и обработка
```python
# Размытие
cv2.blur(src, ksize)                     # простое среднее
cv2.GaussianBlur(src, ksize, sigmaX)      # гауссово
cv2.medianBlur(src, ksize)                # медианное
cv2.bilateralFilter(src, d, sigmaColor, sigmaSpace)  # билатеральное

# Морфологические операции
cv2.erode(src, kernel, iterations)
cv2.dilate(src, kernel, iterations)
cv2.morphologyEx(src, op, kernel)
# op: cv2.MORPH_OPEN, cv2.MORPH_CLOSE, cv2.MORPH_GRADIENT, cv2.MORPH_TOPHAT, cv2.MORPH_BLACKHAT

cv2.getStructuringElement(shape, ksize)
# shape: cv2.MORPH_RECT, cv2.MORPH_ELLIPSE, cv2.MORPH_CROSS

# Градиенты
cv2.Sobel(src, ddepth, dx, dy, ksize)
cv2.Scharr(src, ddepth, dx, dy)
cv2.Laplacian(src, ddepth)
cv2.Canny(image, threshold1, threshold2)   # детектор границ

# Пороговая обработка
cv2.threshold(src, thresh, maxval, type)
# type: cv2.THRESH_BINARY, cv2.THRESH_BINARY_INV, cv2.THRESH_TRUNC
#       cv2.THRESH_TOZERO, cv2.THRESH_TOZERO_INV, cv2.THRESH_OTSU

cv2.adaptiveThreshold(src, maxValue, adaptiveMethod, thresholdType, blockSize, C)
# adaptiveMethod: cv2.ADAPTIVE_THRESH_MEAN_C, cv2.ADAPTIVE_THRESH_GAUSSIAN_C
```

---

### 9. Поиск контуров
```python
cv2.findContours(image, mode, method)
# mode: cv2.RETR_EXTERNAL, cv2.RETR_LIST, cv2.RETR_TREE, cv2.RETR_CCOMP
# method: cv2.CHAIN_APPROX_SIMPLE, cv2.CHAIN_APPROX_NONE

cv2.drawContours(image, contours, contourIdx, color, thickness)

cv2.contourArea(contour)
cv2.arcLength(curve, closed)
cv2.boundingRect(array)
cv2.minAreaRect(points)
cv2.minEnclosingCircle(points)
cv2.convexHull(points)
cv2.isContourConvex(contour)
cv2.matchShapes(contour1, contour2, method, parameter)

# Моменты
cv2.moments(array)
# m00 - площадь, m10/m00 - центр X, m01/m00 - центр Y
```

---

### 10. Особые точки и дескрипторы
```python
# SIFT (требуется opencv-contrib-python)
sift = cv2.SIFT_create()
kp, des = sift.detectAndCompute(gray, None)
img = cv2.drawKeypoints(gray, kp, img)

# ORB
orb = cv2.ORB_create()
kp, des = orb.detectAndCompute(gray, None)

# AKAZE
akaze = cv2.AKAZE_create()
kp, des = akaze.detectAndCompute(gray, None)

# BRISK
brisk = cv2.BRISK_create()
kp, des = brisk.detectAndCompute(gray, None)

# Сопоставление
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)
matches = sorted(matches, key=lambda x: x.distance)

img_matches = cv2.drawMatches(img1, kp1, img2, kp2, matches[:10], None)
```

---

### 11. Гистограммы
```python
cv2.calcHist(images, channels, mask, histSize, ranges)

cv2.equalizeHist(src)  # выравнивание гистограммы

cv2.compareHist(H1, H2, method)
# method: cv2.HISTCMP_CORREL, cv2.HISTCMP_CHISQR, cv2.HISTCMP_INTERSECT
#         cv2.HISTCMP_BHATTACHARYYA, cv2.HISTCMP_HELLINGER

cv2.calcBackProject(images, channels, hist, ranges, scale)
```

---

### 12. Детекторы лиц и объектов
```python
# Загрузка каскадов Хаара
face_cascade = cv2.CascadeClassifier('haarcascade_frontalface_default.xml')
eye_cascade = cv2.CascadeClassifier('haarcascade_eye.xml')

# Детекция
faces = face_cascade.detectMultiScale(gray, scaleFactor, minNeighbors, flags, minSize, maxSize)
# scaleFactor: коэффициент масштабирования (1.1-1.3)
# minNeighbors: минимальное количество соседей (3-6)

# HOG дескриптор (для людей)
hog = cv2.HOGDescriptor()
hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())
rects, weights = hog.detectMultiScale(img)
```

---

### 13. Камера Raspberry Pi (специфичные настройки)
```python
# Для Pi Camera через OpenCV
cap = cv2.VideoCapture(0)

# Включение/отключение автоэкспозиции
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)  # 0.25 - выкл, 0.75 - вкл

# Ручная установка экспозиции
cap.set(cv2.CAP_PROP_EXPOSURE, 100)

# Баланс белого
cap.set(cv2.CAP_PROP_WHITE_BALANCE_BLUE_U, 100)

# Поворот изображения (для Pi Camera)
# через V4L2
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('M','J','P','G'))
```

---

### 14. Полезные константы
```python
# Типы данных
cv2.CV_8U, cv2.CV_8S, cv2.CV_16U, cv2.CV_16S, cv2.CV_32S, cv2.CV_32F, cv2.CV_64F

# Кодеки FourCC
cv2.VideoWriter_fourcc('M','J','P','G')  # Motion JPEG
cv2.VideoWriter_fourcc('X','V','I','D')  # XVID
cv2.VideoWriter_fourcc('H','2','6','4')  # H.264
cv2.VideoWriter_fourcc('M','P','4','V')  # MP4
```
