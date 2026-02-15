#IT
## Импорт
```python
import matplotlib.pyplot as plt
```
## Создание одного графика
```python
plt.plot(x, y1, label='График 1')
```
## Легенда
```python
plt.legend()
```
## Вывод графиков
```python
plt.show(block=false) # не блокирует поток
```
## Создание нескольких графиков
```python
fig, (ax1, ax2) = plt.subplots(2, 1)  # 2 строки, 1 колонка

ax1.plot(x, y1, color='blue')
ax1.set_title('График 1')

ax2.plot(x, y2, color='green')
ax2.set_title('График 2')

plt.tight_layout()  # Чтобы графики не налезали друг на друга
plt.show()
```
## Создание нескольких окон
```python
fig1 = plt.figure()
plt.plot(x, y1)
plt.title('Первое окно')

fig2 = plt.figure()
plt.plot(x, y2)
plt.title('Второе окно')

plt.show()
```