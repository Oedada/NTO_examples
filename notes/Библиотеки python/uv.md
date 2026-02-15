#программирование
## Создание среды
```python
uv init venv --python 3.13
cd venv
```
## Обновление
```python
uv self update
```
## Запуск программ
```python
uv run hello.py
```
## Установка пакетов
```python
uv add requests aiohttp==2.3.1
```
## Установка пакетов из `dependencies`
```python
uv sync
```
# Просмотр зависимостей
```python
uv tree
```
