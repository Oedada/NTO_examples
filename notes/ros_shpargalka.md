# Shell
## подготовка к работе
```bash
mkdir -p ~/catkin_ws/src
catkin_make
```
## работа с пакетами
```bash
catkin_create_pkg *название_пакета* *зависимость1* *зависимость2*..
```

## работа с нодами
```bash
rosnode list
rosnode info *нода*
rosnode kill *нода*
```

## работа с топиками
```bash
rostopic list
rostopic echo *имя топика*
```

## после сборки
### настройка окружения
```bash
source ~/catkin_ws/devel/setup.bash
```
### запуск roscore
```bash
roscore
```
### запуск нода/нодов
```bash
rosrun *нода* *исполняющий_файл*
```

# Python
## примеры
### publisher
```python
#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy

from std_msgs.msg import String

# создаем publisher
# (название_топика, тип передаваемых данных, очередь)
publisher = rospy.Publisher("publisher", String, queue_size=10)

# задаем название для узла (node)
rospy.init_node("publisher")

# с помощью этого метода можно делать логи и
# выводить сообщение в консоль (при запуске узла)
rospy.loginfo("random message")

# задаем задержку вывода сообщений от publisher'а (в герц)
delay = rospy.Rate(10)

# будет выполняться, пока программа активна
while not rospy.is_shutdown():
    # отправляем сообщение
    publisher.publish("something")

    # задержка
    delay.sleep()

```
## subscriber
```python
#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String

def pub_callback(message):
    rospy.loginfo(message)

# инициализируем  
rospy.init_node("subscriber", anonymous=True)

# (название топика, тип сообщения, ссылка на функцию (при получении сообщения)
rospy.Subscriber("publisher", String, pub_callback)

# обязательно указываем, чтобы программа не завершалась
rospy.spin()

```

### обязательно писать в начале программы каждой ноды!
```python
#!/usr/bin/env python3
#-*- coding: utf-8 -*-
```

## полезное
### типы данных (std_msgs.msg):
Bool, Byte, Char, Int8/16/32/64, Float32/64, String и др.
### логирование:
```python
rospy.loginfo(string)
rospy.logwarn(string)
rospy.logerr(string)
rospy.logfatal(string)
```
### сон
```python
rospy.Rate(частота).sleep()
rospy.sleep(время_в_секундах)
```

