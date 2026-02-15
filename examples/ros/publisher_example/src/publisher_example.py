#!/usr/bin/env python3
#-*- coding: utf-8 -*-

import rospy

# типы данных: Bool, Byte, Char, Int8/16/32/64, Float32/64, String и др.
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
