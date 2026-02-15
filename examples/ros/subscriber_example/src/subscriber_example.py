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



