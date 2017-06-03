mport roslib
roslib.load_manifest('tutorial')

#Цепляем нужные нам библиотеки. rospy привязка ROS к Python, 
#String - обертка над очевидным типом данных для формирования сообщений
import rospy
from std_msgs.msg import String
def talker():
#Создаем шину под названием chatter для публикации сообщений типа String
    pub = rospy.Publisher('chatter', String)
#Говорим ROS, как называется наш узел
    rospy.init_node('talker')
#Пока ROS работает, публикуем строку
    while not rospy.is_shutdown():
        str = "Zooo! %s"%rospy.get_time()
        rospy.loginfo(str)
        pub.publish(String(str))
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
