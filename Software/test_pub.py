#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def talker():
    # 初始化節點
    rospy.init_node('my_publisher', anonymous=True)
    # 建立 Publisher，topic 名稱需和 subscriber 相同
    pub = rospy.Publisher('/your_topic_name', String, queue_size=10)

    rate = rospy.Rate(1)  # 每秒發送一次
    count = 0

    while not rospy.is_shutdown():
        msg = f"Hello ROS! Message {count}"
        rospy.loginfo(f"Publishing: {msg}")
        pub.publish(msg)
        count += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
