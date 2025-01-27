#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class RobotMovementStatus:
    def __init__(self):
        self.robot_moving = False
        self.move_pub = rospy.Publisher('/robot_moving_status', Bool, queue_size=1)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.odom_callback)

    def odom_callback(self, msg):
        """
        로봇의 위치 정보를 확인하여 이동 여부를 판단하고 퍼블리시
        """
        velocity = msg.twist.twist.linear  # 속도 정보를 확인
        if velocity.x != 0 or velocity.y != 0 or velocity.z != 0:
            # 로봇이 이동 중이면
            if not self.robot_moving:
                self.robot_moving = True
                self.move_pub.publish(True)
                rospy.loginfo("로봇이 움직이고 있습니다.")
        else:
            # 로봇이 멈추면
            self.robot_moving = False
            self.move_pub.publish(False)
            rospy.loginfo("로봇이 멈췄습니다.")

if __name__ == '__main__':
    rospy.init_node('robot_movement_status_node', anonymous=True)
    movement_status = RobotMovementStatus()

    rospy.loginfo("로봇 움직임 상태 퍼블리시 노드 시작")
    rospy.spin()  # 이 노드에서만 콜백을 처리하도록 하여, 별도의 작업을 추가하지 않고도 실행

