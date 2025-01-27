#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

class RobotMovementStatus:
    def __init__(self):
        self.robot_moving = False

        #/robot_moving_status 토픽으로  publish 하는 publisher 생성성
        self.move_pub = rospy.Publisher('/robot_moving_status', Bool, queue_size=1)

        #odom 토픽을 구독하여 odom값을 받아온다
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

    #노드 이름을 robot_movement_status_node 로 설정 및 노드 초기화
    rospy.init_node('robot_movement_status_node', anonymous=True)
    
    #RobotMovementStatus 의 객체 생성성
    movement_status = RobotMovementStatus()

    rospy.loginfo("로봇 움직임 상태 퍼블리시 노드 시작")
    
    #지금 부터 스레드는 ROS의 pub,sub 관련 event loop만 돌며 계속해서 pub,sub을 동작시킨다.
    rospy.spin()  

