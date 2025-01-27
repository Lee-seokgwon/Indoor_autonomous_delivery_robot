#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from actionlib_msgs.msg import GoalStatusArray

class RobotMovementStatus:
    def __init__(self):
        self.robot_moving = False

        #/robot_moving_status 토픽으로  publish 하는 publisher 생성성
        self.move_pub = rospy.Publisher('/robot_moving_status', Bool, queue_size=1)

        #odom 토픽을 구독하여 odom값을 받아온다
        self.status = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

    def status_callback(self, msg):
        """
        로봇의 이동 상태 정보를 받아서 퍼블리시시
        """
        if msg.status_list:  # 상태 리스트가 비어있지 않을 때만 처리
            status = msg.status_list[0].status
            if status == 1:
                rospy.loginfo("로봇이 목적지로 정상적으로 이동 중 입니다.")
                self.move_pub.publish(True)
            elif status == 2:
                rospy.loginfo("로봇의 기존 목적지가 취소 당했습니다. 별도의 publish 하지않겠습니다.")
            elif status == 3:
                rospy.loginfo("로봇이 목적지에 정상적으로 도착했습니다다")
                self.move_pub.publish(False)
            elif status == 4:
                rospy.loginfo("목적지에 도착을 실패하였습니다.")
        else:
            rospy.loginfo("status가 1,2,3,4에 속하지 않습니다. 다른 status 입니다.")


if __name__ == '__main__':

    #노드 이름을 robot_movement_status_node 로 설정 및 노드 초기화
    rospy.init_node('robot_movement_status_node', anonymous=True)
    
    #RobotMovementStatus 의 객체 생성
    movement_status = RobotMovementStatus()

    rospy.loginfo("로봇 움직임 상태 퍼블리시 노드 시작")
    
    #지금 부터 스레드는 ROS의 pub,sub 관련 event loop만 돌며 계속해서 pub,sub을 동작시킨다.
    rospy.spin()  

