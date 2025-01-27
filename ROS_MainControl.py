# -*- coding: utf-8 -*-
#!/usr/bin/env python

#이 코드에서 로봇의 이동제어를 담당 (이동하면서 발생하는 이벤트 등도 처리)

import rospy
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseActionResult
from std_msgs.msg import String, Int32, Bool
import os
from sensor_msgs.msg import Image
import time
import threading
#esp8266에 HTTP요청을 보내기 위한 라이브러리리
import requests
import json
from collections import deque
import webbrowser

class MainControl():
    def __init__(self):
        
        # 객체의 users 변수에 호출자의 위치 저장 (실제로는 DB에서 데이터를 가져오거나 다른 방법을 사용)
        self.users = {
        'user1': {'location': {'pos_x': 0.0616436451674, 'pos_y': 1.57818627357, 'ori_z': 0.898063068045, 'ori_w': 0.439866713691}},
        'user2': {'location': {'pos_x': -0.0987980738282, 'pos_y': 0.411867380142, 'ori_z': 0.315396049112, 'ori_w': 0.948960132042}},
        }
        self.position = PoseStamped()
        self.current_goal = PoseStamped()

        self.sub_position = rospy.Subscriber("/summon",PoseStamped,self.get_destination)

        #Navigation Stack에 활용되는 토픽으로, 이 토픽으로 좌표 보내면 로봇이 그 좌표로 움직인다.
        self.pub_goal = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        ##Navigation Stack에 활용되는 토픽으로, 로봇의 도착 상태 (성공,실패) 등을 이 토픽으로 수신받는다.
        self.sub_result = rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.callback, queue_size=1)


        self.esp_command_pub = rospy.Publisher("/esp8266_command", String, queue_size=10)
       #------------------------로봇 호출 후 수령인 ID 받기 위해 웹서버로 request--------------#
        self.server_url = "http://localhost:5000"  # 젯슨웹 서버 주소
        self.text_from_server = ""
        self.customer_list = deque() #수령인 ID 저장 리스트

        #-----------------------------------------------------------------------------------#

    #ID에 따른 해당 인원 좌표값 파싱하는 함수
    def get_location_info(self,user_id):
        user = self.users.get(user_id)
        if user:
            location = user.get('location')
            if location:
                return {
                    location.get('pos_x'),
                    location.get('pos_y'),
                    location.get('ori_z'),
                    location.get('ori_w')
                }
        return (None,None,None,None)
    
    # 호출위치 좌표값 받아오고 설정하는 부분.
    def get_destination(self,msg):
        # 구독한 메시지에서 좌표 값 가져오기
        self.position.pose.position.x = msg.pose.position.x
        self.position.pose.position.y = msg.pose.position.y
        self.position.pose.orientation.z = msg.pose.orientation.z
        self.position.pose.orientation.w = msg.pose.orientation.w

        #---------------------좌표 이외의 부분 정의--------------#
        self.position.pose.position.z = 0.0
        self.position.pose.orientation.x = 0
        self.position.pose.orientation.y = 0
        self.position.header.seq = 0
        self.position.header.stamp.secs = 0
        self.position.header.stamp.nsecs = 0
        self.position.header.frame_id = 'map'
        #-------------------------------------------------------#
        
        #호출위치로 이동수행
        self.move_command(self.position.pose.position.x,self.position.pose.position.y,self.position.pose.orientation.z,self.position.pose.orientation.w)

    def move_command(self, pos_x, pos_y, ori_z, ori_w): #좌표값 인자로 받아서 실제 로봇이 움직이도록 메시지를 퍼블리쉬하는 함수
        self.position.header.seq = 0
        self.position.header.stamp = rospy.Time.now()

        self.position.header.frame_id = 'map'

        self.position.pose.position.x = pos_x
        self.position.pose.position.y = pos_y
        self.position.pose.position.z = 0.0


        self.position.pose.orientation.x = 0
        self.position.pose.orientation.y = 0
        self.position.pose.orientation.z = ori_z
        self.position.pose.orientation.w = ori_w

        #실제 로봇 이동 (아래 한줄), pub goal의 토픽인 /move_base_simple_goal은 ROS가 항상 구독중중
        self.pub_goal.publish(self.position)

        self.current_goal.pose.position.x = pos_x
        self.current_goal.pose.position.y = pos_y
        self.current_goal.pose.orientation.z = ori_z
        self.current_goal.pose.orientation.w = ori_w

    #도착결과에 따라 로봇을 이동시키는 코드 .... 여기에 mcu 관련 제어코드. 대기모드 코드 이런거...
    def callback(self,test):

        if (test.status.status == 3):
            # 목표 도착 시 esp8266제어 명령 전송
            rospy.loginfo("Robot has reached the destination.")

            webbrowser.open('http://localhost:5000/')

            self.esp_command_pub.publish("Arrival") #esp8266 제어노드에에 작동명령 publish
        else:
            rospy.loginfo("fail arrival")    

    def exit(self): 
        self.close() 
        os.system("rosnode kill /rosout") 
        time.sleep(1) 
        os.system("shutdown now") 

    def exit_process(self): 
        os.system("rosnode kill /amcl") 
        os.system("rosnode kill /base_link_to_laser4")
        os.system("rosnode kill /map_server")
        os.system("rosnode kill /md_robot_node")
        os.system("rosnode kill /move_base") 
        os.system("rosnode kill /robot_state_publisher") 
        os.system("rosnode kill /rviz")
        os.system("rosnode kill /turtlebot3_core") 
        os.system("rosnode kill /turtlebot3_diagnostics") 
        os.system("rosnode kill /ydlidar_lidar_publisher")


if __name__ == '__main__':
    try:
        #노드의 이름을 main_control로 설정 및 노드 초기화
        rospy.init_node('main_control', anonymous=True)

        #MainControl 객체 생성
        main_control = MainControl()

        #지금 부터 스레드는 ros pub,sub 관련 event loop만 돌며 계속해서 pub,sub을 동작시킨다.
        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS_MainControl 노드 동작 중지")
