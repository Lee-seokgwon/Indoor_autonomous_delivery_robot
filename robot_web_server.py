#!/usr/bin/env python
# -*- coding: utf-8 -*-
from flask import Flask, render_template, request, redirect, url_for, session, jsonify
from werkzeug.security import generate_password_hash, check_password_hash
import os
import rospy
from collections import deque
from std_msgs.msg import Int32, Bool
import threading
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult

app = Flask(__name__)
app.secret_key = os.urandom(24)  # 세션 암호화용 비밀키
task_queue = deque() #로봇의 행동을 담을 큐 (여러사람 작업 처리위해)
summoner_queue = deque() #호출한 사람들을 ID 담아두는 리스트
is_submit_done = False #로봇이 사람과 만나서 점유된 상태인지에 대한 플래그 변수
cnt = 1
# 예시 사용자 데이터 (실제로는 DB에서 데이터를 가져오거나 다른 방법을 사용)
users = {
    'user1': {'password': generate_password_hash('password1'), 'location': {'pos_x': 0.0616436451674, 'pos_y': 1.57818627357, 'ori_z': 0.898063068045, 'ori_w': 0.439866713691}},
    'user2': {'password': generate_password_hash('password2'), 'location': {'pos_x': -0.0987980738282, 'pos_y': 0.411867380142, 'ori_z': 0.315396049112, 'ori_w': 0.948960132042}},
}

#-------------------------- 콜백 함수 정의부 ------------------------#

#얘는 계속해서 호출된다는 것을 잊지말자
def robot_scheduler(msg): 
    global is_submit_done
    if msg.data==True: #robot_moving_status_publisher 노드에서 받아온 로봇 움직임 값 (status 1이면 True, 3이면 False)
        rospy.loginfo("로봇이 이동중입니다.")
    elif task_queue and is_submit_done:
        # 로봇이 이동 중이지 않으면 큐를 확인하고 작업을 처리
        position = task_queue.popleft()
        move_pub.publish(position)  # 로봇에 이동 명령을 발행
        rospy.loginfo("큐에 담겨있던 이동명령을 발행합니다.")
        is_submit_done = False
    else:
        # 큐가 비었으면 대기
        rospy.loginfo("큐가 비어있습니다.")

#-------------------------- 콜백 함수 정의부 ------------------------#



rospy.init_node('flask_server', anonymous=True)
move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
rospy.Subscriber('/robot_moving_status',Bool, robot_scheduler)


def run_flask():
    """Flask 서버 실행"""
    app.run(host='0.0.0.0', port=5000, debug=True)

def ros_spin():
    rospy.spin()


@app.route('/')
def index():
    if 'user_id' in session:
        user_id = session['user_id']
        return render_template('ROS_mode_select.html', user_id=user_id, location=users[user_id]['location'])
    else:
        return redirect(url_for('login'))

@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        user_id = request.form['user_id']  # 로그인 ID
        password = request.form['password']  # 로그인 비밀번호

        # 사용자가 존재하는지 확인하고 비밀번호 체크
        user = users.get(user_id)
        if user and check_password_hash(user['password'], password):
            session['user_id'] = user_id  # 세션에 사용자 ID 저장
            return redirect(url_for('index'))  # 로그인 후 메인 페이지로 리디렉션
        else:
            return 'Invalid credentials', 401  # 로그인 실패

    return render_template('ROS_login.html')  # 로그인 페이지로 이동


@app.route('/redirect_summon_robot_web', methods=['GET'])
def redirect_summon_robot_web():
    return redirect(url_for('summon_robot_web_open'))

@app.route('/summon_robot_web_open', methods=['GET'])
def summon_robot_web_open():
    return render_template('ROS_summon_web.html')

@app.route('/redirect_pick_up_item_web', methods=['GET'])
def redirect_pick_up_item_web():
    return redirect(url_for('pick_up_item_web_open'))

@app.route('/pick_up_item_web_open', methods=['GET'])
def pick_up_item_web_open():
    return render_template('ROS_pick_up_item_web.html')

@app.route('/redirect_get_text_web', methods=['GET'])
def redirect_get_text_web():
    return redirect(url_for('get_text'))

@app.route('/get_text', methods=['GET']) 
def get_text():
    return render_template('get_text.html')

@app.route('/item_received', methods=['GET'])
def item_received():
    global is_submit_done
    is_submit_done = True
    return redirect(url_for('index'))

@app.route('/ROS_no_more_summon', methods=['GET'])
def ROS_no_more_summon():
    return render_template('ROS_no_more_summon.html')

@app.route('/redirect_to_index', methods=['GET'])
def redirect_to_index():
    return redirect(url_for('index'))


# user가 웹에서 호출 버튼 누름 -> js가 /summon_robot으로 요청 보냄, 호출자의 좌표와 함께 ->
# 아래 함수가 실행되어 호출자의 좌표가 큐에 쌓임
@app.route('/summon_robot', methods=['POST'])
def summon_robot():
    global cnt
    try:
        
        if 'user_id' not in session:
            raise ValueError('User ID not found in session.')

        user_id = session['user_id']  # 자바스크립트에서 전송된 user_id
        user_location = users.get(user_id, {}).get('location')  # 해당 사용자의 위치 찾기

        if not user_location:
            raise ValueError('User location not found')
        position = PoseStamped()
        position.header.frame_id = "map"
        position.header.stamp = rospy.Time.now()
        position.pose.position.x = float(user_location['pos_x'])
        position.pose.position.y = float(user_location['pos_y'])
        position.pose.orientation.z = float(user_location['ori_z'])
        position.pose.orientation.w = float(user_location['ori_w'])
        
        if cnt == 1: #임시 땜방 셋업 - 일단 첫번째 호출로 무지성 이동시킴.
            move_pub.publish(position)
            summoner_queue.append(position)
            cnt +=1
        else:
            # 첫번째 호출 무지성 이동후부터는 얘가 실행됨. 호출자가 '호출'누를때마다 호출자의 좌표가 큐에쌓임.
            if len(summoner_queue) == 2:
                redirect(url_for('ROS_no_more_summon'))
            else:
                task_queue.appendleft(position)
                summoner_queue.append(position)
                cnt+=1 
        return redirect(url_for('ROS_robot_is_summoned'))

    except Exception as e:
        print("Error: {}".format(e))  # 서버에서 발생한 오류를 로그로 출력
        return 'Error occurred: {}'.format(e), 500  # 오류를 클라이언트에 반환

@app.route('/ROS_robot_is_summoned')
def ROS_robot_is_summoned():
    return render_template('ROS_robot_is_summoned.html')   

#user가 물건 다 담고 서랍 닫은후에, 목적지 좌표도 알려주면 실행되는 라우팅
@app.route('/submit_text', methods=['POST'])
def submit_text():
    global is_submit_done
    try:
        user_text = request.form.get('user_text', '')  # 사용자가 입력한 텍스트 (ID)

        # user_text가 users 딕셔너리에서 존재하는 사용자 ID인지 확인
        user_location = users.get(user_text, {}).get('location')
        
        if not user_location:
            return jsonify({"status": "failure", "message": "User not found"}), 404  # 사용자 찾을 수 없는 경우
        #목적지 좌표로 position 객체 초기화
        position = PoseStamped()
        position.header.frame_id = "map"
        position.header.stamp = rospy.Time.now()
        position.pose.position.x = float(user_location['pos_x'])
        position.pose.position.y = float(user_location['pos_y'])
        position.pose.orientation.z = float(user_location['ori_z'])
        position.pose.orientation.w = float(user_location['ori_w'])
        

        #task_queue 의 오른쪽에 호출자가 원하는 배송 목적지의 좌표 정보가 담긴 position 객체를 넣어준다.
        task_queue.append(position)
        is_submit_done = True
        return redirect(url_for('index'))

    except Exception as e:
        print("Error: {}".format(e))  # 서버에서 발생한 오류를 로그로 출력
        return jsonify({"status": "failure", "message": "Error occurred: {e}"}), 500  # 오류를 클라이언트에 반환

if __name__ == '__main__':
    # ROS 스레드 시작, target에 ros_spin(위에보면 rospy.spin() 담겨있음.)적어두면, 아래 세줄로 모든 ros섭스크라이브 콜백들이 쓰레드로 작동함.
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True  # 메인 스레드 종료 시 함께 종료
    ros_thread.start()

    # Flask 서버 실행
    run_flask()
