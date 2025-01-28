#!/usr/bin/env python
# -*- coding: utf-8 -*-
from flask import Flask, render_template, request, redirect, url_for, session, jsonify
from werkzeug.security import generate_password_hash, check_password_hash
import sqlite3
import os
import rospy
from collections import deque
from std_msgs.msg import Int32, Bool
import threading
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseActionResult


app = Flask(__name__)

DB_PATH = os.path.join(os.getcwd(), 'data', 'company.db')

app.secret_key = os.urandom(24)  # 세션 암호화용 비밀키
robot_scheduling_queue = deque() #로봇의 행동을 담을 큐 (여러사람 작업 처리위해)
is_with_person = False #로봇이 사람과 만나서 점유된 상태인지에 대한 플래그 변수
cnt = 0
# 예시 사용자 데이터 (실제로는 DB에서 데이터를 가져오거나 다른 방법을 사용)
users = {
    'user1': {'password': generate_password_hash('password1'), 'location': {'pos_x': 0.0616436451674, 'pos_y': 1.57818627357, 'ori_z': 0.898063068045, 'ori_w': 0.439866713691}},
    'user2': {'password': generate_password_hash('password2'), 'location': {'pos_x': -0.0987980738282, 'pos_y': 0.411867380142, 'ori_z': 0.315396049112, 'ori_w': 0.948960132042}},
}

#-------------------------- 콜백 함수 정의부 ------------------------#

#얘는 계속해서 호출된다는 것을 잊지말자
def robot_scheduler(msg): 
    global is_with_person
    if msg.data==True: #robot_moving_status_publisher 노드에서 받아온 로봇 움직임 값 (status 1이면 True, 3이면 False)
        rospy.loginfo("로봇이 이동중입니다.")
    else:
        # 로봇이 이동 중이지 않으면 큐를 확인하고 작업을 처리
        if can_robot_go():
            position = robot_scheduling_queue.popleft()
            is_with_person = True
            move_pub.publish(position)  # 로봇에 이동 명령을 발행
            rospy.loginfo("큐에 담겨있던 이동명령을 발행합니다.")
        else:
            # 큐가 비었으면 대기
            rospy.loginfo("큐가 비어있습니다.")

#-------------------------- 콜백 함수 정의부 ------------------------#

def can_robot_go():
    global is_with_person
     #robot_scheduling_queue가 1개라도 채워져있고, 로봇이 사람과 만나지 않은 경우 True 반환
    if robot_scheduling_queue and not is_with_person:
        return True
    return False


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
    global is_with_person
    is_with_person = False
    return redirect(url_for('index'))

@app.route('/redirect_to_index', methods=['GET'])
def redirect_to_index():
    return redirect(url_for('index'))


# user가 웹에서 호출 버튼 누름 -> js가 /summon_robot으로 요청 보냄, 호출자의 좌표와 함께 ->
# 아래 함수가 실행되어 호출자의 좌표가 큐에 쌓임
@app.route('/summon_robot', methods=['POST'])
def summon_robot():
    global is_with_person
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
        
        if cnt == 0: #임시 땜방 셋업 - 일단 첫번째 호출로 무지성 이동시킴.
            move_pub.publish(position)
            cnt +=1
        else:
            # 첫번째 호출 무지성 이동후부터는 얘가 실행됨. 호출자가 '호출'누를때마다 호출자의 좌표가 큐에쌓임.
            robot_scheduling_queue.appendleft(position) 
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
    # 사용자로부터 입력 받은 이름
    name = request.form['name']

    # DB에서 해당 이름의 부서 정보와 좌표 조회
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    try:
        # 직원 정보 조회
        cursor.execute("SELECT department FROM employees WHERE name = ?", (name,))
        
        employee = cursor.fetchone()

        if not employee:
            return jsonify({'error': '해당 이름의 직원이 존재하지 않습니다.'}), 404
        else : 
            rospy.loginfo('DB조회해서 직원의 이름으로 부서 찾기 성공')

        department = employee[0]

        # 부서 좌표 정보 조회
        cursor.execute("SELECT pos_x, pos_y, ori_z, ori_w FROM departments WHERE department = ?", (department,))
        coords = cursor.fetchone()

        if not coords:
            return jsonify({'error': '해당 부서의 좌표 정보가 존재하지 않습니다.'}), 404
        else :
            rospy.loginfo('DB조회해서 부서 이름으로 좌표 찾기 성공')

        # 좌표를 기반으로 PoseStamped 객체 초기화
        position = PoseStamped()
        position.header.frame_id = "map"
        position.header.stamp = rospy.Time.now()
        position.pose.position.x = coords[0]
        position.pose.position.y = coords[1]
        position.pose.orientation.z = coords[2]
        position.pose.orientation.w = coords[3]

        # PoseStamped 객체 정보 출력 (f-string 대신 .format() 사용)
        rospy.loginfo("이름: {}, 부서: {}, 좌표: (x: {}, y: {}, z: {}, w: {})".format(
            name, department, coords[0], coords[1], coords[2], coords[3]))

        # 리디렉션 URL을 포함하여 응답
        return jsonify({
            'name': name,
            'department': department,
            'coordinates': {
                'pos_x': coords[0],
                'pos_y': coords[1],
                'ori_z': coords[2],
                'ori_w': coords[3]
            },
            'message': '좌표가 정상적으로 초기화되었습니다.',
            'redirect_url': url_for('success')  # 리디렉션 URL
        })

    except sqlite3.Error as e:
        return jsonify({'error': 'DB 에러 발생: {}'.format(e)}), 500

    finally:
        conn.close()


if __name__ == '__main__':
    # ROS 스레드 시작, target에 ros_spin(위에보면 rospy.spin() 담겨있음.)적어두면, 아래 세줄로 모든 ros섭스크라이브 콜백들이 쓰레드로 작동함.
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True  # 메인 스레드 종료 시 함께 종료
    ros_thread.start()

    # Flask 서버 실행
    run_flask()
