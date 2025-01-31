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
import logging



app = Flask(__name__)

app.secret_key = os.urandom(24)  # 세션 암호화용 비밀키

DB_PATH = os.path.join(os.getcwd(), 'data', 'company.db')

task_queue = deque() #로봇의 행동을 담을 큐 (여러사람 작업 처리위해)
is_submit_done = False #로봇이 사람과 만나서 점유된 상태인지에 대한 플래그 변수
summoner_queue = deque()
cnt = 0
# 예시 사용자 데이터 (실제로는 DB에서 데이터를 가져오거나 다른 방법을 사용)
users = {
    'user1': {'password': generate_password_hash('password1'), 'location': {'pos_x': 0.0616436451674, 'pos_y': 1.57818627357, 'ori_z': 0.898063068045, 'ori_w': 0.439866713691}},
    'user2': {'password': generate_password_hash('password2'), 'location': {'pos_x': -0.0987980738282, 'pos_y': 0.411867380142, 'ori_z': 0.315396049112, 'ori_w': 0.948960132042}},
}#이거 이제 삭제해도 될듯. db구현했으니!

#-------------------------- 콜백 함수 정의부 ------------------------#

#얘는 계속해서 호출된다는 것을 잊지말자
def robot_scheduler(moving): 
    global is_submit_done
    global cnt
    if moving.data==True: #robot_moving_status_publisher 노드에서 받아온 로봇 움직임 값 (status 1이면 True, 3이면 False)
        logger.info("로봇이 이동중입니다.")
    elif task_queue and is_submit_done:
        # 로봇이 이동 중이지 않으면 큐를 확인하고 작업을 처리
        position = task_queue.popleft()
        move_pub.publish(position)  # 로봇에 이동 명령을 발행
        rospy.loginfo("큐에 담겨있던 이동명령을 발행합니다.")
        is_submit_done = False
    elif not task_queue and is_submit_done and summoner_queue:
        # 로봇이 대기 상태이고, 모든 작업이 끝났으며, summoner_queue 초기화 조건을 만족할 때
        summoner_queue.clear()
        is_submit_done = False #마지막 수령인이 수령확인후 True가 되버리니까, False로 돌려줘야함함
        cnt = 0
        rospy.loginfo("모든 작업이 완료되었습니다. summoner_queue를 초기화합니다.")
    else:
        # 큐가 비었으면 대기
        rospy.loginfo("로봇이 submit 대기중입니다.")

#-------------------------- 콜백 함수 정의부 ------------------------#



#---------------------------------------------DB조회 메소드(START)---------------------------------------------------#
def verify_user_credentials(user_id, password):
    """
    데이터베이스에서 ID와 비밀번호를 확인합니다.
    """
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # 데이터베이스에서 사용자 정보 조회
    cursor.execute('SELECT password FROM employees WHERE id = ?', (user_id,))
    user = cursor.fetchone()
    conn.close()

    # 사용자 존재 여부 및 비밀번호 검증
    if user is None:
        logger.info("user가 잘못된 id 혹은 password를 입력했습니다.")
        return False  # 사용자가 존재하지 않음
    logger.info("user가 로그인에 성공하였습니다!.")
    stored_password = user[0]
    return stored_password == password  # 비밀번호 비교 (평문 저장 기준)


# 사용자의 부서를 찾는 함수
def get_user_department_from_db(user_id):
    """
    주어진 user_id에 대해 DB에서 부서 정보를 조회하고 해당 부서를 반환합니다.
    """
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # user_id에 해당하는 사용자의 부서 정보 가져오기
    cursor.execute('''
        SELECT department
        FROM employees
        WHERE id = ?
    ''', (user_id,))

    result = cursor.fetchone()
    conn.close()

    if result:
        # 부서가 존재하면 반환
        return result[0]
    else:
        # 부서가 없으면 예외 처리
        raise ValueError('User department not found in the database')

# 부서에 해당하는 좌표를 찾는 함수
def get_department_position_from_db(department):
    """
    주어진 부서에 대해 DB에서 좌표 정보를 조회하고 해당 좌표를 반환합니다.
    """
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    # 부서에 해당하는 좌표 정보 가져오기
    cursor.execute('''
        SELECT pos_x, pos_y, ori_z, ori_w
        FROM departments
        WHERE department = ?
    ''', (department,))

    result = cursor.fetchone()
    conn.close()

    if result:
        # 좌표가 존재하면 반환
        return {
            'pos_x': result[0],
            'pos_y': result[1],
            'ori_z': result[2],
            'ori_w': result[3]
        }
    else:
        # 좌표가 없으면 예외 처리
        raise ValueError('부서는 존재하나 해당 부서에 해당하는 좌표값이 존재하지 않습니다!')

def get_position_for_user_from_session(session):
    """
    세션에서 user_id를 가져와 DB에서 부서를 찾고, 해당 부서에 맞는 좌표를 조회하여
    position 객체를 생성하여 반환하는 함수입니다.
    """
    # 세션에서 user_id 가져오기
    user_id = session.get('user_id')

    if not user_id:
        raise ValueError('User not logged in')

    # DB에서 user_id에 해당하는 부서 정보 가져오기
    department = get_user_department_from_db(user_id)

    # DB에서 부서에 해당하는 좌표 정보 가져오기
    department_location = get_department_position_from_db(department)

    # 위치 정보를 바탕으로 PoseStamped 객체 생성
    position = PoseStamped()
    position.header.frame_id = "map"
    position.header.stamp = rospy.Time.now()
    position.pose.position.x = float(department_location['pos_x'])
    position.pose.position.y = float(department_location['pos_y'])
    position.pose.orientation.z = float(department_location['ori_z'])
    position.pose.orientation.w = float(department_location['ori_w'])
    
    return position


def run_flask():
    """Flask 서버 실행"""
    app.run(host='0.0.0.0', port=5000, debug=True)

def ros_spin():
    rospy.spin()

#---------------------------------------------DB조회 메소드(FINISH)---------------------------------------------------#

#---------------------------------------------Setting up Pub&Sub-------------------------------------------------#
move_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
rospy.Subscriber('/robot_moving_status',Bool, robot_scheduler)
#---------------------------------------------Setting up Pub&Sub-------------------------------------------------#



@app.route('/')
def index():
    if 'user_id' in session:
        user_id = session['user_id']
        return render_template('ROS_mode_select.html', user_id=user_id)
    else:
        return redirect(url_for('login'))



@app.route('/login', methods=['GET', 'POST'])
def login():
    if request.method == 'POST':
        user_id = request.form['user_id']  # 입력된 ID
        password = request.form['password']  # 입력된 비밀번호

        # ID와 비밀번호 검증
        if verify_user_credentials(user_id, password):
            session['user_id'] = user_id  # 세션에 사용자 ID 저장
            logger.info("user의 로컬 컴퓨터에 stored session cookie 를 저장하였습니다.")
          
            return redirect(url_for('index'))  # 메인 페이지로 리디렉션
        else:
            return 'Invalid credentials', 401  # 로그인 실패 메시지 및 401 상태 코드

    # GET 요청의 경우 로그인 페이지 반환
    return render_template('ROS_login.html')

#여기 라우터도 잘 보고 js로 json 넘겨주는 경우이면 렌더 템플릿만 남기면 될듯.
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
    return redirect(url_for('get_recipients_name_open'))

@app.route('/get_recipients_name_open', methods=['GET']) 
def get_text():
    return render_template('get_recipients_name.html')

@app.route('/item_received', methods=['GET'])
def item_received():
    global is_submit_done
    is_submit_done = True
    return redirect(url_for('index'))

@app.route('/redirect_to_index', methods=['GET'])
def redirect_to_index():
    return redirect(url_for('index'))

# user가 웹에서 호출 버튼 누름 -> js가 /summon_robot으로 요청 보냄, 호출자의 좌표와 함께 ->
# 아래 함수가 실행되어 호출자의 좌표가 큐에 쌓임
@app.route('/summon_robot', methods=['POST'])
def summon_robot():
    global cnt
    logger.info("sumon_robot is routed")
    try:
        if 'user_id' not in session:
            raise ValueError('User ID not found in session.')

        user_id = session['user_id']  # 세션에서 user_id를 가져옵니다.

        # user_id로부터 부서 정보를 찾고 해당 부서의 좌표를 가져옵니다.
        position = get_position_for_user_from_session(session)
        
        logger.info("로봇을 호출한 user의 position 정보 x: {}, y: {}, z: {}, w: {}".format(
            position.pose.position.x,
            position.pose.position.y,
            position.pose.orientation.z,
            position.pose.orientation.w
        ))

        ######################석권!!!!!!!!!!!여기가 세션기반으로 DB조회해서 client의 좌표로 position 초기화한 부분이야
        #######client의 좌표담긴 position객체 쓰고싶으면 여기 바로위에있는 position객체 쓰면돼!!!!!!!!!!!!!!!!!!!!1

        if cnt == 0:  # 첫 번째 호출 시 무지성 이동
            move_pub.publish(position)
            summoner_queue.append(position)
            cnt += 1
            return jsonify({'redirect': url_for('ROS_robot_is_summoned')})  # 🔥 JSON 응답
        else:
            # 첫 번째 호출 이후에는 호출자의 좌표가 큐에 쌓입니다.
            if len(summoner_queue) == 2:
                return jsonify({'redirect': url_for('ROS_no_more_summon')})  # 🔥 JSON 응답
            else:
                task_queue.appendleft(position)
                summoner_queue.append(position)
                logger.info("Redirecting to ROS_robot_is_summoned,,,,,,,,")
                return jsonify({'redirect': url_for('ROS_robot_is_summoned')})  # 🔥 JSON 응답
    
    except Exception as e:
        print("Error: {}".format(e))  # 서버에서 발생한 오류를 로그로 출력
        return jsonify({'error': str(e)}), 500  # 🔥 JSON 에러 응답
        #return 'Error occurred: {}'.format(e), 500  # 오류를 클라이언트에 반환


@app.route('/ROS_robot_is_summoned')
def ROS_robot_is_summoned():
    logger.info("will render ROS_robot_is_summoned.html")
    return render_template('ROS_robot_is_summoned.html')  
 
@app.route('/ROS_no_more_summon')
def ROS_robot_is_summoned():
    logger.info("will render ROS_robot_is_summoned.html")
    return render_template('ROS_robot_is_summoned.html')  

#user가 물건 다 담고 서랍 닫은후에, 목적지 좌표도 알려주면 실행되는 라우팅
@app.route('/submit_text', methods=['POST'])
def submit_text():
    global is_submit_done
    # 사용자로부터 입력 받은 이름
    name = request.form['name']

    # DB에서 해당 이름의 부서 정보와 좌표 조회
    conn = sqlite3.connect(DB_PATH)
    cursor = conn.cursor()

    try:
        # 직원 정보 조회
        # employees 테이블에서 이름이 ? 인 사람의 department(부서)를 선택하겠다.
        cursor.execute("SELECT department FROM employees WHERE name = ?", (name,))
        
        employee = cursor.fetchone()

        if not employee:
            return jsonify({'error': '해당 이름의 직원이 존재하지 않습니다.'}), 404
        else : 
            logger.info('DB조회해서 직원의 이름으로 부서 찾기 성공')

        department = employee[0]

        # 부서 좌표 정보 조회
        # departments 테이블에서 부서가 ? 인 사람의 x,y,z,w를 선택하겠다.
        cursor.execute("SELECT pos_x, pos_y, ori_z, ori_w FROM departments WHERE department = ?", (department,))
        coords = cursor.fetchone()

        if not coords:
            return jsonify({'error': '해당 부서의 좌표 정보가 존재하지 않습니다.'}), 404
        else :
            logger.info('DB조회해서 부서 이름으로 좌표 찾기 성공')

        # 좌표를 기반으로 PoseStamped 객체 초기화
        position = PoseStamped()
        position.header.frame_id = "map"
        position.header.stamp = rospy.Time.now()
        position.pose.position.x = coords[0]
        position.pose.position.y = coords[1]
        position.pose.orientation.z = coords[2]
        position.pose.orientation.w = coords[3]

        # position 객체에서 x, y, z, w 값을 추출하여 로그 출력
        x = position.pose.position.x
        y = position.pose.position.y
        z = position.pose.orientation.z
        w = position.pose.orientation.w

        logger.info("서버의 DB 조회 성공 !!! 이름: {}, 부서: {}, 좌표: (x: {}, y: {}, z: {}, w: {})".format(
            name, department, x, y, z, w))
        
        task_queue.append(position)
        is_submit_done = True

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
            'redirect_url': url_for('index')  # 리디렉션 URL
        })
        

    except sqlite3.Error as e:
        logger.info("DB조회 과정에서 오류 발생")
        return jsonify({'error': 'DB 에러 발생: {}'.format(e)}), 500

    finally:
        conn.close()


if __name__ == '__main__':
     #-----------------------------------------LOGGING SYSTEM------------------------------------------------------
     # 로깅 설정
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    logger = logging.getLogger()
    logger.setLevel(logging.DEBUG)
     #-----------------------------------------LOGGING SYSTEM------------------------------------------------------


    rospy.init_node('flask_server', anonymous=True)
    logger.info("ROS Node Initialized!")


     #-----------------------------------------LOGGING SYSTEM------------------------------------------------------
    # 로깅 재설정 (ROS 노드가 로깅을 덮어쓰는 문제 해결하기위해)
    logger.handlers = []  # 기존 핸들러 제거
    logging.basicConfig(level=logging.INFO, format="%(asctime)s - %(levelname)s - %(message)s")
    
     #-----------------------------------------LOGGING SYSTEM------------------------------------------------------

    # ROS 스레드 시작, target에 ros_spin(위에보면 rospy.spin() 담겨있음.)적어두면, 아래 세줄로 모든 ros섭스크라이브 콜백들이 쓰레드로 작동함.
    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True  # 메인 스레드 종료 시 함께 종료
    ros_thread.start()

    logger.info("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!서버가 실행됩니다!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!")  
    # Flask 서버 실행
    run_flask()