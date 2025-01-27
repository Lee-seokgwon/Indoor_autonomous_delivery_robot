import rospy
import requests
import json
from std_msgs.msg import String


#----------------------------------실제로 ESP8266과 통신해서 로봇의 움직임 상태 쏴주는 노드----------------------------

class ESP8266Communication:
    def __init__(self):
        # ESP8266의 IP 주소 (D1 R1의 펌웨어 상에서 아래의의 IP를 고정 IP로 할당받도록 설정함.)
        self.esp8266_url = "http://192.168.1.107/mcu_1"  
        
        # "/esp8266_command" 토픽을 구독독하면 로봇 움직임 상태를 전달받을 수 있다.
        rospy.Subscriber("/esp8266_command", String, self.handle_esp_command)

    def send_command(self, command):
        payload = {"command": command}
        headers = {"Content-Type": "application/json"}
        
        try:
            # https 프로토콜 이용해서 실제로 esp8266에게 command를 전송
            response = requests.post(self.esp8266_url, data=json.dumps(payload), headers=headers, timeout=5)
            # 정상적으로 command 전송시 response.status_code에 200이 저장된다.
            if response.status_code == 200:
                rospy.loginfo(f"{command} 명령이 ESP8266으로 성공적으로 전달되었습니다.: {response.text}")
            else:
                rospy.logwarn(f" {command} 명령을 ESP8266 전달하지 못했습니다.")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"{command} 명령을 ESP8266으로 보내는데 에러가 발생하였습니다.: {e}")

    def handle_esp_command(self, msg):
        # 메시지가 "Arrival"이면 Arrival 명령을 ESP8266으로 보냄
        if msg.data == "Arrival":
            self.send_command("Arrival")
        elif msg.data == "Departure":
            self.send_command("Departure")

if __name__ == '__main__':
    try:
        # 노드 이름을 esp8266_communication 으로 설정하고 초기화
        rospy.init_node('esp8266_communication', anonymous=True)
        # ESP8266Communication 객체 반환
        esp8266_comm = ESP8266Communication()
        # 지금부터 pub,sub 전부 할 수 있게되고 스레드는 pub,sub 루프만 관리
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ESP8266 communicationode 노드가 예기치 못하게 종료되었습니다.")