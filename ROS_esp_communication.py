import rospy
import requests
import json
from std_msgs.msg import String

class ESP8266Communication:
    def __init__(self):
        # ESP8266의 IP 주소와 엔드포인트
        self.esp8266_url = "http://192.168.1.107/mcu_1"  
        
        # "/esp8266_command" 주제에 대한 subscriber 추가
        rospy.Subscriber("/esp8266_command", String, self.handle_esp_command)

    def send_command(self, command):
        payload = {"command": command}
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(self.esp8266_url, data=json.dumps(payload), headers=headers, timeout=5)
            if response.status_code == 200:
                rospy.loginfo(f"{command} Command sent to ESP successfully: {response.text}")
            else:
                rospy.logwarn(f"Failed to send {command} command to ESP8266")
        except requests.exceptions.RequestException as e:
            rospy.logerr(f"Error sending {command} command to ESP8266: {e}")

    def handle_esp_command(self, msg):
        # 메시지가 "Arrival"이면 Arrival 명령을 ESP8266으로 보냄
        if msg.data == "Arrival":
            self.send_command("Arrival")
        elif msg.data == "Departure":
            self.send_command("Departure")

if __name__ == '__main__':
    try:
        rospy.init_node('esp8266_communication', anonymous=True)
        esp8266_comm = ESP8266Communication()
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("ESP8266 communication node terminated")