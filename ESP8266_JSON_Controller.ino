#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Adafruit_NeoPixel.h>
#include <MFRC522.h>
#include <ArduinoJson.h>

// Wi-Fi 설정
const char* ssid = "MERCUSYS_A452";       // Wi-Fi SSID
const char* password = "15919724";        // Wi-Fi 비밀번호

//고정IP설정(ESP8266이 전원 연결시 마다 로컬ip가 바뀌는 문제 해결위함.)
IPAddress local_IP(192, 168, 1, 50); // 고정 IP 주소
IPAddress gateway(192, 168, 1, 1); // 게이트웨이 주소
IPAddress subnet(255, 255, 255, 0); //서브넷마스크

ESP8266WebServer server(80);              // 웹 서버 객체 생성

// 네오픽셀 설정
const int NEOPIXEL_PIN = D8;              // 네오픽셀 데이터 핀
const int NUM_PIXELS = 12;                // 네오픽셀 LED 개수

Adafruit_NeoPixel strip(NUM_PIXELS, NEOPIXEL_PIN, NEO_GRB + NEO_KHZ800);

// RFID 설정
#define RFID_SDA_PIN D2 // RFID SDA 핀
#define RFID_RST_PIN D3 // RFID RST 핀
MFRC522 rfid(RFID_SDA_PIN, RFID_RST_PIN);

// 특정 색으로 모든 네오픽셀 설정
void setAllPixels(uint8_t red, uint8_t green, uint8_t blue) {
  for (int i = 0; i < NUM_PIXELS; i++) {
    strip.setPixelColor(i, strip.Color(red, green, blue));
  }
  strip.show(); // 변경 사항 적용
}

// 초록색 LED 켜기
void Green_LED_ON() {
  setAllPixels(0, 255, 0); // 초록색 네오픽셀 켜기
  Serial.println("초록 LED가 켜졌습니다.");
}

// 빨간색 LED 켜기
void RED_LED_ON() {
  setAllPixels(255, 0, 0); // 빨간색 네오픽셀 켜기
  Serial.println("빨간 LED가 켜졌습니다.");
}

// RFID 카드 UID 체크 함수
bool checkRFIDCardUID() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    byte expectedUID[] = {0x36, 0xE8, 0x33, 0x03}; // UID 설정
    if (rfid.uid.size != sizeof(expectedUID)) {
      return false;
    }
    for (byte i = 0; i < rfid.uid.size; i++) {
      if (rfid.uid.uidByte[i] != expectedUID[i]) {
        return false;
      }
    }
    rfid.PICC_HaltA();
    return true;
  }
  return false;
}

// 리니어 액추에이터 제어 함수 (추후 구현 필요)
void Linear_LOCK_OFF() {
  Serial.println("리니어 액추에이터 잠금 해제");
  // TODO: 리니어 액추에이터 잠금 해제 코드 추가
}

void Linear_LOCK_ON() {
  Serial.println("리니어 액추에이터 잠금");
  // TODO: 리니어 액추에이터 잠금 코드 추가
}

void handle_LOCK_LED_Control() {
  if (!server.hasArg("plain")) {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"No payload\"}");
    return;
  }

  String body = server.arg("plain");
  Serial.println("Received payload: " + body);

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, body);

  if (error) {
    Serial.println("JSON 파싱 실패: " + String(error.c_str()));
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
    return;
  }

  const char* command = doc["command"];
  bool rfidAuthorized = false;

  // RFID 카드 태그를 기다림
  unsigned long startTime = millis(); // 태그 기다리기 시작하는 시간 등록, handle_LOCK_LED_Control 함수가 한번 실행될때 등록됨.
  unsigned long timeout = 10000; // 10초 동안 rfid 카드 태그 대기
  while (millis() - startTime < timeout) { //조건문안의 millis는 계속 갱신(증가)됨.
    if (checkRFIDCardUID()) {
      rfidAuthorized = true;
      break;
    }
    delay(100); // RFID 확인 간격
  }

  if (!rfidAuthorized) {
    server.send(403, "application/json", "{\"status\":\"error\",\"message\":\"RFID Unauthorized or Timeout\"}");
    return;
  }

  // RFID 인증 성공 후 LED 및 리니어 액추에이터 제어
  if (strcmp(command, "Arrival") == 0) {
    Green_LED_ON();
    Linear_LOCK_OFF();
    server.send(200, "application/json", "{\"status\":\"success\",\"action\":\"LOCKOFF\"}");
  } else if (strcmp(command, "Departure") == 0) {
    RED_LED_ON();
    Linear_LOCK_ON();
    server.send(200, "application/json", "{\"status\":\"success\",\"action\":\"LOCKON\"}");
  } else {
    server.send(400, "application/json", "{\"status\":\"error\",\"message\":\"Unknown command\"}");
  }
}

// Wi-Fi 연결
void connectToWiFi() {
  WiFi.config(local_IP, gateway, subnet);
  WiFi.begin(ssid, password);
  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(1000);
    Serial.print(".");
    attempts++;
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nConnected to WiFi");
    Serial.println("IP address: " + WiFi.localIP().toString());
  } else {
    Serial.println("\nWiFi 연결 실패! 재시도 중...");
    connectToWiFi(); // 재귀 호출로 재시도
  }
}

void setup() {
  Serial.begin(115200);

  // 네오픽셀 초기화
  strip.begin();
  strip.show();
  setAllPixels(0, 0, 0); // 초기화 상태로 LED 끄기

  // RFID 초기화
  SPI.begin();
  rfid.PCD_Init();

  // Wi-Fi 연결
  connectToWiFi();

  // 서버 핸들러 등록
  server.on("/mcu_1", HTTP_POST, handle_LOCK_LED_Control);
  server.begin();
  Serial.println("Server started");
}

void loop() {
  server.handleClient(); // 클라이언트 요청 처리
}
