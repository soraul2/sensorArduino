/*
 * 스마트팜 수질 및 환경 센서 모니터링 코드 (MQTT 통합 버전)
 * (MIRACLE FARM - millis() 기반 비동기 구조)
 *
 * [MQTT 기능]
 * - 보드: Arduino UNO R4 WiFi
 * - 라이브러리: WiFiS3, ArduinoMqttClient, ArduinoJson
 * - 브로커: HiveMQ Cloud (eafc441602df4e36aed5f15ad6df2e4c.s1.eu.hivemq.cloud)
 * - 포트: 8883 (SSL/TLS 사용)
 * - 페이로드: {"serial": "YOUR_SERIAL", "value": 12.34} 형식의 JSON
 * - 토픽: plantofactory/sensor/[센서명]
 *
 * [기존 센서 기능]
 * - DHT11, DS18B20, TDS, pH
 * - 릴레이를 이용한 pH/TDS 간섭 회피
 * - 14비트 ADC 및 3점 교정 적용
 *
 * [!중요!]
 * 1. Arduino IDE의 '라이브러리 매니저'에서
 * "ArduinoMqttClient" 와 "ArduinoJson" 을 '반드시' 설치해야 합니다.
 * 2. 아래 '--- 0. 사용자 설정 (필수!) ---' 섹션에
 * WiFi 정보와 장치 시리얼 번호를 입력하세요.
 */

// --- 0. 사용자 설정 (필수!) ---
#define YOUR_WIFI_SSID "daesin_302" // 여기에 WiFi SSID 입력
#define YOUR_WIFI_PASS "ds123456" // 여기에 WiFi 비밀번호 입력
#define YOUR_DEVICE_SERIAL_NUMBER "PLANTOFACTORY_SENSOR_001" // DB에 등록할 장치 시리얼 (예: "MIRACLE_FARM_001")
// --- 0. 설정 끝 ---


// --- 1. 라이브러리 포함 ---
// WiFi / MQTT 라이브러리
#include <WiFiS3.h>
#include <WiFiSSLClient.h>      // Port 8883 (SSL)용
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>        // JSON 페이로드 생성용

// 센서 라이브러리
#include <OneWire.h>
#include <DallasTemperature.h>
#include <DHT.h>
#include <math.h>

// --- 2. 핀 번호 정의 ---
#define ONE_WIRE_BUS_PIN 2      // DS18B20
#define DHT_PIN 3               // DHT11
#define TDS_PIN A0
#define PH_PIN A1
#define PH_RELAY_VCC_PIN 4
#define PH_RELAY_GND_PIN 5
#define RANDOM_SEED_PIN A5      // 랜덤 클라이언트 ID 생성을 위한 노이즈 핀

// --- 3. MQTT 브로커 정보 ---
const char* broker = "eafc441602df4e36aed5f15ad6df2e4c.s1.eu.hivemq.cloud";
const int port = 8883;
const char* mqttUser = "daesin_302";
const char* mqttPass = "!Ds123456";
const char* deviceSerial = YOUR_DEVICE_SERIAL_NUMBER;

// --- 4. 객체 생성 ---
// WiFi / MQTT 객체
WiFiSSLClient sslClient; // R4 WiFi의 SSL 클라이언트
MqttClient mqttClient(sslClient);

// 센서 객체
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);
DHT dht(DHT_PIN, DHT11);

// --- 5. 전역 변수 ---
// 센서 값
float airTemperature = 0.0, airHumidity = 0.0, absoluteHumidity = 0.0;
float waterTemperature = 25.0, tdsValue = 0.0, ecValue = 0.0, phValue = 7.0;

// 타이밍
unsigned long previousDhtMillis = 0;
const long dhtInterval = 2000;
unsigned long previousWaterSensorMillis = 0;
const long phStabilizeTime = 3000;
const long cycleInterval = 10000; // MQTT 전송을 고려해 10초 유지 (필요시 늘려도 됨)

// 상태 머신
enum WaterSensorState { STATE_READ_TDS_TEMP, STATE_WAIT_PH_STABILIZE, STATE_READ_PH, STATE_WAIT_NEXT_CYCLE };
WaterSensorState currentWaterState = STATE_READ_TDS_TEMP;

// ADC 설정
const float VOLTAGE_REFERENCE = 5.0;
const float ADC_RESOLUTION = 16383.0; // 14비트


void setup() {
  Serial.begin(9600);
  Serial.println("--- 스마트팜 센서 시스템 (MQTT) 시작 ---");

  // --- 1. ADC 및 핀 설정 ---
  analogReadResolution(14);
  randomSeed(analogRead(RANDOM_SEED_PIN)); // 랜덤 클라이언트 ID를 위한 시드 설정
  Serial.println("ADC 14비트, 랜덤 시드 설정 완료.");

  pinMode(PH_RELAY_VCC_PIN, OUTPUT);
  pinMode(PH_RELAY_GND_PIN, OUTPUT);
  digitalWrite(PH_RELAY_VCC_PIN, LOW); // pH 센서 OFF (Active HIGH)
  digitalWrite(PH_RELAY_GND_PIN, LOW);

  // --- 2. 센서 시작 ---
  sensors.begin();
  dht.begin();

  // --- 3. WiFi 연결 ---
  connectWiFi();
  
  // --- 4. MQTT 연결 ---
  // (WiFi 연결이 성공해야 MQTT 연결 시도)
  connectMQTT();
}


void loop() {
  // 현재 시간
  unsigned long currentMillis = millis();

  // --- [중요] MQTT 클라이언트 연결 유지 및 메시지 수신 ---
  // 이 함수는 매 루프마다 호출되어야 합니다.
  mqttClient.poll();

  // --- 작업 0: MQTT 연결 끊김 감지 및 재연결 ---
  // (WiFi는 R4 보드가 자동 재연결을 시도하지만, MQTT는 수동으로 해줘야 함)
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.println("경고: MQTT 연결 끊김. 5초 후 재연결 시도...");
    delay(5000); // 5초 대기
    connectMQTT(); // 재연결 시도
  }

  // --- 작업 1: DHT11 공기 온/습도 측정 (2초마다) ---
  manageDhtSensor(currentMillis);

  // --- 작업 2: 수질 측정 상태 머신 ---
  manageWaterSensors(currentMillis);
}


// --- WiFi / MQTT 연결 함수 ---

/**
 * @brief WiFi에 연결합니다.
 */
void connectWiFi() {
  Serial.print("WiFi 연결 시도 중: ");
  Serial.println(YOUR_WIFI_SSID);

  while (WiFi.status() != WL_CONNECTED) {
    WiFi.begin(YOUR_WIFI_SSID, YOUR_WIFI_PASS);
    delay(5000); // 5초마다 재시도
    Serial.print(".");
  }

  Serial.println("\nWiFi 연결 성공!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());
}

/**
 * @brief MQTT 브로커에 연결합니다.
 */
void connectMQTT() {
  // 랜덤 클라이언트 ID 생성 (충돌 방지)
  String clientId = "ArduinoClient-";
  clientId += String(random(0, 100000));
  
  Serial.print("MQTT 브로커 연결 시도 중: ");
  Serial.println(broker);
  Serial.print("클라이언트 ID: ");
  Serial.println(clientId);

  // 사용자 이름과 비밀번호 설정
  mqttClient.setUsernamePassword(mqttUser, mqttPass);

  // [수정 1] ArduinoMqttClient 라이브러리는 클라이언트 ID를
  // connect() 함수 호출 전에 setId()로 별도 설정해야 합니다.
  mqttClient.setId(clientId.c_str());

  // [수정 2] connect() 함수에는 브로커와 포트만 전달합니다.
  // 연결 시도 (루프)
  while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT 연결 실패, 오류: ");
    Serial.print(mqttClient.connectError());
    Serial.println(" 5초 후 재시도...");
    delay(5000);
  }

  Serial.println("MQTT 연결 성공!");
}

/**
 * @brief JSON 페이로드를 생성하여 MQTT 토픽으로 전송합니다.
 * @param topic 전송할 토픽 (예: "plantofactory/sensor/water_ph")
 * @param value 전송할 센서 값
 */
void publishMqttMessage(const String& topic, float value) {
  // 1. MQTT가 연결되어 있지 않으면 즉시 종료
  if (!mqttClient.connected()) {
    Serial.println("MQTT 메시지 전송 실패 (연결 끊김)");
    return;
  }
  
  // 2. JSON 페이로드 생성 (ArduinoJson)
  // (200바이트는 {"serial":"...", "value":123.45} 를 담기에 충분)
  StaticJsonDocument<200> doc;
  doc["serial"] = deviceSerial;
  doc["value"] = value;

  // 3. JSON을 문자열 버퍼로 직렬화
  char jsonBuffer[200];
  serializeJson(doc, jsonBuffer);

  // 4. MQTT 메시지 전송
  Serial.print("MQTT 전송: [");
  Serial.print(topic);
  Serial.print("] ");
  Serial.print(jsonBuffer);

  mqttClient.beginMessage(topic);
  mqttClient.print(jsonBuffer);
  
  if (mqttClient.endMessage()) {
    Serial.println(" - 성공");
  } else {
    Serial.println(" - 실패");
  }
}


// --- 센서 관리 함수 ---

/**
 * @brief 작업 1: DHT11 센서 관리 함수 (MQTT 전송 추가)
 */
void manageDhtSensor(unsigned long currentMillis) {
  if (currentMillis - previousDhtMillis >= dhtInterval) {
    previousDhtMillis = currentMillis;

    airHumidity = dht.readHumidity();
    airTemperature = dht.readTemperature();

    if (isnan(airHumidity) || isnan(airTemperature)) {
      Serial.println("오류: DHT11 센서 값을 읽을 수 없습니다!");
      return;
    }

    calculateAbsoluteHumidity();
    
    // [시리얼 출력]
    Serial.println("--- [공기 환경] ---");
    Serial.print("온도: "); Serial.print(airTemperature); Serial.println(" *C");
    Serial.print("상대 습도: "); Serial.print(airHumidity); Serial.println(" %");
    Serial.print("절대 습도: "); Serial.print(absoluteHumidity); Serial.println(" g/m³");
    Serial.println("--------------------");

    // [MQTT 전송 트리거]
    publishMqttMessage("plantofactory/sensor/air_temperature", airTemperature);
    publishMqttMessage("plantofactory/sensor/air_humidity_relative", airHumidity);
    publishMqttMessage("plantofactory/sensor/air_humidity_absolute", absoluteHumidity);
  }
}

/**
 * @brief 작업 2: 수질 센서 상태 머신 (MQTT 전송 추가)
 */
void manageWaterSensors(unsigned long currentMillis) {
  switch (currentWaterState) {

    case STATE_READ_TDS_TEMP: {
      Serial.println("[상태 1] TDS 및 수온 측정 시작...");
      digitalWrite(PH_RELAY_VCC_PIN, LOW); // pH OFF
      digitalWrite(PH_RELAY_GND_PIN, LOW);

      // 수온 측정
      sensors.requestTemperatures();
      waterTemperature = sensors.getTempCByIndex(0);
      if (waterTemperature == DEVICE_DISCONNECTED_C) {
        Serial.println("오류: DS18B20 수온 센서 연결 실패!");
        waterTemperature = 25.0; // 오류 시 기본값
      }

      // EC/TDS 측정
      int tdsAnalogValue = analogRead(TDS_PIN);
      calculateTdsAndEc(tdsAnalogValue, waterTemperature);

      // [시리얼 출력]
      Serial.print("  > 수온: "); Serial.print(waterTemperature); Serial.println(" *C");
      Serial.print("  > EC: "); Serial.print(ecValue); Serial.println(" uS/cm");
      Serial.print("  > TDS: "); Serial.print(tdsValue); Serial.println(" ppm");

      // [MQTT 전송 트리거]
      if (waterTemperature != DEVICE_DISCONNECTED_C) { // DS18B20이 정상일 때만 전송
         publishMqttMessage("plantofactory/sensor/water_temperature", waterTemperature);
      }
      publishMqttMessage("plantofactory/sensor/water_ec", ecValue);
      publishMqttMessage("plantofactory/sensor/water_tds", tdsValue);

      // 다음 상태로 전환
      currentWaterState = STATE_WAIT_PH_STABILIZE;
      previousWaterSensorMillis = currentMillis;
      break;
    }

    case STATE_WAIT_PH_STABILIZE: {
      Serial.println("[상태 2] pH 센서 전원 ON. 안정화 대기 중...");
      digitalWrite(PH_RELAY_VCC_PIN, HIGH); // pH ON (Active HIGH)
      digitalWrite(PH_RELAY_GND_PIN, HIGH);

      if (currentMillis - previousWaterSensorMillis >= phStabilizeTime) {
        currentWaterState = STATE_READ_PH;
      }
      break;
    }

    case STATE_READ_PH: {
      Serial.println("[상태 3] pH 측정 시작...");

      // pH 측정
      int phAnalogValue = analogRead(PH_PIN);
      phValue = convertAnalogToPH(phAnalogValue, waterTemperature);

      // [시리얼 출력]
      Serial.print("  > pH: "); Serial.print(phValue);
      Serial.print("\t | EC: "); Serial.print(ecValue); Serial.println(" uS/cm");

      // [MQTT 전송 트리거]
      publishMqttMessage("plantofactory/sensor/water_ph", phValue);

      // pH 센서 전원 차단
      Serial.println("  > pH 측정 완료. 전원 차단.");
      digitalWrite(PH_RELAY_VCC_PIN, LOW);
      digitalWrite(PH_RELAY_GND_PIN, LOW);

      // 다음 상태로 전환
      currentWaterState = STATE_WAIT_NEXT_CYCLE;
      previousWaterSensorMillis = currentMillis;
      break;
    }

    case STATE_WAIT_NEXT_CYCLE: {
      if (currentMillis - previousWaterSensorMillis >= cycleInterval) {
        Serial.println("\n[상태 4] 다음 측정 사이클 시작...\n");
        currentWaterState = STATE_READ_TDS_TEMP;
      }
      break;
    }
  }
}


// --- 센서 계산 함수 ---

/**
 * @brief (PLACEHOLDER) TDS 아날로그 값을 EC(uS/cm)와 TDS(ppm)로 변환합니다.
 */
void calculateTdsAndEc(int analogValue, float temperature) {
  float voltage = analogValue * (VOLTAGE_REFERENCE / ADC_RESOLUTION);
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;
  ecValue = (133.42 * compensationVoltage * compensationVoltage * compensationVoltage 
             - 255.86 * compensationVoltage * compensationVoltage 
             + 857.39 * compensationVoltage);
  float k_factor = 0.5;
  tdsValue = ecValue * k_factor;
}

/**
 * @brief (교정 완료) pH 아날로그 값을 pH 값으로 변환합니다.
 */
float convertAnalogToPH(int analogValue, float temperature) {
  const float VOLTAGE_PH6_86 = 2.833; 
  const float VOLTAGE_PH4_01 = 3.356; 
  const float VOLTAGE_PH9_18 = 2.410;

  float voltage = analogValue * (VOLTAGE_REFERENCE / ADC_RESOLUTION);
  float ph;

  if (voltage > VOLTAGE_PH6_86) {
    float slope_acid = (6.86 - 4.01) / (VOLTAGE_PH6_86 - VOLTAGE_PH4_01);
    ph = 6.86 + (voltage - VOLTAGE_PH6_86) * slope_acid;
  } else {
    float slope_base = (9.18 - 6.86) / (VOLTAGE_PH9_18 - VOLTAGE_PH6_86);
    ph = 6.86 + (voltage - VOLTAGE_PH6_86) * slope_base;
  }
  return ph;
}

/**
 * @brief (새로운 함수) 절대 습도(g/m³)를 계산합니다.
 */
void calculateAbsoluteHumidity() {
  if (isnan(airTemperature) || isnan(airHumidity)) {
    absoluteHumidity = 0.0;
    return;
  }
  double svp = 6.112 * exp((17.67 * airTemperature) / (airTemperature + 243.5));
  double avp = svp * (airHumidity / 100.0);
  absoluteHumidity = (avp * 216.74) / (airTemperature + 273.15);
}

