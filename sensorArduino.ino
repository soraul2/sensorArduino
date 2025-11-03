/*
 * 스마트팜 수질 및 환경 센서 모니터링 코드 (MQTT 통합 버전)
 * (MIRACLE FARM - millis() 기반 비동기 구조)
 *
 * [수정됨] R4 보드의 '내장 RTC'를 사용하도록 변경되었습니다. (RTC.h 라이B
 *
 * [!중요!]
 * 1. [필수] 이 코드는 "Arduino UNO R4 Boards" 패키지에 포함된 'RTC.h'를 사용합니다.
 * 2. [필수] '외부 RTC 모듈(DS3231 등)'은 더 이상 필요하지 않습니다.
 * 3. [복원] 인터넷 시간(NTP) 동기화 기능이 복원되었습니다.
 * - WiFi 연결 시 자동으로 RTC 시간이 인터넷 시간과 동기화됩니다.
 *
 * [자동 감지 센서]
 * - 부팅 시 I2C 버스를 스캔하여 SCD41, VEML7700 센서를 찾습니다.
 * - SCD41이 감지되면: SCD41 모드 (온도/습도/CO2)
 * - SCD41이 없으면: DHT11 모드 (온도/습도) (3번 핀 연결 가정)
 * - VEML7700이 감지되면: 광량 측정 활성화
 * - VEML7700이 없으면: 광량 측정 비활성화
 */

// --- 0. 사용자 설정 (필수!) ---
#define YOUR_WIFI_SSID "Farm_2.4g" // 여기에 WiFi SSID 입력
#define YOUR_WIFI_PASS "20240603" // 여기에 WiFi 비밀번호 입력
#define YOUR_DEVICE_SERIAL_NUMBER "PLANTOFACTORY_SENSOR_A001" // DB에 등록할 장치 시리얼
// --- 0. 설정 끝 ---


// --- 1. 라이브러리 포함 ---
// WiFi / MQTT 라이브러리
#include <WiFiS3.h>
#include <WiFiSSLClient.h>      // Port 8883 (SSL)용
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>        // JSON 페이로드 생성용
#include <math.h>
#include <Wire.h>               // --- I2C 공용 라이브러리 ---

// --- [RTC 라이브러리 변경] ---
#include "RTC.h"       // [변경] R4 보드 내장 RTC 라이브러리 (가이드 기반)
// ---

// 공통 센서 라이브러리
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_VEML7700.h>  // --- VEML7700 (Lux) 라이브러리 ---

// --- 자동 감지를 위해 두 센서 라이브러리 모두 포함 ---
#include <SensirionI2cScd4x.h> 
#include <DHT.h>
// ---


// --- 2. 핀 번호 정의 ---
#define ONE_WIRE_BUS_PIN 2      // DS18B20
#define TDS_PIN A0
#define PH_PIN A1
#define PH_RELAY_VCC_PIN 4
#define PH_RELAY_GND_PIN 5
#define RANDOM_SEED_PIN A5      
#define DHT_PIN 3               // DHT11 사용 시 3번 핀 (고정)
// (SCD41, VEML7700은 I2C 핀(SDA/SCL)을 사용함)


// --- 3. MQTT 브로커 정보 ---
const char* broker = "eafc441602df4e36aed5f15ad6df2e4c.s1.eu.hivemq.cloud";
const int port = 8883;
const char* mqttUser = "daesin_302";
const char* mqttPass = "!Ds123456"; 
const char* deviceSerial = "PLANTOFACTORY_SENSOR_A001";

// --- 4. 객체 생성 ---
// WiFi / MQTT 객체
WiFiSSLClient sslClient;
MqttClient mqttClient(sslClient);

// --- [RTC 객체 변경] ---
// (RTC.h 라이브러리는 별도 객체 생성이 필요 없으며, 'RTC' static 클래스 사용)
// ---

// 공통 센서 객체
OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);
Adafruit_VEML7700 veml = Adafruit_VEML7700(); // VEML7700 객체

// --- 두 센서 객체 모두 생성 ---
SensirionI2cScd4x scd4x;
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);
// ---


// --- 5. 전역 변수 ---
// (이전과 동일)
float airTemperature = 0.0, airHumidity = 0.0, absoluteHumidity = 0.0;
float waterTemperature = 25.0, tdsValue = 0.0, ecValue = 0.0, phValue = 7.0;
float lux = 0.0; 
float co2 = 0.0; 
bool isSCD41_active = false; 
bool isVEML7700_active = false; 
unsigned long previousDhtMillis = 0;
unsigned long previousLightMillis = 0; 
const long dhtInterval = 2000; 
const long lightInterval = 2000; 
unsigned long previousWaterSensorMillis = 0;
const long phStabilizeTime = 3000;
const long cycleInterval = 10000; 
enum WaterSensorState { STATE_READ_TDS_TEMP, STATE_WAIT_PH_STABILIZE, STATE_READ_PH, STATE_WAIT_NEXT_CYCLE };
WaterSensorState currentWaterState = STATE_READ_TDS_TEMP;
const float VOLTAGE_REFERENCE = 5.0;
const float ADC_RESOLUTION = 16383.0; // 14비트


void setup() {
  Serial.begin(9600);
  Serial.println("--- 스마트팜 센서 시스템 (MQTT) 시작 ---");
  
  // --- 1. ADC 및 핀 설정 ---
  analogReadResolution(14);
  randomSeed(analogRead(RANDOM_SEED_PIN)); 
  Serial.println("ADC 14비트, 랜덤 시드 설정 완료.");

  pinMode(PH_RELAY_VCC_PIN, OUTPUT);
  pinMode(PH_RELAY_GND_PIN, OUTPUT);
  digitalWrite(PH_RELAY_VCC_PIN, LOW); // pH 센서 OFF
  digitalWrite(PH_RELAY_GND_PIN, LOW);

  // --- 2. 센서 시작 ---
  sensors.begin(); // DS18B20 시작
  Wire.begin();    // I2C 버스 시작 (SCD41, VEML7700, RTC 공용)

  // --- [RTC 시작 로직 변경] ---
  Serial.println("내장 RTC 시작 중...");
  RTC.begin(); // R4 내장 RTC 초기화
  // (WiFi 연결 후 syncRtcTime()에서 시간 설정)
  // ---

  // VEML7700 (Lux 센서) 초기화 및 자동 감지
  Serial.println("VEML7700(I2C) 센서 스캔 중...");
  if (!veml.begin()) {
    Serial.println("VEML7700 감지 실패. 광량 측정을 비활성화합니다.");
    isVEML7700_active = false;
  } else {
    Serial.println("VEML7700 감지 성공. 광량 측정을 활성화합니다.");
    isVEML7700_active = true;
  }

  // --- [공기 센서 자동 감지] ---
  Serial.println("SCD41(I2C) 센서 스캔 중...");
  uint16_t error;
  char errorMessage[256];
  
  scd4x.begin(Wire, SCD40_I2C_ADDR_62); 
  
  error = scd4x.stopPeriodicMeasurement(); // 기존 측정 중지 (초기화)
  error = scd4x.startPeriodicMeasurement(); // 새 측정 시작

  if (error) {
    // --- SCD41 실패 -> DHT11 모드로 전환 ---
    Serial.print("SCD41 감지 실패. DHT11 모드로 전환합니다. (Pin 3)");
    isSCD41_active = false;
    dht.begin(); // DHT11 초기화
  } else {
    // --- SCD41 성공 ---
    Serial.println("SCD41 감지 성공. SCD41 모드로 시작합니다.");
    isSCD41_active = true;
  }
  // --- [자동 감지 끝] ---

  // --- 3. WiFi 연결 ---
  connectWiFi();
  
  // --- 4. [복원] RTC 시간 동기화 ---
  syncRtcTime();

  // --- 5. MQTT 연결 ---
  connectMQTT();
}


void loop() {
  // 현재 시간
  unsigned long currentMillis = millis();

  // --- [중요] MQTT 클라이언트 연결 유지 및 메시지 수신 ---
  mqttClient.poll();

  // --- 작업 0: MQTT 연결 끊김 감지 및 재연결 ---
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.println("경고: MQTT 연결 끊김. 5초 후 재연결 시도...");
    delay(5000); // 5초 대기
    connectMQTT(); // 재연결 시도
  }

  // --- 작업 1: 공기 센서 측정 (자동 감지된 센서 호출) ---
  if (isSCD41_active) {
    manageScd41Sensor(currentMillis); // SCD41 모드
  } else {
    manageDhtSensor(currentMillis);   // DHT11 모드
  }

  // --- 작업 2: 수질 측정 상태 머신 ---
  manageWaterSensors(currentMillis);

  // --- 작업 3: 광량 측정 (VEML7700, 감지된 경우에만) ---
  if (isVEML7700_active) {
    manageLightSensor(currentMillis);
  }
}


// --- WiFi / MQTT / RTC 함수 ---

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
 * @brief [복원/수정됨] WiFi(NTP)를 통해 내장 RTC 시간을 동기화합니다. (RTC.h 기반)
 */
void syncRtcTime() {
  Serial.println("RTC 상태 확인 중...");
  if (!RTC.isRunning()) {
    Serial.println("RTC가 실행 중이 아닙니다. 인터넷 시간(NTP)으로 동기화 시도...");
    
    unsigned long epochTime = 0;
    int attempts = 0;

    do {
      // WiFi.getTime()은 Unix time(초)을 반환합니다.
      epochTime = WiFi.getTime(); 
      
      if (epochTime == 0) {
        Serial.println("시간 동기화 실패. 2초 후 재시도...");
        delay(2000);
        attempts++;
      }
      if(attempts > 5) {
        Serial.println("NTP 동기화 실패. RTC 기본 시간으로 작동합니다.");
        return; // 5회 실패 시 포기
      }
    } while (epochTime == 0);

    Serial.println("NTP 시간(Epoch)을 RTCTime 객체로 변환합니다...");
    
    // (KST = UTC+9, 9시간(32400초) 추가)
    epochTime += 32400; // 한국 시간으로 변환
    
    int sec = epochTime % 60;
    epochTime /= 60;
    int min = epochTime % 60;
    epochTime /= 60;
    int hr = epochTime % 24;
    epochTime /= 24;

    // 1970년 1월 1일 (목요일) 기준 일수
    long days = epochTime; 
    int dayOfWeek = (days + 4) % 7; // 요일 (0=일요일, ..., 4=목요일)
    
    int yr = 1970;
    while (true) {
        bool isLeap = (yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0);
        int daysInYear = isLeap ? 366 : 365;
        if (days < daysInYear) break;
        days -= daysInYear;
        yr++;
    }

    int mo = 1;
    const int daysInMonth[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    while (true) {
        int daysThisMonth = (mo == 2 && ((yr % 4 == 0 && yr % 100 != 0) || (yr % 400 == 0))) ? 29 : daysInMonth[mo - 1];
        if (days < daysThisMonth) break;
        days -= daysThisMonth;
        mo++;
    }
    
    int dy = days + 1; // 1일 기반
    
    // --- [오류 수정] ---
    // RTCTime 생성자는 8개의 인자가 필요합니다. (마지막: 서머타임)
    // 'SAVING_TIME_NOT_ACTIVE' -> 'SAVING_TIME_INACTIVE'
    RTCTime timeToSet(dy, (Month)mo, yr, hr, min, sec, (DayOfWeek)dayOfWeek, SaveLight::SAVING_TIME_INACTIVE);
    // --- [수정 끝] ---

    RTC.setTime(timeToSet); // setEpoch 대신 setTime 사용
    
    Serial.println("RTC 시간 동기화 성공!");

  } else {
    Serial.println("RTC가 이미 실행 중입니다. (시간 동기화 건너뜀)");
  }
  
  Serial.print("현재 RTC 시간: ");
  Serial.println(getIsoTimestamp()); // 동기화된 시간 확인
}


// --- [오류 수정] ---
// 'Month2int' 헬퍼 함수 삭제. (RTC.h 라이브러리에 이미 포함되어 있음)
// --- [수정 끝] ---


/**
 * @brief [수정됨] 현재 시간을 ISO8601 형식 문자열로 반환합니다. (RTC.h 사용)
 */
String getIsoTimestamp() {
  if (!RTC.isRunning()) { // RTC가 작동 안하면
    return "1970-01-01T00:00:00Z"; // 기본 타임스탬프 반환
  }
  
  RTCTime currenttime;
  RTC.getTime(currenttime); // R4 내장 RTC에서 시간 가져오기
  
  char timestamp[30];
  // YYYY-MM-DDTHH:MM:SSZ (UTC 기준)
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
          currenttime.getYear(),
          Month2int(currenttime.getMonth()), // 헬퍼 함수 사용 (라이브러리의 함수 호출)
          currenttime.getDayOfMonth(),
          currenttime.getHour(),
          currenttime.getMinutes(),
          currenttime.getSeconds());
  return String(timestamp);
}

/**
 * @brief MQTT 브로커에 연결합니다.
 */
void connectMQTT() {
  String clientId = "ArduinoClient-";
  clientId += String(random(0, 100000));
  
  Serial.print("MQTT 브로커 연결 시도 중: ");
  Serial.println(broker);
  Serial.print("클라이언트 ID: ");
  Serial.println(clientId);

  mqttClient.setUsernamePassword(mqttUser, mqttPass);
  mqttClient.setId(clientId.c_str());

  while (!mqttClient.connect(broker, port)) {
    Serial.print("MQTT 연결 실패, 오류: ");
    Serial.print(mqttClient.connectError());
    Serial.println(" 5초 후 재시도...");
    delay(5000);
  }

  Serial.println("MQTT 연결 성공!");
}

/**
 * @brief JSON 페이로드를 생성하여 MQTT 토픽으로 전송합니다. [타임스탬프 추가됨]
 */
void publishMqttMessage(const String& topic, float value, const String& timestamp) {
  if (!mqttClient.connected()) {
    Serial.println("MQTT 메시지 전송 실패 (연결 끊김)");
    return;
  }
  
  StaticJsonDocument<256> doc; 
  doc["serial"] = deviceSerial;
  doc["value"] = value;
  doc["timestamp"] = timestamp; 

  char jsonBuffer[256];
  serializeJson(doc, jsonBuffer);

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
 * @brief 작업 1 (SCD41): SCD41 센서 관리 함수 [MQTT 호출 수정됨]
 */
void manageScd41Sensor(unsigned long currentMillis) { 
  if (currentMillis - previousDhtMillis >= dhtInterval) {
    previousDhtMillis = currentMillis;

    // --- SCD41 측정 로직 ---
    uint16_t error;
    char errorMessage[256];
    
    uint16_t co2_int = 0; 
    error = scd4x.readMeasurement(co2_int, airTemperature, airHumidity);
    co2 = (float)co2_int;

    if (error) {
      Serial.print("오류: SCD41 센서 값을 읽을 수 없습니다! ");
      errorToString(error, errorMessage, 256);
      Serial.println(errorMessage);
      co2 = 0.0;
      airTemperature = 0.0;
      airHumidity = 0.0;
      absoluteHumidity = 0.0;
      return; 
    } else if (co2 == 0) {
      Serial.println("SCD41 센서 값 읽는 중... (초기화 중일 수 있음)");
      return; 
    }

    // 절대 습도 계산
    calculateAbsoluteHumidity();
    
    // [시리얼 출력]
    Serial.println("--- [공기 환경 (SCD41)] ---");
    Serial.print("CO2: "); Serial.print(co2); Serial.println(" ppm");
    Serial.print("온도: "); Serial.print(airTemperature); Serial.println(" *C");
    Serial.print("상대 습도: "); Serial.print(airHumidity); Serial.println(" %");
    Serial.print("절대 습도: "); Serial.print(absoluteHumidity); Serial.println(" g/m³");
    Serial.println("--------------------");

    // [MQTT 전송 트리거]
    String now = getIsoTimestamp(); // --- [수정] 내장 RTC 시간 가져오기 ---
    publishMqttMessage("plantofactory/sensor/air_co2", co2, now); 
    publishMqttMessage("plantofactory/sensor/air_temperature", airTemperature, now);
    publishMqttMessage("plantofactory/sensor/air_humidity_relative", airHumidity, now);
    publishMqttMessage("plantofactory/sensor/air_humidity_absolute", absoluteHumidity, now);
  }
}

/**
 * @brief 작업 1 (DHT11): DHT11 센서 관리 함수 [MQTT 호출 수정됨]
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
    Serial.println("--- [공기 환경 (DHT11)] ---");
    Serial.print("온도: "); Serial.print(airTemperature); Serial.println(" *C");
    Serial.print("상대 습도: "); Serial.print(airHumidity); Serial.println(" %");
    Serial.print("절대 습도: "); Serial.print(absoluteHumidity); Serial.println(" g/m³");
    Serial.println("--------------------");

    // [MQTT 전송 트리거]
    String now = getIsoTimestamp(); // --- [수정] 내장 RTC 시간 가져오기 ---
    publishMqttMessage("plantofactory/sensor/air_temperature", airTemperature, now);
    publishMqttMessage("plantofactory/sensor/air_humidity_relative", airHumidity, now);
    publishMqttMessage("plantofactory/sensor/air_humidity_absolute", absoluteHumidity, now);
  }
}



/**
 * @brief 작업 2: 수질 센서 상태 머신 (MQTT 전송 추가) [MQTT 호출 수정됨]
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
      String now = getIsoTimestamp(); // --- [수정] 내장 RTC 시간 가져오기 ---
      if (waterTemperature != DEVICE_DISCONNECTED_C) { // DS18B20이 정상일 때만 전송
         publishMqttMessage("plantofactory/sensor/water_temperature", waterTemperature, now);
      }
      publishMqttMessage("plantofactory/sensor/water_ec", ecValue, now);
      publishMqttMessage("plantofactory/sensor/water_tds", tdsValue, now);

      // 다음 상태로 전환
      currentWaterState = STATE_WAIT_PH_STABILIZE;
      previousWaterSensorMillis = currentMillis;
      break;
    }

    case STATE_WAIT_PH_STABILIZE: {
      //Serial.println("[상태 2] pH 센서 전원 ON. 안정화 대기 중..."); 
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
      String now = getIsoTimestamp(); // --- [수정] 내장 RTC 시간 가져오기 ---
      publishMqttMessage("plantofactory/sensor/water_ph", phValue, now);

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

/**
 * @brief 작업 3: 광량(Lux) 센서 관리 함수 (VEML7700) [MQTT 호출 수정됨]
 */
void manageLightSensor(unsigned long currentMillis) {
  if (currentMillis - previousLightMillis >= lightInterval) {
    previousLightMillis = currentMillis;

    lux = veml.readLux();

    // [시리얼 출력]
    Serial.println("--- [광량 (VEML7700)] ---");
    Serial.print("Lux: "); Serial.print(lux); Serial.println(" lx");
    Serial.println("-------------------------");

    // [MQTT 전송 트리거]
    String now = getIsoTimestamp(); // --- [수정] 내장 RTC 시간 가져T(MQTT 통합):MIRACLE_FARM_Sensors_MQTT.ino
    publishMqttMessage("plantofactory/sensor/light_lux", lux, now);
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
  const float VOLTAGE_PH6_86 = 2.813; 
  const float VOLTAGE_PH4_01 = 3.266; 
  const float VOLTAGE_PH9_18 = 2.405;

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
 * @brief 절대 습도(g/m³)를 계산합니다. (SCD41/DHT11 공용)
 */
void calculateAbsoluteHumidity() {
  if (isnan(airTemperature) || isnan(airHumidity) || airHumidity == 0) { 
    absoluteHumidity = 0.0;
    return;
  }
  double svp = 6.112 * exp((17.67 * airTemperature) / (airTemperature + 243.5));
  double avp = svp * (airHumidity / 100.0);
  absoluteHumidity = (avp * 216.74) / (airTemperature + 273.15);
}

