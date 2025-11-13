/*
 * 스마트팜 수질 및 환경 센서 모니터링 코드 (MQTT 통합 버전)
 * (MIRACLE FARM - millis() 기반 비동기 구조)
 *
 * [수정 v3.8 - pH 온도 보정 적용]
 * 1. (v3.7) 릴레이 로직 변수화
 * 2. (v3.8) 'convertAnalogToPH' 함수에 Nernst 방정식을 기반으로 한 수온 보정 로직 추가.
 * (DS18B20이 실패하면 25도 기준으로 자동 계산됨)
 */

// --- 0. 사용자 설정 (필수!) ---
#define YOUR_WIFI_SSID "sfarm_2.4g"
#define YOUR_WIFI_PASS "ds123456"
#define YOUR_DEVICE_SERIAL_NUMBER "PLANTOFACTORY_SENSOR_A001" // DB 등록 시리얼 (설정 파일 값)
// --- 0. 설정 끝 ---


// --- 1. 라이브러리 포함 ---
// WiFi / MQTT 라이브러리
#include <WiFiS3.h>
#include <WiFiSSLClient.h>
#include <ArduinoMqttClient.h>
#include <ArduinoJson.h>
#include <math.h>
#include <Wire.h>

// --- [RTC 라이브러리] ---
#include "RTC.h"
// ---

// --- [NTPClient 라이브러리] (펠티어 코드에서 추가) ---
#include <WiFiUdp.h>
#include <NTPClient.h>
// ---

// 공통 센서 라이브러리
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Adafruit_VEML7700.h>

// --- 자동 감지를 위해 두 센서 라이브러리 모두 포함 ---
#include <SensirionI2cScd4x.h>
#include <DHT.h>
// ---


// --- 2. 핀 번호 정의 ---
#define ONE_WIRE_BUS_PIN 2
#define TDS_PIN A0
#define PH_PIN A1
#define PH_RELAY_VCC_PIN 4
#define PH_RELAY_GND_PIN 5
#define RANDOM_SEED_PIN A5
#define DHT_PIN 3

// --- [추가] pH 릴레이 로직 설정 (Active HIGH/LOW) ---
// Active LOW (HIGH=ON, LOW=OFF) - 기본값  
#define PH_RELAY_ON LOW
#define PH_RELAY_OFF HIGH

/*
// Active HIGH (LOW=ON, HIGH=OFF) - 이 설정을 사용하려면 위 두 줄을 주석 처리하고 아래 두 줄을 활성화하세요.
#define PH_RELAY_ON LOW
#define PH_RELAY_OFF HIGH
*/
// ---


// --- 3. MQTT 브로커 정보 ---
const char* broker = "eafc441602df4e36aed5f15ad6df2e4c.s1.eu.hivemq.cloud";
const int port = 8883;
const char* mqttUser = "daesin_302";
const char* mqttPass = "!Ds123456";
const char* deviceSerial = "leafyvegetables_lack01"; // [수정된 값 적용됨]

// --- 4. 객체 생성 ---
WiFiSSLClient sslClient;
MqttClient mqttClient(sslClient);

// --- [NTPClient 객체 생성] (펠티어 코드에서 추가) ---
const long gmtOffset_sec = 32400; // KST (UTC+9)
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", gmtOffset_sec);
// ---

OneWire oneWire(ONE_WIRE_BUS_PIN);
DallasTemperature sensors(&oneWire);
Adafruit_VEML7700 veml = Adafruit_VEML7700();

SensirionI2cScd4x scd4x;
#define DHT_TYPE DHT11
DHT dht(DHT_PIN, DHT_TYPE);


// --- 5. 전역 변수 ---
float airTemperature = 0.0, airHumidity = 0.0, absoluteHumidity = 0.0;
float waterTemperature = 25.0, tdsValue = 0.0, ecValue = 0.0, phValue = 7.0;
float lux = 0.0;
float co2 = 0.0;
bool isSCD41_active = false;
bool isVEML7700_active = false;

// [수정된 간격 적용됨]
unsigned long previousDhtMillis = 0;
const long dhtInterval = 20000; // 20초 (20000ms)
unsigned long previousLightMillis = 0;
const long lightInterval = 2000;
unsigned long previousWaterSensorMillis = 0;
const long phStabilizeTime = 10000; // 10초 안정화
const long cycleInterval = 10000;

enum WaterSensorState { STATE_READ_TDS_TEMP, STATE_WAIT_PH_STABILIZE, STATE_READ_PH, STATE_WAIT_NEXT_CYCLE };
WaterSensorState currentWaterState = STATE_READ_TDS_TEMP;

// [수정] 10비트 ADC로 변경
const float VOLTAGE_REFERENCE = 5.0;
const float ADC_RESOLUTION = 1023.0; // 10비트 (0-1023)

// [수정] TDS 센서 보정 계수 (const 제거, 일반 변수로 변경, 1.0으로 초기화)
float tdsCalibrationFactor = 1.0;

// --- [추가] pH 보정값 전역 변수 (MQTT로 업데이트 가능) ---
float calVoltage_PH6_86 = 2.5;
float calVoltage_PH4_01 = 3.0;
float calVoltage_PH9_18 = 2.0;
// --- [추가] pH Raw Voltage 전송용 변수 ---
float phVoltageRaw = 0.0;
// ---


void setup() {
  Serial.begin(9600);
  Serial.println("--- 스마트팜 센서 시스템 (MQTT) 시작 ---");
  
  // --- 1. ADC 및 핀 설정 ---
  analogReadResolution(10); // [수정] 10비트로 변경
  randomSeed(analogRead(RANDOM_SEED_PIN));
  Serial.println("ADC 10비트, 랜덤 시드 설정 완료.");

  pinMode(PH_RELAY_VCC_PIN, OUTPUT);
  pinMode(PH_RELAY_GND_PIN, OUTPUT);
  // [릴레이 로직 수정] 초기 상태: pH 센서 전원 OFF (설정값 적용)
  digitalWrite(PH_RELAY_VCC_PIN, PH_RELAY_OFF);
  digitalWrite(PH_RELAY_GND_PIN, PH_RELAY_OFF);

  // --- 2. 센서 시작 (I2C 및 SCD41, VEML7700 자동 감지) ---
  sensors.begin(); // DS18B20 시작
  Wire.begin();    // I2C 버스 시작 (SCD41, VEML7700, RTC 공용)

  Serial.println("내장 RTC 시작 중...");
  RTC.begin(); // R4 내장 RTC 초기화

  // VEML7700 (Lux 센서) 초기화 및 자동 감지
  Serial.println("VEML7700(I2C) 센서 스캔 중...");
  if (!veml.begin()) {
    Serial.println("VEML7700 감지 실패. 광량 측정을 비활성화합니다.");
    isVEML7700_active = false;
  } else {
    Serial.println("VEML7700 감지 성공. 광량 측정을 활성화합니다.");
    isVEML7700_active = true;
  }

  // 공기 센서 자동 감지
  Serial.println("SCD41(I2C) 센서 스캔 중...");
  uint16_t error;
  char errorMessage[256];
  
  scd4x.begin(Wire, SCD40_I2C_ADDR_62);
  error = scd4x.stopPeriodicMeasurement();
  error = scd4x.startPeriodicMeasurement();

  if (error) {
    Serial.print("SCD41 감지 실패. DHT11 모드로 전환합니다. (Pin 3)");
    isSCD41_active = false;
    dht.begin();
  } else {
    Serial.println("SCD41 감지 성공. SCD41 모드로 시작합니다.");
    isSCD41_active = true;
  }

  // --- 3. WiFi 연결 ---
  connectWiFi();
  
  // --- 4. [수정] NTPClient 라이브러리를 사용하여 시간 동기화 (펠티어 코드에서 가져옴) ---
  Serial.println("NTPClient 시작...");
  timeClient.begin();
  Serial.print("NTP 서버에서 시간 가져오는 중...");
  
  // 💡 [추가] 1초에서 5초 사이의 랜덤 딜레이를 주어 동시 접속을 방지
  long randomDelay = random(1000, 5000);
  Serial.print("(랜덤 대기: "); Serial.print(randomDelay); Serial.println("ms)");
  delay(randomDelay);

  while (!timeClient.forceUpdate()) { // 성공할 때까지 차단
    Serial.print(".");
    delay(500);
  }
  Serial.println("\n시간 동기화 성공!");
  
  // Epoch 시간을 RTCTime 객체로 변환 (gmtOffset_sec는 이미 적용됨)
  RTCTime timeToSet(timeClient.getEpochTime()); 
  RTC.setTime(timeToSet); // 내장 RTC 설정
  timeClient.end(); // NTP 클라이언트 종료
  Serial.println("내장 RTC가 NTP 시간으로 설정되었습니다.");
  
  Serial.print("현재 RTC 시간: ");
  Serial.println(getIsoTimestamp()); 
  // --- 시간 동기화 끝 ---

  // --- 5. MQTT 연결 및 구독 ---
  connectMQTT();
}


void loop() {
  unsigned long currentMillis = millis();

  mqttClient.poll();

  // --- 작업 0: MQTT 연결 끊김 감지 및 재연결 ---
  if (WiFi.status() == WL_CONNECTED && !mqttClient.connected()) {
    Serial.println("경고: MQTT 연결 끊김. 5초 후 재연결 시도...");
    delay(5000);
    connectMQTT();
  }

  // --- 작업 1: 공기 센서 측정 (자동 감지된 센서 호출) ---
  if (isSCD41_active) {
    manageScd41Sensor(currentMillis);
  } else {
    manageDhtSensor(currentMillis);
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
    delay(5000);
    Serial.print(".");
  }

  Serial.println("\nWiFi 연결 성공!");
  Serial.print("IP 주소: ");
  Serial.println(WiFi.localIP());
}

// 💡 [제거] 기존의 syncRtcTime() 함수는 NTPClient 로직으로 대체되었으므로 삭제되었습니다.


/**
 * @brief 현재 시간을 ISO8601 형식 문자열로 반환합니다. (이 함수는 변경할 필요 없음)
 */
String getIsoTimestamp() {
  if (!RTC.isRunning()) {
    return "1970-01-01T00:00:00Z";
  }
  
  RTCTime currenttime;
  RTC.getTime(currenttime);
  
  char timestamp[30];
  // YYYY-MM-DDTHH:MM:SSZ (UTC 기준)
  sprintf(timestamp, "%04d-%02d-%02dT%02d:%02d:%02dZ",
           currenttime.getYear(),
           Month2int(currenttime.getMonth()),
           currenttime.getDayOfMonth(),
           currenttime.getHour(),
           currenttime.getMinutes(),
           currenttime.getSeconds());
  return String(timestamp);
}

/**
 * @brief [MQTT 로직] MQTT 브로커에 연결하고 구독합니다.
 * [강화] 클라이언트 ID를 (랜덤이 아닌) 고유한 MAC 주소 기반으로 생성합니다.
 */
void connectMQTT() {
  String clientId;
  
  // 💡 [수정] WiFi MAC 주소를 가져오는 방식 (WiFiS3 라이브러리)
  uint8_t macArray[6];
  WiFi.macAddress(macArray); // 라이브러리가 macArray를 채워줍니다.
  
  clientId = "Sensor-"; // 접두사
  char hex[3]; // "FF\0" (2글자 + 널 문자)
  
  for (int i = 0; i < 6; i++) {
    sprintf(hex, "%02X", macArray[i]); // 바이트를 2자리 16진수 문자열로 변환
    clientId += hex;
  }
  // clientId는 이제 "Sensor-E05A1B2C3D4E"와 같은 고유한 ID가 됩니다.

  Serial.print("MQTT 브로커 연결 시도 중: ");
  Serial.println(broker);

  Serial.print("고유 클라이언트 ID (MAC 기반): ");
  Serial.println(clientId);

  mqttClient.setUsernamePassword(mqttUser, mqttPass);
  
  // 💡 ID는 고정값이므로 루프 밖에서 한 번만 설정
  mqttClient.setId(clientId.c_str());

  // r4 보드에서는 WiFiSSLClient 사용 (저장된 정보 사용)
  while (!mqttClient.connected()) {
    
    // ID는 이미 설정됨. 연결만 시도.
    if (mqttClient.connect(broker, port)) {
      Serial.println("MQTT 연결 성공!");
      // break; // 루프 조건이 알아서 종료시킴
    } else {
      Serial.print("MQTT 연결 실패, 오류: ");
      Serial.print(mqttClient.connectError());
      Serial.println(" 5초 후 재시도... (ID 충돌이 아님)");
      delay(5000);
      // ID가 고유하므로, 여기서 실패는 ID 충돌이 아니라 네트워크 문제일 가능성이 높음
    }
  }

  // 연결 성공 시에만 구독 로직 실행
  if (mqttClient.connected()) {
    // --- [추가] pH 보정 토픽 구독 ---
    String calibrationPhTopic = "plantofactory/command/calibration/pH/";
    calibrationPhTopic += deviceSerial;
    
    Serial.print("pH 보정 토픽 구독: ");
    Serial.println(calibrationPhTopic);
    
    if (mqttClient.subscribe(calibrationPhTopic.c_str())) {
        Serial.println("구독 성공!");
    } else {
        Serial.println("구독 실패!");
    }
    
    // --- [추가] EC 보정 토픽 구독 ---
    String calibrationEcTopic = "plantofactory/command/calibration/EC/";
    calibrationEcTopic += deviceSerial;

    Serial.print("EC 보정 토픽 구독: ");
    Serial.println(calibrationEcTopic);

    if (mqttClient.subscribe(calibrationEcTopic.c_str())) {
        Serial.println("구독 성공!");
    } else {
      Serial.println("구독 실패!");
    }
    
    // 메시지 수신 콜백 함수 설정
    mqttClient.onMessage(onMqttMessageReceived);
  }
}

/**
 * @brief [MQTT 콜백] MQTT 메시지를 수신했을 때 호출되는 함수
 */
void onMqttMessageReceived(int messageSize) {
    String topic = mqttClient.messageTopic();
    String payload = "";

    Serial.print("메시지 수신 - 토픽: ");
    Serial.println(topic);

    // 페이로드 읽기
    while (mqttClient.available()) {
        payload += (char)mqttClient.read();
    }
    
    Serial.print("페이로드: ");
    Serial.println(payload);

    StaticJsonDocument<100> doc;
    DeserializationError error = deserializeJson(doc, payload);

    if (error) {
        Serial.print("JSON 파싱 실패: ");
        Serial.println(error.c_str());
        return;
    }
    
    // --- pH 보정 처리 ---
    String calibrationPhTopicPrefix = "plantofactory/command/calibration/pH/";
    calibrationPhTopicPrefix += deviceSerial;
    
    if (topic == calibrationPhTopicPrefix) {
        float phPoint = doc["ph_point"] | 0.0;
        float voltage = doc["voltage"] | 0.0;

        if (voltage == 0.0) {
            Serial.println("오류: 전압 값이 0입니다.");
            return;
        }

        if (abs(phPoint - 6.86) < 0.01) {
            calVoltage_PH6_86 = voltage;
            Serial.print(">> pH 6.86 보정값 업데이트: "); Serial.println(calVoltage_PH6_86, 3);
        } else if (abs(phPoint - 4.01) < 0.01) {
            calVoltage_PH4_01 = voltage;
            Serial.print(">> pH 4.01 보정값 업데이트: "); Serial.println(calVoltage_PH4_01, 3);
        } else if (abs(phPoint - 9.18) < 0.01) {
            calVoltage_PH9_18 = voltage;
            Serial.print(">> pH 9.18 보정값 업데이트: "); Serial.println(calVoltage_PH9_18, 3);
        } else {
            Serial.print("경고: 알 수 없는 pH 포인트 ("); Serial.print(phPoint); Serial.println(")");
        }
    }
    
    // --- [추가] EC 보정 처리 ---
    String calibrationEcTopicPrefix = "plantofactory/command/calibration/EC/";
    calibrationEcTopicPrefix += deviceSerial;

    if (topic == calibrationEcTopicPrefix) {
        float ecTarget = doc["ec_target"] | 0.0; // 외부 측정기로 잰 참값
        float ecRaw = doc["ec_raw"] | 0.0;      // 아두이노가 보낸 보정 전 값

        if (ecRaw > 0.0 && ecTarget > 0.0) {
            tdsCalibrationFactor = ecTarget / ecRaw;
            Serial.print(">> EC 보정 계수 업데이트 (Target/Raw): ");
            Serial.println(tdsCalibrationFactor, 3);
            Serial.print("   (참값: "); Serial.print(ecTarget); Serial.print(" / 측정값: "); Serial.print(ecRaw); Serial.println(")");
        } else {
            Serial.println("오류: EC 보정 데이터가 유효하지 않습니다 (Raw/Target=0).");
        }
    }
}

/**
 * @brief JSON 페이로드를 생성하여 MQTT 토픽으로 전송합니다.
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
 * @brief 작업 1 (SCD41): SCD41 센서 관리 함수
 */
void manageScd41Sensor(unsigned long currentMillis) {
  if (currentMillis - previousDhtMillis >= dhtInterval) {
    previousDhtMillis = currentMillis;

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

    calculateAbsoluteHumidity();
    
    Serial.println("--- [공기 환경 (SCD41)] ---");
    Serial.print("CO2: "); Serial.print(co2); Serial.println(" ppm");
    Serial.print("온도: "); Serial.print(airTemperature); Serial.println(" *C");
    Serial.print("상대 습도: "); Serial.print(airHumidity); Serial.println(" %");
    Serial.print("절대 습도: "); Serial.print(absoluteHumidity); Serial.println(" g/m³");
    Serial.println("--------------------");

    String now = getIsoTimestamp();
    publishMqttMessage("plantofactory/sensor/air_co2", co2, now);
    publishMqttMessage("plantofactory/sensor/air_temperature", airTemperature, now);
    publishMqttMessage("plantofactory/sensor/air_humidity_relative", airHumidity, now);
    // 💡 [수정] 오타 수정: publishMtqMessage -> publishMqttMessage
    publishMqttMessage("plantofactory/sensor/air_humidity_absolute", absoluteHumidity, now);
  }
}

/**
 * @brief 작업 1 (DHT11): DHT11 센서 관리 함수
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
    
    Serial.println("--- [공기 환경 (DHT11)] ---");
    Serial.print("온도: "); Serial.print(airTemperature); Serial.println(" *C");
    Serial.print("상대 습도: "); Serial.print(airHumidity); Serial.println(" %");
    Serial.print("절대 습도: "); Serial.print(absoluteHumidity); Serial.println(" g/m³");
    Serial.println("--------------------");

    String now = getIsoTimestamp();
    publishMqttMessage("plantofactory/sensor/air_temperature", airTemperature, now);
    publishMqttMessage("plantofactory/sensor/air_humidity_relative", airHumidity, now);
    publishMqttMessage("plantofactory/sensor/air_humidity_absolute", absoluteHumidity, now);
  }
}



/**
 * @brief 작업 2: 수질 센서 상태 머신
 */
void manageWaterSensors(unsigned long currentMillis) {
  switch (currentWaterState) {

    case STATE_READ_TDS_TEMP: {
      Serial.println("[상태 1] TDS 및 수온 측정 시작...");
      // [릴레이 상태 명확화]
      Serial.println("  > 릴레이 상태: pH 센서 OFF");
      // [릴레이 로직 - 설정값 적용]
      digitalWrite(PH_RELAY_VCC_PIN, PH_RELAY_OFF); // pH 센서 전원 차단 (OFF)
      digitalWrite(PH_RELAY_GND_PIN, PH_RELAY_OFF);

      // 수온 측정
      sensors.requestTemperatures();
      waterTemperature = sensors.getTempCByIndex(0);
      if (waterTemperature == DEVICE_DISCONNECTED_C) {
        Serial.println("오류: DS18B20 수온 센서 연결 실패!");
        waterTemperature = 25.0; // 오류 시 기본값
      }

      // --- TDS 평균값 계산 로직 (pH와 동일하게 50회 샘플링) ---
      Serial.println("  > TDS/EC 측정: 50회 샘플링 (2.5초 소요)");
      const int numTdsSamples = 50;
      long totalTdsAnalogValue = 0;

      for (int i = 0; i < numTdsSamples; i++) {
        totalTdsAnalogValue += analogRead(TDS_PIN);
        delay(50); // 50ms 딜레이
      }

      int averageTdsAnalogValue = totalTdsAnalogValue / numTdsSamples;
      // --- 수정 끝 ---

      // [수정] ecValue 계산 시, ecRaw 값 전송을 위해 tdsCalibrationFactor를 곱하지 않음
      calculateTdsAndEc(averageTdsAnalogValue, waterTemperature);

      Serial.print("  > 수온: "); Serial.print(waterTemperature); Serial.println(" *C");
      Serial.print("  > EC: "); Serial.print(ecValue); Serial.println(" uS/cm");
      Serial.print("  > TDS: "); Serial.print(tdsValue); Serial.println(" ppm");
      
      // --- [추가] EC Raw 값 전송 (보정용) ---
      // 보정 전 순수 EC 값 (tdsCalibrationFactor 적용 전)을 전송
      float ecRaw = ecValue / tdsCalibrationFactor;

      String now = getIsoTimestamp();
      if (waterTemperature != DEVICE_DISCONNECTED_C) {
          publishMqttMessage("plantofactory/sensor/water_temperature", waterTemperature, now);
      }
      // EC/TDS (보정 완료된 값)
      publishMqttMessage("plantofactory/sensor/water_ec", ecValue, now);
      publishMqttMessage("plantofactory/sensor/water_tds", tdsValue, now);
      // EC Raw (보정용)
      publishMqttMessage("plantofactory/sensor/water_ec_raw", ecRaw, now);

      currentWaterState = STATE_WAIT_PH_STABILIZE;
      previousWaterSensorMillis = currentMillis;
      break;
    }

    case STATE_WAIT_PH_STABILIZE: {
      Serial.println("[상태 2] pH 센서 전원 ON. 안정화 대기 중...");
      // [릴레이 상태 명확화]
      Serial.println("  > 릴레이 상태: pH 센서 ON");
      // [릴레이 로직 - 설정값 적용]
      digitalWrite(PH_RELAY_VCC_PIN, PH_RELAY_ON); // pH ON
      digitalWrite(PH_RELAY_GND_PIN, PH_RELAY_ON);

      if (currentMillis - previousWaterSensorMillis >= phStabilizeTime) {
        currentWaterState = STATE_READ_PH;
      }
      break;
    }

    case STATE_READ_PH: {
      Serial.println("[상태 3] pH 측정 시작 (50회 평균값 계산)...");
      // [릴레이 상태 명확화]
      Serial.println("  > 릴레이 상태: pH 센서 ON 상태 유지");

      // --- pH 평균값 계산 로직 (delay 사용) ---
      const int numPhSamples = 50;
      long totalTdsAnalogValue = 0;

      for (int i = 0; i < numPhSamples; i++) {
        totalTdsAnalogValue += analogRead(PH_PIN);
        delay(50); // 50ms 딜레이 (총 2.5초 소요)
      }

      int averagePhAnalogValue = totalTdsAnalogValue / numPhSamples;
      // --- 수정 끝 ---

      // [추가] Raw Voltage 계산 및 저장
      phVoltageRaw = averagePhAnalogValue * (VOLTAGE_REFERENCE / ADC_RESOLUTION);
      
      // pH 측정 (평균 아날로그 값 사용)
      phValue = convertAnalogToPH(averagePhAnalogValue, waterTemperature);

      // [시리얼 출력]
      Serial.print("  > [평균] Analog: "); Serial.print(averagePhAnalogValue);
      Serial.print("  | Raw Voltage: "); Serial.print(phVoltageRaw, 3); // Raw Voltage 출력
      Serial.print(" V | pH: "); Serial.print(phValue);
      Serial.print("\t | EC: "); Serial.print(ecValue); Serial.println(" uS/cm");

      // [MQTT 전송 트리거]
      String now = getIsoTimestamp();
      publishMqttMessage("plantofactory/sensor/water_ph", phValue, now);
      // [추가] Raw Voltage 전송
      publishMqttMessage("plantofactory/sensor/water_ph_voltage", phVoltageRaw, now);

      // pH 센서 전원 차단
      Serial.println("  > pH 측정 완료. 릴레이 전원 차단.");
      // [릴레이 로직 - 설정값 적용]
      digitalWrite(PH_RELAY_VCC_PIN, PH_RELAY_OFF); // pH OFF
      digitalWrite(PH_RELAY_GND_PIN, PH_RELAY_OFF);

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
 * @brief 작업 3: 광량(Lux) 센서 관리 함수 (VEML7700)
 */
void manageLightSensor(unsigned long currentMillis) {
  if (isVEML7700_active && currentMillis - previousLightMillis >= lightInterval) {
    previousLightMillis = currentMillis;

    lux = veml.readLux();

    Serial.println("--- [광량 (VEML7700)] ---");
    Serial.print("Lux: "); Serial.print(lux); Serial.println(" lx");
    Serial.println("-------------------------");

    String now = getIsoTimestamp();
    publishMqttMessage("plantofactory/sensor/light_lux", lux, now);
  }
}


// --- 센서 계산 함수 ---

/**
 * @brief (보정 계수 적용됨) TDS 아날로그 값을 EC(uS/cm)와 TDS(ppm)로 변환합니다.
 */
void calculateTdsAndEc(int analogValue, float temperature) {
  // 10비트 ADC 기준
  float voltage = analogValue * (VOLTAGE_REFERENCE / ADC_RESOLUTION);
  float compensationCoefficient = 1.0 + 0.02 * (temperature - 25.0);
  float compensationVoltage = voltage / compensationCoefficient;
  
  // 1. EC 순수값 계산 (DFRobot Cubic Curve Formula 사용)
  // 이 값은 ecRaw 값 계산의 기준이 됩니다.
  
  // [수정 v3.6] 3차 다항식의 마지막 항 부호 오류 수정 (- -> +)
  float ecBase = (133.42 * pow(compensationVoltage, 3)
              - 255.86 * pow(compensationVoltage, 2)
              + 857.39 * compensationVoltage); // <--- 부호 수정됨

  float k_factor = 0.5; // EC to TDS 변환 비율 (0.5 ~ 0.7)
  
  // 2. 최종 EC 및 TDS 값 계산 (tdsCalibrationFactor 적용)
  ecValue = ecBase * tdsCalibrationFactor;
  tdsValue = (ecBase * k_factor) * tdsCalibrationFactor;

  // 참고: ecRaw 값은 ecValue / tdsCalibrationFactor 로 계산하여 MQTT로 전송됨.
}

/**
 * @brief (교정 완료) pH 아날로그 값을 pH 값으로 변환합니다.
 * [수정 v3.8] Nernst 방정식을 기반으로 수온을 보정합니다.
 */
float convertAnalogToPH(int analogValue, float temperature) {
  // --- [수정] 전역 변수를 참조하도록 변경 ---
  const float VOLTAGE_PH6_86 = calVoltage_PH6_86;
  const float VOLTAGE_PH4_01 = calVoltage_PH4_01;
  const float VOLTAGE_PH9_18 = calVoltage_PH9_18;
  // ---

  // 10비트 ADC 기준
  float voltage = analogValue * (VOLTAGE_REFERENCE / ADC_RESOLUTION);
  float ph;

  // --- [추가 v3.8] Nernst 방정식을 이용한 온도 보정 ---
  // 1. 25°C(298.15K) 기준 보정 슬로프(단위: pH/V) 계산
  //    (pH = 6.86 + (V - V_6.86) * slope)
  float slope_acid_25C = (6.86 - 4.01) / (VOLTAGE_PH6_86 - VOLTAGE_PH4_01);
  float slope_base_25C = (9.18 - 6.86) / (VOLTAGE_PH9_18 - VOLTAGE_PH6_86);

  // 2. 온도 보정 계수 계산 ( T(K) / 298.15K )
  //    (temperature 변수는 ds18b20에서 측정되었거나, 실패 시 25.0으로 설정된 값)
  float temp_coefficient = (273.15 + temperature) / (273.15 + 25.0);

  // 3. 현재 온도로 보정된 슬로프 계산
  //    전압(V) 차이는 온도(K)에 비례합니다.
  //    따라서 슬로프(pH/V)는 온도(K)에 반비례합니다.
  float slope_acid = slope_acid_25C / temp_coefficient;
  float slope_base = slope_base_25C / temp_coefficient;
  // --- [수정 끝] ---


  // 센서 특성 확인: VOLTAGE_PH4_01 (4.05) > VOLTAGE_PH6_86 (3.58) > VOLTAGE_PH9_18 (3.15)
  // 전압이 높을수록 산성 (로직 정상)
  
  if (voltage > VOLTAGE_PH6_86) {
    // 중성점보다 높다 -> 산성 영역 예상
    // [수정 v3.8] 온도 보정된 슬로프 사용
    ph = 6.86 + (voltage - VOLTAGE_PH6_86) * slope_acid;
  } else {
    // 중성점보다 낮다 -> 염기성 영역 예상
    // [수정 v3.8] 온도 보정된 슬로프 사용
    ph = 6.86 + (voltage - VOLTAGE_PH6_86) * slope_base;
  }
  
  // [수정 v3.8] 온도 보정이 적용됨
  
  return ph;
}

/**
 * @brief 절대 습도(g/m³)를 계산합니다.
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