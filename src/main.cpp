
#include <Arduino.h>
#include <Wire.h>
#include <SHT31.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>
#include <addons/TokenHelper.h>
#include <WiFiManager.h>
#include <WiFi.h>
#include <ESP32Ping.h>
#include <LiquidCrystal_I2C.h>

#include "driver/pcnt.h"                                                
#include "soc/pcnt_struct.h"
#include "time.h"
#include "DHT.h"

const int LONG_PRESS_TIME = 5000;
int lastState = LOW;
int currentState;
int lcdColumns = 20;
int lcdRows = 4;

LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); 

unsigned long pressedTime = 0;
unsigned long releasedTime = 0;

long previousLCDMillis = 0;
long lcdInterval = 4000;

int screen = 0;    
int screenMax = 2;
bool screenChanged = true;

WiFiManager wifiManager;
bool AutoGeneratedAP_Name;

#define TZ_INFO "CET-1CEST,M3.5.0,M10.5.0/3"

WiFiManagerParameter _USER_EMAIL("influxDbUrl", "Enter your Mail Address: ", "omerfarukoruc35@gmail.com", 100);
WiFiManagerParameter _USER_PASSWORD("influxDbToken", "Enter your Password:", "deneme", 100);
WiFiManagerParameter _API_KEY("influxDbOrganization", "Enter your Firebase API Key: ", "AIzaSyDl-0kC9UBSvSjqFy3HwQt2d3pe4zY5GCs", 100);
WiFiManagerParameter _DATABASE_URL("influxDbBucket", "Enter your Firebase Database URL: ", "btcss-1-default-rtdb.europe-west1.firebasedatabase.app", 100);

#define pinRelay 3 // 8 den 3 e aldım bunu
#define allFansPWMPin 40 // 37 den aldım

#define oneWireBus 18
#define buttonPin 38 // GPIO23 pin connected to button http://espblockly.com/ButtonLongPress.html#How_To_Detect_Long_Press 23 den aldım.
#define flowMeterPin 2

#define FREQ_PIN_FAN1 GPIO_NUM_4
#define FREQ_PIN_FAN2 GPIO_NUM_5
#define FREQ_PIN_FAN3 GPIO_NUM_6
#define FREQ_PIN_FAN4 GPIO_NUM_7
#define FREQ_PIN_FAN5 GPIO_NUM_15 // 15 den aldım bunu
#define FREQ_PIN_FAN6 GPIO_NUM_16 // 16 den aldım bunu

FirebaseData fbdo;
FirebaseJson jsonStatic, jsonDynamic;
FirebaseAuth auth;
FirebaseConfig config;

String a, b, c, d;
String deneme;
int pwmFromFirebase;
String uid;
String databaseStaticReadingsPath, databaseDynamicReadingsPath;
String temperatureOutsidePath = "/temperatureOutside";
String UsersData = "/UsersData/";
String humPath = "/humidity";
String temperatureInsidePath = "/temperatureInside";
String timePath = "/timestamp";

String rpmPath = "/fans";
String rpmPath1 = rpmPath + "/fan1";
String rpmPath2 = rpmPath + "/fan2";
String rpmPath3 = rpmPath + "/fan3";
String rpmPath4 = rpmPath + "/fan4";
String rpmPath5 = rpmPath + "/fan5";
String rpmPath6 = rpmPath + "/fan6";

String readingsPath = "/readings";
String staticReadingsPath = "/staticReadings";
String dynamicReadingsPath = "/dynamicReadings";

String parentStaticReadingsPath, parentDynamicReadingsPath;
int timestamp;

const char *ntpServer = "0.tr.pool.ntp.org"; // tr ye aldım
unsigned long sendDataPrevMillis = 0;
unsigned long timerDelay = 5000; ///firebase'e ne aralıkla veri gönderilecek.

SHT31 shtSensor;
OneWire oneWire(oneWireBus);
DallasTemperature probe(&oneWire);
uint32_t chipId = 0;

bool wm_nonblocking = false;

const int PWMFreq = 5000;
const int PWMChannel = 0;
const int PWMResolution = 8;

bool            flag_fan1          = true;                                   
uint32_t        overflow_fan1      = 20000;                                  
uint32_t        overflow_cnt_fan1  = 0;                                        
volatile double frequency_fan1     = 0;
uint16_t result_fan1 = 0;

bool            flag_fan2          = true;                                   
uint32_t        overflow_fan2      = 20000;                                  
uint32_t        overflow_cnt_fan2  = 0;                                        
volatile double frequency_fan2     = 0;
uint16_t result_fan2 = 0;

bool            flag_fan3          = true;                                   
uint32_t        overflow_fan3      = 20000;                                  
uint32_t        overflow_cnt_fan3  = 0;                                        
volatile double frequency_fan3     = 0;
uint16_t result_fan3 = 0;

bool            flag_fan4          = true;                                   
uint32_t        overflow_fan4      = 20000;                                  
uint32_t        overflow_cnt_fan4  = 0;                                        
volatile double frequency_fan4    = 0;
uint16_t result_fan4 = 0;

volatile double frequency_fan5    = 0;
volatile double frequency_fan6    = 0;
volatile double average_fans = 0;

const int coefficientMovingAverage = 10;
int readings [coefficientMovingAverage];
int readIndex = 0;
long total = 0;

long currentMillis = 0;
long previousMillis = 0;
int intervalFlowMeter = 5000;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
int interruptCounterFan5, interruptCounterFan6, rpm;

esp_timer_create_args_t timer_args_fan1;                                   
esp_timer_handle_t timer_handle_fan1;                                       
portMUX_TYPE timer_mux_fan1 = portMUX_INITIALIZER_UNLOCKED;

esp_timer_create_args_t timer_args_fan2;                                    
esp_timer_handle_t timer_handle_fan2;                                          
portMUX_TYPE timer_mux_fan2 = portMUX_INITIALIZER_UNLOCKED;

esp_timer_create_args_t timer_args_fan3;                                    
esp_timer_handle_t timer_handle_fan3;                                          
portMUX_TYPE timer_mux_fan3 = portMUX_INITIALIZER_UNLOCKED;

esp_timer_create_args_t timer_args_fan4;                                    
esp_timer_handle_t timer_handle_fan4;                                          
portMUX_TYPE timer_mux_fan4 = portMUX_INITIALIZER_UNLOCKED;

typedef struct 
{
  int pulse_gpio_num;
  int ctrl_gpio_num;
  pcnt_ctrl_mode_t lctrl_mode;
  pcnt_ctrl_mode_t hctrl_mode;
  pcnt_count_mode_t pos_mode;
  pcnt_count_mode_t neg_mode;
  uint16_t counter_h_lim;
  uint16_t counter_l_lim;
  pcnt_unit_t unit;
  pcnt_channel_t channel;
} my_pcnt_config_t;

void pcnt_config(my_pcnt_config_t* config, int pulse_gpio_num, pcnt_unit_t unit) 
{
  config->pulse_gpio_num = pulse_gpio_num;
  config->ctrl_gpio_num = -1;
  config->lctrl_mode = PCNT_MODE_KEEP;
  config->hctrl_mode = PCNT_MODE_KEEP;
  config->pos_mode = PCNT_COUNT_INC;
  config->neg_mode = PCNT_COUNT_INC;
  config->counter_h_lim = 20000;
  config->counter_l_lim = 0;
  config->unit = unit;
  config->channel = PCNT_CHANNEL_0;
}

typedef my_pcnt_config_t _pcnt_config_t;

void IRAM_ATTR pcnt_event_handler(void *arg)
{
  auto params = static_cast<std::tuple<pcnt_unit_t, volatile uint32_t *, portMUX_TYPE *> *>(arg);
  pcnt_unit_t unit = std::get<0>(*params);
  volatile uint32_t *overflow_cnt = std::get<1>(*params);
  portMUX_TYPE *timer_mux = std::get<2>(*params);

  portENTER_CRITICAL_ISR(timer_mux);
  (*overflow_cnt)++;
  PCNT.int_clr.val = BIT(unit);
  portEXIT_CRITICAL_ISR(timer_mux);
}

void IRAM_ATTR pulseCounterForFlowMeter()
{
  pulseCount++;
}

void countUpFan5() 
{
  interruptCounterFan5++;
}

void countUpFan6() 
{
  interruptCounterFan6++;
}

void saveParamsCallback()
{
  a = _USER_EMAIL.getValue();
  b = _USER_PASSWORD.getValue();
  c = _API_KEY.getValue();
  d = _DATABASE_URL.getValue();
}

long simpleMovingAverage() 
{
  total = total - readings[readIndex];
  readings[readIndex] = rpm;
  total = total + readings[readIndex];
  readIndex = readIndex + 1;
  if (readIndex >= coefficientMovingAverage) 
  {
    readIndex = 0;
  }

  return total / coefficientMovingAverage;
}

float calculateFlowMeter()
{
  currentMillis = millis();
  if (currentMillis - previousMillis > intervalFlowMeter) 
  {
    pulse1Sec = pulseCount;
    pulseCount = 0;
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    
    Serial.print("Flow rate: ");
    Serial.print(double(flowRate)); 
    Serial.print("L/min");
    Serial.print("\t");
  }
  return flowRate;
}

esp_err_t pcnt_isr_register_custom(pcnt_unit_t unit, volatile uint32_t *overflow_cnt, portMUX_TYPE *timer_mux)
{
  auto params = new std::tuple<pcnt_unit_t, volatile uint32_t *, portMUX_TYPE *>(unit, overflow_cnt, timer_mux);
  return pcnt_isr_register([](void *arg) { pcnt_event_handler(arg); }, params, 0, NULL);
}

void pcnt_get_counter(void *arg)
{
  auto params = static_cast<std::tuple<pcnt_unit_t, int16_t *, bool *> *>(arg);
  pcnt_unit_t unit = std::get<0>(*params);
  int16_t *result = std::get<1>(*params);
  bool *flag = std::get<2>(*params);

  pcnt_counter_pause(unit);
  pcnt_get_counter_value(unit, result);
  *flag = true;
}


float calculateRpmFan1()
{
  if (flag_fan1 == true)
  {
    flag_fan1 = false;
    frequency_fan1 =  result_fan1 + (overflow_cnt_fan1*20000); 
    overflow_cnt_fan1 = 0; 
    pcnt_counter_clear(PCNT_UNIT_0); 
    pcnt_counter_resume(PCNT_UNIT_0); 
    overflow_cnt_fan1 = 0;    
  
    Serial.print(frequency_fan1 * 15);
    Serial.println(" RPM FAN1");

    pcnt_counter_clear(PCNT_UNIT_0);
    esp_timer_start_once(timer_handle_fan1, 1000000);
    
  }
  return frequency_fan1 * 15;
}

float calculateRpmFan2()
{
  if (flag_fan2 == true)
  {
    flag_fan2 = false;
    frequency_fan2 =  result_fan2 + (overflow_cnt_fan2 * 20000); 
    overflow_cnt_fan2 = 0; 
    pcnt_counter_clear(PCNT_UNIT_1); 
    pcnt_counter_resume(PCNT_UNIT_1); 
    overflow_cnt_fan2 = 0;    
  
    Serial.print(frequency_fan2 * 15);
    Serial.println(" RPM FAN2");

    pcnt_counter_clear(PCNT_UNIT_1);
    esp_timer_start_once(timer_handle_fan2, 1000000);
  }
  return frequency_fan2 * 15;
}

float calculateRpmFan3()
{
  if (flag_fan3 == true)
  {
    flag_fan3 = false;
    frequency_fan3 =  result_fan3 + (overflow_cnt_fan3 * 20000); 
    overflow_cnt_fan3 = 0; 
    pcnt_counter_clear(PCNT_UNIT_2); 
    pcnt_counter_resume(PCNT_UNIT_2); 
    overflow_cnt_fan3 = 0;    
  
    Serial.print(frequency_fan3 * 15);
    Serial.println(" RPM FAN3");

    pcnt_counter_clear(PCNT_UNIT_2);
    esp_timer_start_once(timer_handle_fan3, 1000000);
  }
  return frequency_fan3 * 15;
}

float calculateRpmFan4()
{  
  if (flag_fan4 == true)
  {
    flag_fan4 = false;
    frequency_fan4 =  result_fan4 + (overflow_cnt_fan4 * 20000); 
    overflow_cnt_fan4 = 0; 
    pcnt_counter_clear(PCNT_UNIT_3); 
    pcnt_counter_resume(PCNT_UNIT_3); 
    overflow_cnt_fan4 = 0;    
  
    Serial.print(frequency_fan4 * 15);
    Serial.println(" RPM FAN4");

    pcnt_counter_clear(PCNT_UNIT_3);
    esp_timer_start_once(timer_handle_fan4, 1000000);
  }
  return frequency_fan4 * 15;
}

float calculateRpmFan5() 
{
  interruptCounterFan5 = 0;
  attachInterrupt(digitalPinToInterrupt(FREQ_PIN_FAN5), countUpFan5, RISING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(FREQ_PIN_FAN5));
  frequency_fan5 = ((interruptCounterFan5 / 2) * 60);
  Serial.println(frequency_fan5);
  return frequency_fan5;
}

float calculateRpmFan6() 
{
  interruptCounterFan6 = 0;
  attachInterrupt(digitalPinToInterrupt(FREQ_PIN_FAN6), countUpFan6, RISING);
  delay(1000);
  detachInterrupt(digitalPinToInterrupt(FREQ_PIN_FAN6));
  frequency_fan6 = ((interruptCounterFan6 / 2) * 60);
  Serial.println(frequency_fan6);
  return frequency_fan6;
}

void initFlowMeter()
{
  pinMode(flowMeterPin, INPUT_PULLUP);
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  previousMillis = 0;

  attachInterrupt(digitalPinToInterrupt(flowMeterPin), pulseCounterForFlowMeter, FALLING);
}

void checkIfLongPress(void) 
{
  currentState = digitalRead(buttonPin);

  if (lastState == HIGH && currentState == LOW)  // button is pressed
  {
    pressedTime = millis();
  }

  else if (lastState == LOW && currentState == HIGH)  // button is released
  {
    releasedTime = millis();

    long pressDuration = releasedTime - pressedTime;

    if (pressDuration > LONG_PRESS_TIME) 
    {
      Serial.println("A long press is detected");
      //wifiManager.resetSettings();
    }
  }
  lastState = currentState;
}

void initSensors() 
{
  probe.begin();
  Wire.begin();
  shtSensor.begin();
  Wire.setClock(100000);
}

void initWiFi() 
{
  Serial.print("Connecting to WiFi ..");
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print('.');
    delay(1000);
  }
  Serial.println(WiFi.localIP());
  Serial.println();
  configTime(0, 0, ntpServer);
}

unsigned long getTime() 
{
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) 
  {
    Serial.println("Failed to obtain time");
  }

  time(&now);
  return now;
}

void showFirstScreen(void)
{  
  probe.requestTemperatures();
  float temperatureProbe = probe.getTempCByIndex(0);
  float humidity = shtSensor.getHumidity();
  float outsideTemperature = shtSensor.getTemperature();
  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("_KYRA  TECHNOLOGY_");  

  lcd.setCursor(1, 1);
  lcd.print("BARRUS ICISI: ");
  int temp1 = round(temperatureProbe);
  lcd.print(temp1);
  lcd.print("'C");

  lcd.setCursor(2, 2);
  lcd.print("ORTAM ISISI: ");
  int temp2 = round(outsideTemperature);
  lcd.print(temp2);
  lcd.print("'C");

  lcd.setCursor(2, 3);
  lcd.print("ORTAM  NEMI: ");
  lcd.print("%");
  int temp3 = round(humidity);
  lcd.print(temp3);
  
}

void showSecondScreen(void)
{
  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("_KYRA  TECHNOLOGY_");

  lcd.setCursor(2, 1);
  lcd.print("DEBIMETRE: ");
  int temp4 = round(calculateFlowMeter());
  lcd.print(temp4);
  lcd.print(" L/M");

  lcd.setCursor(0, 2);
  lcd.print("FAN ORTALAMA: ");
  int temp5 = round((calculateRpmFan1() + calculateRpmFan2() + calculateRpmFan3() + calculateRpmFan4() + calculateRpmFan5() + calculateRpmFan6()) / 6);
  lcd.print(temp5);
}

void showThirdScreen(void)
{  
  lcd.clear();

  lcd.setCursor(1, 0);
  lcd.print("_KYRA  TECHNOLOGY_");

  lcd.setCursor(0, 1);
  lcd.print("FAN1:");
  int temp6 = round(calculateRpmFan1());
  lcd.print(temp6);
  lcd.print(" ");
  lcd.print("FAN4:");
  int temp7 = round(calculateRpmFan4());
  lcd.print(temp7);

  lcd.setCursor(0, 2);
  lcd.print("FAN2:");
  int temp8 = round(calculateRpmFan2());
  lcd.print(temp8);
  lcd.print(" ");
  lcd.print("FAN5:");
  int temp9 = round(calculateRpmFan5());
  lcd.print(temp9);

  lcd.setCursor(0, 3);
  lcd.print("FAN3:");
  int temp10 = round(calculateRpmFan3());
  lcd.print(temp10);
  lcd.print(" ");
  lcd.print("FAN6:");
  int temp11 = round(calculateRpmFan6());
  lcd.print(temp11);
}

void initLcdScreen(void)
{
  lcd.init();                   
  lcd.backlight();
}

void writeLcdScreen(void)
{
  shtSensor.read();
  unsigned long currentLCDMillis = millis();

  if(currentLCDMillis - previousLCDMillis > lcdInterval)
  {
    previousLCDMillis = currentLCDMillis; 
    screen++;
    if (screen > screenMax) 
      screen = 0;
    screenChanged = true; 
  }

  if (screenChanged)
  {
    screenChanged = false;
    switch(screen)
    {
      case 0: 
        showFirstScreen(); 
        break;

      case 1: 
        showSecondScreen();
        break;

      case 2:
        showThirdScreen();
        break;

      default:
        break;
    }
  }
}

void firebaseSendData(void) 
{
  databaseStaticReadingsPath = UsersData + uid + readingsPath + staticReadingsPath; // burada string toplamasında bir sıkıntı var.
  databaseDynamicReadingsPath = UsersData + uid + readingsPath + dynamicReadingsPath; // burada string toplamasında bir sıkıntı var.

  if (Firebase.ready() && (millis() - sendDataPrevMillis > timerDelay || sendDataPrevMillis == 0)) 
  {
    sendDataPrevMillis = millis();
    timestamp = getTime();

    shtSensor.read();
    probe.requestTemperatures();

    float temperatureProbe = probe.getTempCByIndex(0);
    float humidity = shtSensor.getHumidity();
    float outsideTemperature = shtSensor.getTemperature();

    Serial.print("time: ");
    Serial.println(timestamp);

    parentStaticReadingsPath = databaseStaticReadingsPath;
    parentDynamicReadingsPath = databaseDynamicReadingsPath + "/" + timestamp;

    jsonDynamic.set(temperatureOutsidePath.c_str(), outsideTemperature);
    jsonDynamic.set(humPath.c_str(), humidity);
    jsonDynamic.set(temperatureInsidePath.c_str(), temperatureProbe);
    jsonDynamic.set(timePath, timestamp);
    
    jsonStatic.set(rpmPath1.c_str(), calculateRpmFan1());
    jsonStatic.set(rpmPath2.c_str(), calculateRpmFan2());
    jsonStatic.set(rpmPath3.c_str(), calculateRpmFan3());
    jsonStatic.set(rpmPath4.c_str(), calculateRpmFan4());
    jsonStatic.set(rpmPath5.c_str(), calculateRpmFan5());
    jsonStatic.set(rpmPath6.c_str(), calculateRpmFan6());

    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentStaticReadingsPath.c_str(), &jsonStatic) ? "ok" : fbdo.errorReason().c_str());
    Serial.printf("Set json... %s\n", Firebase.RTDB.setJSON(&fbdo, parentDynamicReadingsPath.c_str(), &jsonDynamic) ? "ok" : fbdo.errorReason().c_str());

    //writeLcdScreen();

    neopixelWrite(RGB_BUILTIN, 4, 0, 4);
    delay(500);
    neopixelWrite(RGB_BUILTIN, 0, 0, 0);
  }
}

void countRPM(void) 
{
  calculateRpmFan1();
  calculateRpmFan2();
  calculateRpmFan3();
  calculateRpmFan4();
  calculateRpmFan5();
  calculateRpmFan6();
  
  average_fans = (frequency_fan1 + frequency_fan2 + frequency_fan3 + frequency_fan4 + frequency_fan5 + frequency_fan6) / 6 * 15;
  Serial.print(average_fans);
  Serial.println(" Average RPM'S of 6 fans");
}

void alertBuzzer(void) 
{
    
}

void checkIfDisconnected(void)  // Baglantı koptuguna dair buzzer vs. ile sesli bildirim yapılacak.
{
  bool internetConnection = Ping.ping("www.google.com");
  if (internetConnection) 
  {
  // Internet connectionunda sıkıntı yoksa diger işlemlere devam edecek.
  }

  else 
  {
    Serial.println("Failed to connect");
    alertBuzzer();
  }
}

void relayControl(void) 
{
  if (Firebase.RTDB.getString(&fbdo, F("/UsersData/bmU3MvDjN1RtaC0qk2wkPEVkwur2/relay/value")))  // dynamic user relay control implement edilecek.
  {
    deneme = fbdo.to<const char *>();
    printf("%s\n", deneme);
    if (deneme == "open") 
    {
      digitalWrite(pinRelay, 0);
    }

    if (deneme == "close") 
    {
      digitalWrite(pinRelay, 1);
    }
  }

  else
  {
    Serial.println(fbdo.errorReason());
  }
}

void pwmControl(void)
{
  if (Firebase.RTDB.getInt(&fbdo, F("/UsersData/bmU3MvDjN1RtaC0qk2wkPEVkwur2/pwm/value")))  // dynamic user relay control implement edilecek.
  {
    pwmFromFirebase = fbdo.to<int>();
    ledcWrite(PWMChannel, pwmFromFirebase);
  }

  else
  {
    Serial.println(fbdo.errorReason());
  }
}

void initResetButton(void)
{
  pinMode(buttonPin, INPUT_PULLUP);
  pinMode(15, OUTPUT);
}

void initPWM(void)
{
  ledcSetup(PWMChannel, PWMFreq, PWMResolution);
  ledcAttachPin(allFansPWMPin, PWMChannel);
}

void initAPMode(void)
{
  WiFi.mode(WIFI_STA);
  wifiManager.resetSettings();

  if(wm_nonblocking) 
    wifiManager.setConfigPortalBlocking(false);
    
  wifiManager.setClass("invert");
  wifiManager.setScanDispPerc(true);
  wifiManager.setConfigPortalTimeout(180);

  wifiManager.addParameter(&_USER_EMAIL);
  wifiManager.addParameter(&_USER_PASSWORD);
  wifiManager.addParameter(&_API_KEY);
  wifiManager.addParameter(&_DATABASE_URL);
  wifiManager.setSaveParamsCallback(saveParamsCallback);

  AutoGeneratedAP_Name = wifiManager.autoConnect();  // auto generated AP name from chipid
  if (!AutoGeneratedAP_Name)
  {
    Serial.println("Failed to connect.");
    ESP.restart();
  }

  else 
  {
    Serial.println("Succesfully connected.");
  }
}

void initRelay(void)
{
  pinMode(pinRelay, OUTPUT);
}

void pcnt_init_all_fans(void) 
{
  _pcnt_config_t pcnt_config_fan1, pcnt_config_fan2, pcnt_config_fan3, pcnt_config_fan4;
  pcnt_init_fan_generic(&pcnt_config_fan1, PCNT_UNIT_0, FREQ_PIN_FAN1, &overflow_cnt_fan1, &timer_mux_fan1, &result_fan1, &flag_fan1, &timer_handle_fan1);
  pcnt_init_fan_generic(&pcnt_config_fan2, PCNT_UNIT_1, FREQ_PIN_FAN2, &overflow_cnt_fan2, &timer_mux_fan2, &result_fan2, &flag_fan2, &timer_handle_fan2);
  pcnt_init_fan_generic(&pcnt_config_fan3, PCNT_UNIT_2, FREQ_PIN_FAN3, &overflow_cnt_fan3, &timer_mux_fan3, &result_fan3, &flag_fan3, &timer_handle_fan3);
  pcnt_init_fan_generic(&pcnt_config_fan4, PCNT_UNIT_3, FREQ_PIN_FAN4, &overflow_cnt_fan4, &timer_mux_fan4, &result_fan4, &flag_fan4, &timer_handle_fan4);
}

void setup()
{
  Serial.begin(115200);

  initLcdScreen();
  initResetButton();
  initPWM();
  initSensors();
  initAPMode();
  initWiFi();
  initRelay();

  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) 
  {
    Serial.print(".");
    delay(300);
  }

  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();
  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

  config.database_url = d;
  config.api_key = c;
  auth.user.email = a;
  auth.user.password = b;

  Firebase.reconnectWiFi(true);
  fbdo.setResponseSize(4096);
  config.token_status_callback = tokenStatusCallback;
  config.max_token_generation_retry = 5;
  Firebase.begin(&config, &auth);

  Serial.println("Getting User UID");
  while ((auth.token.uid) == "") 
  {
    Serial.print('.');
    delay(1000);
  }

  uid = auth.token.uid.c_str();
  Serial.print("User UID: ");
  Serial.println(uid);

  for (int i = 0; i < 17; i = i + 8) 
  {
    chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
  }

  Serial.print("Chip ID: ");
  Serial.println(chipId);

  pcnt_init_all_fans();
  initFlowMeter();

  pinMode(FREQ_PIN_FAN1, INPUT);
  pinMode(FREQ_PIN_FAN2, INPUT);
  pinMode(FREQ_PIN_FAN3, INPUT);
  pinMode(FREQ_PIN_FAN4, INPUT);
  pinMode(FREQ_PIN_FAN5, INPUT);
  pinMode(FREQ_PIN_FAN6, INPUT);

  delay(1000);
}

void loop() 
{
  if(wm_nonblocking) 
    wifiManager.process();

  checkIfDisconnected();
  checkIfLongPress();
  relayControl();
  pwmControl();
  calculateFlowMeter();
  firebaseSendData();
  writeLcdScreen();
  delay(1000);
}
