// TODO: ADD ALL FANS RPM
// TODO: ADD ALL TEMPERATURES
// TODO: ADD HUMIDITY
// TODO: ADD FLOWMETER
// TODO: ADD PWM SET CHANGE FAN SPEED / GET FROM FIREBASE
// TODO: ADD FAN SPEED CONTROL WITH TEMPERATURE
// TODO: ADD RELAY CONTROL
// TODO: ADD BUTTON TO RESTART ESP32 WITH LONG PRESS (DEBOUNCE CHECK)
// TODO: ADD A RTOS TASK TO CHECK IF DEVICE DISCONNECTED FROM WIFI AND RECONNECT
// TODO: ADD A RTOS TASK TO CHECK IF DEVICE DISCONNECTED FROM FIREBASE AND RECONNECT
// TODO: INIT LCD SCREEN AND SHOW ALL DATA
// TODO: IMPLEMENT A LOGIC TO LOG DATA ONLINE(CLOUD) AND OFFLINE(SD CARD)
// TODO: IMPLEMENT .h FILE FOR VARIABLES AND FUNCTIONS
// TODO: IMPLEMENT CLASS FOR FANS, TEMPERATURES, HUMIDITY, FLOWMETER, RELAYS, LCD SCREEN, BUTTONS, LEDS, WIFI, FIREBASE, SD CARD (v2.0) / NOT URGENT


// ------------------------ LIBRARIES ------------------------

#include <Arduino.h>
#include <WiFiManager.h>
#include <Firebase_ESP_Client.h>
#include <addons/RTDBHelper.h>
#include <addons/TokenHelper.h>
#include <DallasTemperature.h>
#include <OneWire.h>

#include "driver/pcnt.h"                                                
#include "soc/pcnt_struct.h"

// ------------------------ DEFINES ------------------------

#define FREQ_PIN_FAN1 4
#define oneWireBus 18

// ------------------------ VARIABLES ------------------------

TaskHandle_t wifiTaskHandle = NULL;
TaskHandle_t resetButtonHandle = NULL;
TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t firebaseTaskHandle = NULL;

FirebaseConfig firebaseConfig;
FirebaseData firebaseData;
FirebaseAuth firebaseAuth;
FirebaseJson jsonStatic, jsonDynamic;

WiFiManager wifiManager;
WiFiManagerParameter _USER_EMAIL("a", "Enter your Mail Address: ", "omerfarukoruc35@gmail.com", 100);
WiFiManagerParameter _USER_PASSWORD("b", "Enter your Password:", "deneme", 100);
WiFiManagerParameter _API_KEY("c", "Enter your Firebase API Key: ", "AIzaSyDl-0kC9UBSvSjqFy3HwQt2d3pe4zY5GCs", 100);
WiFiManagerParameter _DATABASE_URL("d", "Enter your Firebase Database URL: ", "btcss-1-default-rtdb.europe-west1.firebasedatabase.app", 100);

String USER_EMAIL, USER_PASSWORD, API_KEY, DATABASE_URL;
String databaseStaticReadingsPath, databaseDynamicReadingsPath;
String parentStaticReadingsPath, parentDynamicReadingsPath;
String UsersData = "/UsersData/", readingsPath = "/readings", staticReadingsPath = "/staticReadings", dynamicReadingsPath = "/dynamicReadings";
String temperatureOutsidePath = "/temperatureOutside";
String uid;
String rpmPath1 = "/fans/fan1";
String rpmPath2 = "/fans/fan2";
String rpmPath3 = "/fans/fan3";
String rpmPath4 = "/fans/fan4";
String rpmPath5 = "/fans/fan5";
String rpmPath6 = "/fans/fan6";

const char *ntpServer = "0.tr.pool.ntp.org"; // dynamic setup the ntp server

uint32_t chipId = 0;
unsigned long sendDataPrevMillis = 0;
int timestamp;

OneWire oneWire(oneWireBus);
DallasTemperature probe(&oneWire);

bool            flag_fan1             = true;                                   
uint32_t        overflow_cnt_fan1     = 0;                                        
volatile double frequency_fan1        = 0;
double          global_frequency_fan1 = 0;
uint16_t        result_fan1           = 0;

esp_timer_create_args_t timer_args_fan1;
esp_timer_handle_t timer_handle_fan1;
portMUX_TYPE timer_mux_fan1 = portMUX_INITIALIZER_UNLOCKED;

pcnt_config_t pcnt_config_fan1 = 
{
  .pulse_gpio_num    = FREQ_PIN_FAN1,
  .ctrl_gpio_num     = -1,
  .lctrl_mode        = PCNT_MODE_KEEP,
  .hctrl_mode        = PCNT_MODE_KEEP,
  .pos_mode          = PCNT_COUNT_INC,
  .neg_mode          = PCNT_COUNT_INC,
  .counter_h_lim     = 20000,
  .counter_l_lim     = 0,
  .unit              = PCNT_UNIT_0, 
  .channel           = PCNT_CHANNEL_0
};

// ------------------------ FUNCTION DEFINES ------------------------

void configModeCallback(WiFiManager *myWiFiManager);
void initWiFi();
void WiFiEvent(WiFiEvent_t event);
void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info);
void setupGPIO();
void firebaseConnect();

void wifiTask(void *pvParameters);
void resetButtonTask(void *pvParameters); 
void fan1Task(void *pvParameters);
void firebaseTask(void *pvParameters);
void pcnt_get_counter_fan1(void *p);
void pcnt_init_fan1(void);
void IRAM_ATTR pcnt_event_handler_fan1(void *arg);

float calculateRpmFan1();
unsigned long getTime();

bool isWiFiConnected;
bool wifiManagerNonblocking = false;
bool isInConfigMode = false;
bool isFirebaseConnected = false;

void IRAM_ATTR pcnt_event_handler_fan1(void *arg)
{
  portENTER_CRITICAL_ISR(&timer_mux_fan1);
  overflow_cnt_fan1++; 
  PCNT.int_clr.val = BIT(PCNT_UNIT_0);
  portEXIT_CRITICAL_ISR(&timer_mux_fan1);
}     

void pcnt_get_counter_fan1(void *p) 
{ 
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_get_counter_value(PCNT_UNIT_0, (int16_t*) &result_fan1); 
  flag_fan1 = true;
}

void pcnt_init_fan1(void)                                                     
{  
  pinMode(FREQ_PIN_FAN1, INPUT_PULLUP); // TODO : INPUT 

  pcnt_unit_config(&pcnt_config_fan1);
  pcnt_isr_register(pcnt_event_handler_fan1, NULL, 0, NULL);
  pcnt_set_filter_value(PCNT_UNIT_0, 1000);
  pcnt_filter_enable(PCNT_UNIT_0);
  pcnt_counter_pause(PCNT_UNIT_0);
  pcnt_counter_clear(PCNT_UNIT_0);
  pcnt_event_enable(PCNT_UNIT_0, PCNT_EVT_H_LIM);
  pcnt_counter_resume(PCNT_UNIT_0);

  timer_args_fan1.callback = pcnt_get_counter_fan1;
  timer_args_fan1.arg      = NULL;
  timer_args_fan1.name     = "one shot timer";

  if(esp_timer_create(&timer_args_fan1, &timer_handle_fan1) != ESP_OK) 
  {
    //ESP_LOGE(TAG, "timer create");
  }

  timer_args_fan1.callback = pcnt_get_counter_fan1; 
  esp_timer_create(&timer_args_fan1, &timer_handle_fan1);
}

void saveParamsCallback()
{
  USER_EMAIL    = _USER_EMAIL.getValue();
  USER_PASSWORD = _USER_PASSWORD.getValue();
  API_KEY       = _API_KEY.getValue();
  DATABASE_URL  = _DATABASE_URL.getValue();
  isInConfigMode = true;
}

void initWiFi() 
{
  WiFi.onEvent(WiFiEvent);
  WiFi.onEvent(WiFiGotIP, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_GOT_IP);

  WiFiEventId_t eventID = WiFi.onEvent([](WiFiEvent_t event, WiFiEventInfo_t info)
  {
    Serial.print("WiFi lost connection. Reason: ");
    Serial.println(info.wifi_sta_disconnected.reason);
  }, WiFiEvent_t::ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  Serial.print("WiFi Event ID: ");
  Serial.println(eventID);

  wifiManager.resetSettings();

  wifiManager.setDarkMode(true);
  wifiManager.setScanDispPerc(true);
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setShowPassword(true);
  wifiManager.setTitle("Wifi Settings");

  wifiManager.addParameter(&_USER_EMAIL);
  wifiManager.addParameter(&_USER_PASSWORD);
  wifiManager.addParameter(&_API_KEY);
  wifiManager.addParameter(&_DATABASE_URL);

  if(wifiManagerNonblocking)
    wifiManager.setConfigPortalBlocking(false);
  wifiManager.setSaveParamsCallback(saveParamsCallback);

  isWiFiConnected = wifiManager.autoConnect("ESP32-AP");

  if (!isWiFiConnected)
  {
    Serial.println("Failed to connect.");
    ESP.restart();
  }

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

void wifiTask(void *pvParameters) 
{
  while(1) 
  {
    if(wifiManagerNonblocking)
      wifiManager.process();
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void resetButtonTask(void *pvParameters)
{
  while(1) 
  {
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void fan1Task(void *pvParameters) 
{
  while (1) 
  {
    calculateRpmFan1();
    vTaskDelay(1000 / portTICK_RATE_MS);
  }
}

void firebaseTask(void *pvParameters) 
{
  while (1) 
  {
    if (isWiFiConnected && !isFirebaseConnected) 
    {
      firebaseConnect();
    }

    vTaskDelay(500 / portTICK_RATE_MS); // maybe thats not necessary

    databaseStaticReadingsPath = UsersData + uid + readingsPath + staticReadingsPath;

    if(isWiFiConnected && isFirebaseConnected)
    {

      if (Firebase.ready())
      {
        sendDataPrevMillis = millis();
        timestamp = getTime();

        Serial.print("time: ");
        Serial.println(timestamp);

        parentStaticReadingsPath = databaseStaticReadingsPath;
  
        jsonStatic.set("/fans/fan1", global_frequency_fan1);
        Serial.printf("Sending fan RPM value into Firebase %s\n", Firebase.RTDB.setJSON(&firebaseData, parentStaticReadingsPath.c_str(), &jsonStatic) ? "" : firebaseData.errorReason().c_str());
        vTaskDelay(4000 / portTICK_RATE_MS);
      }
      vTaskDelay(500 / portTICK_RATE_MS);
    }
  }
}

void firebaseConnect() 
{
  //Firebase.reconnectNetwork(true);

  firebaseAuth.user.email = USER_EMAIL;
  firebaseAuth.user.password = USER_PASSWORD;
  firebaseConfig.database_url = DATABASE_URL;
  firebaseConfig.api_key = API_KEY;
  firebaseConfig.token_status_callback = tokenStatusCallback;
  firebaseConfig.max_token_generation_retry = 5;
  firebaseData.setResponseSize(4096);
  // TODO: TCP Keep Alive

  Firebase.begin(&firebaseConfig, &firebaseAuth);

  if (Firebase.ready()) 
  {
    uid = firebaseAuth.token.uid.c_str();
    Serial.print("User UID: ");
    Serial.println();
    
    Serial.println("Firebase connection successful!");
    neopixelWrite(RGB_BUILTIN, 0, 64, 64); // means connected to firebase

    for (int i = 0; i < 17; i = i + 8) 
    {
      chipId |= ((ESP.getEfuseMac() >> (40 - i)) & 0xff) << i;
    }
    Serial.print("Chip ID: ");
    Serial.println(chipId);

    isFirebaseConnected = true;
  } 
  
  else 
  {
    Serial.println("Firebase connection failed!");
    isFirebaseConnected = false;
  }
}

void WiFiEvent(WiFiEvent_t event)
{
  // TODO: All events should have a different led color blinking.
  switch (event) 
  {
    case ARDUINO_EVENT_WIFI_READY: 
      Serial.println("*** 1. WiFi interface ready ***");
      break;

    case ARDUINO_EVENT_WIFI_SCAN_DONE:
      Serial.println("*** 2. Completed scan for access points ***");
      break;

    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.println("*** 3. WiFi client started ***");
      break;

    case ARDUINO_EVENT_WIFI_STA_STOP:
      Serial.println("*** 4. WiFi clients stopped ***");
      for(int i = 0; i < 5; i++)
      {
        neopixelWrite(RGB_BUILTIN, 64, 0 , 0);
        delay(250);
        neopixelWrite(RGB_BUILTIN, 0, 64, 0); 
        delay(250);
      }
      neopixelWrite(RGB_BUILTIN, 64, 0, 64); 
      break;

    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.println("*** 5. Connected to access point ***");
      break;

    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED:
      if(isInConfigMode)
      {
        Serial.println("*** 6. Disconnected from WiFi access point ***");
        for(int i = 0; i < 5; i++)
        {
          neopixelWrite(RGB_BUILTIN, RGB_BRIGHTNESS, 0 , 0);
          delay(250);
          neopixelWrite(RGB_BUILTIN, 0, RGB_BRIGHTNESS, 0); 
          delay(250);
        }
        neopixelWrite(RGB_BUILTIN, 64, 0, 64);
      }
      break;

    case ARDUINO_EVENT_WIFI_STA_AUTHMODE_CHANGE:
      Serial.println("*** 7. Authentication mode of access point has changed ***");
      break;

    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.print("*** 8. Obtained IP address: ");
      Serial.println(WiFi.localIP());
      break;

    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.println("*** 9. Lost IP address and IP address is reset to 0 ***");
      break;

    case ARDUINO_EVENT_WIFI_AP_START:
      Serial.println("*** 10. WiFi access point started ***");
      break;

    case ARDUINO_EVENT_WIFI_AP_STOP:
      Serial.println("*** 11. WiFi access point stopped ***");
      break;

    case ARDUINO_EVENT_WIFI_AP_STACONNECTED:
      Serial.println("*** 12. Client connected ***");
      break;

    case ARDUINO_EVENT_WIFI_AP_STADISCONNECTED:
      Serial.println("*** 13. Client disconnected ***");
      break;

    case ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED:
      Serial.println("*** 14. Assigned IP address to client ***");
      break;

    case ARDUINO_EVENT_WIFI_AP_PROBEREQRECVED:
      Serial.println("*** 15. Received probe request ***");
      break;

    default:
      break;
  }
}

void WiFiGotIP(WiFiEvent_t event, WiFiEventInfo_t info)
{
    Serial.println("WiFi connected");
    Serial.println("IP address: ");
    Serial.println(IPAddress(info.got_ip.ip_info.ip.addr));
}

void setupGPIO()
{
  pinMode(RGB_BUILTIN, OUTPUT);
  probe.begin();
}

void setup() 
{
  WiFi.mode(WIFI_AP);
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  delay(1000);
  setupGPIO();
  initWiFi();

  pcnt_init_fan1();

  xTaskCreatePinnedToCore(wifiTask,        "WiFi Task",         10000, NULL, 1, &wifiTaskHandle,     0);
  xTaskCreatePinnedToCore(resetButtonTask, "Reset Button Task", 10000, NULL, 1, &resetButtonHandle , 0);
  xTaskCreatePinnedToCore(firebaseTask,    "Firebase Task",     10000, NULL, 1, &firebaseTaskHandle, 0);
  xTaskCreatePinnedToCore(fan1Task,        "Fan1 Task",         10000, NULL, 1, &sensorTaskHandle,   0);
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
    pcnt_counter_clear(PCNT_UNIT_0);
    esp_timer_start_once(timer_handle_fan1, 1000000);
  }
  global_frequency_fan1 = frequency_fan1 * 15;
  return frequency_fan1 * 15;
}

void loop() 
{
  // printf("Free Heap: %d\n", esp_get_free_heap_size());
  // printf("Min Free Heap: %d\n", esp_get_minimum_free_heap_size());
}
