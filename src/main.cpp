#include <Arduino.h>
#include "credentials/credentials.h"

#include <SPI.h>
#include <time.h>

#define DEBUG

//SPI
#define SCK  18   
#define MISO 19
#define MOSI 23

unsigned long sketchTime = 0;

////////////////////////////////
// OLED
////////////////////////////////
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_I2CDevice.h>

//OLED pins
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire);

boolean invert_display = false;

void initOLED () {
  Wire.begin(SDA, SCL);
  if(!display.begin(0x02, 0x3c, false, false)) {
  #ifdef DEBUG
    Serial.println(F("SSD1306 allocation failed"));
  #endif
    for(;;); // Don't proceed, loop forever
  }
  display.ssd1306_command(/*SSD1306_DISPLAYON*/ 0xAF);
  display.clearDisplay();
  display.setRotation(0);
  display.setTextColor(1);
  display.setTextSize(1);
  // display.startscrollright(0x00, 0x0F);
  display.display();
}

#define LORA_HEIGHT   24
#define LORA_WIDTH    16
static const unsigned char PROGMEM LoRaLogo[] =
{ B00000011, B11000000,
  B00111111, B11111100,
  B11110000, B00001111,
  B01000011, B11000010,
  B00011111, B11111000,
  B00110000, B00001100,
  B00000111, B11100000,
  B00011000, B00011000,
  B00000011, B11000000,
  B00000110, B01100000,
  B00001100, B00110000,
  B00001100, B00110000,
  B00001100, B00110000,
  B00001100, B00110000,
  B00000110, B01100000,
  B00000011, B11000000,
  B00011000, B00011000,
  B00000111, B11100000,
  B00110000, B00001100,
  B00011111, B11111000,
  B01000011, B11000010,
  B11110000, B00011111,
  B00111111, B11111100,
  B00000111, B11100000 };


#define WIFI_HEIGHT   13
#define WIFI_WIDTH    16
static const unsigned char PROGMEM WiFiLogo[] =
{ B00000111, B11100000,
  B00111111, B11111100,
  B01111000, B00011110,
  B11100111, B11100111,
  B00011111, B11111000,
  B00111100, B00111100,
  B00000011, B11000000,
  B00000111, B11100000,
  B00000100, B00000000,
  B00000001, B10000000,
  B00000011, B11000000,
  B00000011, B11000000,
  B00000001, B10000000 };

#define NOWIFI_HEIGHT   13
#define NOWIFI_WIDTH    16
static const unsigned char PROGMEM noWiFiLogo[] =
{ B11100111, B11100000,
  B01111111, B11111100,
  B01111000, B00011110,
  B11111111, B11100111,
  B00011111, B11111000,
  B00111111, B00111100,
  B00000011, B11000000,
  B00000111, B11100000,
  B00000100, B11100000,
  B00000001, B11110000,
  B00000011, B11111000,
  B00000011, B11011100,
  B00000001, B10001110 };

////////////////////////////////
// RTC
// ////////////////////////////////
// #include <RTClib.h>
// RTC_DS3231 rtc;

// DateTime lastReceived;

////////////////////////////////
// LED
////////////////////////////////
#define LED_PIN 2

void blink(int nb, int ms = 100) {
  for (int i=0 ; i<nb ; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(ms);
    digitalWrite(LED_PIN, LOW);
    delay(ms);
  }
}

////////////////////////////////
// Voltage & Current SENSOR
////////////////////////////////
#define LIPOBAT_PIN 27

uint32_t lipoBat=0;
int16_t maxGlobalCurrent = 0;
float currentList[10] = {0};
float averageCurrent = 0;
//long int numiter = 0;
long int numCollect = 0;

bool collectData();

////////////////////////////////
// Program state
////////////////////////////////
enum STATE {
  INIT,
  WAIT,
  WAKEUP,
  RECEIVING,
  MANAGE,
  SEND,
  BROKER,
  DATA,
  SLEEP,
  ERROR
};

volatile STATE state = INIT;
volatile STATE prevState;

void stateToStr(char *text){
  switch(state) {
    case INIT:      strcpy(text, "INIT"); break;
    case WAIT:      strcpy(text, "WAIT"); break;
    case WAKEUP:    strcpy(text, "WAKEUP"); break;
    case RECEIVING: strcpy(text, "RECEIVING"); break;
    case MANAGE:    strcpy(text, "MANAGE"); break;
    case SEND:      strcpy(text, "SEND"); break;
    case BROKER:    strcpy(text, "BROKER"); break;
    case DATA:      strcpy(text, "DATA"); break;
    case SLEEP:     strcpy(text, "SLEEP"); break;
    case ERROR:     strcpy(text, "ERROR"); break;
    default:        strcpy(text, "UNKNOWN"); break;
  }
}


////////////////////////////////
// JSon
////////////////////////////////
#include <ArduinoJson.h>

//StaticJsonDocument<200> doc;

////////////////////////////////
// LORA
////////////////////////////////
#include <LoRa.h>
#define BAND 870E6
#define SF   7
#define SS   4
#define RST  33
#define DIO0 32
// #define DIO1 35

int packetSize;

void onReceive(int packetSize) {
  // received a packet
  Serial.printf("------ Received Packet : %d bytes\n",packetSize);
  state = RECEIVING;
}

////////////////////////////////
// WiFi
////////////////////////////////
#include <WiFi.h>
#include <WiFiClient.h>

WiFiClient wifiClient; 

void onWifiEvent (system_event_id_t event, system_event_info_t info) {
#ifdef DEBUG
    Serial.printf ("[WiFi-event] event: %d - ", event);
    switch (event) {
    case SYSTEM_EVENT_WIFI_READY:    
      Serial.printf ("WiFi ready\n"); 
      break;
    case SYSTEM_EVENT_STA_START:     
      Serial.printf ("WiFi start\n"); 
      break;
    case SYSTEM_EVENT_STA_CONNECTED:
      Serial.printf ("Connected to %s. Asking for IP address\n", info.connected.ssid);
      break;
    case SYSTEM_EVENT_STA_STOP:
      Serial.printf ("Station Stop\n");
      break;
    case SYSTEM_EVENT_STA_LOST_IP:
      Serial.printf ("Lost IP\n");
      break;
    case SYSTEM_EVENT_STA_GOT_IP:
      Serial.printf ("Got IP: %s\n", IPAddress (info.got_ip.ip_info.ip.addr).toString ().c_str ());
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED: 
      Serial.printf ("Disconnected from SSID: %s\n", info.disconnected.ssid);
      break;
		default:
      Serial.printf ("Unknown event\n");
      break;
    }
#endif
}


bool WiFiConnect() {
  #ifdef DEBUG
  Serial.println("--> WiFi Connect");
  #endif
  bool retval = false;
  if(!WiFi.isConnected()) WiFi.begin(AP_NAME, AP_PASSRHRASE);
  for (int t=0; t<1000; t++) {
    if (WiFi.isConnected()) {
      retval = true;
      break;
    }
    delay(10);
  }
  #ifdef DEBUG
  Serial.print("--> WiFi Connect End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif
  return retval;
}

////////////////////////////////
// NTP
////////////////////////////////
#include <WiFiUdp.h>
#include <NTPClient.h>

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "fr.pool.ntp.org", 3600, 60000);

time_t lastReceived;

////////////////////////////////
// MQTT
////////////////////////////////
#include <PubSubClient.h>

const char* mqtt_server_ip = MQTT_SERVER;
const int   mqtt_server_port = MQTT_PORT;
const char* mqtt_server_username = MQTT_LOGIN ;
const char* mqtt_server_password = MQTT_PASSWD ;

PubSubClient mqttClient(wifiClient);

String message;
String device = "N/A";

bool sendBrokerData();
bool sendMQTT();
bool managePacket(String message);

void callback(char* topic, byte* payload, unsigned int length) {
  #ifdef DEBUG
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif
}

bool reconnect();

void displayStats();

////////////////////////////////////////////////////
// OTA
////////////////////////////////////////////////////
// #include <ArduinoOTA.h>

/////////////////////////////////////////
// ADS1115
/////////////////////////////////////////
// #include "Adafruit_ADS1015.h"
#include "ADS1115.h"

#define I2Caddress 0x48
#define ADS1115_READY_PIN 25

ADS1115 adc0(/*ADS1115_DEFAULT_ADDRESS*/ 0x48);

void initADC(){
  #ifdef DEBUG
  Serial.println("Testing device connections...");
  Serial.println(adc0.testConnection() ? "ADS1115 connection successful" : "ADS1115 connection failed");
  #endif
  
  adc0.initialize();
  adc0.setMode(/*ADS1115_MODE_SINGLESHOT*/ 0x01);
  adc0.setRate(/*ADS1115_RATE_64*/ 0x03);
  adc0.setGain(/*ADS1115_PGA_6P144*/ 0x00);
  
  // ALERT/RDY pin will indicate when conversion is ready
  // pinMode(ADS1115_READY_PIN,INPUT);
  // adc0.setConversionReadyPinMode();

  // To get output from this method, you'll need to turn on the 
  //#define ADS1115_SERIAL_DEBUG // in the ADS1115.h file
  // adc0.showConfigRegister();
  // Serial.print("HighThreshold="); Serial.println(adc0.getHighThreshold(),BIN);
  // Serial.print("LowThreshold="); Serial.println(adc0.getLowThreshold(),BIN);
}


////////////////////////////////
// Interrupts
////////////////////////////////
#include <esp_sleep.h>
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define BUTTON_PIN 26

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

hw_timer_t *timer = NULL;

hw_timer_t *timer2 = NULL;

struct Button {
    const uint8_t PIN;
    uint32_t numberKeyPresses;
    bool pressed;
};

Button button1 = {BUTTON_PIN, 0, false};

void IRAM_ATTR isr(void* arg) {
    Button* s = static_cast<Button*>(arg);
    s->numberKeyPresses += 1;
    s->pressed = true;
    state = SEND;
}

void IRAM_ATTR LoraIRQ() {
  portENTER_CRITICAL_ISR(&mux);
  state = WAKEUP;
  portEXIT_CRITICAL_ISR(&mux);
}

#define TIME_TO_SEND_BROKER_DATA 60

void IRAM_ATTR onBrokerTimer() {
  portENTER_CRITICAL_ISR(&mux);
  //numiter++;
  prevState=state;
  state = BROKER;
  portEXIT_CRITICAL_ISR(&mux);
}

#define TIME_TO_COLLECT_DATA 9 * 5

void IRAM_ATTR onDataTimer() {
  portENTER_CRITICAL_ISR(&mux);
  prevState = state;
  numCollect++;
  state = DATA;
  portEXIT_CRITICAL_ISR(&mux);
}

void print_wakeup_reason(){

  #ifdef DEBUG
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause(); 

  switch(wakeup_reason)
  {
    case ESP_SLEEP_WAKEUP_EXT0      : Serial.println("Wakeup caused by external signal using RTC_IO");  break;
    case ESP_SLEEP_WAKEUP_EXT1      : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER     : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD  : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP       : Serial.println("Wakeup caused by ULP program"); break;
    case ESP_SLEEP_WAKEUP_UNDEFINED : Serial.println("Wakeup cause is undefined"); break;
    default : Serial.println("Wakeup was not caused by deep sleep"); break;
    
  }
  #endif
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  // pinMode(LIPOBAT_PIN, INPUT_PULLUP);
  // pinMode(LED_PIN, OUTPUT);

  // pinMode(button1.PIN, INPUT_PULLUP);
  // attachInterruptArg(button1.PIN, isr, &button1, FALLING);

  //attachInterrupt(digitalPinToInterrupt(DIO0), LoraIRQ, RISING);

  print_wakeup_reason();
  //pinMode(GPIO_NUM_32,INPUT_PULLDOWN);

  initOLED();

  // init timer to send broker Data 
  timer = timerBegin(1, 80, true);
  timerAttachInterrupt(timer, &onBrokerTimer, true);
  timerAlarmWrite(timer, TIME_TO_SEND_BROKER_DATA * uS_TO_S_FACTOR, true);
  timerAlarmEnable(timer);

  // init timer to collect Data 
  timer2 = timerBegin(2, 80, true);
  timerAttachInterrupt(timer2, &onDataTimer, true);
  timerAlarmWrite(timer2, TIME_TO_COLLECT_DATA * uS_TO_S_FACTOR, true);
  timerAlarmEnable(timer2);

  // WiFi init
  WiFi.begin(AP_NAME, AP_PASSRHRASE);
  WiFi.onEvent (onWifiEvent);

  // Init MQTT Client
  mqttClient.setServer(mqtt_server_ip,mqtt_server_port);
  //mqttLocalClient.setServer(mqtt_local_ip,mqtt_local_port);
  
  // Init LoRa 
  LoRa.setPins(SS,RST,DIO0);
  while (!LoRa.begin(BAND)) {
    #ifdef DEBUG
    Serial.println("Starting LoRa failed!");
    #endif
    state = ERROR;
    delay(2000);
  }

  LoRa.onReceive(onReceive);
  LoRa.receive();
  #ifdef DEBUG
  Serial.println("LoRa started OK in recieve mode");
  #endif

  // NTPClient init
  timeClient.begin();
  #ifdef DEBUG
  Serial.println("NTPClient started");
  #endif

  initADC();
  
  #ifdef DEBUG
  Serial.println("--------- SETUP ENDED ---------");
  #endif

}

////////////////////////////////
// LOOP
////////////////////////////////

void loop() {

  sketchTime = millis();

  switch (state) {
  case INIT:
    if(WiFi.isConnected()) timeClient.update();
    delay(1000);
    state=WAIT;
    break;
  
  case ERROR:
    if(WiFi.isConnected()) WiFi.disconnect();
    //if(LoRa.available()) LoRa.flush();
    break;

  case WAIT:
    if(WiFi.isConnected()) timeClient.update();
    break;

  case RECEIVING:
    state = MANAGE;
    message.clear();
    while (LoRa.available()) {
      message += (char)LoRa.read();
      if(message.length() > 200) {
        #ifdef DEBUG
        Serial.println("Message too long !");
        #endif
        LoRa.flush();
        state=ERROR;
        break;
      }
    }
    #ifdef DEBUG
    Serial.println(message);
    #endif
    break;
  
  case MANAGE:
    invert_display = invert_display ? false : true; 
    lastReceived = timeClient.getEpochTime();
    state=SEND;
    if (!managePacket(message)) { 
      state = ERROR;
      device = "MSG Err";
    }
  break;

  case SEND:
    state = WAIT;
    if (!sendMQTT()) { 
      state = ERROR;
      device = "SEND Err";
    }
    WiFi.disconnect(true);
    break;

  case BROKER:
    state=WAIT;
    displayStats();
    if (!sendBrokerData()) {
      state = ERROR;
      device = "Broker Err";
    }
    WiFi.disconnect(true);
    break;

  case DATA:
    //Serial.println("--> collect data");
    collectData();
    state = prevState;
    break;

  case SLEEP:
    display.clearDisplay();
    display.display();
    Serial.flush();
    LoRa.flush();
    LoRa.end();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32,1);
    esp_deep_sleep_start();
    state=WAIT;
    break;

  default:
    displayStats();
    #ifdef DEBUG
      Serial.println("State unknown !");
    #endif
    break;
  }

  displayStats();

} 


////////////////////////////////
// Manage LoRa / JSon Packet
////////////////////////////////

bool managePacket(String message) {

  #ifdef DEBUG
  Serial.println("--> Manage Packet");
  Serial.printf("RSSI: %d SNR: %.2f\n",LoRa.packetRssi(), LoRa.packetSnr());
  #endif
  bool retval=true;

  StaticJsonDocument<200> doc;
  DeserializationError Derror = deserializeJson(doc, message);
  #ifdef DEBUG
  Serial.print("Deserialization : "); Serial.println(Derror.c_str());
  #endif

  if (Derror) {
    #ifdef DEBUG
    Serial.println("Deserialization FAILED !");
    #endif
    device = "unknown";
    retval=false;
  } else {
    String dev = doc["device"];
    device = dev;  
  }

  #ifdef DEBUG  
  Serial.print("device = "); Serial.print(device);
  size_t docsize = doc.size();
  Serial.printf(" with %d elements\n",docsize);
  Serial.print("--> Manage Packet End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif

  return retval;
}

////////////////////////////////
// Send data
////////////////////////////////

#define WAIT 150

bool sendMQTT() {
  #ifdef DEBUG
  Serial.println("--> Publish MQTT");
  #endif
  bool retval = true;
  StaticJsonDocument<200> doc;
  char topic[64];
  //char topicLocal[64];
  char payload[64];

  if (!mqttClient.connected()) reconnect();
  //if (!mqttLocalClient.connected()) reconnect();

  DeserializationError Derror = deserializeJson(doc, message);
  if (Derror) {
    #ifdef DEBUG
    Serial.println("Deserialization FAILED ");
    #endif
    return false;
  } else {
    Serial.println("Deserialization OK ");
  }

  String dev = doc["device"];
  device = dev;

  JsonObject obj = doc.as<JsonObject>();

  for (JsonPair p : obj) {
    if (strcmp(p.key().c_str(),"device") == 0) continue; 

    //sprintf(topic,"%s/f/%s.%s",mqtt_server_username,device.c_str(),p.key().c_str());
    sprintf(topic,"home/%s/%s",device.c_str(),p.key().c_str());

    #ifdef DEBUG
    //Serial.printf("topic External = %s\n",topic);
    Serial.printf("topic = %s\n",topic);
    #endif

    if (p.value().is<char*>()) {
      const char* s = p.value();
      sprintf(payload,"%s",s);
    }

    if(p.value().is<float>()) {
      float f = p.value();
      sprintf(payload,"%.4f",f);
    }

    #ifdef DEBUG
    Serial.printf("payload External = %s\n",payload );
    #endif

    if (!mqttClient.publish(topic,payload)) {
      Serial.println("external publish failed !");
      retval=false;
    }

    /*if (!mqttLocalClient.publish(topicLocal,payload)) {
      Serial.println("local publish failed !");
      retval=false;
    }*/
  }
  delay(WAIT);


  sprintf(topic,"home/%s/snr",device.c_str());
  sprintf(payload,"%.2f",LoRa.packetSnr());
  if (!mqttClient.publish(topic,payload)) retval = false;
  //if (!mqttLocalClient.publish(topicLocal,payload)) retval = false;
  delay(WAIT);

  sprintf(topic,"home/%s/rssi", device.c_str());
  sprintf(payload,"%d",LoRa.packetRssi());
  if (!mqttClient.publish(topic,payload)) retval = false;
  //if (!mqttLocalClient.publish(topicLocal,payload)) retval = false;
  delay(WAIT);

  sprintf(topic,"home/%s/frequencyerror", device.c_str());
  sprintf(payload,"%li",LoRa.packetFrequencyError());
  if (!mqttClient.publish(topic,payload)) retval = false;
  //if (!mqttLocalClient.publish(topicLocal,payload)) retval = false;
  delay(WAIT);


  #ifdef DEBUG
  Serial.print("--> Publish MQTT End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif
  return retval;
}


////////////////////////////////
// Collect data
////////////////////////////////

bool collectData(){
  int16_t VCC     = adc0.getConversionP0GND();
  int16_t vGlobal = adc0.getConversionP2GND();
  float GlobalmA  = (VCC - vGlobal) * adc0.getMvPerCount();

  currentList[numCollect % 10] = GlobalmA;
  averageCurrent = 0;

  for (int i=0; i<10; i++) {
    averageCurrent =  averageCurrent + currentList[i];
  }
  averageCurrent =  averageCurrent / 10;
  return true;
}

////////////////////////////////
// Send broker LoRa / JSon Packet
////////////////////////////////

bool sendBrokerData() {
  #ifdef DEBUG
  Serial.println("--> Publish Broker stats");
  #endif
  bool retval=true;

  char topic[64];
  char payload[64];

  int16_t VCC     = adc0.getConversionP0GND();;
  //int16_t vGlobal = adc0.getConversionP2GND();
  //float GlobalmA  = (VCC - vGlobal) * adc0.getMvPerCount();

  if (!mqttClient.connected()) reconnect();
  
  const char dev[]="broker";
  
  //sprintf(topic,"%s/f/%s.vbat", mqtt_server_username,dev);
  sprintf(topic,"home/%s/vbat", dev);
  sprintf(payload,"%.4f", VCC * adc0.getMvPerCount() / 1000 );
  if (!mqttClient.publish(topic,payload)){
    #ifdef DEBUG
      Serial.printf("publish to %s with port %d failed !\n", mqtt_server_ip, mqtt_server_port);
    #endif  
    retval=false;
  #ifdef DEBUG
    Serial.printf("topic:[%s] payload:[%s]\n",topic,payload);
  #endif
  }
  delay(WAIT);

  // sprintf(topic,"%s/f/%s.vbatglobal", mqtt_server_username,dev);
  // sprintf(payload,"%.4f", vGlobal * adc0.getMvPerCount() / 1000 );
  // if (!mqttClient.publish(topic,payload)) retval=false;
  // #ifdef DEBUG
  // Serial.printf("topic:[%s] payload:[%s]\n",topic,payload);
  // #endif
  // delay(WAIT);

  // sprintf(topic,"%s/f/%s.last", mqtt_server_username,dev);
  // tm *myTimeLocal = localtime(&lastReceived);
  // sprintf(payload,"%s - %02d:%02d:%02d %02d/%02d",device.c_str(), myTimeLocal->tm_hour, myTimeLocal->tm_min, myTimeLocal->tm_sec, myTimeLocal->tm_mday, myTimeLocal->tm_mon + 1);
  // #ifdef DEBUG
  // Serial.printf("topic:[%s] payload:[%s]\n",topic,payload);
  // #endif
  // if (!mqttClient.publish(topic,payload)) retval=false;
  // delay(WAIT);

  sprintf(topic,"home/%s/current", dev);
  sprintf(payload,"%.4f",averageCurrent);
  #ifdef DEBUG
    Serial.printf("topic:[%s] payload:[%s]\n",topic,payload);
  #endif
  if (!mqttClient.publish(topic,payload)) {
    #ifdef DEBUG
      Serial.printf("publish to %s with port %d failed !\n", mqtt_server_ip, mqtt_server_port);
    #endif
    retval=false;
  }
  delay(WAIT);

  // sprintf(topic,"%s/f/%s.iter", mqtt_server_username,dev);
  // sprintf(payload,"%li",numiter);
  // #ifdef DEBUG
  // Serial.printf("topic:[%s] payload:[%s]\n",topic,payload);
  // #endif
  // if (!mqttClient.publish(topic,payload)) retval=false;
  // delay(WAIT);

  #ifdef DEBUG
  Serial.print("--> Publish Broker stats END : "); Serial.println(retval ? "OK" : "FAILED");
  #endif
  return retval;
}

////////////////////////////////
// MQTT specific
////////////////////////////////

bool reconnect() {
  bool retval = true;

  #ifdef DEBUG
  Serial.println("--> reconnect MQTT");
  #endif

  WiFiConnect();

  Serial.print("Attempting MQTT connection...");

  String clientId = "ESP32Logger-";
  clientId += String(random(0xffff), HEX);

  for (int i=1 ; i < 11 ; i++) {
    if (mqttClient.connect(clientId.c_str(),MQTT_LOGIN,MQTT_PASSWD )) {
      #ifdef DEBUG
        Serial.println("external connected");
      #endif
      retval=true;
      break;
    } else {
      #ifdef DEBUG
        Serial.print("failed, rc=");
        Serial.print(mqttClient.state());
        Serial.print(" try="); Serial.print(i);
        Serial.println(" try again in 1 seconds");
      #endif
      retval = false;
      delay(1000);
    }
    return retval;
  }

  #ifdef DEBUG
  Serial.print("--> reconnect MQTT End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif

  return true;
}

////////////////////////////////
// Display Statistics
////////////////////////////////

void displayStats() {
  //display.ssd1306_command(SSD1306_DISPLAYON);
  // #ifdef DEBUG
  // Serial.printf(".");
  // #endif
  display.clearDisplay();
  //display.stopscroll();
  display.invertDisplay(invert_display);
  display.setRotation(2);
  display.setCursor(0,0);
  char strstate[16]; stateToStr(strstate);
  display.printf("Broker - %s\n",strstate);

  display.printf("Dev  %s\n",device.c_str());
  
  if (timeClient.getSeconds() % 2) display.fillCircle(display.width()-4, display.height()-4,2,0x1);
  
  time_t nowTime = timeClient.getEpochTime();
  tm *n = localtime(&nowTime);
  display.printf("Time %02d:%02d:%02d %02d/%02d\n",n->tm_hour,n->tm_min,n->tm_sec,n->tm_mday,n->tm_mon+1);

  tm *myTimeLocal = localtime(&lastReceived);
  display.printf("Last %02d:%02d:%02d\n", myTimeLocal->tm_hour, myTimeLocal->tm_min, myTimeLocal->tm_sec );
  display.printf("Rx   %3db\n",message.length() );
  display.printf("RSNR %d %.2f\n",LoRa.packetRssi(),LoRa.packetSnr() );
  //display.printf("FqErr: %li\n",LoRa.packetFrequencyError());

  if (LoRa.available()) display.drawBitmap(display.width() - LORA_WIDTH, /*display.height()/2*/ 40 - LORA_HEIGHT/2, LoRaLogo, LORA_WIDTH, LORA_HEIGHT, 0x1);
  // if (!mqttClient.connected()) { display.printf("MQTT : No Cnx\n"); } else {display.printf("MQTT : connected\n"); }
  // switch (mqttClient.state()) {
  //   case -4 /*MQTT_CONNECTION_TIMEOUT*/    : display.printf("MQTT cnx timeout!\n"); break;
  //   case -3 /*MQTT_CONNECTION_LOST*/       : display.printf("MQTT cnx lost!\n"); break;
  //   case -2 /*MQTT_CONNECT_FAILED*/        : display.printf("MQTT cnx failed!\n"); break;
  //   case -1 /*MQTT_DISCONNECTED*/          : display.printf("MQTT disconnected\n"); break;
  //   case 0 /*MQTT_CONNECTED*/              : display.printf("MQTT connected\n"); break;
  //   case 1 /*MQTT_CONNECT_BAD_PROTOCOL*/   : display.printf("MQTT Bad Protocol!\n"); break;
  //   case 2 /*MQTT_CONNECT_BAD_CLIENT_ID*/  : display.printf("MQTT BAD ID\n"); break;
  //   case 3 /*MQTT_CONNECT_UNAVAILABLE*/    : display.printf("MQTT unavailable\n"); break;
  //   case 4 /*MQTT_CONNECT_BAD_CREDENTIALS*/: display.printf("MQTT BAD CRED!\n"); break;
  //   case 5 /*MQTT_CONNECT_UNAUTHORIZED*/   : display.printf("MQTT Unauthorized!\n"); break; 
  //   default:                                 display.printf("MQTT Unknown\n"); break;
  // }

  if (WiFi.isConnected()) { 
    display.drawBitmap(display.width() - WIFI_WIDTH, 0, WiFiLogo,WIFI_WIDTH, WIFI_HEIGHT, 0x1); 
    // display.printf("WiFi %s\r\n",WiFi.localIP().toString().c_str());
  } else { 
    display.drawBitmap(display.width() - NOWIFI_WIDTH, 0, noWiFiLogo,NOWIFI_WIDTH, NOWIFI_HEIGHT, 0x1); 
    // display.printf("WiFi No IP\r\n");
  }

  int16_t VCC     = adc0.getConversionP0GND();;
  int16_t vGlobal = adc0.getConversionP2GND();
  
  float GlobalmA  = (VCC - vGlobal) * adc0.getMvPerCount();
  display.printf("Curr %3.0fmA %3.0fmA\n", GlobalmA,averageCurrent);
  // if (GlobalmA > maxGlobalCurrent) maxGlobalCurrent = (int16_t) GlobalmA;
  // display.printf("Max   %5dmA\n", maxGlobalCurrent);
  // int16_t vESP32  = adc0.getConversionP3GND();
  // float ESP32mA  = (vGlobal - vESP32) * adc0.getMvPerCount();
  // display.printf("uC    %5.0fmA\n", ESP32mA);
  display.printf("VCC %5.0fmV\n", VCC * adc0.getMvPerCount() );
  display.display();
  delay(20);
  // #ifdef DEBUG
  // Serial.printf("\r");
  // #endif
}