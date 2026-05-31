#include <Arduino.h>
#include "credentials/credentials.h"

#include <SPI.h>
#include <time.h>

#define DEBUG

// specifique TTGO lora OLED pins
// I2C OLED Display works with SSD1306 driver
#define OLED_SDA   4
#define OLED_SCL  15
#define OLED_RST  16

// SPI LoRa Radio
#define LORA_SCK   5      // GPIO5  - SX1276 SCK
#define LORA_MISO 19      // GPIO19 - SX1276 MISO
#define LORA_MOSI 27      // GPIO27 - SX1276 MOSI
#define LORA_CS   18      // GPIO18 - SX1276 CS
#define LORA_RST  14      // GPIO14 - SX1276 RST
#define LORA_IRQ  26      // GPIO26 - SX1276 IRQ (interrupt request)

unsigned long sketchTime = 0;

portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

//////////////////////////////
// Program state
//////////////////////////////
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

//////////////////////////////////////
// LoRa 
//////////////////////////////////////
#include <SPI.h>
#include <LoRa.h>

#define LORA_BAND                                   870E6    // Hz
#define LORA_TX_POWER                               20        // dBm
#define LORA_BANDWIDTH                              125E3     
#define LORA_SPREADING_FACTOR                       7         // [SF7..SF12]
#define LORA_CODINGRATE_DENOMINATOR                 5         // 4/5

String textSend, textRecv ;

const uint8_t LoRa_buffer_size = 128; // Define the payload size here
char txpacket[LoRa_buffer_size];

void IRAM_ATTR onSend() {
  portENTER_CRITICAL_ISR(&mux);
  state = SEND;
  portEXIT_CRITICAL_ISR(&mux);
}

void LoRa_rxMode(){
  LoRa.disableInvertIQ();               // normal mode
  LoRa.receive();                       // set receive mode
}

void onTxDone() {
  portENTER_CRITICAL_ISR(&mux);
  //state = SENT;
  portEXIT_CRITICAL_ISR(&mux);
}

void LoRa_sendMessage() {
  LoRa.beginPacket();                 // start packet
  LoRa.print(textSend);               // add payload
  LoRa.endPacket(true);               // finish packet and send it
  Serial.println("send message => "); 
  Serial.println(textSend);
}

// LoRa interrupt handler for hardware IRQ pin
void IRAM_ATTR onLoRaInterrupt() {
  portENTER_CRITICAL_ISR(&mux);
  state = RECEIVING;
  portEXIT_CRITICAL_ISR(&mux);
}

void initLoRa(){
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);
  
  if (!LoRa.begin(LORA_BAND)) {
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setCodingRate4(LORA_CODINGRATE_DENOMINATOR);
    LoRa.setSignalBandwidth(LORA_BANDWIDTH);
    LoRa.setTxPower(LORA_TX_POWER);
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }
  
  // Attach hardware interrupt for LoRa IRQ pin
  pinMode(LORA_IRQ, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LORA_IRQ), onLoRaInterrupt, RISING);
  
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();
  
  Serial.println("LoRa initialized with hardware interrupt on pin " + String(LORA_IRQ));
}

//////////////////////////////
// WiFi
//////////////////////////////
#include <WiFi.h>
#include <WiFiClient.h>

WiFiClient wifiClient; 

void onWifiEvent (arduino_event_t* event) {
#ifdef DEBUG
    Serial.printf ("[WiFi-event] event: %d - ", event->event_id);
    switch (event->event_id) {
    case ARDUINO_EVENT_WIFI_READY:    
      Serial.printf ("WiFi ready\n"); 
      break;
    case ARDUINO_EVENT_WIFI_SCAN_DONE:     
      Serial.printf ("WiFi scan done\n"); 
      break;
    case ARDUINO_EVENT_WIFI_STA_START:
      Serial.printf ("WiFi station start\n");
      break;
    case ARDUINO_EVENT_WIFI_STA_CONNECTED:
      Serial.printf ("Connected to SSID\n");
      break;
    case ARDUINO_EVENT_WIFI_STA_STOP:
      Serial.printf ("Station Stop\n");
      break;
    case ARDUINO_EVENT_WIFI_STA_LOST_IP:
      Serial.printf ("Lost IP\n");
      break;
    case ARDUINO_EVENT_WIFI_STA_GOT_IP:
      Serial.printf ("Got IP: %s\n", IPAddress (event->event_info.got_ip.ip_info.ip.addr).toString ().c_str ());
      break;
    case ARDUINO_EVENT_WIFI_STA_DISCONNECTED: 
      Serial.printf ("Disconnected from SSID\n");
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

//////////////////////////////
// JSon
//////////////////////////////
#include <ArduinoJson.h>

/////////////////////////////////
// OLED Display
/////////////////////////////////
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
bool displayFound = false;

/////////////////////////////////
// InfluxDB3
/////////////////////////////////
#include <HTTPClient.h>

const char* influxdb_server = INFLUXDB_SERVER;
int influxdb_port = INFLUXDB_PORT;
const char* influxdb_token = INFLUXDB_TOKEN;
const char* influxdb_db = INFLUXDB_DB;

String message;
String device = "N/A";

bool sendInfluxDB3();
bool managePacket(String message);

//////////////////////////////
// Interrupts
//////////////////////////////
#include <esp_sleep.h>
#define uS_TO_S_FACTOR 1000000ULL  /* Conversion factor for micro seconds to seconds */
#define BUTTON_PIN 12

hw_timer_t *timer = NULL;

hw_timer_t *timer2 = NULL;

// For display timeout
unsigned long lastDisplayUpdate = 0;
#define DISPLAY_TIMEOUT 10000 // 10 seconds

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


#define TIME_TO_SEND_BROKER_DATA 60

void IRAM_ATTR onBrokerTimer() {
  portENTER_CRITICAL_ISR(&mux);
  prevState=state;
  state = BROKER;
  portEXIT_CRITICAL_ISR(&mux);
}

#define TIME_TO_COLLECT_DATA 9 * 5

void IRAM_ATTR onDataTimer() {
  portENTER_CRITICAL_ISR(&mux);
  prevState = state;
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

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// SETUP
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void setup() {
  #ifdef DEBUG
  Serial.begin(115200);
  #endif

  // Initialize OLED display and run test
  delay(200); // Give more time for power to stabilize
  
  Serial.println("=== OLED Display Troubleshooting ===");
  Serial.print("SDA pin: "); Serial.println(OLED_SDA);
  Serial.print("SCL pin: "); Serial.println(OLED_SCL);
  Serial.print("RST pin: "); Serial.println(OLED_RST);
  
  // Configure reset pin if needed
  pinMode(OLED_RST, OUTPUT);
  digitalWrite(OLED_RST, HIGH);
  delay(50);
  digitalWrite(OLED_RST, LOW);
  delay(50);
  digitalWrite(OLED_RST, HIGH);
  delay(50);
  
  Wire.begin(OLED_SDA, OLED_SCL);
  
  // Try all possible I2C addresses for SSD1306
  uint8_t addresses[] = {0x3C, 0x3D, 0x78, 0x7A};
  
  for (uint8_t addr : addresses) {
    Serial.print("Trying address 0x");
    Serial.println(addr, HEX);
    if(!display.begin(SSD1306_SWITCHCAPVCC, addr)) {
      Serial.print("  Failed at 0x");
      Serial.println(addr, HEX);
    } else {
      displayFound = true;
      Serial.print("  SUCCESS! Found at 0x");
      Serial.println(addr, HEX);
      // Increase display brightness
      display.ssd1306_command(SSD1306_SETCONTRAST);
      display.ssd1306_command(0xFF); // Maximum brightness
      break;
    }
  }
  
  if (displayFound) {
    Serial.println("Display found! Configuring...");
    
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.cp437(true); // Use full 256 char 'Code Page 437' font
    
    display.println("OLED TEST");
    display.println("ESP32");
    display.println("LoRa32");
    display.println("Working!");
    
    display.display();
    Serial.println("Test pattern displayed");
    
    unsigned long startTime = millis();
    while (millis() - startTime < 10000) {
      delay(100);
      if ((millis() / 500) % 2 == 0) {
        display.drawPixel(120, 56, SSD1306_WHITE);
      } else {
        display.drawPixel(120, 56, SSD1306_BLACK);
      }
      display.display();
    }
    
    display.clearDisplay();
    display.display();
    Serial.println("OLED test completed");
    
  } else {
    Serial.println("ERROR: No SSD1306 display found!");
    Serial.println("Starting I2C bus scan...");
    
    byte error, address;
    int nDevices = 0;
    for(address = 1; address < 127; address++ ) {
      Wire.beginTransmission(address);
      error = Wire.endTransmission();
      if (error == 0) {
        Serial.print("I2C device found at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
        nDevices++;
      } else if (error == 4) {
        Serial.print("Unknown error at address 0x");
        if (address < 16) Serial.print("0");
        Serial.println(address, HEX);
      }
    }
    
    if (nDevices == 0) {
      Serial.println("No I2C devices found!");
    } else {
      Serial.print("Found ");
      Serial.print(nDevices);
      Serial.println(" I2C device(s)");
    }
    
    Serial.println("Trying alternative initialization...");
    for (uint8_t addr : addresses) {
      Serial.print("Trying external VCC at 0x");
      Serial.println(addr, HEX);
      if(display.begin(SSD1306_EXTERNALVCC, addr)) {
        Serial.println("SUCCESS with external VCC!");
        display.clearDisplay();
        display.setTextSize(2);
        display.setTextColor(SSD1306_WHITE);
        display.setCursor(0, 0);
        display.println("EXT VCC");
        display.println("Working!");
        display.display();
        delay(5000);
        break;
      }
    }
  }

  print_wakeup_reason();

  // WiFi init
  WiFi.begin(AP_NAME, AP_PASSRHRASE);
  WiFi.onEvent (onWifiEvent, ARDUINO_EVENT_WIFI_STA_GOT_IP);
  WiFi.onEvent (onWifiEvent, ARDUINO_EVENT_WIFI_STA_DISCONNECTED);

  initLoRa();

  #ifdef DEBUG
  Serial.println("LoRa started OK in receive mode");
  #endif
  
  #ifdef DEBUG
  Serial.println("--------- SETUP ENDED ---------");
  #endif

}

//////////////////////////////
// LOOP
//////////////////////////////

void loop() {

  sketchTime = millis();

  switch (state) {
  case INIT:
    state=WAIT;
    break;
  
  case ERROR:
    delay(5000);
    state = WAIT;
    break;

  case WAIT:
    // Check if display timeout has expired
    if (displayFound && (millis() - lastDisplayUpdate) > DISPLAY_TIMEOUT) {
      display.startscrollright(0x00, 0x0F);
      delay(10000); // Scroll for 10 seconds
      display.stopscroll();
      display.clearDisplay();
      display.display();
    }
    break;

  case RECEIVING: {
    int packetSize = LoRa.parsePacket();
    
    if (packetSize > 0) {
      state = MANAGE;
      message.clear();
      
      while (LoRa.available()) {
        message += (char)LoRa.read();
        if(message.length() > 200) {
          #ifdef DEBUG
          Serial.println("Message too long !");
          #endif
          LoRa.flush();
          state = ERROR;
          break;
        }
      }
      
      #ifdef DEBUG
      Serial.print("Received packet, size: ");
      Serial.print(packetSize);
      Serial.print(", RSSI: ");
      Serial.print(LoRa.packetRssi());
      Serial.print(", SNR: ");
      Serial.print(LoRa.packetSnr());
      Serial.println(" dB");
      Serial.print("Message: ");
      Serial.println(message);
      #endif
    } else {
      // No valid packet, go back to WAIT
      state = WAIT;
    }
    
    // Display received packet info on OLED
    if (displayFound) {
      display.clearDisplay();
      display.setTextSize(2);
      display.setTextColor(SSD1306_WHITE);
      display.setCursor(0, 0);
      display.println("Packet");
      display.setCursor(0, 16);
      display.printf("Dev:%s", device.c_str());
      display.setCursor(0, 32);
      display.printf("Size:%d", message.length());
      
      display.display();
      lastDisplayUpdate = millis();
    }
    break;
  }
  
  case MANAGE:
    state=SEND;
    if (!managePacket(message)) { 
      state = ERROR;
      device = "MSG Err";
    }
  break;

  case SEND:
    state = WAIT;
    if (!sendInfluxDB3()) { 
      state = ERROR;
    }
    break;

  case BROKER:
    state=WAIT;
    break;

  case DATA:
    state = prevState;
    break;

  case SLEEP:
    Serial.flush();
    LoRa.flush();
    LoRa.end();
    esp_sleep_enable_ext0_wakeup(GPIO_NUM_32,1);
    esp_deep_sleep_start();
    state=WAIT;
    break;

  default:
    #ifdef DEBUG
      Serial.println("State unknown !");
    #endif
    break;
  }

}

//////////////////////////////
// Manage LoRa / JSon Packet
//////////////////////////////

bool managePacket(String message) {

  #ifdef DEBUG
  Serial.println("--> Manage Packet");
  Serial.printf("RSSI: %d SNR: %.2f\n",LoRa.packetRssi(), LoRa.packetSnr());
  #endif
  bool retval=true;

  JsonDocument doc;
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

/////////////////////////////////
// Send data to InfluxDB3
/////////////////////////////////

bool sendInfluxDB3() {
  #ifdef DEBUG
  Serial.println("--> Send to InfluxDB3");
  #endif
  bool retval = true;
  
  if (WiFi.status() != WL_CONNECTED) {
    if (!WiFiConnect()) {
      #ifdef DEBUG
      Serial.println("WiFi connection failed for InfluxDB3");
      #endif
      return false;
    }
  }
  
  JsonDocument doc;
  DeserializationError Derror = deserializeJson(doc, message);
  if (Derror) {
    #ifdef DEBUG
    Serial.println("Deserialization FAILED for InfluxDB3");
    #endif
    return false;
  }
  
  String dev = doc["device"];
  device = dev;
  
  // Build InfluxDB3 line protocol data
  String table = "home";
  JsonObject obj = doc.as<JsonObject>();
  
  String influxData = table + ",device_id=" + device;
  bool firstField = true;
  
  for (JsonPair p : obj) {
    if (strcmp(p.key().c_str(),"device") == 0) continue; 
    if (p.value().is<const char*>()) {
      const char* s = p.value();
      if (!firstField) influxData += ",";
      else influxData += " ";
      influxData += String(p.key().c_str()) + "=" + String(s);
    }
    else if(p.value().is<float>()) {
      float f = p.value();
      if (!firstField) influxData += ",";
      else influxData += " ";
      influxData += String(p.key().c_str()) + "=" + String(f, 1);
    }
    else if(p.value().is<int>()) {
      int i = p.value();
      if (!firstField) influxData += ",";
      else influxData += " ";
      influxData += String(p.key().c_str()) + "=" + String(i) + "i";
    }
    firstField = false;
  }
  
  // Add LoRa metrics
  influxData += ",rssi=" + String(LoRa.packetRssi()) + "i";
  influxData += ",snr=" + String(LoRa.packetSnr(), 2);
  
  #ifdef DEBUG
  Serial.print("InfluxDB3 data: ");
  Serial.println(influxData);
  
  String url = "http://" + String(influxdb_server) + ":" + String(influxdb_port) + "/api/v3/write_lp?db=" + String(influxdb_db);
  Serial.print("InfluxDB3 URL: ");
  Serial.println(url);
  #endif
  
  HTTPClient http;
  http.begin(url);
  http.addHeader("Authorization", "Bearer " + String(influxdb_token));
  http.addHeader("Content-Type", "text/plain");
  
  int httpResponseCode = http.POST(influxData);
  
  if (httpResponseCode > 0) {
    #ifdef DEBUG
    Serial.printf("InfluxDB3 HTTP Response code: %d\n", httpResponseCode);
    #endif
    if (httpResponseCode != 204) {
      retval = false;
    }
  } else {
    #ifdef DEBUG
    Serial.printf("InfluxDB3 HTTP POST failed, error: %s\n", http.errorToString(httpResponseCode).c_str());
    #endif
    retval = false;
  }
  
  http.end();
  
  #ifdef DEBUG
  Serial.print("--> Send to InfluxDB3 End : "); Serial.println(retval ? "OK" : "FAILED");
  #endif
  return retval;
}
