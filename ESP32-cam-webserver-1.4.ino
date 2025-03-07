#include "esp_camera.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <WiFiClient.h>
/** WiFiClient class to communicate with GCM server */

#include <Ticker.h>
#include <TimeLib.h>

#include <UrlEncode.h>


//needed for WifiManager library
#include <DNSServer.h>
#include <WebServer.h>
#include <WiFiManager.h>  //https://github.com/tzapu/WiFiManager
//flag for saving data
bool shouldSaveConfig = false;

int threshold = 40;           //Threshold value for touchpads pins
bool touch6detected = false;  //touch6 used to launch WifiManager (hold it while reseting)
long lastTouch;

//callback notifying us of the need to save config
void saveConfigCallback() {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

//Preferences
#include <Preferences.h>
Preferences preferences;


String device = "eCatFeeder";
String theMAC = "";  // to store MAC address

//External pins on AI Thinker board available for SDcard : 14, 15, 2, 4, 12, 13
// GPIO4 is also used or the flash LED (LOW for ON)
//GIO33 is the built in red led
#define PIN_LED 33
#define PIR_PIN 2     //pin GPIO2, used to wake up ESP32 from PIR sensor with PIR output (3.3V)
#define SERVO_PIN 15  // Pin for servo
#define TIMER_WIDTH 11
#define PIN_CLOCK 12  //output to generate clock on Hx711
#define PIN_DOUT 13   //input Dout from Hx711
#define PIN_CAL50 T6
#define PIN_TARE T0


/* This sketch is a extension/expansion/reork of the 'official' ESP32 Camera example
    sketch from Expressif:
    https://github.com/espressif/arduino-esp32/tree/master/libraries/ESP32/examples/Camera/CameraWebServer

    It is modified to allow control of Illumination LED Lamps's (present on some modules),
    greater feedback via a status LED, and the HTML contents are present in plain text
    for easy modification.

    A camera name can now be configured, and wifi details can be stored in an optional
    header file to allow easier updated of the repo.

    The web UI has had minor changes to add the lamp control when present, I have made the
    'Start Stream' controls more accessible, and add feedback of the camera name/firmware.


   note: Make sure that you have either selected ESP32 AI Thinker,
         or another board which has PSRAM enabled to use high resolution camera modes
*/

// Select camera board model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
//#define CAMERA_MODEL_M5STACK_NO_PSRAM
#define CAMERA_MODEL_AI_THINKER

// Select camera module used on the board
#define CAMERA_MODULE_OV2640
//#define CAMERA_MODULE_OV3660



// A Name for the Camera. (can be set in myconfig.h)
#ifdef CAM_NAME
char myName[] = CAM_NAME;
#else
char myName[] = "ESP32 camera server";
#endif

// This will be displayed to identify the firmware
char myVer[] PROGMEM = __DATE__ " @ " __TIME__;


#include "camera_pins.h"

// Status and illumination LED's
#ifdef LAMP_PIN
int lampVal = 0;  // Current Lamp value, range 0-100, Start off
#else
int lampVal = -1;  // disable Lamp
#endif
int lampChannel = 7;          // a free PWM channel (some channels used by camera)
const int pwmfreq = 50000;    // 50K pwm frequency
const int pwmresolution = 9;  // duty cycle bit range
// https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
const int pwmIntervals = 100;  // The number of Steps between the output being on and off
float lampR;                   // The R value in the PWM graph equation (calculated in setup)

void startCameraServer();

//servo inits

unsigned long timeSERVO = 0;
#define ANGLE_POS_HAUT 200    //Servo forward
#define ANGLE_POS_BAS 100     //Servo reverse
#define POSITION_INCREMENT 1  //nombre incrémentation entre chaque position
const int MILIEU = (ANGLE_POS_HAUT + ANGLE_POS_BAS) / 2;
const int RANGE = (ANGLE_POS_BAS - ANGLE_POS_HAUT) / 2;
int positionServo = -RANGE;

Ticker ServoTicker;

//TIME
boolean noTime = false;
int TimeZone = 1;
int dst = 0;

//feed hours
int LastFeedHour = 0;
int LastFeedDay = 0;
int FeedHour1 = 7;
int FeedHour2 = 19;
int FeedDuration = 2;  //was 6
int FeedNow = 0;
int Plate = 25;
struct tm* timeinfo;  //current time

/** Flag for push request */
int doStop = 0;
int doGCM = 0;

unsigned long Timeout = 0;

//WhatsApp callmebot : https://www.callmebot.com/blog/free-api-whatsapp-messages/
// +international_country_code + phone number
// France +33, example: +36010100101
String phoneNumber = "+your_phone_number";
String apiKey = "your_API_key";

//touchpad

RTC_DATA_ATTR int bootCount = 0;
touch_pad_t touchPin;
void callback() {
  //placeholder callback function
}
boolean TouchWake = false;
boolean SendMAC = false;

//external wake Up with PIR sensor
//Only RTC IO can be used as a source for external wake They are pins: 0,2,4,12-15,25-27,32-39.


#define PIR_PIN_BITMASK 0x4  // 2^2 (exponent is GPIO number) in hex

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
int timeToSleep = 60 * 60 * 4;    /* Time ESP32 will go to sleep (in seconds) */

String st;
//String strssid = "";
//String strpassword = "";
String message = "";
float vdd;
int battery;

String ssid = "";
String password = "";
boolean hasWifiCredentials = false;

//scale
unsigned long CalibZero = 0;
long Calib = 1;
float AverageWeight = 0;
float RealTimeWeight = 0;

boolean SendAlarm = false;
boolean SendEating = true;
String Mess;

//UDP --------------
unsigned int localPort = 5000;  // local port to listen on
char packetBuffer[64];          //buffer to hold incoming packet
char AndroidConnected = 0;

WiFiUDP Udp;
//end UDP-----------

/* ===CODE_STARTS_HERE========================================== */

#define xDEBUG_OUT
//#define W_DEBUG     //debug Wifi and firebase
#define G_DEBUG  //debug GCM serveur
#define DEBUG_OUT
#define DEBUG
//#define xxDEBUG
//#define TEST


void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;

  wakeup_reason = esp_sleep_get_wakeup_cause();

  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0: Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1: Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER: Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD:
      Serial.println("Wakeup caused by touchpad");
      digitalWrite(PIN_LED, LOW);
      TouchWake = true;
      break;
    case ESP_SLEEP_WAKEUP_ULP: Serial.println("Wakeup caused by ULP program"); break;
    default: Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(true);
  Serial.println();
  Serial.println("====");
  Serial.print("esp32-cam-webserver: ");
  Serial.println(myName);
  Serial.print("Code Built: ");
  Serial.println(myVer);

  //Print the wakeup reason for ESP32
  esp_sleep_enable_ext1_wakeup(PIR_PIN_BITMASK, ESP_EXT1_WAKEUP_ANY_HIGH);  //this will be the code to enter deep sleep and wakeup with pin GPIO2 high
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(timeToSleep * uS_TO_S_FACTOR);
  //Preferences
  preferences.begin("eCatFeeder", false);

  //preferences.clear();              // Remove all preferences under the opened namespace
  //preferences.remove("counter");   // remove the counter key only
  ssid = preferences.getString("ssid", "mySSID");  // Get the ssid  value, if the key does not exist, return a default value of ""
  password = preferences.getString("password", "myPassword");
  Calib = preferences.getLong("Calib", 0);
  LastFeedHour = preferences.getInt("LastFeedHour", 0);
  LastFeedDay = preferences.getInt("LastFeedDay", 0);
  FeedHour1 = preferences.getInt("FeedHour1", 7);
  FeedHour2 = preferences.getInt("FeedHour2", 20);

  //preferences.end();  // Close the Preferences

  //servo
  ledcSetup(5, 50, TIMER_WIDTH);  // channel 5, 50 Hz, 11-bit width, generates the PPM signal
  ledcAttachPin(SERVO_PIN, 5);    // GPIO 22 assigned to channel 1
  ledcWrite(5, 0);
  //  ledcDetachPin(SERVO_PIN);                                           // ************************ <-- Code pour détacher les servos
  //  pinMode(SERVO_PIN, INPUT);                                          // ************************ <-- Code pour détacher les servos


#ifdef LED_PIN  // If we have a notification LED set it to output
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LED_OFF);
#endif

#ifdef LAMP_PIN
  ledcSetup(lampChannel, pwmfreq, pwmresolution);  // configure LED PWM channel
  ledcWrite(lampChannel, lampVal);                 // set initial value
  ledcAttachPin(LAMP_PIN, lampChannel);            // attach the GPIO pin to the channel
  // Calculate the PWM scaling R factor:
  // https://diarmuid.ie/blog/pwm-exponential-led-fading-on-arduino-or-other-platforms
  lampR = (pwmIntervals * log10(2)) / (log10(pow(2, pwmresolution)));
#endif

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  //init with high specs to pre-allocate larger buffers
  if (psramFound()) {
    config.frame_size = FRAMESIZE_UXGA;
    config.jpeg_quality = 10;
    config.fb_count = 2;
  } else {
    config.frame_size = FRAMESIZE_SVGA;
    config.jpeg_quality = 12;
    config.fb_count = 1;
  }

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  // camera init
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t* s = esp_camera_sensor_get();
  //initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1);        //flip it back
    s->set_brightness(s, 1);   //up the blightness just a bit
    s->set_saturation(s, -2);  //lower the saturation
  }
  //drop down frame size for higher initial frame rate
  s->set_framesize(s, FRAMESIZE_SVGA);

#if defined(CAMERA_MODEL_M5STACK_WIDE)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

  // Feedback that hardware init is complete and we are now attempting to connect
  Serial.println("");

  if (ftouchRead(T6) < threshold) touch6detected = true;  //detect touchpad for T6 (labelled CAL50 on the PCB)

  //WiFiManager
  //Local intialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  //set config save notify callback
  wifiManager.setSaveConfigCallback(saveConfigCallback);

  //reset settings - for testing
  //wifiManager.resetSettings();

  //sets timeout until configuration portal gets turned off
  //useful to make it all retry or go to sleep
  //in seconds
  wifiManager.setTimeout(300);

  if (touch6detected)  //then launch WifiManager
  {
    //fetches ssid and pass and tries to connect
    //if it does not connect it starts an access point with the specified name
    //here  "AutoConnectAP"
    //and goes into a blocking loop awaiting configuration
    if (!wifiManager.startConfigPortal("JP eCatfeeder")) {
      Serial.println("failed to connect and hit timeout");
      delay(3000);
      //reset and try again, or maybe put it to deep sleep
      ESP.restart();
      delay(5000);
    }
  }
  delay(2000);
  //  //save the custom WifiManager's parameters if needed
  if (shouldSaveConfig) {
    Serial.println("saving Wifi credentials ");
    //read updated parameters

    preferences.putString("password", WiFi.psk());
    preferences.putString("ssid", WiFi.SSID());
    delay(2000);
    ESP.restart();
    delay(5000);
  }




  //connect to WiFi
  WiFi.begin(ssid.c_str(), password.c_str());
  long start = millis();
  hasWifiCredentials = false;

  while ((WiFi.status() != WL_CONNECTED) && (millis() - start < 20000)) {
    delay(500);
    Serial.print(".");
  }
  if (WiFi.status() == WL_CONNECTED) hasWifiCredentials = true;

  //if you get here you may be connected to the WiFi
  Serial.print("connected to Wifi: ");
  Serial.println(hasWifiCredentials);



  if (hasWifiCredentials) {
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }

  //if you get here you have connected to the WiFi
  Serial.println("connected...yeey :)");
  theMAC = WiFi.macAddress();
  theMAC.replace(":", "");

  // init interfaces
  SetIF();


  // feedback that we are connected
  Serial.println("WiFi connected");
  Serial.println("");
  flashLED(200);
  delay(100);
  flashLED(200);
  delay(100);
  flashLED(200);

  // Start the Stream server, and the handler processes for the Web UI.
  startCameraServer();
  //access here:http://192.168.x.x:81/stream" http://192.168.x.x:80 for config

  Serial.print("Camera Ready!  Use 'http://");
  Serial.print(WiFi.localIP());
  Serial.println("' to connect and :81/stream for video stream");

  //scale init HX711
  pinMode(PIN_CLOCK, OUTPUT);  // initialize digital pin 4 as an output.(clock)
  digitalWrite(PIN_CLOCK, HIGH);
  delayMicroseconds(100);        //be sure to go into sleep mode if > 60µs
  digitalWrite(PIN_CLOCK, LOW);  //exit sleep mode*/
  pinMode(PIN_DOUT, INPUT);      // initialize digital pin 5 as an input.(data Out)



  //Time
  //init and get the time
  Serial.println("trying to get time 1");
  configTime(TimeZone * 3600, dst * 0, "pool.ntp.org");
  printLocalTime();

  //init and get the time
  Serial.println("trying to get time 2");  //call it twice to have a well synchronized time on soft reset... Why ? because it works...
  delay(2000);
  configTime(TimeZone * 3600, dst * 0, "pool.ntp.org");
  printLocalTime();

  time_t now;
  struct tm* timeinfo;
  time(&now);
  timeinfo = localtime(&now);

  //    hours = timeinfo->tm_hour;
  //    seconds = timeinfo->tm_sec;
  //    minutes = timeinfo->tm_min;
  //
  //    days = timeinfo->tm_mday;
  //    months = timeinfo->tm_mon + 1;
  //    years = timeinfo->tm_year + 1900;
  //
  //
  //
  //    if (noTime)
  //    {
  //      hours = hour();
  //      seconds = second();
  //      minutes = minute();
  //    }

  if (timeinfo->tm_mday != LastFeedDay)  // we not on the same day as last time
  {
    LastFeedDay = timeinfo->tm_mday;
    LastFeedHour = 0;
    preferences.putInt("LastFeedHour", LastFeedHour);
    preferences.putInt("LastFeedDay", LastFeedDay);
  }
  if ((timeinfo->tm_hour >= FeedHour2) && (LastFeedHour < FeedHour2)) {
    doGCM = 1;
  }
  if ((timeinfo->tm_hour >= FeedHour1) && (LastFeedHour < FeedHour1)) {
    doGCM = 1;
  }
#if defined TEST
  Serial.println("force sending Message ");
  doGCM = 1;
#endif
  if (doGCM == 0) {
#if defined DEBUG
    Serial.println("No need to feed the cat");
#endif
  }

  doGCM = 1;  //force feeeding (Get Cat Message)

  AverageWeight = GetRawWeight();
  AverageWeight = CalibZero - AverageWeight;
  AverageWeight = AverageWeight * 50.0 / Calib;
  RealTimeWeight = AverageWeight;

  Timeout = millis();  //arm software watchdog
}

void printLocalTime() {
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    Serial.println("Failed to obtain time");
    noTime = true;
    return;
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
}

// Notification LED
void flashLED(int flashtime) {
#ifdef LED_PIN                     // If we have it; flash it.
  digitalWrite(LED_PIN, LED_ON);   // On at full power.
  delay(flashtime);                // delay
  digitalWrite(LED_PIN, LED_OFF);  // turn Off
#else
  return;          // No notifcation LED, do nothing, no delay
#endif
}

void SetIF(void) {

  //Start UDP
  Udp.begin(localPort);  // UDP messages are activated and could ba used to send various messagesto the cat feeder (see below into the code)
}


void loop() {
  // Just loop forever.
  // The stream and URI handler processes initiated by the startCameraServer() call at the
  // end of setup() will handle the camera and UI processing from now on.

  //scale
  AverageWeight = GetRawWeight();
  AverageWeight = CalibZero - AverageWeight;
  AverageWeight = AverageWeight * 50.0 / Calib;

  if ((abs(AverageWeight - RealTimeWeight) > 10.0) && (SendEating == true))  // if weight has changed more than 10g then cat is eating
  {
    SendAlarm = true;
    Mess = "Cat is eating. Live stream here \n\"http://";
    Mess += IpAddress2String(WiFi.localIP());
    Mess += ":81/stream\"";
    SendEating = false;  // clear the flag to only send one notification !
  }
  RealTimeWeight = AverageWeight;  // and update the real time weight

  if ((AverageWeight >= Plate) && (FeedNow > 0))  //stop feeding when daily ration is reached
  {
    FeedNow = 0;
    ServoTicker.detach();
    ledcWrite(5, 0);
    //ledcDetachPin(SERVO_PIN);
    //pinMode(SERVO_PIN, INPUT);
    SendAlarm = true;
    Mess = "Cat's plate is full";
  }
  if (SendAlarm) {
#ifdef DEBUG
    Serial.println("Feeder message");
    Serial.println(Mess);
    Serial.print("calibrated weight : ");
    Serial.println(AverageWeight);
#endif
    SendAlarm = false;
    sendWhatsAppMessage(Mess);
  }

#if defined xxDEBUG
  Serial.println("");
  Serial.print("calibrated weight : ");
  Serial.println(AverageWeight);
#endif


  // UDP process : if there's data available, read a packet
  int packetSize = Udp.parsePacket();
  if (packetSize) {
    Timeout = millis();  //rearm software watchdog

#if defined xDEBUG
    Serial.print("Received packet of size ");
    Serial.println(packetSize);
    Serial.print("From ");
    IPAddress remoteIp = Udp.remoteIP();
    Serial.print(remoteIp);
    Serial.print(", port ");
    Serial.println(Udp.remotePort());
#endif
    // read the packet into packetBufffer
    int len = Udp.read(packetBuffer, 255);
    if (len > 0) {
      packetBuffer[len] = 0;
    }
#if defined DEBUG
    Serial.println("UDP Contents:");
    Serial.println(packetBuffer);
#endif
    String Res;
    String test;
    int i;
    test = packetBuffer;

    if (test == "ID")  // send a reply, to the IP address and port that sent us the packet we received
    {
      AndroidConnected = 1;
      Res = device;
      SendUDP(Res);
      doGCM = 0;  //configuration mode block GCM messaging
    }

    else if (test.startsWith("CalibZero"))  // calibration Zero of scale
    {
      CalibZero = GetRawWeight();
      preferences.putULong("CalibZero", CalibZero);
      SendAlarm = true;
      Mess = "Calib zero";
      SendEating = false;
      Serial.print("calibZero ");
      Serial.println(CalibZero);
    } else if (test.startsWith("Calib50"))  // calibration 50g of scale
    {
      Calib = (-GetRawWeight() + CalibZero);
      preferences.putLong("Calib", Calib);
      Serial.print("Calib50 = ");
      Serial.println(Calib);
    } else if (test.startsWith("STOP"))  // send a reply, to the IP address and port that sent us the packet we received
    {
      SendUDP("okSTOP");
      doStop = 1;
    }
  }

  //send the message via notification
  if (doGCM == 1) {
#if defined DEBUG
    Serial.println("Feed cat");
#endif
    doGCM = 0;
    FeedCat();
#if defined TEST
    Mess = "simple test";
    sendWhatsAppMessage(Mess);
#endif
  } else delay(100);

  //manage touch pads
  if ((millis() - lastTouch) > 1000) {

    if (ftouchRead(PIN_CAL50) < threshold)  //calibration plate weight
    {
      lastTouch = millis();
      Calib = (-GetRawWeight() + CalibZero);
      preferences.putLong("Calib", Calib);
      Serial.print("Calib50 = ");
      Serial.println(Calib);

    } else if (ftouchRead(PIN_TARE) < threshold)  //ON & MINUS = decrease highPressure
    {
      lastTouch = millis();
      CalibZero = GetRawWeight();
      preferences.putULong("CalibZero", CalibZero);
      SendAlarm = true;
      Mess = "Calib zero";
      SendEating = false;
      Serial.print("calibZero ");
      Serial.println(CalibZero);
    }
  }

  if ((millis() - Timeout) > 50000)  //if more than 40s without connection then sleep
  {
#if defined DEBUG_OUT
    Serial.println("Time out connection --> stop now");
#endif
    doStop = 1;  //force stop
  }
  if ((doStop == 1) && (digitalRead(PIR_PIN) == LOW)) {
#if defined DEBUG_OUT
    Serial.println("Going to sleep now");
    delay(200);
#endif
    esp_deep_sleep_start();  //enter deeep sleep mode if PIR sensor is off and time out
  }
}


void SendUDP(String Res) {
  char ReplyBuffer[Res.length() + 1];  // a string to send back
  Res.toCharArray(ReplyBuffer, Res.length() + 1);
  Udp.beginPacket(Udp.remoteIP(), Udp.remotePort());
  Udp.print(ReplyBuffer);  //was write...
  Udp.endPacket();
}


void FeedCat(void) {
  time_t now;
  struct tm* timeinfo;
  time(&now);
  timeinfo = localtime(&now);
  LastFeedHour = timeinfo->tm_hour;
  preferences.putInt("LastFeedHour", LastFeedHour);
  if (AverageWeight < Plate)  //feed the cat if plate empty
  {
    SendEating = false;
    FeedNow = 2 * FeedDuration;  //feed the cat now (servoTicker and flip routine)
    //ledcAttachPin(SERVO_PIN, 5);   // GPIO 22 assigned to channel 5
    ledcWrite(5, ANGLE_POS_HAUT);
    ServoTicker.attach(.1, flip);  //start feeding .1s in reverse direction
  } else {
    Serial.print("plate full ");
    Serial.println(AverageWeight);
  }
}

void flip() {

  if (FeedNow == 0) {
    ServoTicker.detach();

    //ledcAttachPin(SERVO_PIN, 5);   // GPIO 22 assigned to channel 5
    ledcWrite(5, 0);
    //    ledcDetachPin(SERVO_PIN);
    //    pinMode(SERVO_PIN, INPUT);

    // if (AverageWeight < Plate)
    {
      //send alarm jam
      SendAlarm = true;
      Mess = "Feeder empty ?";
    }
    return;
  }
  FeedNow--;
  //Serial.println(FeedNow);
  if (FeedNow % 2 == 0)  //normal way
  {
    ServoTicker.attach(3, flip);
    //    ledcAttachPin(SERVO_PIN, 5);   // GPIO 22 assigned to channel 1
    ledcWrite(5, ANGLE_POS_BAS);
#ifdef xDEBUG
    Serial.println("Feed Cat forward");
#endif
  } else {
    ServoTicker.attach(1, flip);
    ledcWrite(5, ANGLE_POS_HAUT);
#ifdef xDEBUG
    Serial.println("Feed Cat reverse");
#endif
  }
}

float GetRawWeight(void) {
  float AverageWeight;
  unsigned long RawWeight;
  // wait for the chip to become ready
  long startTime = millis();
  while ((digitalRead(PIN_DOUT) == HIGH) && ((millis() - startTime) < 80))
    ;

  AverageWeight = 0;
  for (int j = 0; j < 200; j++) {
    RawWeight = 0;
    // pulse the clock pin 24 times to read the data
    for (char i = 0; i < 24; i++) {
      digitalWrite(PIN_CLOCK, HIGH);
      delayMicroseconds(2);
      RawWeight = RawWeight << 1;
      if (digitalRead(PIN_DOUT) == HIGH) RawWeight++;
      digitalWrite(PIN_CLOCK, LOW);
    }
    // set the channel and the gain factor (A 128) for the next reading using the clock pin (one pulse)
    digitalWrite(PIN_CLOCK, HIGH);
    RawWeight = RawWeight ^ 0x800000;
    digitalWrite(PIN_CLOCK, LOW);
    AverageWeight += RawWeight;
    delayMicroseconds(60);
  }
  AverageWeight = AverageWeight / 200;
#if defined xDEBUG
  Serial.print("Raw weight : ");
  Serial.println(AverageWeight);
  Serial.println("");
#endif

  return AverageWeight;
}



int ftouchRead(int gpio)  // this will filter false readings of touchRead() function...
{
  int val = 0;
  int readVal;
  for (int i = 0; i < 10; i++) {
    readVal = touchRead(gpio);
    val = max(val, readVal);
  }
  return val;
}

void sendWhatsAppMessage(String message) {

  // Data to send with HTTP POST
  String url = "https://api.callmebot.com/whatsapp.php?phone=" + phoneNumber + "&apikey=" + apiKey + "&text=" + urlEncode(message);
  HTTPClient http;
  http.begin(url);

  // Specify content-type header
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  // Send HTTP POST request
  int httpResponseCode = http.POST(url);
  if (httpResponseCode == 200) {
    Serial.print("Message sent successfully");
  } else {
    Serial.println("Error sending the message");
    Serial.print("HTTP response code: ");
    Serial.println(httpResponseCode);
  }

  // Free resources
  http.end();
}

String IpAddress2String(const IPAddress& ipAddress) {
  return String(ipAddress[0]) + String(".") + String(ipAddress[1]) + String(".") + String(ipAddress[2]) + String(".") + String(ipAddress[3]);
}
