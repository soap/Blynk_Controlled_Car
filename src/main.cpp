/**
 * Blynk controlled car using ESP8266 Motor Shield and NodeMCU V.2
 * By Prasit Gebsaap
 */ 
#include <FS.h>                   //this needs to be first, or it all crashes and burns...
#if defined(ESP8266)
#include <ESP8266WiFi.h>          //https://github.com/esp8266/Arduino
#else
#include <WiFi.h>
#endif
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>
#include <Servo.h> 
#include <ArduinoJson.h>          //https://github.com/bblanchon/ArduinoJson v.5 only
#include <BlynkSimpleEsp8266.h>
#include <WiFiManager.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>

#define BLYNK_PRINT Serial  // Let Blynk write to serial port 
#define MAX_SPEED       1020  // 100 percent speed value
// front wheels driving 
#define LeftMotorSpeed  4     // GPIO-5 or D1 Pin to control motor speed using PWM
#define LeftMotorDir    2     // GPIO-0 or D3, Pin to control motor direction
#define RightMotorSpeed 5     // GPIO-4 or D2 Pin to control motor speed using PWM
#define RightMotorDir   0     // GPIO-2 or D4, Pin to control motor direction
#define ForwardDir      LOW   // LOW level for forward direction
#define BackwardDir     HIGH  // HIGH level for backward direction

char blynk_token[] = "kIh2dwtVeLnSHUruROs9lk-xxxxxxxxx"; //"kIh2dwtVeLnSHUruROs9lk-g5sfxbzgx";

int minRange = 312;
int maxRange = 712;

int speedPercent = 10;
int minSpeed = 450;
int maxSpeed = 1020;
int noSpeed = 0;

int trigPin = 12;  // D6 on NodeMCU
int echoPin = 13;  // D7 on NodeMCU
int servoPin = 15;  // D8 on Wemos D1 Mini

int PingTimer;            // TimerID
int SrvoPos;              // Servo position
long duration, distance;  // For Ping sensor 
int autoMode = 0;         // Auto driving mode

BlynkTimer timer;
Servo servoMotor;         // Servo's name

ESP8266WebServer server(80);

bool shouldSaveConfig = false;

void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/** 
 * Ultrasonic (Ping) Distance Sensor 
 */
int readPingInCm() {  
    // Pulse
    digitalWrite(trigPin, LOW);
    delayMicroseconds(2);
    digitalWrite(trigPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(trigPin, LOW);

    // End Pulse & Calculate distance
    duration = pulseIn(echoPin, HIGH);
    distance = (duration / 2) * 0.0343; // Speed of sound is: 343m/s = 0.0343 cm/uS = 1/29.1 cm/uS
    
    return distance; 
}

/** 
 * Ultrasonic (Ping) Distance Sensor called by timer
 */
void pingInCm() {  
    int distance = readPingInCm();
    Blynk.virtualWrite(V2, distance);  // To Display Widget
}

int lookForward() {
  servoMotor.write(90);
  distance = readPingInCm();
  return distance;
  
}

int lookRight() {
  servoMotor.write(30);
  delay(500);
  int distance = readPingInCm();
  delay(100);
  servoMotor.write(90);
  return distance;
}

int lookLeft() {
  servoMotor.write(160);
  delay(500);
  int distance = readPingInCm();
  delay(100);
  servoMotor.write(90);
  return distance;
}

void moveForward() {
  digitalWrite(RightMotorDir, ForwardDir); 
  digitalWrite(LeftMotorDir, ForwardDir);
  
  analogWrite(RightMotorSpeed, maxSpeed);
  analogWrite(LeftMotorSpeed, maxSpeed);
}

void moveBackward() {
  digitalWrite(RightMotorDir, BackwardDir);
  digitalWrite(LeftMotorDir, BackwardDir);
  
  analogWrite(RightMotorSpeed, maxSpeed);
  analogWrite(LeftMotorSpeed, maxSpeed);  
}

void turnForwardLeft() {
  digitalWrite(RightMotorDir, ForwardDir);
  digitalWrite(LeftMotorDir, ForwardDir);
  
  analogWrite(RightMotorSpeed, maxSpeed);
  analogWrite(LeftMotorSpeed, minSpeed);
}

void turnForwardRight() {
  digitalWrite(RightMotorDir, ForwardDir);
  digitalWrite(LeftMotorDir, ForwardDir);
  
  analogWrite(RightMotorSpeed, minSpeed);
  analogWrite(LeftMotorSpeed, maxSpeed);
}

void turnBackwardRight() {
  digitalWrite(RightMotorDir, BackwardDir);
  digitalWrite(LeftMotorDir, BackwardDir);
  
  analogWrite(RightMotorSpeed, maxSpeed);
  analogWrite(LeftMotorSpeed, minSpeed);
}

void turnBackwardLeft() {
  digitalWrite(RightMotorDir, BackwardDir);
  digitalWrite(LeftMotorDir, BackwardDir);
  
  analogWrite(RightMotorSpeed, minSpeed);
  analogWrite(LeftMotorSpeed, maxSpeed);   
}

void stopMovement() {
  
  analogWrite(RightMotorSpeed, noSpeed);
  analogWrite(LeftMotorSpeed, noSpeed);
}

/**
 * @param x int X position of Joystick
 * @param y int Y position of Joystick
 */
void processMovement(int x, int y) {
  
  if(y >= maxRange && x >= minRange && x <= maxRange){
    moveForward();
    Blynk.virtualWrite(V0, "Forward");
  }
  // move forward right
  else if(x >= maxRange && y >= maxRange) {
    turnForwardRight();
    Blynk.virtualWrite(V0, "Forward Right");
  }
  // move forward left
  else if(x <= minRange && y >= maxRange) {
    turnForwardLeft();
    Blynk.virtualWrite(V0, "Forward Left");
  }
  // neutral zone
  else if(y < maxRange && y > minRange && x < maxRange && x > minRange) {
    stopMovement();
    Blynk.virtualWrite(V0, "Stop");
  }
  // move back
  else if(y <= minRange && x >= minRange && x <= maxRange) {
    moveBackward();
    Blynk.virtualWrite(V0, "Backward");
  }
  // move back and left
  else if(y <= minRange && x <= minRange) {
    turnBackwardLeft();
    Blynk.virtualWrite(V0, "Backward Left");    
  }
  // move back and right
  else if(y <= minRange && x >= maxRange) {
    turnBackwardRight();
    Blynk.virtualWrite(V0, "Backward Right");        
  }
}

/**
 * Receive virtual pin writing from Blynk application
 */
BLYNK_WRITE(V1) {
  if (autoMode == 1) return;
  
  int x = param[0].asInt();
  int y = param[1].asInt();

  processMovement(x, y);
}

BLYNK_WRITE(V5) {
  int distance = lookLeft();
  Blynk.virtualWrite(V7, distance);  // To Display Widget
}

BLYNK_WRITE(V6) {
  int distance = lookRight();
  Blynk.virtualWrite(V8, distance);  // To Display Widget
}

BLYNK_WRITE(V3) {
  maxSpeed = (int)(MAX_SPEED * param.asInt()/100.0);
  minSpeed = (int)(maxSpeed * 0.00001);
}


BLYNK_WRITE(V4) {
  autoMode = param.asInt();
  if (autoMode == 1) {
    timer.disable(PingTimer);  // Disable timer until needed
  }else{
    stopMovement();
    timer.enable(PingTimer);  // enable timer in manual mode
  }  
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.println("Booting");

    //read configuration from FS json
  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json")) {
      //file exists, reading and loading
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile) {
        Serial.println("opened config file");
        size_t size = configFile.size();
        // Allocate a buffer to store contents of the file.
        std::unique_ptr<char[]> buf(new char[size]);

        configFile.readBytes(buf.get(), size);
        DynamicJsonBuffer jsonBuffer;
        JsonObject& json = jsonBuffer.parseObject(buf.get());
        json.printTo(Serial);
        if (json.success()) {
          Serial.println("\nparsed json");
          strcpy(blynk_token, json["blynk_token"]);
        } else {
          Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }
  
  //end read
  //WiFi.disconnect();              //uncomment to clear SSID and PASSWORD from EEPROM
  WiFi.hostname("KPS_SmartCar");  //uncomment to change hostname for device.  Replace with your own unique name
  
  WiFiManagerParameter custom_blynk_token("blynk", "blynk token", blynk_token, 34);
  WiFiManager wifiManager;
  wifiManager.setConfigPortalTimeout(180);
  wifiManager.setSaveConfigCallback(saveConfigCallback);
  wifiManager.addParameter(&custom_blynk_token);
  
  if (!wifiManager.autoConnect("KPS_SmartCar")) {
    Serial.println("WiFi failed to connect and hit timeout, reseting....");
    delay(3000);
    ESP.reset();
    delay(5000);
  }
  
  //read updated parameters 
  strcpy(blynk_token, custom_blynk_token.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig) {
    Serial.println("saving config");
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
    json["blynk_token"] = blynk_token;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile) {
      Serial.println("failed to open config file for writing");
    }

    json.printTo(Serial);
    json.printTo(configFile);
    configFile.close();
    //end save
  }
  ArduinoOTA.setHostname("KPS_SmartCar");
  ArduinoOTA.onStart([]() {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH) {
      type = "sketch";
    } else { // U_FS
      type = "filesystem";
    }

    // NOTE: if updating FS this would be the place to unmount FS using FS.end()
    Serial.println("Start updating " + type);
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) {
      Serial.println("Auth Failed");
    } else if (error == OTA_BEGIN_ERROR) {
      Serial.println("Begin Failed");
    } else if (error == OTA_CONNECT_ERROR) {
      Serial.println("Connect Failed");
    } else if (error == OTA_RECEIVE_ERROR) {
      Serial.println("Receive Failed");
    } else if (error == OTA_END_ERROR) {
      Serial.println("End Failed");
    }
  });
  
  ArduinoOTA.begin();  // For OTA
  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());
  Blynk.config(blynk_token);
  //Blynk.begin(auth, ssid, pass);
  Serial.print("Blynk token : "); 
  Serial.println(custom_blynk_token.getValue());

  Serial.println("Try Blynk connection ....");
  while (Blynk.connect() == false) {
    // Wait until connected
    Serial.print(".");
  } 
  Serial.println(".");
  Serial.println("Blynk conected.");
  // initial setting for Ultrasonic sensor
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  
  servoMotor.attach(servoPin);
  servoMotor.write(90);
  
   // ===== Timer Setup =====
  PingTimer = timer.setInterval(250L, pingInCm);  // Ping distance routine - Button Controlled
  
  // initial settings for motors off and direction forward
  
  pinMode(RightMotorSpeed, OUTPUT);
  pinMode(LeftMotorSpeed, OUTPUT);
  pinMode(RightMotorDir, OUTPUT);
  pinMode(LeftMotorDir, OUTPUT);
 
  digitalWrite(RightMotorSpeed, LOW);
  digitalWrite(LeftMotorSpeed, LOW);
  digitalWrite(RightMotorDir, ForwardDir);
  digitalWrite(LeftMotorDir, ForwardDir);
  Blynk.virtualWrite(V0, "Stop");
}

void loop() {
  int distanceRight = 0;
  int distanceLeft = 0;
  int distance = lookForward();
  // put your main code here, to run repeatedly:
  Blynk.run();
  timer.run(); // Initiates BlynkTimer
  
  ArduinoOTA.handle();  // For OTA
  
  if (autoMode == 1) {
    Blynk.virtualWrite(V2, distance);
    if (distance < 20) {
      stopMovement();
      delay(300);
      moveBackward();
      delay(300);
      stopMovement();
      distanceRight = lookRight();
      Blynk.virtualWrite(V8, distanceRight);
      delay(300);
      distanceLeft = lookLeft();
      Blynk.virtualWrite(V7, distanceLeft);
      delay(300);
      if (distanceRight >= distanceLeft) {
        turnForwardRight();
        delay(300);
        stopMovement();
      }else{
        turnForwardLeft();
        delay(300);
        stopMovement();
      }           
    }else{
      moveForward();
    }
    distance = lookForward();
  }
}