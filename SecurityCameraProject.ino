#include <IRremote.hpp>

#include "esp_camera.h"
#include <WiFi.h>
#include <Wire.h>
#include "pitches.h"
#include <Keypad.h>
#include <ESP32Servo.h>


// =======================
// Setup for UltraSonic
// =======================
#define echoPin 14
#define trigPin 12
long duration, distance;

void getUltrasonicSignal(){
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
}


// ========================
// Setup for Warning LED
// ========================
const int activeBuzzer = 0;//the pin of the LED
#define led_pin 0

void turnOnLED(){
  digitalWrite(led_pin, HIGH);
}

void turnOffLED(){
  digitalWrite(led_pin, LOW);
}


// ========================
// Notes for Passive Buzzer
// ========================
int melody[] = {
  NOTE_C5, NOTE_D5, NOTE_E5, NOTE_F5, NOTE_G5, NOTE_A5, NOTE_B5, NOTE_C6};
const int noteDuration = 100;  //duration of notes
const int passiveBuzzer = 13; //passive buzzer pin

void powerOnSound(){
  tone(passiveBuzzer, melody[4], noteDuration);
  tone(passiveBuzzer, melody[6], noteDuration);
  tone(passiveBuzzer, melody[7], noteDuration);

  delay(2000);
}

void correctPasscodeSound(){
  tone(passiveBuzzer, melody[4], noteDuration);
  tone(passiveBuzzer, melody[6], noteDuration);
  tone(passiveBuzzer, melody[4], noteDuration);

  delay(2000);
}

void incorrectPasscodeSound(){
  tone(passiveBuzzer, melody[2], noteDuration);
  tone(passiveBuzzer, melody[2], noteDuration);
  tone(passiveBuzzer, melody[2], noteDuration);

  delay(2000);
}


// =======================
// Setup for Servo Motor
// =======================
Servo myservo;
int servoPin = 2; //servo pin location
int startAngle = 90; // initial angle  for servo
int angleStep = 45; // angle change from IR remote
int currentPosition = startAngle; // variable used for changing position accordingly
int newPosition; // used to track new positions for servo

void moveMotor(int irValue){
  switch (irValue){ //irrecv.decodedIRData.decodedRawData
    case 0xF807FF00: 
      Serial.println("RIGHT"); 
      newPosition = currentPosition - angleStep;
      for(int pos = currentPosition; pos >= newPosition; pos -= 1){
        myservo.write(pos);
        delay(20);
      }
      if(currentPosition != 0){
        currentPosition = newPosition;  
      }
      break;
    case 0xF609FF00:
      Serial.println("LEFT"); 
      newPosition = currentPosition + angleStep;
      for(int pos = currentPosition; pos <= newPosition; pos += 1){
        myservo.write(pos);
        delay(20);
      }
      if(currentPosition != 180){
        currentPosition = currentPosition + angleStep;  
      }
      break;
    default:
      Serial.println("Other button pressed: " + irValue);
  }
}


// =======================
// Setup for IR Reciever
// =======================
int receiver = 15; // Signal Pin of IR receiver to Pin 15
IRrecv irrecv(receiver); // create instance of 'irrecv'
decode_results irValue; //variable to store value from IR remote

uint32_t last_decodedRawData = 0;

void resetIR(){
  last_decodedRawData = irrecv.decodedIRData.decodedRawData;
  delay(200);
  irrecv.resume(); // receive the next value
}


void systemPowerOn(){ //function in setup() to enable the system
  bool poweredOn = false;
  Serial.println("---SYSTEM POWER OFF---");

  while (poweredOn == false) {
    if(irrecv.decode()){
      if(irrecv.decodedIRData.decodedRawData == 0xBA45FF00){
        Serial.println("---SYSTEM POWER ON---");
        powerOnSound();
        poweredOn = true;
      }
      resetIR();
    }
  }
  resetIR();
}


void authenticatePassword(){ //correct password is '3179'
  bool authenticated = false;
  String password = "9317";
  String enteredNumber;
  int passwordLength = 4;

  Serial.println("ENTER PASSWORD:");
  Serial.println("---------------");

  while(authenticated == false){
    if(irrecv.decode()){
      switch (irrecv.decodedIRData.decodedRawData){
      case 0xBA45FF00: enteredNumber += "POWER"; break;
      case 0xB847FF00: enteredNumber += "FUNC/STOP"; break;
      case 0xB946FF00: enteredNumber += "VOL+"; break;
      case 0xBB44FF00: enteredNumber += "FAST BACK";    break;
      case 0xBF40FF00: enteredNumber += "PAUSE";    break;
      case 0xBC43FF00: enteredNumber += "FAST FORWARD";   break;
      case 0xF807FF00: enteredNumber += "DOWN";    break;
      case 0xEA15FF00: enteredNumber += "VOL-";    break;
      case 0xF609FF00: enteredNumber += "UP";
      case 0xE619FF00: enteredNumber += "EQ";    break;
      case 0xF20DFF00: enteredNumber += "ST/REPT";    break;
      case 0xE916FF00: enteredNumber += "0";    break;
      case 0xF30CFF00: enteredNumber += "1";    break;
      case 0xE718FF00: enteredNumber += "2";    break;
      case 0xA15EFF00: enteredNumber += "3";    break;
      case 0xF708FF00: enteredNumber += "4";    break;
      case 0xE31CFF00: enteredNumber += "5";    break;
      case 0xA55AFF00: enteredNumber += "6";    break;
      case 0xBD42FF00: enteredNumber += "7";    break;
      case 0xAD52FF00: enteredNumber += "8";    break;
      case 0xB54AFF00: enteredNumber += "9";    break;
      default: enteredNumber += "?";
      }
      
    resetIR();
    Serial.println(enteredNumber);

    if(enteredNumber.length() == passwordLength){
      if(enteredNumber == password){
        correctPasscodeSound();
        Serial.println("CORRECT PASSWORD");
        authenticated = true;
      } else{
        incorrectPasscodeSound();
        Serial.println("INCORRECT PASSWORD");
        enteredNumber = "";
      }
    }
    }
  }
}


// ===================
// Select camera model
// ===================
#define CAMERA_MODEL_WROVER_KIT
#include "camera_pins.h"


// ===========================
// WiFi credentials
// ===========================
const char* ssid = "WIFI NAME HERE";
const char* password = "WIFI PASSWORD HERE";

void startCameraServer();
void setupLedFlash(int pin);


//-----------------------------------------------------------------------------------------------------


void setup() {
  //SET UP FOR LED
  pinMode(led_pin, OUTPUT);
  turnOnLED();
  delay(3000);
  turnOffLED();


  //SET UP FOR ULTRASONIC SENSOR
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);


  //SETUP FOR IR RECIEVER
  Serial.println("IR Receiver Button Decode");
  irrecv.enableIRIn();
  systemPowerOn();
  resetIR();
  authenticatePassword();
  resetIR();


  //SETUP FOR SERVO
  myservo.attach(servoPin);
  myservo.write(startAngle); //setting start position to center position -> 90°


  //SET UP FOR ESP32CAM AND WEBSERVER
  Serial.setDebugOutput(true);
  Serial.println();

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
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_UXGA;
  config.pixel_format = PIXFORMAT_JPEG; // for streaming
  // config.pixel_format = PIXFORMAT_RGB565; // for face detection/recognition
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 1;
  
  if(config.pixel_format == PIXFORMAT_JPEG){
    if(psramFound()){
      config.jpeg_quality = 10;
      config.fb_count = 2;
      config.grab_mode = CAMERA_GRAB_LATEST;
    } else {
      // Limit the frame size when PSRAM is not available
      config.frame_size = FRAMESIZE_SVGA;
      config.fb_location = CAMERA_FB_IN_DRAM;
    }
  } else {
    // Best option for face detection/recognition
    config.frame_size = FRAMESIZE_240X240;

  }
#if CONFIG_IDF_TARGET_ESP32S3
  config.fb_count = 2;
#endif

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

  sensor_t * s = esp_camera_sensor_get();
  // initial sensors are flipped vertically and colors are a bit saturated
  if (s->id.PID == OV3660_PID) {
    s->set_vflip(s, 1); // flip it back
    s->set_brightness(s, 1); // up the brightness just a bit
    s->set_saturation(s, -2); // lower the saturation
  }
  // drop down frame size for higher initial frame rate
  if(config.pixel_format == PIXFORMAT_JPEG){
    s->set_framesize(s, FRAMESIZE_QVGA);
  }

#if defined(CAMERA_MODEL_M5STACK_WIDE) || defined(CAMERA_MODEL_M5STACK_ESP32CAM)
  s->set_vflip(s, 1);
  s->set_hmirror(s, 1);
#endif

#if defined(CAMERA_MODEL_ESP32S3_EYE)
  s->set_vflip(s, 1);
#endif

// Setup LED FLash if LED pin is defined in camera_pins.h
#if defined(LED_GPIO_NUM)
  setupLedFlash(LED_GPIO_NUM);
#endif

  WiFi.begin(ssid, password);
  WiFi.setSleep(false);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected");

  startCameraServer();

  Serial.print("Camera Ready! Use http://");
  Serial.print(WiFi.localIP());
}


//-----------------------------------------------------------------------------------------------------


void loop() {
  getUltrasonicSignal();

  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;
  if(distance < 100){ //turn on and flasg light if person detected within a meter of the camera
    turnOnLED();
    delay(500);
  }
  turnOffLED();

  // String disp = String(distance); //code for displaying current distance from ultrasonic sensor if needed
  // Serial.print("Distance: ");
  // Serial.print(disp);
  // Serial.println(" cm");
  // delay(300);

  if (irrecv.decode()) // have we received an IR signal?
  {
    moveMotor(irrecv.decodedIRData.decodedRawData);
    resetIR();
  }
}

