/**
 * @file Trim-tab.ino
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @author Matthew Gomes (mhgomes@wpi.edu) - 2023/2024
 * @brief File containing the execution code for the controller embedded within the adjustable trim tab
 * @version 2.0.2
 * @date 2022-4-11
 * @copyright Copyright (c) 2022
 */

 // white turns into black: GND
 // green turns into white: SNG
 // red stays red: 5V
/* File containing all constants for the trim tab to operate; mainly pins and comms */
#include "Constants.h"


CRGB leds[NUM_LEDS];

enum TRIM_STATE {TRIM_STATE_MIN_LIFT, TRIM_STATE_MAX_DRAG_PORT, TRIM_STATE_MAX_DRAG_STBD, TRIM_STATE_MAX_LIFT_PORT, TRIM_STATE_MAX_LIFT_STBD, TRIM_STATE_MANUAL};

// WiFi credentials
const char *ssid = "sailbot_trimtab_ap";
const char *password = "sailbot123";
std::string websockets_server_host = "serverip_or_name"; // Enter server adress
uint16_t websockets_server_port = 8080;             // Enter server port
WebSocketsClient webSocket;

/* Control variables */
volatile int ledState;                    // For controlling the state of an LED
bool bleConnected = false;                // Set to True if a device is connected

// Global variables for the Jetson status messages
bool tailscale_connected = true;
bool found_buoy = false;
bool launch_complete = true;
bool trim_auto = false;
bool rudder_auto = false;
bool connection_status = true;
reach_buoy = false;

auto LEDTimer = timer_create_default();   // Sets the LED timer function to be called asynchronously on an interval
Servo servo;                              // Servo object            // Mapped reading from wind direction sensor on the front of the sail
int control_angle = 0;                        // The current angle that the servo is set to
int currentAngle = 0;
unsigned long lastTrimAdjustTime = 0;
TRIM_STATE state;                         // The variable responsible for knowing what state the trim tab is in
StaticJsonDocument<200> currentData;

float trimTabRollScale = 1.0; // roll-controlled trimtab setpoint scale. If we're rolled too much, we want a lower AOA.
unsigned long lastRollTime = 0;

float windAngles[NUM_WIND_READINGS];
int windIndex = 0;
int maxI = 0;
volatile float currentWindAngle = 0;
bool move_flag = false;
int targetAngle = -1;
int startAngle = 0;
int t_iter = 0;
int light_i = 0;

TaskHandle_t WebSocket;
TaskHandle_t ServoController;
TaskHandle_t WindVaneRead;
TaskHandle_t SenData;

bool readWind(){
  float windAngle = analogRead(potPin) / POT_TICKS_PER_DEGREE; //- POT_HEADWIND; // reads angle of attack data and centers values on headwind
  windAngle -= 180; 

  //windAngle = (((windAngle - POT_MIN) * (180 - -180)) / (POT_MAX - POT_MIN)) + -180;
  windAngles[windIndex] = windAngle;
  if (maxI<NUM_WIND_READINGS-1){
    maxI++;
  }
  float sum = 0;
  for(unsigned int i=0; i<maxI; i++){
    sum += windAngles[i];
  }
  sum /= maxI;
  currentWindAngle = sum;
  // Serial.print("Current wind: ");
  // Serial.println(currentWindAngle);
  //Serial.println(currentWindAngle);

  windIndex++;
  windIndex%=NUM_WIND_READINGS-1;
  return true;
}

bool SendJson(){
  currentData["wind_angle"] = currentWindAngle;
  std::string jsonString;
  serializeJson(currentData, jsonString);
  webSocket.sendTXT(jsonString.c_str(), jsonString.length());
  return true;
}

float lastTimestamp = 0.0f;
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length){
  String json_str;
  DynamicJsonDocument doc(1024);
  DeserializationError err;
  switch (type)
  {
  case WStype_DISCONNECTED:
    Serial.printf("[WSc] Disconnected!\n");
    break;
  case WStype_CONNECTED:
    Serial.printf("[WSc] Connected to url: %s\n", payload);
    break;
  case WStype_TEXT:
    Serial.printf("[WSc] Received text: %s\n", payload);

    err = deserializeJson(doc, payload);
    if (err){
      Serial.println("deserialization error");
    } else {
      // if (doc.containsKey("timestamp")){
      //   float timestamp = doc["timestamp"].as<float>();
      //   if (timestamp<lastTimestamp){
      //     Serial.print("Skipping out-of-order command. last timestamp: ");
      //     Serial.println(lastTimestamp);
      //     break;
      //   }
      //   lastTimestamp = timestamp;
      // }
      if (doc.containsKey("angle")){
        control_angle = doc["angle"].as<float>();
        if(control_angle > SERVO_HI_LIM){
          control_angle = SERVO_HI_LIM;
        } else if (control_angle < SERVO_LO_LIM){
          control_angle = SERVO_LO_LIM;
        }
        Serial.print("Control angle: ");
        Serial.print(control_angle);
        Serial.println("");

      } if (doc.containsKey("state")){
        String newState = doc["state"].as<String>();
        if (newState == "min_lift"){
          state = TRIM_STATE_MIN_LIFT;
          Serial.println("State: min_lift");
        }
        else if (newState == "max_drag_port"){
          state = TRIM_STATE_MAX_DRAG_PORT;
          Serial.println("State: max_drag_port");
        }
        else if (newState == "max_drag_starboard"){
          state = TRIM_STATE_MAX_DRAG_STBD;
          Serial.println("State: max_drag_starboard");
        }
        else if (newState == "max_lift_port"){
          state = TRIM_STATE_MAX_LIFT_PORT;
          Serial.println("State: max_lift_port");
        }
        else if (newState == "max_lift_starboard"){
          state = TRIM_STATE_MAX_LIFT_STBD;
          Serial.println("State: max_lift_starboard");
        }
        else if (newState == "max_lift_port"){
          state = TRIM_STATE_MAX_LIFT_PORT;
          Serial.println("State: max_lift_port");
        }
        else if (newState == "manual"){
          state = TRIM_STATE_MANUAL;
          Serial.println("State: manual");
        }
      }
      if (doc.containsKey("roll")){
        unsigned long current_time = millis();
        //Only take roll data every 500ms
        if(current_time-lastRollTime<500){
          break;
        }
        lastRollTime = current_time;
        
        float currentRoll = doc["roll"].as<float>();
        float currentRollMagnitude = abs(currentRoll)-30; // Don't scale until after 30 degrees roll
        if(currentRollMagnitude>0.0){
          trimTabRollScale-=0.1;
          //don't invert the tab
          if(trimTabRollScale<0.0){
            trimTabRollScale = 0.0;
          }
        } else {
          trimTabRollScale += 0.1;
          if(trimTabRollScale>1.0){
            trimTabRollScale = 1.0;
          }
        }
        // trimTabRollScale = -0.05*currentRollMagnitude+1; // Inverse linear scaling from 20-40 degrees of roll
        // trimTabRollScale = min(max(trimTabRollScale, 0.0f), 1.0f); //bound 0->1
      } 
      if (doc.containsKey("clear_winds")){ // used for tacking
        float lastAngle = windAngles[windIndex];
        maxI = 0;
        windIndex = 0;
        windAngles[0]=lastAngle;
        int current_control = control_angle-SERVO_CTR;
        control_angle = SERVO_CTR+(current_control*-1.0);
      }
      if (doc.containsKey("tailscale")){
        tailscale_connected = doc["tailscale"].as<bool>();
      }
      if (doc.containsKey("found_buoy")){
        found_buoy = doc["found_buoy"].as<bool>();
      }
      if (doc.containsKey("trim_auto")){
        trim_auto = doc["trim_auto"].as<bool>();
      }
      if (doc.containsKey("rudder_auto")){
        rudder_auto = doc["rudder_auto"].as<bool>();
      }
      if (doc.containsKey("reach_buoy")){
        reach_buoy = doc["reach_buoy"].as<bool>();
      }
      if (doc.containsKey("battery_ok")){
        connection_status = doc["battery_ok"].as<bool>();
      }
    }
    break;
  case WStype_BIN:
    Serial.printf("[WSc] Received binary data.\n");
    break;
  }
}
float moveToProfile(float diff, float curtime) {
    float A=Amax;
    float Vm=Vmax;
    if (diff < 0){
        A=-Amax;
        Vm=-Vmax;
    }
    float t_accel = Vm/A;
    float acceldist = 0.5*A*pow(t_accel,2);
    float halfdiff = diff/2;
    if (t_accel > abs(halfdiff)) {
      t_accel = pow((halfdiff/(2*A)),0.5);
    }
    acceldist = 0.5*A*pow(t_accel,2);
    float dist_at_max = diff - 2*acceldist;
    float  t_decel = t_accel+(dist_at_max/Vm);
    float t_fin = t_accel+t_decel;
    if (curtime > t_fin) {
      return diff;
    }
    if (curtime < t_accel) {
      return 0.5*A*pow(curtime,2);
    } else if (curtime < t_decel) {
      return acceldist+Vm*(curtime-t_accel);
    } else {
      float dist_at_decel = Vm*(curtime-t_decel)- 0.5*A*pow((curtime-t_decel),2);//x = x0 + vt + at^2
      return acceldist+ dist_at_max+dist_at_decel;
    }
}
//motion helper function
void movementHandler(int goal_angle) {
  if (goal_angle != targetAngle) {
      move_flag = true;
      t_iter=0;
      targetAngle = goal_angle;
      startAngle = currentAngle;
    }else{
      if (move_flag) {
        Serial.print(currentAngle);
        Serial.print(" : ");
        Serial.println(targetAngle);
        int angleChange = moveToProfile(targetAngle-startAngle, t_iter);
        currentAngle = angleChange + startAngle;
        servo.write(currentAngle);
        Serial.println(currentAngle);
        t_iter=t_iter+1;
        if (currentAngle == targetAngle) {
          Serial.println("DONE");
          t_iter=0;
          move_flag = false;
        }
      }
    }
}

// LED helper functions
static int middleIndex() {
  return NUM_LEDS / 2;
}

static void setSectionSolid(int sectionIndex, const CRGB &color) {
  const int mid = middleIndex();
  if (sectionIndex < 0 || sectionIndex > 5) {
    return;
  }

  const int totalHalf = (NUM_LEDS - 1) / 2;
  const int baseHalves[5] = {13, 13, 13, 14, 14};
  int sumBase = 0;
  for (int i = 0; i < 5; i++) {
    sumBase += baseHalves[i];
  }
  const int remainingHalf = max(0, totalHalf - sumBase);
  const int halfHeights[6] = {13, 13, 13, 14, 14, remainingHalf};

  if (halfHeights[sectionIndex] == 0) {
    return;
  }
  int offset = 0;
  for (int i = 0; i < sectionIndex; i++) {
    offset += halfHeights[i];
  }

  if (sectionIndex == 0) {
    const int leftStart = max(0, mid - halfHeights[0]);
    const int leftEnd = max(0, mid - 1);
    const int rightStart = min(NUM_LEDS - 1, mid + 1);
    const int rightEnd = min(NUM_LEDS - 1, mid + halfHeights[0]);
    for (int i = leftStart; i <= leftEnd; i++) {
      leds[i] = color;
    }
    for (int i = rightStart; i <= rightEnd; i++) {
      leds[i] = color;
    }
    leds[mid] = color;
    return;
  }

  const int leftStart = max(0, mid - (offset + halfHeights[sectionIndex]));
  const int leftEnd = max(0, mid - (offset + 1));
  const int rightStart = min(NUM_LEDS - 1, mid + (offset + 1));
  const int rightEnd = min(NUM_LEDS - 1, mid + (offset + halfHeights[sectionIndex]));

  for (int i = leftStart; i <= leftEnd; i++) {
    leds[i] = color;
  }
  for (int i = rightStart; i <= rightEnd; i++) {
    leds[i] = color;
  }
}

bool lightLED( void *) {
  const CRGB defaultColors[6] = {CRGB::Red,    CRGB::Green,  CRGB::Blue,
                                 CRGB::Yellow, CRGB::Purple, CRGB::Orange};
                          
  // Start with everything off so section colors are obvious.
  fill_solid(leds, NUM_LEDS, CRGB::Black);            
  if(reach_buoy){
    setSectionSolid(0, CRGB::Green);
    setSectionSolid(1, CRGB::Green);
    setSectionSolid(2, CRGB::Green);
    setSectionSolid(3, CRGB::Green);
    setSectionSolid(4, CRGB::Green);
  } else {
    
    if (move_flag) { //eventually do one that goes from green ->yellow -> red for hull battery
      setSectionSolid(0, CRGB::Red);
    } else {
      setSectionSolid(0, CRGB::Black);
    }
    if (found_buoy) {
      setSectionSolid(1, CRGB::Blue);
    } else {
      setSectionSolid(1, CRGB::Black);
    }
    if (trim_auto){
      setSectionSolid(2, CRGB::Green);
    } else {
      setSectionSolid(2, CRGB::Black);
    }
    if (rudder_auto){
      setSectionSolid(3, CRGB::Yellow);
    } else {
      setSectionSolid(3, CRGB::Black);
    }
    if (light_i>100){
      setSectionSolid(4, CRGB::Purple);
      if(light_i > 200) {
        light_i = 0;
      }
    } else {
      setSectionSolid(4, CRGB::Black);
    }
    if (connection_status){
      setSectionSolid(5, CRGB::Orange);
    } else {
      setSectionSolid(5, CRGB::Black);
    }
  }
  FastLED.show();
  light_i++;
  return true;
}

bool servoControl();

void setup(){
  /* Setting the mode of the pins necessary */
  pinMode(powerLED, OUTPUT); // Green LED
  pinMode(bleLED, OUTPUT);   // Blue LED
  pinMode(errorLED, OUTPUT); // Red LED
  pinMode(potPin, INPUT_PULLDOWN);

  // Led setup
  Serial.println("Setting up LEDs");
  FastLED.addLeds<LED_TYPE, LED_DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
  FastLED.setBrightness(100);

  /* Set up battery monitoring */
  //battery.begin(3300, 1.43, &sigmoidal);
  /* Initializing variables to initial conditions */
  ledState = LOW;                         // LED starts in off position
  control_angle = SERVO_CTR;              // Trim tab starts off centralized
  state = TRIM_STATE_MIN_LIFT; // The state is set to be in the center position or what we consider to be min lift
  Serial.begin(115200);
  /* Giving feedback that the power is on */
  digitalWrite(powerLED, HIGH);
  /* Initializing the servo and setting it to its initial condition */
  servo.attach(servoPin, 500 , 2500);
  currentAngle = control_angle;
  Serial.print("Setup Current Angle");
  Serial.println(currentAngle);
  servo.write(control_angle);

  // For recalibration
  // while(true){
  //   float windAngle = analogRead(potPin) - POT_HEADWIND; // reads angle of attack data and centers values on headwind
  //   windAngle = (((windAngle - POT_MIN) * (180 - -180)) / (POT_MAX - POT_MIN)) + -180;
  //   Serial.println(windAngle);
  // }

  WiFi.mode(WIFI_STA);
  WiFi.disconnect(); // Disconnect any existing connections
  delay(100); // Short delay after disconnect

  Serial.println("Scanning for WiFi networks...");

  int n = WiFi.scanNetworks(); // Start scanning for networks
  if (n == 0) {
    Serial.println("No networks found");
  } else {
    Serial.print(n);
    Serial.println(" networks found");
    for (int i = 0; i < n; ++i) {
      // Print each network's SSID and RSSI
      Serial.print(i + 1);
      Serial.print(": ");
      Serial.print(WiFi.SSID(i));
      Serial.print(" (");
      Serial.print(WiFi.RSSI(i));
      Serial.print(" dBm)");
      Serial.println((WiFi.encryptionType(i) == WIFI_AUTH_OPEN)?" ":"*");
      delay(10); // Short delay between prints
    }
  }
  Serial.println("");

  // Connect to WiFi
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nWiFi connected");

  Serial.println("Connected to Wifi, doing mDNS lookup.");
  if (!MDNS.begin("sailbot-trimtab-esp32"))
  {
    Serial.println("Error starting mDNS");
    return;
  }

  Serial.println("Sending mDNS query");
  int n_services = MDNS.queryService("sailbot-trimtab", "tcp");
  if (n_services == 0)
  {
    Serial.println("no services found");
  }
  else
  {
    for (int i = 0; i < n; ++i)
    {
      // Print details for each service found
      if (MDNS.hostname(i) != "sailbot-trimtab-local"){
        continue;
      }
      Serial.print("Found service ");
      Serial.print(MDNS.hostname(i));
      Serial.print(" (");
      Serial.print(MDNS.IP(i));
      websockets_server_host = MDNS.IP(i).toString().c_str();
      Serial.print(":");
      Serial.print(MDNS.port(i));
      websockets_server_port = MDNS.port(i);
      Serial.println(")");
      break;
    }
  }
  Serial.println("Found mDNS, Connecting to server.");
  // Setup WebSocket
  webSocket.begin(websockets_server_host.c_str(), websockets_server_port);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  LEDTimer.every(100, lightLED);

  Serial.println("moving servo");
  delay(500);
  servo.write(SERVO_CTR+1);
  delay(500);
  servo.write(SERVO_CTR+5);
  delay(500);
  servo.write(SERVO_CTR+10);
  delay(500);
  servo.write(SERVO_CTR+5);
  delay(500);
  control_angle = SERVO_CTR;
  currentAngle = SERVO_CTR;
  Serial.print("Setup Current Angle Post Warmup");
  Serial.println(currentAngle);
  servo.write(SERVO_CTR);

  xTaskCreatePinnedToCore(
    ServoTask,   /* Task function. */
    "ServoTask",     /* name of task. */
    100000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    10,           /* priority of the task */
    &ServoController,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */   

  xTaskCreatePinnedToCore(
    WebSocketTask,   /* Task function. */
    "WebSocketTask",     /* name of task. */
    100000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    9,           /* priority of the task */
    &WebSocket,      /* Task handle to keep track of created task */
    0);          /* pin task to core 1 */

  xTaskCreatePinnedToCore(
    VaneRead,   /* Task function. */
    "VaneRead",     /* name of task. */
    100000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    8,           /* priority of the task */
    &WindVaneRead,      /* Task handle to keep track of created task */
    1); 

  xTaskCreatePinnedToCore(
    SendData,   /* Task function. */
    "SendData",     /* name of task. */
    100000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &SenData,      /* Task handle to keep track of created task */
    1); 
    
}

void VaneRead(void * pvParameters){
  for(;;){
    readWind();
    vTaskDelay(100);
  }
}

void WebSocketTask(void * pvParameters){
  for(;;){
    webSocket.loop();
  }
}

void ServoTask(void * pvParameters){
  for(;;){
    servoControl();
    vTaskDelay(50);
  }
}

void SendData(void * pvParameters){
  for(;;){
    SendJson();
    vTaskDelay(500);
  }
}

void loop(){
  LEDTimer.tick();
}

/**
 * @author Irina Lavryonova
 * @brief Sets the angle of the servo based on the angle of attack read from the encoder at the front
 * @TODO: send this state to the telemetry interface
 */

bool servoControl(){
  // Read, format, and scale angle of attack reading from the encoder
  
  // Serial.println(windAngle);

  // Set debug LEDs to on to indicate servo control is active
  // digitalWrite(led1Pin, HIGH);
  // digitalWrite(led2Pin, HIGH);

  // Write servo position to one read from the Arduino
  //  servo.write(SERVO_CTR + control_angle - 200 - 90);
  switch (state)
  {
  case TRIM_STATE_MAX_LIFT_STBD: {
      float windAngleInverse = currentWindAngle * -1;
      unsigned long currentTime = millis();
      if(currentTime-lastTrimAdjustTime>TRIM_ADJUST_INTERVAL_MS){
        lastTrimAdjustTime = currentTime;
        Serial.print("Current wind: ");
        Serial.println(windAngleInverse); 
        if (MAX_LIFT_ANGLE > windAngleInverse+angle_tolerance)
        {
          Serial.println("Increasing angle ");
          control_angle += 2;
          if(control_angle>SERVO_HI_LIM){
            control_angle = SERVO_HI_LIM;
          }
        }
        else if ((MAX_LIFT_ANGLE+angle_tolerance < windAngleInverse))
        {
          
          Serial.println("Decreasing angle ");
          control_angle -= 2;
          if(control_angle<SERVO_CTR+10){
            control_angle = SERVO_CTR+10;
          }
        }else{
          Serial.println("");
        }
      }
      int current_control_angle = min(max(control_angle, (SERVO_CTR - 55)), (SERVO_CTR + 55))*trimTabRollScale;
      // Serial.print("Trimtab roll scale:");
      // Serial.println(trimTabRollScale);

      // Serial.print("Control angle: ");
      // Serial.println(control_angle);
      currentAngle = current_control_angle;
      movementHandler(current_control_angle);
      //Serial.println("Max lift port");
    }
    break; 
  case TRIM_STATE_MAX_LIFT_PORT: {
      unsigned long currentTime = millis();
      if(currentTime-lastTrimAdjustTime>TRIM_ADJUST_INTERVAL_MS){
        Serial.print("Max Lift PORT: ");
        Serial.println(currentWindAngle); 
        lastTrimAdjustTime = currentTime;
        if (MAX_LIFT_ANGLE > currentWindAngle+angle_tolerance)
        {
          control_angle -= 2;
          if(control_angle<SERVO_LO_LIM){
            control_angle = SERVO_LO_LIM;
          }
        }
        else if ((MAX_LIFT_ANGLE+angle_tolerance < currentWindAngle))
        {
          control_angle += 2;
          if(control_angle>SERVO_CTR-10){
            control_angle = SERVO_CTR-10;
          }
        }
      }
      int current_control_angle = min(max(control_angle, (SERVO_CTR - 55)), (SERVO_CTR + 55))*trimTabRollScale;
      currentAngle = current_control_angle;
      movementHandler(current_control_angle);
      //Serial.println("Max lift stbd");
    }
    break;
  case TRIM_STATE_MAX_DRAG_STBD:
    movementHandler(SERVO_LO_LIM*trimTabRollScale);
    //Serial.println("Max drag port");
    break;
  case TRIM_STATE_MAX_DRAG_PORT:
    movementHandler(SERVO_HI_LIM*trimTabRollScale);
    //Serial.println("Max drag stbd");
    break;
  case TRIM_STATE_MIN_LIFT:
    movementHandler(SERVO_CTR);
    //Serial.println("Min lift");
    break;
  case TRIM_STATE_MANUAL:
    movementHandler(control_angle);
    break;
  default:
    state = TRIM_STATE_MIN_LIFT;
    Serial.println("State: manual");
    break;
  }

  return true;
}