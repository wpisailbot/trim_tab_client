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

// WiFi credentials
static WebSocketsClient webSocket;

// Global variables for the Jetson status messages
bool tailscale_connected = true;
bool found_buoy = false;
bool trim_auto = false;
bool rudder_auto = false;
bool connection_status = true;
bool reach_buoy = false;

int control_angle = 0;                        // The current angle that the servo is set to

TRIM_STATE state;                         // The variable responsible for knowing what state the trim tab is in

float windAngles[NUM_WIND_READINGS];
int windIndex = 0;
int maxI = 0;

TaskHandle_t WingLEDTimer;
TaskHandle_t ServoController;

volatile float readWind(){
  static volatile float currentWindAngle = 0;
  float windAngle = (analogRead(potPin) / POT_TICKS_PER_DEGREE)+ WIND_OFFSET; //- POT_HEADWIND; // reads angle of attack data and centers values on headwind
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
  #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_WIND)
    Serial.println("");
    Serial.print("Current wind: ");
    Serial.print(currentWindAngle);
  #endif

  windIndex++;
  windIndex%=NUM_WIND_READINGS-1;
  return currentWindAngle;
}

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length){
  static unsigned long lastRollTime = 0;
  String json_str;
  DynamicJsonDocument doc(1024);
  DeserializationError err;
  switch (type){
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
      if (doc.containsKey("angle")){
        control_angle = doc["angle"].as<float>();
        if(control_angle > SERVO_HI_LIM){
          control_angle = SERVO_HI_LIM;
        } else if (control_angle < SERVO_LO_LIM){
          control_angle = SERVO_LO_LIM;
        }
        #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_MANUAL)
          Serial.print("Control angle: ");
          Serial.print(control_angle);
          Serial.println("");
        #endif

      } 
      #ifndef TRIM_STATE_OVERRIDE
      if (doc.containsKey("state")){
        String newState = doc["state"].as<String>();
        if (newState == "min_lift"){
          state = TRIM_STATE_MIN_LIFT;
          #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_STATE)
            Serial.println("State: min_lift");
          #endif
        }
        else if (newState == "max_drag_port"){
          state = TRIM_STATE_MAX_DRAG_PORT;
          #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_STATE)
            Serial.println("State: max_drag_port");
          #endif
        }
        else if (newState == "max_drag_starboard"){
          state = TRIM_STATE_MAX_DRAG_STBD;
          #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_STATE)
            Serial.println("State: max_drag_starboard");
          #endif
        }
        else if (newState == "max_lift_port"){
          state = TRIM_STATE_MAX_LIFT_PORT;
          #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_STATE)
            Serial.println("State: max_lift_port");
          #endif
        }
        else if (newState == "max_lift_starboard"){
          state = TRIM_STATE_MAX_LIFT_STBD;
          #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_STATE)
            Serial.println("State: max_lift_starboard");
          #endif
        }
        else if (newState == "manual"){
          state = TRIM_STATE_MANUAL;
          #if defined(MY_DEBUG_MODE) || defined(MY_DEBUG_MODE_STATE)
            Serial.println("State: manual");
          #endif
        }
      }
      #endif
      if (doc.containsKey("roll")){
        unsigned long current_time = millis();
        //Only take roll data every 500ms
        if(current_time-lastRollTime<500){
          break;
        }
        lastRollTime = current_time;
        
        float currentRoll = doc["roll"].as<float>();
        float currentRollMagnitude = abs(currentRoll)-30; // Don't scale until after 30 degrees roll
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

void setup(){
  //wifi
  static std::string websockets_server_host = SERVER_HOST; // Enter server adress
  static uint16_t websockets_server_port = SERVER_PORT;

  /* Setting the mode of the pins necessary */
  pinMode(potPin, INPUT_PULLDOWN);

  // Led setup
  LED_setup();

  /* Initializing variables to initial conditions */
  control_angle = SERVO_CTR;              // Trim tab starts off centralized
  state = TRIM_STATE_MIN_LIFT; // The state is set to be in the center position or what we consider to be min lift
  Serial.begin(115200);

  /* Initializing the servo and setting it to its initial condition */
  servo.attach(servoPin, 500 , 2500);
  Serial.print("Setup Current Angle");
  servo.write(control_angle);
  
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
  WiFi.begin(HULL_SSID, PASSWORD);
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

  Serial.println("moving servo");
  servo.write(SERVO_CTR+1);
  delay(500);
  servo.write(SERVO_CTR+5);
  delay(500);
  control_angle = SERVO_CTR;
  Serial.println("Setup Current Angle Post Warmup");
  servo.write(SERVO_CTR);
  #ifdef TRIM_STATE_OVERRIDE
  Serial.println("TRIM_STATE_MIN_LIFT: 1");
  Serial.println("TRIM_STATE_MAX_DRAG_PORT: 2");
  Serial.println("TRIM_STATE_MAX_DRAG_STBD: 3");
  Serial.println("TRIM_STATE_MAX_LIFT_PORT: 4");
  Serial.println("TRIM_STATE_MAX_LIFT_STBD: 5");
  Serial.println("TRIM_STATE_MANUAL: 6");
  Serial.println("Select state: ");
  #endif

  xTaskCreatePinnedToCore(
    ServoTask,   /* Task function. */
    "ServoTask",     /* name of task. */
    8000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    10,           /* priority of the task */
    &ServoController,      /* Task handle to keep track of created task */
    1);          /* pin task to core 0 */ 
  xTaskCreatePinnedToCore(
    LEDTimerTask,   /* Task function. */
    "LEDTimerTask",     /* name of task. */
    4000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    9,           /* priority of the task */
    &WingLEDTimer,      /* Task handle to keep track of created task */
    0);          /* pin task to core 1 */
}

void LEDTimerTask(void * pvParameters){
  for(;;){
    //Serial.println("websocket run");
    lightLED(reach_buoy,move_flag,found_buoy,trim_auto,rudder_auto,connection_status);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void ServoTask(void * pvParameters){
  for(;;){
    servoControl(readWind(), state);
    //Serial.println("servo task run");
    vTaskDelay(pdMS_TO_TICKS(50));
  }
}

void loop(){
  webSocket.loop();
  #ifdef TRIM_STATE_OVERRIDE
  static String buffer;
  while (Serial.available()) {
      char c = Serial.read();

      if (c == '\n' || c == '\r') {
          int input_state = buffer.toInt();
          buffer = "";

          switch (input_state) {
              case 1: state = TRIM_STATE_MIN_LIFT; break;
              case 2: state = TRIM_STATE_MAX_DRAG_PORT; break;
              case 3: state = TRIM_STATE_MAX_DRAG_STBD; break;
              case 4: state = TRIM_STATE_MAX_LIFT_PORT; break;
              case 5: state = TRIM_STATE_MAX_LIFT_STBD; break;
              case 6: state = TRIM_STATE_MANUAL; break;
          }
      } else {
          buffer += c;
      }
  }
  #endif
}
