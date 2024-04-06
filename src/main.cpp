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

#include <WiFi.h>
#include <WebSocketsClient.h>
#include <ESPmDNS.h>
#include <pb_encode.h>
#include <pb_decode.h>
#include <ESP32Servo.h>    // Driver code for operating servo
#include <arduino-timer.h> // Library to handle non-blocking function calls at a set interval
#include <Battery.h>       // Library for monitoring battery level
#include <ArduinoJson.h>
/* File containing all constants for the trim tab to operate; mainly pins and comms */
#include "Constants.h"
#include "messages.pb.c" // Protobuf message definitions

/* Battery */
Battery battery = Battery(3000, 4200, batteryPin);

// WiFi credentials
const char *ssid = "sailbot_trimtab_ap";
const char *password = "sailbot123";
std::string websockets_server_host = "serverip_or_name"; // Enter server adress
uint16_t websockets_server_port = 8080;             // Enter server port
WebSocketsClient webSocket;

/* Control variables */
volatile int ledState;                    // For controlling the state of an LED
bool batteryWarning = false;              // If True, battery level is low (at or below 20%)
bool bleConnected = false;                // Set to True if a device is connected
auto LEDTimer = timer_create_default();   // Sets the LED timer function to be called asynchronously on an interval
auto servoTimer = timer_create_default(); // Sets the servo timer function to be called asynchronously on an interval
auto dataTimer = timer_create_default(); // Sets the data timer function to be called asynchronously on an interval
Servo servo;                              // Servo object
volatile float windAngle = 5;             // Mapped reading from wind direction sensor on the front of the sail
int control_angle = 0;                        // The current angle that the servo is set to
TRIM_STATE state;                         // The variable responsible for knowing what state the trim tab is in
StaticJsonDocument<200> currentData;

bool encode_string(pb_ostream_t *stream, const pb_field_t *field, void *const *arg)
{
  const char *str = (const char *)(*arg);

  if (!pb_encode_tag_for_field(stream, field))
    return false;

  return pb_encode_string(stream, (uint8_t *)str, strlen(str));
}

bool SendJson(void *)
{
  currentData["wind_angle"] = windAngle;
  currentData["battery_level"] = battery.level();
  std::string jsonString;
  serializeJson(currentData, jsonString);
  webSocket.sendTXT(jsonString.c_str(), jsonString.length());
  return true;
}
uint lastTimestamp = 0;
void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
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
      //   uint timestamp = doc["timestamp"].as<float>();
      //   if (timestamp<lastTimestamp){
      //     Serial.println("Skipping out-of-order command");
      //     break;
      //   }
      //   lastTimestamp = timestamp;
      // }
      if (doc.containsKey("angle")){
        control_angle = doc["angle"].as<float>();
        Serial.print("Control angle: ");
        Serial.print(control_angle);
        Serial.println("");

      } if (doc.containsKey("state")){
        String newState = doc["state"].as<String>();
        if (newState == "min_lift"){
          state = TRIM_STATE_TRIM_STATE_MIN_LIFT;
          Serial.println("State: min_lift");
        }
        else if (newState == "max_drag_port"){
          state = TRIM_STATE_TRIM_STATE_MAX_DRAG_PORT;
          Serial.println("State: max_drag_port");
        }
        else if (newState == "max_drag_starboard"){
          state = TRIM_STATE_TRIM_STATE_MAX_DRAG_STBD;
          Serial.println("State: max_drag_starboard");
        }
        else if (newState == "max_lift_port"){
          state = TRIM_STATE_TRIM_STATE_MAX_LIFT_PORT;
          Serial.println("State: max_lift_port");
        }
        else if (newState == "max_lift_starboard"){
          state = TRIM_STATE_TRIM_STATE_MAX_LIFT_STBD;
          Serial.println("State: max_lift_starboard");
        }
        else if (newState == "max_lift_port"){
          state = TRIM_STATE_TRIM_STATE_MAX_LIFT_PORT;
          Serial.println("State: max_lift_port");
        }
        else if (newState == "manual"){
          state = TRIM_STATE_TRIM_STATE_MANUAL;
          Serial.println("State: manual");
        }
      }
    }
    break;
  case WStype_BIN:
    Serial.printf("[WSc] Received binary data.\n");
    ControlMessage message = ControlMessage_init_zero;
    pb_istream_t stream = pb_istream_from_buffer(payload, length);
    bool status = pb_decode(&stream, ControlMessage_fields, &message);
    if (!status)
    {
      Serial.printf("Decoding failed: %s\n", PB_GET_ERROR(&stream));
      return;
    }
    switch (message.control_type)
    {
    case CONTROL_MESSAGE_CONTROL_TYPE_CONTROL_MESSAGE_CONTROL_TYPE_STATE:
      state = message.state;
      Serial.print("State: ");
      Serial.println(state);
      break;
    case CONTROL_MESSAGE_CONTROL_TYPE_CONTROL_MESSAGE_CONTROL_TYPE_ANGLE:
      control_angle = message.control_angle;
      Serial.print("Received angle: ");
      Serial.println(control_angle);
      break;
    }
    break;
  }
}

bool servoControl(void *);

void setup()
{
  /* Setting the mode of the pins necessary */
  pinMode(powerLED, OUTPUT); // Green LED
  pinMode(bleLED, OUTPUT);   // Blue LED
  pinMode(errorLED, OUTPUT); // Red LED
  pinMode(potPin, INPUT_PULLDOWN);

  /* Set up battery monitoring */
  battery.begin(3300, 1.43, &sigmoidal);
  /* Initializing variables to initial conditions */
  ledState = LOW;                         // LED starts in off position
  control_angle = SERVO_CTR;              // Trim tab starts off centralized
  state = TRIM_STATE_TRIM_STATE_MIN_LIFT; // The state is set to be in the center position or what we consider to be min lift
  Serial.begin(115200);
  /* Giving feedback that the power is on */
  digitalWrite(powerLED, HIGH);
  /* Initializing the servo and setting it to its initial condition */
  servo.attach(servoPin);
  servo.write(control_angle);
  Serial.println("moving servo");

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

  /* Starting the asynchronous function calls */
  servoTimer.every(10, servoControl);
  dataTimer.every(500, SendJson);
}

void loop()
{
  webSocket.loop();
  if (battery.level() < 20)
  {
    batteryWarning = true;
  }
  else
  {
    batteryWarning = false;
  }
  servoTimer.tick();
  LEDTimer.tick();
  dataTimer.tick();
}

/**
 * @author Irina Lavryonova
 * @brief Sets the angle of the servo based on the angle of attack read from the encoder at the front
 * @TODO: send this state to the telemetry interface
 */
bool servoControl(void *)
{
  // Read, format, and scale angle of attack reading from the encoder
  windAngle = analogRead(potPin) - POT_HEADWIND;                                            // reads angle of attack data and centers values on headwind
  windAngle = (((windAngle - POT_MIN) * (180 - -180)) / (POT_MAX - POT_MIN)) + -180;
  // Serial.println(windAngle);

  // Set debug LEDs to on to indicate servo control is active
  // digitalWrite(led1Pin, HIGH);
  // digitalWrite(led2Pin, HIGH);

  // Write servo position to one read from the Arduino
  //  servo.write(SERVO_CTR + control_angle - 200 - 90);

  switch (state)
  {
  case TRIM_STATE_TRIM_STATE_MAX_LIFT_PORT:
    if (MAX_LIFT_ANGLE > windAngle)
    {
      control_angle += 2;
    }
    else if ((MAX_LIFT_ANGLE < windAngle))
    {
      control_angle -= 2;
    }
    control_angle = min(max(control_angle, (SERVO_CTR - 55)), (SERVO_CTR + 55));
    servo.write(control_angle);
    break;
  case TRIM_STATE_TRIM_STATE_MAX_LIFT_STBD:
    windAngle *= -1;
    if (MAX_LIFT_ANGLE > windAngle)
    {
      control_angle -= 2;
    }
    else if ((MAX_LIFT_ANGLE < windAngle))
    {
      control_angle += 2;
    }
    control_angle = min(max(control_angle, (SERVO_CTR - 55)), (SERVO_CTR + 55));
    servo.write(control_angle);
    break;
  case TRIM_STATE_TRIM_STATE_MAX_DRAG_PORT:
    servo.write(SERVO_CTR - 55);
    break;
  case TRIM_STATE_TRIM_STATE_MAX_DRAG_STBD:
    servo.write(SERVO_CTR + 55);
    break;
  case TRIM_STATE_TRIM_STATE_MIN_LIFT:
    servo.write(SERVO_CTR);
    break;
  case TRIM_STATE_TRIM_STATE_MANUAL:
    servo.write(control_angle);
    break;
  default:
    servo.write(control_angle);
    break;
  }

  return true;
}