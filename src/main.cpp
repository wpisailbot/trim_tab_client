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
int control_angle;                        // The current angle that the servo is set to
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

void webSocketEvent(WStype_t type, uint8_t *payload, size_t length)
{
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
bool blinkState(void *);

void setup()
{
  /* Setting the mode of the pins necessary */
  pinMode(powerLED, OUTPUT); // Green LED
  pinMode(bleLED, OUTPUT);   // Blue LED
  pinMode(errorLED, OUTPUT); // Red LED

  /* Set up battery monitoring */
  battery.begin(3300, 1.43, &sigmoidal);

  /* Initializing variables to initial conditions */
  ledState = LOW;                         // LED starts in off position
  control_angle = SERVO_CTR;              // Trim tab starts off centralized
  state = TRIM_STATE_TRIM_STATE_MIN_LIFT; // The state is set to be in the center position or what we consider to be min lift
  Serial.begin(115200);

  /* Giving feedback that the power is on */
  digitalWrite(powerLED, HIGH);

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
  int n = MDNS.queryService("sailbot-trimtab", "tcp");
  if (n == 0)
  {
    Serial.println("no services found");
  }
  else
  {
    for (int i = 0; i < n; ++i)
    {
      // Print details for each service found
      Serial.print("Found service ");
      Serial.print(MDNS.hostname(i));
      Serial.print(" (");
      Serial.print(MDNS.IP(i));
      websockets_server_host = MDNS.IP(i).toString().c_str();
      Serial.print(":");
      Serial.print(MDNS.port(i));
      websockets_server_port = MDNS.port(i);
      Serial.println(")");
    }
  }
  Serial.println("Found mDNS, Connecting to server.");
  // Setup WebSocket
  webSocket.begin(websockets_server_host.c_str(), websockets_server_port);
  webSocket.onEvent(webSocketEvent);
  webSocket.setReconnectInterval(5000);

  /* Initializing the servo and setting it to its initial condition */
  servo.attach(servoPin);
  servo.write(control_angle);

  /* Starting the asynchronous function calls */
  servoTimer.every(10, servoControl);
  LEDTimer.every(1000, blinkState);
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
  windAngle = windAngle < 0 ? POT_HEADWIND + windAngle + (1023 - POT_HEADWIND) : windAngle; // wraps angle around
  windAngle = windAngle / 1023.0 * 360.0;                                                   // Convert to degrees, positive when wind from 0-180, negative when wind 180-359
  if (windAngle > 180)
  {
    windAngle = (360 - windAngle) * -1;
  }
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

/**
 * @author Irina Lavryonova
 * @author Tom Nurse
 * @brief controls the blinking operations within the LEDS
 */
bool blinkState(void *)
{
  // Toggle state
  ledState = ledState == LOW ? HIGH : LOW;

  digitalWrite(powerLED, HIGH);

  if (batteryWarning)
  {
    digitalWrite(errorLED, HIGH);
  }

  if (bleConnected)
  {
    digitalWrite(bleLED, ledState);
  }
  else
  {
    digitalWrite(bleLED, LOW);
  }

  switch (state)
  {
  case TRIM_STATE_TRIM_STATE_MAX_LIFT_PORT:
    digitalWrite(powerLED, ledState);
    break;
  case TRIM_STATE_TRIM_STATE_MAX_LIFT_STBD:
    if (ledState == HIGH)
    {
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
    }
    else
    {
      digitalWrite(powerLED, LOW);
    }
    break;
  case TRIM_STATE_TRIM_STATE_MAX_DRAG_PORT:
    if (ledState == HIGH)
    {
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
    }
    else
    {
      digitalWrite(powerLED, LOW);
    }
    break;
  case TRIM_STATE_TRIM_STATE_MAX_DRAG_STBD:
    if (ledState == HIGH)
    {
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
    }
    else
    {
      digitalWrite(powerLED, LOW);
    }
    break;
  case TRIM_STATE_TRIM_STATE_MIN_LIFT:
    if (ledState == HIGH)
    {
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
    }
    else
    {
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
      delay(100);
      digitalWrite(powerLED, HIGH);
      delay(100);
      digitalWrite(powerLED, LOW);
    }
    break;
  case TRIM_STATE_TRIM_STATE_MANUAL:
    digitalWrite(powerLED, HIGH);
    break;
  default:
    // Water is wet
    break;
  }

  return true;
}
