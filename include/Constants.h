/**
 * @file Constants.h
 * @author Irina Lavryonova (ilavryonova@wpi.edu) - 2019/2020
 * @author Connor Burri (cjburri@wpi.edu) - 2020/2021
 * @author Tom Nurse (tjnurse@wpi.edu) - 2021/2022
 * @brief File containing variables common to the entire system, centralizing the settings of the sailbot
 * @version 2.0.2
 * @date 2022-4-11
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef Constants_h
#define Constants_h

// necessary libraries for the constants
#include <Arduino.h>

/* Trim servo */
const int SERVO_CTR    = 115;
const int SERVO_LO_LIM = SERVO_CTR-55;
const int SERVO_HI_LIM = SERVO_CTR+55;

/* Wind vane*/
const int POT_HEADWIND = 2060;

/* State angles */
const int MAX_LIFT_ANGLE = 30;

/* Pins */
const int potPin = 32;              //A19
const int batteryPin = 36;          //Battery V+
const int servoPin = 13;            //Servo
const int errorLED = 4;             //red
const int bleLED = 3;               //blue
const int powerLED = 5;             //green

#define POT_MAX 890
#define POT_MIN -830    

#endif