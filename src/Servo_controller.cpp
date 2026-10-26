#include "Constants.h"

bool move_flag = false;
Servo servo;

bool servoControl(int currentWindAngle, TRIM_STATE state){
  // Read, format, and scale angle of attack reading from the encoder
  
  // Serial.println(windAngle);

  // Set debug LEDs to on to indicate servo control is active
  // digitalWrite(led1Pin, HIGH);
  // digitalWrite(led2Pin, HIGH);

  // Write servo position to one read from the Arduino
  //  servo.write(SERVO_CTR + control_angle - 200 - 90);
  static unsigned long lastTrimAdjustTime = 0;
  static int currentAngle = SERVO_CTR;

  switch (state)
  {
  case TRIM_STATE_MAX_LIFT_STBD: {
      float windAngleInverse = currentWindAngle * -1;
      unsigned long currentTime = millis();
      if(currentTime-lastTrimAdjustTime>TRIM_ADJUST_INTERVAL_MS){
        lastTrimAdjustTime = currentTime;
        //Serial.print("Current wind: ");
        //Serial.println(windAngleInverse); 
        if (MAX_LIFT_ANGLE > windAngleInverse+angle_tolerance)
        {
          Serial.println("Increasing angle ");
          control_angle += 5;
          if(control_angle>SERVO_HI_LIM){
            control_angle = SERVO_HI_LIM;
          }
        }
        else if ((MAX_LIFT_ANGLE+angle_tolerance < windAngleInverse))
        {
          
          Serial.println("Decreasing angle ");
          control_angle -= 5;
          if(control_angle<SERVO_CTR+10){
            control_angle = SERVO_CTR+10;
          }
        }else{
          Serial.println("");
        }
      }
      int current_control_angle = min(max(control_angle, (SERVO_CTR - 55)), (SERVO_CTR + 55));

      // Serial.print("Control angle: ");
      // Serial.println(control_angle);
      currentAngle = current_control_angle;
      servo.write(current_control_angle);
      //Serial.println("Max lift port");
    }
    break; 
  case TRIM_STATE_MAX_LIFT_PORT: {
      unsigned long currentTime = millis();
      if(currentTime-lastTrimAdjustTime>TRIM_ADJUST_INTERVAL_MS){
        //Serial.print("Max Lift PORT: ");
        //Serial.println(currentWindAngle); 
        lastTrimAdjustTime = currentTime;
        if (MAX_LIFT_ANGLE > currentWindAngle+angle_tolerance)
        {
          control_angle -= 5;
          if(control_angle<SERVO_LO_LIM){
            control_angle = SERVO_LO_LIM;
          }
        }
        else if ((MAX_LIFT_ANGLE+angle_tolerance < currentWindAngle))
        {
          control_angle += 5;
          if(control_angle>SERVO_CTR-10){
            control_angle = SERVO_CTR-10;
          }
        }
      }
      int current_control_angle = min(max(control_angle, (SERVO_CTR - 55)), (SERVO_CTR + 55));
      currentAngle = current_control_angle;
      servo.write(current_control_angle);
      //Serial.println("Max lift stbd");
    }
    break;
  case TRIM_STATE_MAX_DRAG_STBD:
    movementHandler(SERVO_LO_LIM, currentAngle);
    //Serial.println("Max drag port");
    break;
  case TRIM_STATE_MAX_DRAG_PORT:
    movementHandler(SERVO_HI_LIM, currentAngle);
    //Serial.println("Max drag stbd");
    break;
  case TRIM_STATE_MIN_LIFT:
    movementHandler(SERVO_CTR, currentAngle);
    //Serial.println("Min lift");
    break;
  case TRIM_STATE_MANUAL:
    state = TRIM_STATE_MANUAL;
    movementHandler(control_angle, currentAngle);
    break;
  default:
    state = TRIM_STATE_MANUAL; 
    Serial.println("State: manual");
    break;
  }

  return true;
}

//motion helper function
void movementHandler(int goal_angle, int currentAngle) {
    static int targetAngle = -1;
    static int startAngle = 0;
    static int t_iter = 0;
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