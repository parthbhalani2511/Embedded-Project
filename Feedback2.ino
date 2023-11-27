#include <GyverMotor.h>
#include <ezButton.h>
#define MOTOR_IN 2  
#define MOTOR_PWM 3
#define HC_ECHO 4
#define HC_TRIG 5
#define LIM_SWITCH_UP 8
#define LIM_SWITCH_DOWN 7
#define LED_mode 13
#define LED_floor 12

GMotor motor(DRIVER2WIRE, MOTOR_IN, MOTOR_PWM, HIGH); 
ezButton limitSwitch_UP(LIM_SWITCH_UP);
ezButton limitSwitch_DOWN(LIM_SWITCH_DOWN);

//Initial state and command
int state = -1; // Elevator is anywhere 
char command = 'b'; //OFF mode

//Height of the floor
float height0 = 82.0;
float height1 = 51.5;
float height2 = 18.0;

//Sensor measurement
float dist = 0;

bool flag1 = false, flag2 = false, flag3 = false;

void setup() {
  //PWM 10 bit (Pins D3 and D11 - 31.4 KHz) to avoid "beeping"
  TCCR2B = 0b00000001;  // x1
  TCCR2A = 0b00000001;  // phase correct
  //Activate 10-bit Mode
  motor.setResolution(10);

  //Key on start!
  motor.setMode(FORWARD);

  //Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Start of the program!");

  //Built-in LED to show ON/OFF mode
  pinMode(LED_mode, OUTPUT);
  pinMode(LED_floor, OUTPUT);

  //Distance sensor
  pinMode(HC_TRIG, OUTPUT);
  pinMode(HC_ECHO, INPUT);

  //Set debounce time of Limit switches
  limitSwitch_DOWN.setDebounceTime(50);
  limitSwitch_UP.setDebounceTime(50);
}

void loop() {
  //Handle signals from limit switches
  limitSwitch_DOWN.loop();
  limitSwitch_UP.loop();

  //Emergency case (RED BUTTON)
  if (limitSwitch_UP.getState() == LOW) {    
    stop();
    digitalWrite(LED_mode, LOW);
    digitalWrite(LED_floor, LOW);
    state = -1;
    command = 'b';
    if (flag1 == false) {
      // Serial.println("RED BUTTON!");
      flag1 = true;
    }
  }

  // else if (limitSwitch_DOWN.getState() == LOW) {
  //   stop();
  //   digitalWrite(LED_mode, LOW);
  //   digitalWrite(LED_floor, LOW);
  //   state = -1;
  //   command = 'b';
  // }

  //Read commands from the app
  else if (Serial.available()) 
    command = Serial.read();
  
  //Read the Distance sensor (every 100ms)
  static uint32_t tmr_dist = 0;
  if (millis() - tmr_dist >= 100) { 
    tmr_dist = millis();
    dist = getDist(); 
    // Serial.println(dist);
  }

  //OFF -> ON (go Home)
  if (command == 'a' && state == -1) {
    digitalWrite(LED_mode, HIGH);
    if (flag2 == false) {
      // Serial.println("ON mode");
      flag2 = true;
      flag3 = false;
      flag1 = false;
    }
    goHome();
  } 

  //ON -> OFF (stop)
  else if (command == 'b') {
    stop();
    digitalWrite(LED_mode, LOW);
    digitalWrite(LED_floor, LOW);
    state = -1;
    if (flag3 == false) {
      // Serial.println("OFF mode");
      flag3 = true;
      flag2 = false;
    }      
  }  

  //FIRST or SECOND floor -> GROUND floor
  else if (command == '0' && (state == 1 || state == 2)) {
    goHome();
  }

  else if (command == '1') {
    //GROUND floor -> FIRST floor
    if (state == 0) {
      float err = dist - height1;
      Serial.println(err);
      if (err > 10) {goUp(9);}
      else if (err > 5) {goUp(8);}
      else if (err > 0) {goUp(7);}
      else {
        stop();
        digitalWrite(LED_floor, LOW); 
        state = 1;
        // Serial.println("1-st floor");
      }
    }
    //SECOND floor -> FIRST floor
    else if (state == 2) {
      float err = height1 - dist;
      Serial.println(err);
      if (err > 10) {goDown(8);}
      else if (err > 5) {goDown(7);}
      else if (err > 0) {goDown(6);}
      else {
        stop();
        digitalWrite(LED_floor, LOW); 
        state = 1;
        // Serial.println("1-st floor");
      }
    }
  }

  //GROUND or FIRST floor -> SECOND floor
  else if (command == '2' && (state == 0 || state == 1)) {
    float err = dist - height2;
    Serial.println(err);
    if (err > 10) {goUp(9);}
    else if (err > 5) {goUp(8);}
    else if (err > 0) {goUp(7);}
    else {
      stop();
      digitalWrite(LED_floor, HIGH); 
      state = 2;
      // Serial.println("2-nd floor");
    }
  }

  // else if (command == 'd') {
  //   // Serial.println(getDist());
  //   command = 'b';
  // }
}

void goUp(int duty_cycle) {
  float u = 1024/100 * duty_cycle;
  motor.setSpeed(u);
}

void goDown(int duty_cycle) {
  float u = 1024/100 * duty_cycle;
  motor.setSpeed(-u);
}

void stop() {
  motor.setSpeed(0);
}

//Go Home (GROUND floor)
void goHome() {
  if (limitSwitch_DOWN.getState() == HIGH) {
    float err = height0 - dist;
    // Serial.println(err);
    if (err > 15) {goDown(9);}
    else if (err > 10) {goDown(8);}
    else if (err > 5) {goDown(7);}
    else {goDown(6);}
  }
  else {
    stop(); 
    digitalWrite(LED_floor, HIGH);
    state = 0;
    command = '0';
    // Serial.println("Ground floor");
  }
}

float getDist() {
  digitalWrite(HC_TRIG, HIGH);
  delayMicroseconds(10);    // impulse 10 mcs
  digitalWrite(HC_TRIG, LOW);
  uint32_t us = pulseIn(HC_ECHO, HIGH); // calculate the time of returning impulse
  return (us / 58.3);   // calculate the distance
}