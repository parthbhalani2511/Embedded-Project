#include <GyverMotor.h>
#include <ezButton.h>
#define MOTOR_IN 2 
#define MOTOR_PWM 3
#define LIM_SWITCH_UP 8
#define LIM_SWITCH_DOWN 7
#define TACHO A0
#define LED 13

GMotor motor(DRIVER2WIRE, MOTOR_IN, MOTOR_PWM, HIGH); 
ezButton limitSwitch_UP(LIM_SWITCH_UP);
ezButton limitSwitch_DOWN(LIM_SWITCH_DOWN);

//Initial state and command
int state = -1; // Elevator is anywhere 
char command = '-'; //OFF mode

//Voltages
float source_voltage = 24.0;
float ref_voltage = 5.0; // Reference Voltage for ADC
// float ref_voltage = 4.096;

//Elevator velocity
int duty_cycle = 9; // in percentage (4% is MIN, 10% is MAX)
int u = 1024/100 * duty_cycle; // control input

//Time between floors (in seconds)
float t01 = 6;  
float t12 = 6;
float t02 = 12; 
float t21 = 4;

//Resistors (Voltage Divider for Tacho output)
float R1 = 10.0;
float R2 = 10.0;

void setup() {
  //PWM 10 bit (Pins D3 and D11 - 31.4 KHz) to avoid "beeping"
  TCCR2B = 0b00000001;  // x1
  TCCR2A = 0b00000001;  // phase correct
  //Activate 10-bit Mode
  motor.setResolution(10);

  //Key on start!
  motor.setMode(FORWARD);

  //Use external voltage reference for ADC
  // analogReference(EXTERNAL);

  //Setup Serial Monitor
  Serial.begin(9600);
  Serial.println("Start of the program!");

  //Built-in LED to show ON/OFF mode
  pinMode(LED, OUTPUT);

  //Set debounce time of Limit switches
  limitSwitch_DOWN.setDebounceTime(50);
  limitSwitch_UP.setDebounceTime(50);
}

void loop() {
  //Check signals from limit switches
  limitSwitch_DOWN.loop();
  limitSwitch_UP.loop();
  // readTacho();

  //Read commands from the app
  if (Serial.available()) {command = Serial.read();}
  
  //Emergency cases
  if (command == '-' || limitSwitch_UP.getState() == LOW) {    
    stop();
    digitalWrite(LED, LOW);
    state = -1;
    command = '-';
  }

  //OFF -> ON (go Home)
  if (state == -1 && command == '+') {
    digitalWrite(LED, HIGH);
    goHome();
  }

  //GROUND floor -> FIRST floor
  if (state == 0 && command == '1') {
    static uint32_t tmr01 = millis();
    if (millis() - tmr01 <= t01*1000)
      goUp();
    else {
      stop();
      state = 1;
      command = '1';
      // Serial.println("1-st floor");
    }
  }

  //GROUND floor -> SECOND floor
  if (state == 0 && command == '2') {
    static uint32_t tmr02 = millis();
    if (millis() - tmr02 <= t02*1000)
      goUp();
    else {
      stop(); 
      state = 2;
      command = '2';
      // Serial.println("2-nd floor");
    }
  }

  //FIRST floor -> SECOND floor
  if (state == 1 && command == '2') {
    static uint32_t tmr12 = millis();
    if (millis() - tmr12 <= t12*1000)
      goUp();
    else {
      stop(); 
      state = 2;
      command = '2';
      // Serial.println("2-nd floor");
    }
  }

  //SECOND floor -> FIRST floor
  if (state == 2 && command == '1') {
    static uint32_t tmr21 = millis();
    if (millis() - tmr21 <= t21*1000)
      goDown();
    else {
      stop(); 
      state = 1;
      command = '1';
      // Serial.println("1-st floor");
    }
  }

  //FIRST floor -> GROUND floor
  if (state == 1 && command == '0')
    goHome();

  //SECOND floor -> GROUND floor
  if (state == 2 && command == '0')
    goHome();
}


void goUp() {
  motor.setSpeed(u);
}

void goDown() {
  motor.setSpeed(-u);
}

void stop() {
  motor.setSpeed(0);
}

//Go Home (GROUND floor)
void goHome() {
  if (limitSwitch_DOWN.getState() == HIGH) 
    goDown();
  else {
    // Serial.println("Ground floor (HOME)");
    stop(); 
    state = 0;
    command = '0';
  }
}

void readTacho() {
  static uint32_t tmr = 0;

  if (millis() - tmr >= 500) { // every 0.5 s
    tmr = millis();
    
    //Read the Analog Input
    float adc_value = analogRead(TACHO);

    //Determine voltage at ADC input
    float adc_voltage  = (adc_value / 1024.0) * ref_voltage; 
      
    //Calculate tacho voltage
    float tacho_voltage = adc_voltage / (R2/(R1+R2)); 

    //Calculate angular rate
    int velocity = round(1000/3.25 * tacho_voltage);

    //Print results to Serial Monitor
    Serial.print("Input Voltage = ");
    Serial.println(source_voltage * duty_cycle/100, 2);
    Serial.print("Tacho Voltage = ");
    Serial.println(tacho_voltage, 2);
    Serial.print("Speed = ");
    Serial.println(velocity);
    Serial.println();
  }
}