//helpful PID code link: http://www.electronoobs.com/eng_arduino_tut24_2.php

//temperature sensing libraries for DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

//Data wire is plugged into port EDIT CURRENTLY 2 on Arduino
#define ONE_WIRE_BUS 2

//Temperature rigamarole
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Temperature Variable
float mattressTemp;  //temperature of mattress
const double maxMattressTemp = 43; //maximum allowable mattress temp

//Initialize Output Pin
const int fetPin = 9;  //digital pin 9, PWM, mosfet; change to whatever pin it is

//Initialize Voltage Pin
const int batVoltagePin = A1; //change to whatever pin it is

//Initialize Current Pin
const int currentPin = A2;  //change to whatever pin it is

//Battery Variable(CALCULATIONS ASSUMING 3 RESISTORS AND FINDING VOLTAGE ACROSS 1)
double batLevel;  //voltage of battery
double batConvert = 5 / 1023;  //analog to digital conversion for battery voltage
int numVoltResistors = 3; //number of resistors in mesh with battery to read in voltage

//Current Variables
double currentSense;  //sensed current value
double current; //actual current value
const double maxCurrent = 3;  //upper bound for current
const double RSense = 300;  //sensing resistor resistance

//PID Constants - may have to change based on new circuit/Simulink model
const double Kp = 1; // proportional gain
const double Ki = 0.0009; //integral gain
const double Kd = 0.3; //derivative gain

//PID Variables
double P = 0; //P control term
double I = 0; //I control term
double D = 0; //D control term

double PIDInput; //input to PID Controller (mattress temperature)
double PIDOutput; //output of PID Controller (signal to mosfet controlling current)
double constrainPIDOutput; //constrained PID Output (saturates at maximum current)
double PWMOutput; //PID output constrained to PWM limits (0 to 255)

const double setpoint = 37; //Target Mattress Temperature

//PID Time Variables
unsigned long currTime, preTime; //time and previousTime
double elapsedTime; //time between current and previous time

//PID Error Variables
double error;
double preError;
double iError;
double dError;
const double iErrorMax = 5600; //unsure why this is max - mess around

//File file;

void setup() {
  //file = SD.open("mattress_vals.txt");
  Serial.begin(9600);
  //Serial.print("setup");
  
  //initialize mosfet pin as output
  pinMode(fetPin, OUTPUT);

  //prevent current from flowing
  digitalWrite(fetPin, LOW);
  
  //initialize sensor pins as input
  //pinMode(batVoltagePin, INPUT);
  //pinMode(currentPin, INPUT);

  //begin temperature reading
  sensors.begin();
}

void loop() {
  //read sensors
  
  sensorRead();

  //calculate PID values
  PIDCalc();

  //send Control Signal
  heat();
  
  //Display Information
  infoDisplay();
  
  delay(1000);
}

void sensorRead() {
  //Reading in Temperature
  sensors.requestTemperatures();
  mattressTemp = sensors.getTempCByIndex(0);
  //unsigned long tim = millis();
  //file.write(tim);
  //file.write(mattressTemp);
  Serial.println(mattressTemp);
  //Reading in Battery Voltage
  //batLevel = analogRead(batVoltagePin);
  //batLevel = batLevel * batConvert * numVoltResistors; //analog to digital conversion and multiply by number of resistors (KVL)
  
  //Reading in Current
  //currentSense = analogRead(currentPin);
}

void PIDCalc() {
  //set PID input to mattress temperature
  PIDInput = (double)(mattressTemp);

  //read time
  currTime = millis();

  //calculate elapsed time
  elapsedTime = (double)(currTime - preTime);

  //calculate error
  error = setpoint - PIDInput;

  //calculate integral of error
  iError += error * elapsedTime;
  iError = constrain(iError, -iErrorMax, iErrorMax); //still unsure of why constraining FIGURE OUT

  //calculate derivative of error
  dError = (error - preError)/elapsedTime;

  //calculate P, I, and D terms
  P = Kp * error;
  I = Ki * iError;
  D = Kd * dError;

  //calculate PIDOutput
  PIDOutput = P + I + D;

  //Constrain Output (multiply by 1000 for precision)
  constrainPIDOutput = constrain(PIDOutput, 0, maxCurrent)*1000;
  
  //Map output to PWM
  PWMOutput = map(constrainPIDOutput, 0, maxCurrent*1000, 0, 255);

  //update preTime and preError
  preError = error;
  preTime = currTime;
}

void heat() {
  //Set output to zero if over temp limit
  if(mattressTemp > maxMattressTemp){
    PWMOutput = 0;
  }

  //Send PWMOutput to fet
  analogWrite(fetPin, PWMOutput);
}

void infoDisplay() {
  //print temperature
  //Serial.print("Temp");
  //Serial.print(mattressTemp);
  
  //print battery voltage
  //Serial.print("Battery Voltage");
  //Serial.print(batLevel);
  
  //print current
  //Serial.print("Current");
  //Serial.print(currentSense);
}
