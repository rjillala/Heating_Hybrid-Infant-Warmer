//Temperature read to eliminate humming noise 

//temperature sensing libraries for DS18B20
#include <OneWire.h>
#include <DallasTemperature.h>

//Data wire is plugged into port EDIT CURRENTLY 12 on Arduino
#define ONE_WIRE_BUS 12

//Temperature rigamarole
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

//Temperature variables
float mattressTemp;
int heat = 9;           // the PWM pin the element is attached to
//int fifty = 127;          // how bright the LED is
int duty_cycle = 127;       //0-254 with 254 being 100 percent duty cycle
//int seventyFive = 191;    // how many points to fade the LED by


void setup() {

  //PWM frequency for D9 & D10: 490.20 Hz (The DEFAULT)- https://www.etechnophiles.com/change-frequency-pwm-pins-arduino-uno/
  //TCCR1B = TCCR1B & B11111000 | B00000001; // set timer 1 divisor to 1 for PWM frequency of 31372.55 Hz
  TCCR1B = TCCR1B & B11111000 | B00000010; // for PWM frequency of 3921.16 Hz
  
  // declare pin 9 to be an output:
  pinMode(heat, OUTPUT);
  
  Serial.begin(9600);
  //Serial.print("setup");

  //begin temperature reading
  sensors.begin();
}

void loop() {
  //read sensors
  sensorRead();
  //delay(500);
}

//Function to read temperature sensor values
void sensorRead() {
  // set the brightness of pin 9:
  analogWrite(heat, duty_cycle);
  
  sensors.requestTemperatures();
  
  mattressTemp = sensors.getTempCByIndex(0);  //where does getTempCByIndex(0) come from?
  
  //Serial.print("Temp Celcius: ");
  //Serial.print(mattressTemp);
  Serial.println(mattressTemp);
  //Serial.print("\n");
  delay(500);
}
