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
int fifty = 127;          // how bright the LED is
int seventyFive = 191;    // how many points to fade the LED by


void setup() {
  // declare pin 9 to be an output:
  pinMode(heat, OUTPUT);
  
  Serial.begin(9600);
  Serial.print("setup");

  //begin temperature reading
  sensors.begin();
}

void loop() {
  //read sensors
  sensorRead();
  //delay(500);
}

void sensorRead() {
  // set the brightness of pin 9:
  analogWrite(heat, fifty);
  
  sensors.requestTemperatures();
  
  mattressTemp = sensors.getTempCByIndex(0);
  
  //Serial.print("Temp Celcius: ");
  //Serial.print(mattressTemp);
  Serial.println(mattressTemp);
  //Serial.print("\n");
  delay(100);
}
