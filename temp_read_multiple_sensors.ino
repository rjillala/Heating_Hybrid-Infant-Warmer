#include <OneWire.h>
#include <DallasTemperature.h> 
#include <SPI.h>
#include <SD.h>

#define TEMP_PIN1 2
#define TEMP_PIN2 3//insert pin number

//For SD card
File myFile;

//Temperature rigamarole
OneWire oneWire1(TEMP_PIN1);
DallasTemperature sensor1(&oneWire1);
OneWire oneWire2(TEMP_PIN2);
DallasTemperature sensor2(&oneWire2);

//Temperature Variables
float mattressTemp1;
float mattressTemp2;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600); //setting baud rate

  //Setting up SD card
  Serial.print("Initializing SD card...");
  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    while (1);
  }
  Serial.println("initialization done.");

  //Begin Temperature Reading
  sensor1.begin();
  sensor2.begin();
}

void loop() {
  // put your main code here, to run repeatedly:
  //read sensors
  sensorRead();
}

//Function to read temperature sensor values
void sensorRead(){
  //Get Sensor Data
  sensor1.requestTemperatures();
  sensor2.requestTemperatures();

  mattressTemp1 = sensor1.getTempCByIndex(0);
  mattressTemp2 = sensor2.getTempCByIndex(0);

  //Print to Serial Monitor
  Serial.print(mattressTemp1);
  Serial.print(" ");
  Serial.println(mattressTemp2);
  
  //Store to SD card
  myFile = SD.open("72V-multiple-sensorsn.txt", FILE_WRITE);
  myFile.seek(EOF);
  myFile.print(millis());
  myFile.print(" ");
  myFile.print(mattressTemp1);
  myFile.print(" ");
  myFile.println(mattressTemp2);
  myFile.close();

  delay(500); //500 ms
}
