*/

// Supporting Arduino Forum Topics:
// Waveshare e-paper displays with SPI: http://forum.arduino.cc/index.php?topic=487007.0
// Good Dispay ePaper for Arduino : https://forum.arduino.cc/index.php?topic=436411.0

// mapping suggestion from Waveshare 2.9inch e-Paper to Wemos D1 mini
// BUSY -> D2, RST -> D4, DC -> D3, CS -> D8, CLK -> D5, DIN -> D7, GND -> GND, 3.3V -> 3.3V

// mapping suggestion for ESP32, e.g. LOLIN32, see .../variants/.../pins_arduino.h for your board
// NOTE: there are variants with different pins for SPI ! CHECK SPI PINS OF YOUR BOARD
// BUSY -> 4, RST -> 16, DC -> 17, CS -> SS(5), CLK -> SCK(18), DIN -> MOSI(23), GND -> GND, 3.3V -> 3.3V

// mapping suggestion for AVR, UNO, NANO etc.
// BUSY -> 7, RST -> 9, DC -> 8, CS-> 10, CLK -> 13, DIN -> 11

// include library, include base class, make path known
#include <GxEPD.h>

// select the display class to use, only one
#include <GxGDEP015OC1/GxGDEP015OC1.cpp>    // 1.54" b/w
//#include <GxGDEW0154Z04/GxGDEW0154Z04.cpp>  // 1.54" b/w/r
//#include <GxGDE0213B1/GxGDE0213B1.cpp>      // 2.13" b/w
//#include <GxGDEW0213Z16/GxGDEW0213Z16.cpp>  // 2.13" b/w/r
//#include <GxGDEH029A1/GxGDEH029A1.cpp>      // 2.9" b/w
//#include <GxGDEW029Z10/GxGDEW029Z10.cpp>    // 2.9" b/w/r
//#include <GxGDEW027C44/GxGDEW027C44.cpp>    // 2.7" b/w/r
//#include <GxGDEW042T2/GxGDEW042T2.cpp>      // 4.2" b/w
//#include <GxGDEW075T8/GxGDEW075T8.cpp>      // 7.5" b/w
//#include <GxGDEW075Z09/GxGDEW075Z09.cpp>    // 7.5" b/w/r

// uncomment next line for drawBitmap() test
#include GxEPD_BitmapExamples

// FreeFonts from Adafruit_GFX
#include <Fonts/FreeMonoBold9pt7b.h>
#include <Fonts/FreeMonoBold12pt7b.h>
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>


#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>

#if defined(ESP8266)

// generic/common.h
//static const uint8_t SS    = 15;
//static const uint8_t MOSI  = 13;
//static const uint8_t MISO  = 12;
//static const uint8_t SCK   = 14;
// pins_arduino.h
//static const uint8_t D8   = 15;
//static const uint8_t D7   = 13;
//static const uint8_t D6   = 12;
//static const uint8_t D5   = 14;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 0, 2); // arbitrary selection of D3(=0), D4(=2), selected for default of GxEPD_Class
// GxGDEP015OC1(GxIO& io, uint8_t rst = 2, uint8_t busy = 4);
GxEPD_Class display(io); // default selection of D4(=2), D2(=4)

#elif defined(ESP32)

// pins_arduino.h, e.g. LOLIN32
//static const uint8_t SS    = 5;
//static const uint8_t MOSI  = 23;
//static const uint8_t MISO  = 19;
//static const uint8_t SCK   = 18;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 17, 16); // arbitrary selection of 17, 16
// GxGDEP015OC1(GxIO& io, uint8_t rst = D4, uint8_t busy = D2);
GxEPD_Class display(io, 16, 4); // arbitrary selection of (16), 4

#elif defined(ARDUINO_ARCH_SAMD)

// variant.h of MKR1000
//#define PIN_SPI_MISO  (10u)
//#define PIN_SPI_MOSI  (8u)
//#define PIN_SPI_SCK   (9u)
//#define PIN_SPI_SS    (24u) // should be 4?
// variant.h of MKRZERO
//#define PIN_SPI_MISO  (10u)
//#define PIN_SPI_MOSI  (8u)
//#define PIN_SPI_SCK   (9u)
//#define PIN_SPI_SS    (4u)

GxIO_Class io(SPI, 4, 7, 6);
GxEPD_Class display(io, 6, 5);

#elif defined(_BOARD_GENERIC_STM32F103C_H_)

// STM32 Boards (STM32duino.com)
// Generic STM32F103C series
// aka BluePill
// board.h
//#define BOARD_SPI1_NSS_PIN        PA4
//#define BOARD_SPI1_MOSI_PIN       PA7
//#define BOARD_SPI1_MISO_PIN       PA6
//#define BOARD_SPI1_SCK_PIN        PA5
//enum {
//    PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7, PA8, PA9, PA10, PA11, PA12, PA13,PA14,PA15,
//  PB0, PB1, PB2, PB3, PB4, PB5, PB6, PB7, PB8, PB9, PB10, PB11, PB12, PB13,PB14,PB15,
//  PC13, PC14,PC15
//};
// variant.h
//static const uint8_t SS   = BOARD_SPI1_NSS_PIN;
//static const uint8_t SS1  = BOARD_SPI2_NSS_PIN;
//static const uint8_t MOSI = BOARD_SPI1_MOSI_PIN;
//static const uint8_t MISO = BOARD_SPI1_MISO_PIN;
//static const uint8_t SCK  = BOARD_SPI1_SCK_PIN;

// GxIO_SPI(SPIClass& spi, int8_t cs, int8_t dc, int8_t rst = -1, int8_t bl = -1);
GxIO_Class io(SPI, SS, 8, 9);
// GxGDEP015OC1(GxIO& io, uint8_t rst = 9, uint8_t busy = 7);
GxEPD_Class display(io, 9, 3);

#else

// pins_arduino.h, e.g. AVR
//#define PIN_SPI_SS    (10)
//#define PIN_SPI_MOSI  (11)
//#define PIN_SPI_MISO  (12)
//#define PIN_SPI_SCK   (13)

GxIO_Class io(SPI, SS, 8, 9); // arbitrary selection of 8, 9 selected for default of GxEPD_Class
//GxIO_DESTM32L io;
//GxIO_GreenSTM32F103V io;
GxEPD_Class display(io);

#endif

//Heated Mattress
//12-6-17

//Include ADC Library
#include <Adafruit_ADS1015.h>

Adafruit_ADS1115 ads1115(0x48);

//Include E-Ink Libraries

/*
#include <GxEPD.h>
#include <GxGDEP015OC1/GxGDEP015OC1.cpp>    // 1.54" b/w
#include GxEPD_BitmapExamples
#include <Fonts/FreeMonoBold18pt7b.h>
#include <Fonts/FreeMonoBold24pt7b.h>
#include <GxIO/GxIO_SPI/GxIO_SPI.cpp>
#include <GxIO/GxIO.cpp>


//E-Ink
//GxIO_Class io(SPI, SS, 8, 9); // arbitrary selection of 8, 9 selected for default of GxEPD_Class
//GxEPD_Class display(io);

*/
//Initialize LED Pins
const int gLedPin = 36; //digital pin 36, non PWM, green LED
const int yLedPin = 32; //digital pin 4, non PWM, yellow LED
const int rLedPin = 34; //digital pin 12, non PWM, red LED

//Initialize Output Pin
const int fetPin = 6;  //digital pin 10, PWM, mosfet

//Initialize Temperature Sensor Pins
const int mattressTempPin = A0; //analog pin A1, mattress temperature

//Initialize Voltage Pin
const int batVoltagePin = A2;   //analog pin A2, battery voltage

//Initialize Current Pin
const int currentPin = A10;       //analog pin A3, current

//Initialize Battery Variables
double batLevel;    //variable displaying value to LEDs
const double batUpper =11.5;  //upper bound for yellow led display
const double batLower = 10.5;  //lower bound for yellow led display
const double batMin = 10.25; //minimum allowable battery voltage

//Initialize Current Variables
double currentSense;  //sensed current value
double current; //actual current value
const double maxCurrent = 3;  //upper bound for current
const double RSense = 300;      //sensing resistor resistance

//Initialize Temperature Variables
double mattressTemp; //mattress temperature
const double maxMattressTemp = 43;  //maximum allowable mattress temperature
double avgMattressTemp; //average mattress temperature
double infantTemp; //infant temperature

//Initialize Control Variables
double runtime; //time
double preRuntime = 0;  //previous time
double dt; //time differential

double PIDInput; //input to PID Controller (mattress temperature)
double PIDOutput;  //output of PID controller (signal to mosfet controlling current)
double constrainPIDOutput; //constrained PID Output (saturates at maximum current)
double PWMOutput; //output constrained to PWM limits (0 to 255)

const double setpoint = 26; //Target Mattress Temperature

double error; //difference between setpoint and PID input
double preError = 0; //previous error
double dError; //derivative of error
double iError; //integral of error
double iErrorMax = 5600;

const double Kp = 1; //proportional gain
const double Ki = 0.0009; //integral gain
const double Kd = 0.3; //derivative gain

double P = 0; //P control term
double I = 0; //I control term
double D = 0; //D control term

//Error Shutoff Variables
int errorCode = 0;

//variables for averaging sensor reading
const int numReadings = 10;
double readings[numReadings];
int readIndex = 0;
double total = 0;
double average = 0;

//variables to control display refresh rate
int displayCounter = 0;
int isDisplay;

//adc variables
int16_t adc0;
double mattressTempVoltageRead;
double mattressTempVoltage;

int serialCount = 0;
  
void setup() {
  //setup input and outputs and run startup sequence
 
  
  Serial.begin(9600);
  Serial.print("setup");
  //Set LED and fet pins as OUTPUT
  pinMode(gLedPin, OUTPUT);
  pinMode(yLedPin, OUTPUT);
  pinMode(rLedPin, OUTPUT);
  pinMode(fetPin, OUTPUT);

  //Set sensors and battery voltage and current as INPUT
//  pinMode(infantTempPin, INPUT);
  pinMode(mattressTempPin, INPUT);
  pinMode(batVoltagePin, INPUT);
  pinMode(currentPin, INPUT);

  //prevent any current from flowing
  digitalWrite(fetPin, LOW);

  //initiate E-Ink display
  display.init();

  //activate ADC
  ads1115.begin();

  //initialize vector for temperature averaging
  for (int thisReading = 0; thisReading < numReadings; thisReading++) {
    readings[thisReading] = 0;
  }

  //run startup sequence
  startupSequence();
}


void loop() {
  //loop through all functions
  
  //read sensors
  sensorRead();
 
  
  //calculate PID values
  PIDCalc();
  
  //Send Control Signal
  heat();
  
  //Display Information
  infoDisplay();
  
  //Test Code
  //test();
  }
  
void startupSequence()  {
  
  //startup sequence
  //only runs once
  
  //turn screen on
  //[insert code for turning the screen on]

  //LED sequence - red, yellow, green on, then off, then repeat
  digitalWrite(rLedPin, HIGH);
  delay(500);
  digitalWrite(yLedPin, HIGH);
  delay(500);
  digitalWrite(gLedPin, HIGH);
  delay(1000);
  
  digitalWrite(rLedPin, LOW);
  digitalWrite(yLedPin, LOW);
  digitalWrite(gLedPin, LOW);
  delay(500);
  
  digitalWrite(rLedPin, HIGH);
  delay(500);
  digitalWrite(yLedPin, HIGH);
  delay(500);
  digitalWrite(gLedPin, HIGH);
  delay(1000);
  
  digitalWrite(rLedPin, LOW);
  digitalWrite(yLedPin, LOW);
  digitalWrite(gLedPin, LOW);  
  }


void sensorRead() {
  //Serial.print("sensorRead") ;
  //This function reads in battery voltabe, current, and sensors
  
  //read battery voltage
  batLevel = analogRead(batVoltagePin);
  
  //convert voltage to battery level
  batLevel = batLevel/1023*5*3;

  //read current
  currentSense = analogRead(currentPin);
  

  //read mattress temperature
  adc0 = ads1115.readADC_SingleEnded(0);
  mattressTempVoltageRead = adc0;//analogRead(mattressTempPin);
  mattressTempVoltage = mattressTempVoltageRead*.1875;
  mattressTemp = 30 + (mattressTempVoltage-943)*(1/(-5.194));

  //compute moving average:
    
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = mattressTemp;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  avgMattressTemp = total/numReadings;
  delay(1);
  }


void PIDCalc() {
  //This function calculates PID control values

  //mattress temperature is PID input
  PIDInput = avgMattressTemp;

  //read time
  runtime = millis();

  //calculate time differential
  dt = runtime-preRuntime;

  //calculate error
  error = setpoint-PIDInput;
  
  //calculate derivative of error
  dError = (error-preError)/dt;
  
  //calculate integral of error
  iError = iError+error*dt;
  iError = constrain(iError,-iErrorMax,iErrorMax);

  //calculate P, I, and D terms
  P = Kp*error;
  I = Ki*iError;
 // I = constrain(I,-IMax,IMax);
  D = Kd*dError;

  //sum P, I, and D terms
  PIDOutput = P+I+D;

  //Constrain Output (multiply by 1000 for precision)
  constrainPIDOutput = constrain(PIDOutput, 0, maxCurrent)*1000;
  
  //Map output to PWM
  PWMOutput = map(constrainPIDOutput, 0, maxCurrent*1000, 0, 255);

  //store previous error value
  preError = error;

  //store previous time value
  preRuntime = runtime;
  }


void heat() {
  //This function sends the heat signal to the fet
    //Serial.println("heat");

  //Uncomment for Error
  /* if (batLevel < batMin) {
    errorCode = 1;
    errorShutoff();
  }
  */
  //set output to zero if over current limit
  /*
  current = currentSense/RSense;  
  if (current > maxCurrent)  {
    errorCode = 2;
    errorShutoff();
    }
  */
  
  //Set output to zero if over temp limit
  if(mattressTemp > maxMattressTemp){
    PWMOutput = 0;
    }

  //Send PWMOutput to fet
  analogWrite(fetPin, PWMOutput);
  }


void infoDisplay() {
  //This function controls LED and E-Ink Display
  //battery level display
  //Serial.println("infoDisplay");
  if (batLevel >= batUpper) {
    digitalWrite(gLedPin, HIGH);//displays green
    digitalWrite(yLedPin, LOW); //makes sure only 1 color is displayed
    digitalWrite(rLedPin, LOW); //makes sure only 1 color is displayed
    }
  
  if (batLevel < batUpper && batLevel > batLower) {
    digitalWrite(yLedPin, HIGH);//displays yellow
    digitalWrite(gLedPin, LOW); //makes sure only 1 color is displayed
    digitalWrite(rLedPin, LOW); //makes sure only 1 color is displayed
    }
  
  if  (batLevel<=batLower){
    digitalWrite(rLedPin, HIGH);//displays red
    digitalWrite(gLedPin, LOW); //makes sure only 1 color is displayed
    digitalWrite(yLedPin, LOW); //makes sure only 1 color is displayed
    }

  //e-ink display
  displayCounter = displayCounter + 1;
  
  displayCounter = displayCounter + 1;
  isDisplay = displayCounter % 100;
 
  
  
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(3);

  
  display.setCursor(0, 170);
  display.setFont(&FreeMonoBold24pt7b);
  display.setTextSize(.5);
  display.println(avgMattressTemp);  //replace 37 with infant_tempclyako

  display.setCursor(160, 110);
  display.setFont(&FreeMonoBold24pt7b);
  display.setTextSize(0.5);
  display.println("°C");
  if (isDisplay == 0){
  
  display.drawPaged(infoDisplay);
  //delay(1000);
  //Serial.print("boop");
  }

  //Serial monitor print-outs
 /*Serial.print(total);
  Serial.print("\t");
  Serial.print(avgMattressTemp);
  Serial.print("\t");
  Serial.print(error);
  Serial.print("\t");
  Serial.print(P);
  Serial.print("\t");
  Serial.print(I);
  Serial.print("\t");
  Serial.print(D);
  Serial.print("\t");
  Serial.print(constrainPIDOutput);
  Serial.print("\t");
  Serial.println(PWMOutput);
  //Serial.print("\t")
  */
  serialCount = serialCount+1;
  
  Serial.print(millis());
  Serial.print("\t");
  Serial.print(avgMattressTemp);
  Serial.print("\t");
  Serial.print(currentSense);
  Serial.print("\t");
  Serial.print(batLevel);
  Serial.print("\t");
    Serial.print(I);
  Serial.print("\t");
  Serial.println(PWMOutput);
  
  }
  


/*void errorShutoff() {
    //if error occurs this function shuts off power
    // Serial.println("errorshutoff");
    //ensure no more heat is sent out
    PWMOutput = 0;
    analogWrite(fetPin,PWMOutput);

    //display what error has occured
    if (errorCode == 1) {
      //display error 1
    }

    if (errorCode == 2) {
      //display error 2
    }
    
    while (errorCode != 0)  {
      //blink LEDs until system is manually shut off
      digitalWrite(rLedPin, HIGH);

      delay(500);
  
      digitalWrite(rLedPin, LOW);
      delay(500);
    }
}
*/
    
/*void tempDisplay() {

  displayCounter = displayCounter + 1;
  isDisplay = displayCounter % 100;
 
  Serial.println("tempdisplay");
  
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(3);

  
  display.setCursor(0, 170);
  display.setFont(&FreeMonoBold24pt7b);
  display.setTextSize(3);
  display.println(avgMattressTemp);  //replace 37 with infant_tempclyako

  display.setCursor(160, 110);
  display.setFont(&FreeMonoBold24pt7b);
  display.setTextSize(0.5);
  display.println("°C");
  if (isDisplay == 0){
  display.drawPaged(test);
  delay(1000);
  }

}
*/
    
    


void test(){
analogWrite(fetPin,0);
delay(3000);
analogWrite(fetPin,150);
delay(3000);
analogWrite(fetPin,255);
delay(3000);
/*double  testVoltage = analogRead(A0);
  Serial.println(testVoltage);
  
    //test function
    displayCounter = displayCounter + 1;

    int16_t adc0;
    adc0 = ads1115.readADC_SingleEnded(0);
    
    //E-Ink/temp Demonstration

    //analogReference(INTERNAL);
    double mattressTempVoltageRead = adc0;//analogRead(mattressTempPin);
    double mattressTempVoltage = mattressTempVoltageRead*.1875;
    double mattressTemp = 30 + (mattressTempVoltage-943)*(1/(-5.194));
    
  total = total - readings[readIndex];
  // read from the sensor:
  readings[readIndex] = mattressTemp;
  // add the reading to the total:
  total = total + readings[readIndex];
  // advance to the next position in the array:
  readIndex = readIndex + 1;

  // if we're at the end of the array...
  if (readIndex >= numReadings) {
    // ...wrap around to the beginning:
    readIndex = 0;
  }

  average = total/numReadings;
  delay(1);
    
    

    double fetPWM = map(average, 23, 27, 0, 255);
          fetPWM = constrain(fetPWM, 0, 255);

     //fet demonstration
    //analogWrite(fetPin,255);
    //delay(5000);
    analogWrite(fetPin,fetPWM);
    //delay(5000);

    Serial.print(average);
    Serial.print("\t");
    Serial.println(fetPWM);

    isDisplay = displayCounter % 100;
    Serial.print("isDisplay");
    Serial.print(isDisplay);

  
  
  display.fillScreen(GxEPD_WHITE);
  display.setTextColor(GxEPD_BLACK);
  display.setRotation(3);

  
  display.setCursor(0, 170);
//  display.setFont(&FreeMonoBold24pt7b);
  display.setTextSize(3);
  display.println(average);  //replace 37 with infant_tempclyako

  display.setCursor(160, 110);
 // display.setFont(&FreeMonoBold24pt7b);
  display.setTextSize(0.5);
  display.println("°C");
  if (isDisplay == 0){
  display.drawPaged(test);
  delay(1000);
  }
  
  /////////break here
    //

*/
    
    /*
    digitalWrite(rLedPin,HIGH);
    digitalWrite(yLedPin,HIGH);
    digitalWrite(gLedPin,HIGH);
    delay(500);
    digitalWrite(rLedPin,LOW);
    digitalWrite(yLedPin,LOW);
    digitalWrite(gLedPin,LOW);
    delay(500);
    digitalWrite(rLedPin,HIGH);
    digitalWrite(yLedPin,HIGH);
    digitalWrite(gLedPin,HIGH);
    delay(500);
    digitalWrite(rLedPin,LOW);
    digitalWrite(yLedPin,LOW);
    digitalWrite(gLedPin,LOW);
    delay(500);
    digitalWrite(rLedPin,HIGH);
    digitalWrite(yLedPin,HIGH);
    digitalWrite(gLedPin,HIGH);
    delay(500);
    digitalWrite(rLedPin,LOW);
    digitalWrite(yLedPin,LOW);
    digitalWrite(gLedPin,LOW);
    delay(500);

    analogWrite(fetPin,255);
    delay(500);
    analogWrite(fetPin,120);
    delay(500);
    digitalWrite(fetPin,0);
    delay(500);
    */
//    int16_t adc0;
   // adc0 = ads1115.readADC_SingleEnded();
    //double testVoltageRead = analogRead(A5);
   // double testVoltage = acd0*1.1/1023*1000;
   // double testTempcurrent = 30 + (testVoltage-943)*(1/(-5.194));
  
    /* testTemp1 = testTemp2;
     testTemp2 = testTemp3;
     testTemp3 = testTemp4;
     testTemp4 = testTemp5;
     testTemp5 = testTempcurrent;
     double avgTemp = (testTemp1+testTemp2+testTemp3+testTemp4+testTemp5)/5;
    
    //Serial.print("test");
    Serial.print(avgTemp);
    Serial.print("\t");
    Serial.print(testTemp1);
    Serial.print("\t");
    Serial.print(testTemp2);
    Serial.print("\t");
    Serial.print(testTemp3);
    Serial.print("\t");
        Serial.print(testTemp4);
    Serial.print("\t");
    Serial.println(testTemp5);
    //Serial.print("\t");
    delay(1000);
    
   //double voltageIn = analogRead(A3);
    //Serial.println(voltageIn);
*/
  }
