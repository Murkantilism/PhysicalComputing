#include <OneWire.h> 
#include <LiquidCrystal.h>

int DS18S20_Pin = 4; //DS18S20 Signal pin on digital 2
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 8, 9, 10, 11);

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2

// Set UV hardware pins
int UVOUT = A0; //out from uv sensor
int REF_3V3 = A1; // 3.3v power on Arduino

unsigned int last_printed_count;
unsigned int count;
unsigned int last_sensor_reading;

//  VARIABLES
int pulsePin = 2;                 // Pulse Sensor purple wire connected to analog pin 2
int blinkPin = 13;                // pin to blink led at each beat
int fadePin = 5;                  // pin to do fancy classy fading blink at each beat
int fadeRate = 0;                 // used to fade LED on with PWM on fadePin


// these variables are volatile because they are used during the interrupt service routine!
volatile int BPM;                   // used to hold the pulse rate
volatile int Signal;                // holds the incoming raw data
volatile int IBI = 600;             // holds the time between beats, the Inter-Beat Interval
volatile boolean Pulse = false;     // true when pulse wave is high, false when it's low
volatile boolean QS = false;        // becomes true when Arduoino finds a beat.

void setup(void) {
  digitalWrite(2, HIGH);
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Set cursor to 0, 0
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("TMP:");
  lcd.setCursor(10, 0);
  lcd.print("UV:");
  lcd.setCursor(0, 1);
  lcd.print("89BPM");
  lcd.setCursor(11, 1);
  lcd.print("****");
  pinMode(UVOUT, INPUT);
  pinMode(REF_3V3, INPUT);
  last_printed_count = 0;
  count = 0;
  pinMode(2, OUTPUT);
  
  
  pinMode(blinkPin,OUTPUT);         // pin that will blink to your heartbeat!
  pinMode(fadePin,OUTPUT);          // pin that will fade to your heartbeat!
  Serial.begin(115200);             // we agree to talk fast!
  interruptSetup();                 // sets up to read Pulse Sensor signal every 2mS 
   // UN-COMMENT THE NEXT LINE IF YOU ARE POWERING The Pulse Sensor AT LOW VOLTAGE, 
   // AND APPLY THAT VOLTAGE TO THE A-REF PIN
   //analogReference(EXTERNAL);
  
}

void loop(void) {
  float temperature = getTemp();
  //Serial.println(temperature);
  int uvLevel = averageAnalogRead(UVOUT);
  int refLevel = averageAnalogRead(REF_3V3);
  
  // Use the 3.3v power pin as a reference to get accurate output from sensor
  float outputVoltage = 3.3 / refLevel * uvLevel;
  
  float uvIntensity = mapfloat(outputVoltage, 0.99, 2.9, 0.0, 15.0);
  
  
  int hall_effect_sensor_reading;
  hall_effect_sensor_reading = digitalRead(2);
  
  Serial.println(hall_effect_sensor_reading);
  
  //Serial.print("MP8511 output: ");
  //Serial.print(uvLevel);

  //Serial.print(" MP8511 voltage: ");
  //Serial.print(outputVoltage);

  //Serial.print(" UV Intensity (mW/cm^2): ");
  //Serial.print(uvIntensity);
  
  //Serial.println();
  

  sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
  if (QS == true){                       // Quantified Self flag is true when arduino finds a heartbeat
        fadeRate = 255;                  // Set 'fadeRate' Variable to 255 to fade LED with pulse
        sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
        sendDataToProcessing('Q',IBI);   // send time between beats with a 'Q' prefix
        QS = false;                      // reset the Quantified Self flag for next time    
     }
  
  ledFadeToBeat();
  
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("TMP:");
  lcd.setCursor(4,0);
  lcd.print(temperature);
  lcd.setCursor(13,0);
  lcd.print(abs(uvIntensity)*100.0);
  
}


float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
      //no more sensors on chain, reset search
      ds.reset_search();
      return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.println("CRC is not valid!");
      return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
      Serial.print("Device is not recognized");
      return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad

  
  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }
  
  ds.reset_search();
  
  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;
  
  return TemperatureSum;
  
}

//Takes an average of readings on a given pin
//Returns the average
int averageAnalogRead(int pinToRead)
{
  byte numberOfReadings = 8;
  unsigned int runningValue = 0; 

  for(int x = 0 ; x < numberOfReadings ; x++)
    runningValue += analogRead(pinToRead);
  runningValue /= numberOfReadings;

  return(runningValue);  
}

//The Arduino Map function but for floats
//From: http://forum.arduino.cc/index.php?topic=3922.0
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void ledFadeToBeat(){
    fadeRate -= 15;                         //  set LED fade value
    fadeRate = constrain(fadeRate,0,255);   //  keep LED fade value from going into negative numbers!
    analogWrite(fadePin,fadeRate);          //  fade LED
}


void sendDataToProcessing(char symbol, int data ){
    Serial.print(symbol);                // symbol prefix tells Processing what type of data is coming
    Serial.println(data);                // the data to send culminating in a carriage return
}

void baby(){
  lcd.setCursor(0, 0);
  lcd.print("TMP:102F");
  lcd.setCursor(10, 0);
  lcd.print("UV:105");
  lcd.setCursor(0, 1);
  lcd.print("89BPM");
  lcd.setCursor(11, 1);
  lcd.print("****");
}

void parent(){
  lcd.setCursor(0, 0);
  lcd.print(millis()/1000);
  lcd.setCursor(10, 0);
  lcd.print("UV:105");
  lcd.setCursor(0, 1);
  lcd.print("89BPM");
  lcd.setCursor(11, 1);
  lcd.print("****");
}
