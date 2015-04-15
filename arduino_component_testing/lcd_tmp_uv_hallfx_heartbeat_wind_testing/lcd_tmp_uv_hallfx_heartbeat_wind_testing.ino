#include <OneWire.h> 
#include <LiquidCrystal.h>
#include <Interrupt>

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


#define analogPinForRV    4   // change to pins you the analog pins are using
#define analogPinForTMP   3

// to calibrate your sensor, put a glass over it, but the sensor should not be
// touching the desktop surface however.
// adjust the zeroWindAdjustment until your sensor reads about zero with the glass over it. 

const float zeroWindAdjustment =  .2; // negative numbers yield smaller wind speeds and vice versa.

int TMP_Therm_ADunits;  //temp termistor value from wind sensor
float RV_Wind_ADunits;    //RV output from wind sensor 
float RV_Wind_Volts;
unsigned long lastMillis;
int TempCtimes100;
float zeroWind_ADunits;
float zeroWind_volts;
float WindSpeed_MPH;

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
  
  //   This is code from the Modern Device temp sensor (not required)
  pinMode(A4, INPUT);        // GND pin      
  pinMode(A3, INPUT);        // VCC pin
  digitalWrite(A3, LOW);     // turn off pullups
  
  pinMode(5, OUTPUT); // LED PIN
  pinMode(6, OUTPUT); // LED PIN
  pinMode(7, OUTPUT); // LED PIN
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

  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
  delay(1000);
  digitalWrite(5, HIGH);
  digitalWrite(6, HIGH);
  digitalWrite(7, HIGH);
  delay(1000);
  
  //Serial.print("MP8511 output: ");
  //Serial.print(uvLevel);

  //Serial.print(" MP8511 voltage: ");
  //Serial.print(outputVoltage);

  //Serial.print(" UV Intensity (mW/cm^2): ");
  //Serial.print(uvIntensity);
  

  sendDataToProcessing('S', Signal);     // send Processing the raw Pulse Sensor data
  if (QS == true){                       // Quantified Self flag is true when arduino finds a heartbeat
        fadeRate = 255;                  // Set 'fadeRate' Variable to 255 to fade LED with pulse
        sendDataToProcessing('B',BPM);   // send heart rate with a 'B' prefix
        sendDataToProcessing('Q',IBI);   // send time between beats with a 'Q' prefix
        QS = false;                      // reset the Quantified Self flag for next time    
     }
  
  ledFadeToBeat();
  
  windSensor();
  
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("TMP:");
  lcd.setCursor(4,0);
  lcd.print(temperature);
  lcd.setCursor(13,0);
  lcd.print(abs(uvIntensity)*100.0);
  
}

void windSensor(){
  if (millis() - lastMillis > 200){      // read every 200 ms - printing slows this down further
    
    TMP_Therm_ADunits = analogRead(analogPinForTMP);
    RV_Wind_ADunits = analogRead(analogPinForRV);
    RV_Wind_Volts = (RV_Wind_ADunits *  0.0048828125);

    // these are all derived from regressions from raw data as such they depend on a lot of experimental factors
    // such as accuracy of temp sensors, and voltage at the actual wind sensor, (wire losses) which were unaccouted for.
    TempCtimes100 = (0.005 *((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits)) - (16.862 * (float)TMP_Therm_ADunits) + 9075.4;  

    zeroWind_ADunits = -0.0006*((float)TMP_Therm_ADunits * (float)TMP_Therm_ADunits) + 1.0727 * (float)TMP_Therm_ADunits + 47.172;  //  13.0C  553  482.39

    zeroWind_volts = (zeroWind_ADunits * 0.0048828125) - zeroWindAdjustment;  

    // This from a regression from data in the form of 
    // Vraw = V0 + b * WindSpeed ^ c
    // V0 is zero wind at a particular temperature
    // The constants b and c were determined by some Excel wrangling with the solver.
    
   WindSpeed_MPH =  pow(((RV_Wind_Volts - zeroWind_volts) /.2300) , 2.7265);   
   
    Serial.print("  TMP volts ");
    Serial.print(TMP_Therm_ADunits * 0.0048828125);
    

    Serial.print(" RV volts ");
    Serial.print((float)RV_Wind_Volts);

    Serial.print("\t  TempC*100 ");
    Serial.print(TempCtimes100 );

    Serial.print("   ZeroWind volts ");
    Serial.print(zeroWind_volts);

    lcd.setCursor(10, 1);
    lcd.print((float)WindSpeed_MPH);
    
    Serial.print("   WindSpeed MPH ");
    Serial.println((float)WindSpeed_MPH);
    lastMillis = millis();    
  }  
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
