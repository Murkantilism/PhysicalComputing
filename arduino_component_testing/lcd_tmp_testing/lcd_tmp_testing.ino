#include <OneWire.h> 
#include <LiquidCrystal.h>

int DS18S20_Pin = 4; //DS18S20 Signal pin on digital 2
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 8, 9, 10, 11);

//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2

void setup(void) {
  Serial.begin(9600);
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Set cursor to 0, 0
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("TMP:");
  lcd.setCursor(10, 0);
  lcd.print("UV:105");
  lcd.setCursor(0, 1);
  lcd.print("89BPM");
  lcd.setCursor(11, 1);
  lcd.print("****");
}

void loop(void) {
  float temperature = getTemp();
  Serial.println(temperature);
  
  delay(100);
  lcd.setCursor(0, 0);
  lcd.print("TMP:");
  lcd.setCursor(4,0);
  lcd.print(temperature);
  
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
