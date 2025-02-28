/*
  LiquidCrystal Library - Hello World
 
 Demonstrates the use a 16x2 LCD display.  The LiquidCrystal
 library works with all LCD displays that are compatible with the 
 Hitachi HD44780 driver. There are many of them out there, and you
 can usually tell them by the 16-pin interface.
 
 This sketch prints "Hello World!" to the LCD
 and shows the time.
 
  The circuit:
 * LCD RS pin to digital pin 12
 * LCD Enable pin to digital pin 11
 * LCD D4 pin to digital pin 5
 * LCD D5 pin to digital pin 4
 * LCD D6 pin to digital pin 3
 * LCD D7 pin to digital pin 2
 * LCD R/W pin to ground
 * 10K resistor:
 * ends to +5V and ground
 * wiper to LCD VO pin (pin 3)
 
 Library originally added 18 Apr 2008
 by David A. Mellis
 library modified 5 Jul 2009
 by Limor Fried (http://www.ladyada.net)
 example added 9 Jul 2009
 by Tom Igoe
 modified 22 Nov 2010
 by Tom Igoe
 
 This example code is in the public domain.

 http://www.arduino.cc/en/Tutorial/LiquidCrystal
 */

// include the library code:
#include <LiquidCrystal.h>

float timer = 6000.0;

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(13, 12, 8, 9, 10, 11);

void setup() {
  // set up the LCD's number of columns and rows: 
  lcd.begin(16, 2);
  // Set cursor to 0, 0
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("SKYNET ACTIVE");
  lcd.setCursor(0, 1);
  lcd.print("DETONATE:"); 
  lcd.setCursor(10, 1);
  lcd.print(timer - millis()/100);
}

void loop() {
  lcd.setCursor(0, 0);
  // Print a message to the LCD.
  lcd.print("SKYNET ACTIVE");
  lcd.setCursor(0, 1);
  lcd.print("DETONATE:");
  lcd.setCursor(10, 1);
  lcd.print(timer - millis()/100);
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
