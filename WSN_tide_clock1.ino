/*  Simple test for ebay 128x64 oled run via I2C on a
    Arduino Pro Mini 3.3V (328P), with ebay DS3231 real time
    clock module.

    The sketch will calculate the current tide height for
    the site (assuming clock is set correctly) and display
    the tide height and time on a ssd1306-controller OLED
    128x64 display. Time updates every second, tide updates
    as the last significant digit changes (10-20 seconds).

*/

#include <Wire.h>
#include "SSD1306Ascii.h" // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h" // https://github.com/greiman/SSD1306Ascii
#include <SPI.h> // Required for RTClib to compile properly
#include <RTClib.h> // From https://github.com/millerlp/RTClib
#include <Servo.h>

// Real Time Clock setup
RTC_DS3231 RTC; // Uncomment when using this chip

Servo myservo;  // create servo object to control a servo

// Tide calculation library setup.
// Change the library name here to predict for a different site.
//#include "TidelibSanDiegoSanDiegoBay.h"
#include "TidelibBatemansBayAustralia.h"
// Other sites available at http://github.com/millerlp/Tide_calculator
TideCalc myTideCalc; // Create TideCalc object

// 0X3C+SA0 - 0x3C or 0x3D for oled screen on I2C bus
#define I2C_ADDRESS 0x3C

SSD1306AsciiWire oled; // create oled dispaly object

long oldmillis; // keep track of time

float height; // tide height
unsigned long future; // future tide time
long secondsUntilNext; // seconds until next tide
boolean goingHighTide;
unsigned long halfClockInSeconds = 6 * 3600 + 12 * 60 + 30;
unsigned int servoCentre = 90;
unsigned int maxServoReach = 173;

DateTime now; // define variable to hold date and time
// Enter the site name for display. 11 characters max
char siteName[20] = "Batemans Bay";
int currMinute; // Keep track of current minute value in main loop

//------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  RTC.begin();
  //RTC.adjust(DateTime("2018-07-28", "21:46:00"));  // Time and date is expanded to date and time on your computer at compiletime

  // For debugging output to serial monitor
  Serial.begin(115200); // Set baud rate to 115200 in serial monitor

  myservo.attach(9);  // attaches the servo on pin 9 to the servo objects
  pinMode(8, OUTPUT);          // sets the digital pin as output
  pinMode(7, OUTPUT);          // sets the digital pin as output

  // Start up the oled display
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.set400kHz();
  oled.setFont(Adafruit5x7);
  oled.clear();
  oldmillis = millis();
}
//------------------------------------------------------------------------------
void loop() {

  // Get current time, store in object "now"
  DateTime now = RTC.now();
  // If it is the start of a new minute, calculate new tide height
  if (now.minute() != currMinute) {
    // If now.minute doesn't equal currMinute, a new minute has turned
    // over, so it's time to update the tide height. We only want to do
    // this once per minute.
    currMinute = now.minute(); // update currMinute
    Serial.println();
    printTime(now);

    // Calculate new tide height based on current time
    height = myTideCalc.currentTide(now);

    //*****************************************
    // For debugging
    Serial.print("Tide height: ");
    Serial.print(height, 3);
    Serial.println("m");

    // Calculate new tide height based on current time
    height = myTideCalc.currentTide(now);

    Serial.print("Time to next: ");
    Serial.print(secondsUntilNext / 3600.0);
    Serial.println(" mins");

    Serial.println(); // blank line
  }  // End of if (now.minute() != currMinute) statement

  // The main statement block will run once per second
  if ( oldmillis + 1000 < millis() ) {
    oldmillis = millis(); // update oldmillis
    //now = RTC.now(); // update time

    // Calculate current tide height
    height = myTideCalc.currentTide(now);

    if (secondsUntilNext <= 0) {
      future = now.unixtime() + 30;
      float lastHeight = height;
      float nextTideHeight = myTideCalc.currentTide(future);
      if (lastHeight < nextTideHeight) {
        goingHighTide = true;
      } else {
        goingHighTide = false;
      }
      
      while ((goingHighTide && nextTideHeight >= lastHeight) || (!goingHighTide && nextTideHeight <= lastHeight)) {
        future += 60;
        lastHeight = nextTideHeight;
        nextTideHeight = myTideCalc.currentTide(future);
    
//        Serial.print("Time: ");
//        Serial.print(future);
//        Serial.print(" | ");
//        Serial.print(nextTideHeight);
//        Serial.print(" <= ");
//        Serial.print(lastHeight);
//        Serial.println();
      }

      oled.clear();
    }
    secondsUntilNext = future - now.unixtime();

    char buf[20]; // declare a string buffer to hold the time result
    // Create a string representation of the date and time,
    // which will be put into 'buf'.
    now.toString(buf, 20);
    // Now extract the time by making another character pointer that
    // is advanced 11 places into buf to skip over the date.
    char *timeStr = buf + 11;
    oled.home();
    oled.set2X();  // Enable large font
    oled.print(" ");
    //oled.println(siteName); // Print site name, move to next line
    oled.println(timeStr); // print hour:min:sec
    oled.set1X(); // Enable normal font
    oled.print(siteName); // Print site name, move to next line
    oled.println(" tide:");
    oled.print("  ");
    oled.print(height, 3); // print tide ht. to 3 decimal places
    oled.println("m");
    oled.println();
    oled.print("Time until ");
    if (goingHighTide) {
      oled.print("high");
    } else {
      oled.print("low");
    }
    oled.println(":");
    oled.set2X(); // Enable large font
    //oled.println(subbuf); // print hour:min:sec
    //secondsUntilNext = 15 * 3600l + 54 * 60l + 36;
    short hour = secondsUntilNext / 3600;
    short minute = (secondsUntilNext % 3600) / 60;
    short second = ((secondsUntilNext % 3600) % 60);
    oled.print(" ");
    if (hour < 10) oled.print("0");
    oled.print(hour); // hours until next tide
    oled.print(":");
    if (minute < 10) oled.print("0");
    oled.print(minute); // mins until next tide
    oled.print(":");
    if (second < 10) oled.print("0");
    oled.print(second); // secs until next tide

//    DateTime tideCountdown (secondsUntilNext);
//    tideCountdown.toString(buf, 20);
//    char *countDownStr = buf + 11;
//    oled.println(countDownStr); // print hour:min:sec

    digitalWrite(7, LOW);
    digitalWrite(8, HIGH);

    float percentage = (1.0 * secondsUntilNext / halfClockInSeconds);
    int position = percentage * servoCentre;
    if (goingHighTide) {
      position = 0 + position;
    } else {
      position = servoCentre + position;
    }
    position = max(0, min(maxServoReach, position));
    myservo.write(position);
  }

  delay(499);
}


//*******************************************
// Function for printing the current date/time to the
// serial port in a nicely formatted layout.
void printTime(DateTime now) {
  Serial.print(now.year(), DEC);
  Serial.print("/");
  Serial.print(now.month(), DEC);
  Serial.print("/");
  Serial.print(now.day(), DEC);
  Serial.print("  ");
  Serial.print(now.hour(), DEC);
  Serial.print(":");
  if (now.minute() < 10) {
    Serial.print("0");
    Serial.print(now.minute());
  }
  else if (now.minute() >= 10) {
    Serial.print(now.minute());
  }
  Serial.print(":");
  if (now.second() < 10) {
    Serial.print("0");
    Serial.println(now.second());
  }
  else if (now.second() >= 10) {
    Serial.println(now.second());
  }
} // End of printTime function
//*************************************
