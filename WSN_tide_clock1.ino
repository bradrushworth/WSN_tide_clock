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

SSD1306AsciiWire oled; // create oled display object

unsigned long oldmillis; // keep track of update time
unsigned long screenUpdateMs = 1000; // how often to update the screen (milliseconds)
DateTime now; // define variable to hold date and time
float height; // tide height
unsigned long future; // future tide time
long secondsUntilNext; // seconds until next tide
boolean goingHighTide; // yes or no
unsigned long halfClockInSeconds = 6 * 3600 + 12 * 60 + 30; // how long is a tide cycle on the clock face
unsigned int servoCentre = 90;
unsigned int maxServoReach = 173; // some servos can't fully go to 180
unsigned int searchIncrement = halfClockInSeconds / 180; // time accuracy per degree of servo movement
unsigned long invalidTime = 2000000000l; // fake time when value isn't valid

// Enter the site name for display. 11 characters max
char siteName[20] = "Batemans Bay";

//------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  RTC.begin();
  //RTC.adjust(DateTime("Jul 31 2018", "18:35:00"));  // 1 hr before high tide
  //RTC.adjust(DateTime("Jul 31 2018", "19:20:00"));  // 15 min before high tide
  //RTC.adjust(DateTime("Jul 31 2018", "19:34:30"));  // 1 min before high tide
  //RTC.adjust(DateTime("Jul 31 2018", "19:37:00"));  // 1 min after high tide
  //RTC.adjust(DateTime("Jul 31 2018", "12:25:00"));  // 1 hr before low tide
  //RTC.adjust(DateTime("Jul 31 2018", "13:22:00"));  // 5 min before low tide
  //RTC.adjust(DateTime("Jul 31 2018", "13:25:00"));  // 1 min before low tide
  //RTC.adjust(DateTime("Jul 31 2018", "13:27:00"));  // 1 min after low tide
  //RTC.adjust(DateTime(__DATE__, __TIME__));         // Time and date is expanded to date and time on your computer at compiletime
  //RTC.adjust(DateTime("Jul 31 2018", "19:20:00"));  // Current time

  // For debugging output to serial monitor
  Serial.begin(115200); // Set baud rate to 115200 in serial monitor
  Serial.print("COMPILED AT: date=");
  Serial.print(__DATE__);
  Serial.print(" time=");
  Serial.println(__TIME__);
  
  myservo.attach(9);  // attaches the servo on pin 9 to the servo objects
  pinMode(8, OUTPUT);          // sets the digital pin as output
  pinMode(7, OUTPUT);          // sets the digital pin as output

  // Start up the oled display
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();

  oldmillis = 0;
}
//------------------------------------------------------------------------------
void loop() {
  unsigned int startTime = millis();
  if (startTime < oldmillis) { // Check for long overflow after 58 days
    oldmillis = 0;
  }

  // Get current time, store in object "now"
  DateTime now = RTC.now();

  // The main statement block will run once per second
//  Serial.print("tideHeight: oldmillis=");
//  Serial.print(oldmillis);
//  Serial.print(" (searchIncrement * 1000)=");
//  Serial.print((searchIncrement * 1000));
//  Serial.print(" startTime=");
//  Serial.println(startTime);
  if ( oldmillis == 0 || oldmillis + (searchIncrement * 1000) < startTime ) {
    oldmillis = startTime; // update oldmillis
  
    // Calculate current tide height
    height = myTideCalc.currentTide(now);
    Serial.println();
    Serial.print("height=");
    Serial.println(height, 3);
  }
  
  if (secondsUntilNext <= 0) {
    Serial.println();
    unsigned long highTide = localMax(now.unixtime(), now.unixtime() + halfClockInSeconds * 1.5);
    if (highTide == now.unixtime())
      highTide = invalidTime;
    Serial.println();
    unsigned long lowTide  = localMin(now.unixtime(), now.unixtime() + halfClockInSeconds * 1.5);
    if (lowTide == now.unixtime())
      lowTide = invalidTime;
    Serial.println();
    Serial.print("highTide=");
    Serial.print(highTide);
    Serial.print(" (");
    Serial.print(highTide-now.unixtime());
    Serial.print(") lowTide=");
    Serial.print(lowTide);
    Serial.print(" (");
    Serial.print(lowTide-now.unixtime());
    Serial.print(")");
    if (highTide < lowTide) {
      goingHighTide = true;
      future = highTide;
      Serial.print(" goingHighTide=true");
    } else {
      goingHighTide = false;
      future = lowTide;
      Serial.print(" goingHighTide=false");
    }
    Serial.println();
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

  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);

  float percentage = (1.0 * secondsUntilNext / halfClockInSeconds);
  int position = percentage * servoCentre;
  if (goingHighTide) {
    position = 0 + position; // 0 to 90 degrees
  } else {
    position = servoCentre + position; // 90 to 180 degrees
  }
  position = max(0, min(maxServoReach, position));
  myservo.write(position);

  unsigned int delayTime = screenUpdateMs - (millis() - startTime);
  //Serial.print("delay: delayTime=");
  //Serial.println(delayTime);
  delay(delayTime);
}

// A binary search based function that returns
// index of a local minima.
unsigned long localMinUtil(unsigned long low, unsigned long high, unsigned long n)
{
    // Find index of middle element
    unsigned long mid = low + (high - low)/2;  /* (low + high)/2 */
    Serial.print("localMinUtil: low=");
    Serial.print(low);
    Serial.print(" mid=");
    Serial.print(mid);
    Serial.print(" high=");
    Serial.print(high);
    Serial.print(" currentTide=");
    Serial.print(myTideCalc.currentTide(mid), 5);
    Serial.println();

    // If the mid is the same as the low and high, quit
    if (high - low < searchIncrement)
        return low; //invalidTime;
    
    // Compare middle element with its neighbours
    // (if neighbours exist)
    if ((myTideCalc.currentTide(mid - searchIncrement) > myTideCalc.currentTide(mid)) &&
            (myTideCalc.currentTide(mid + searchIncrement) > myTideCalc.currentTide(mid)))
        return mid;
 
    // If middle element is not minima and its left
    // neighbour is smaller than it, then left half
    // must have a local minima.
    else if (myTideCalc.currentTide(mid - searchIncrement) < myTideCalc.currentTide(mid))
        return localMinUtil(low, (mid - searchIncrement), n);
 
    // If middle element is not minima and its right
    // neighbour is smaller than it, then right half
    // must have a local minima.
    return localMinUtil((mid + searchIncrement), high, n);
}
 
// A wrapper over recursive function localMinUtil()
unsigned long localMin(unsigned long beginning, unsigned long end)
{
    return localMinUtil(beginning, end, end);
}

// A binary search based function that returns
// index of a local maxima.
unsigned long localMaxUtil(unsigned long low, unsigned long high, unsigned long n)
{
    // Find index of middle element
    unsigned long mid = low + (high - low)/2;  /* (low + high)/2 */
    Serial.print("localMaxUtil: low=");
    Serial.print(low);
    Serial.print(" mid=");
    Serial.print(mid);
    Serial.print(" high=");
    Serial.print(high);
    Serial.print(" currentTide=");
    Serial.print(myTideCalc.currentTide(mid), 5);
    Serial.println();
    
    // If the mid is the same as the low and high, quit
    if (high - low < searchIncrement)
        return low; //invalidTime;
        
    // Compare middle element with its neighbours
    // (if neighbours exist)
    if ((myTideCalc.currentTide(mid - searchIncrement) < myTideCalc.currentTide(mid)) &&
            (myTideCalc.currentTide(mid + searchIncrement) < myTideCalc.currentTide(mid)))
        return mid;
 
    // If middle element is not minima and its left
    // neighbour is smaller than it, then left half
    // must have a local minima.
    else if (myTideCalc.currentTide(mid - searchIncrement) > myTideCalc.currentTide(mid))
        return localMaxUtil(low, (mid - searchIncrement), n);
 
    // If middle element is not minima and its right
    // neighbour is smaller than it, then right half
    // must have a local minima.
    return localMaxUtil((mid + searchIncrement), high, n);
}
 
// A wrapper over recursive function localMaxUtil()
unsigned long localMax(unsigned long beginning, unsigned long end)
{
    return localMaxUtil(beginning, end, end);
}
