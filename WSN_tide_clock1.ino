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
#include "LowPower.h"

const boolean verbose = false;

// Real Time Clock setup
RTC_DS3231 RTC; // Uncomment when using this chip

Servo timeServo, heightServo;  // create servo object to control a servo

// Tide calculation library setup.
// Change the library name here to predict for a different site.
//#include "TidelibSanDiegoSanDiegoBay.h"
#include "TidelibBatemansBayAustralia.h"
// Other sites available at http://github.com/millerlp/Tide_calculator
TideCalc myTideCalc; // Create TideCalc object

// 0X3C+SA0 - 0x3C or 0x3D for oled screen on I2C bus
#define I2C_ADDRESS 0x3C
SSD1306AsciiWire oled; // create oled display object

const unsigned int wakeUpPin = 2;
const unsigned int ledPin = 13;
const unsigned int heightServoPin = 9;
const unsigned int timeServoPin = 11;

unsigned long oldUnixtime; // keep track of update time
const unsigned long screenUpdateMs = 1000; // how often to update the screen (milliseconds)
const unsigned long powerDownMs = 500; // how long the CPU is powered down (milliseconds)
DateTime now; // define variable to hold date and time
float height; // tide height
unsigned long future; // future tide time
long secondsUntilNext; // seconds until next tide
boolean goingHighTide; // yes or no
const float maxTideHeight = 2.0; // metres
const unsigned long halfClockInSeconds = 6 * 3600 + 12 * 60 + 30; // how long is a tide cycle on the clock face
const unsigned int servoCentre = 90;
const unsigned int timeMinServoReach = 0;
const unsigned int timeMaxServoReach = 190; // some servos can't fully go to 180
const unsigned int heightMinServoReach = 0;
const unsigned int heightMaxServoReach = 173; // some servos can't fully go to 180
const unsigned int searchIncrement = halfClockInSeconds / timeMaxServoReach; // time accuracy (seconds) per degree of servo movement
const unsigned long invalidTime = 2000000000l; // fake time when value isn't valid
const unsigned int screenOnTimeMs = 10 * screenUpdateMs;
unsigned int screenOnCountdownMs = screenOnTimeMs;

// Enter the site name for display. 11 characters max
char siteName[20] = "Batemans Bay";

//------------------------------------------------------------------------------
void setup() {
  Wire.begin();
  RTC.begin();
  //RTC.adjust(DateTime("Jul 31 2018", "20:21:00"));  // 3 hr before high tide
  //RTC.adjust(DateTime("Jul 31 2018", "22:21:00"));  // 1 hr before high tide
  //RTC.adjust(DateTime("Jul 31 2018", "23:13:00"));  // 15 min before high tide
  //RTC.adjust(DateTime("Jul 31 2018", "23:25:00"));  // 1 min before high tide
  //RTC.adjust(DateTime("Jul 31 2018", "23:30:00"));  // 1 min after high tide
  //RTC.adjust(DateTime("Jul 31 2018", "14:14:00"));  // 3 hr before low tide
  //RTC.adjust(DateTime("Jul 31 2018", "16:15:00"));  // 1 hr before low tide
  //RTC.adjust(DateTime("Jul 31 2018", "17:10:00"));  // 5 min before low tide
  //RTC.adjust(DateTime("Jul 31 2018", "17:14:00"));  // 1 min before low tide
  //RTC.adjust(DateTime("Jul 31 2018", "17:17:00"));  // 1 min after low tide
  //RTC.adjust(DateTime(__DATE__, __TIME__));         // Time and date is expanded to date and time on your computer at compiletime
  //RTC.adjust(DateTime("Aug 06 2018", "20:30:00"));  // Current time

  // For debugging output to serial monitor
  Serial.begin(115200); // Set baud rate to 115200 in serial monitor
  Serial.print("COMPILED AT: date=");
  Serial.print(__DATE__);
  Serial.print(" time=");
  Serial.println(__TIME__);
  
  timeServo.attach(timeServoPin); // attaches the servo on pin 11 (Timer 2 which is usable in low-power state)
  heightServo.attach(heightServoPin);
  pinMode(ledPin, OUTPUT);
  pinMode(wakeUpPin, INPUT);

  // Allow wake up pin to trigger interrupt on high.
  attachInterrupt(0, wakeUp, HIGH);

  // Start up the oled display
  oled.begin(&Adafruit128x64, I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();

  oldUnixtime = 0;
}
//------------------------------------------------------------------------------
void loop() {
  digitalWrite(ledPin, HIGH);
  
  unsigned long startTime = millis();

  // Get current time, store in object "now"
  DateTime now = RTC.now();

  // The main statement block will run once per second
  if (verbose) {
    Serial.print("tideHeight: oldUnixtime=");
    Serial.print(oldUnixtime);
    Serial.print(" searchIncrement=");
    Serial.print(searchIncrement);
    Serial.print(" now.unixtime()=");
    Serial.println(now.unixtime());
  }
  if ( oldUnixtime == 0 || oldUnixtime + searchIncrement < now.unixtime() ) {
    oldUnixtime = now.unixtime(); // update oldUnixtime
  
    // Calculate current tide height
    height = myTideCalc.currentTide(now);
    Serial.println();
    Serial.print("height=");
    Serial.println(height, 3);
  }
  int heightPosition = heightMaxServoReach - (height / maxTideHeight * (servoCentre * 2));
  if (heightPosition < heightMinServoReach) heightPosition = heightMinServoReach;
  if (heightPosition > heightMaxServoReach) heightPosition = heightMaxServoReach;
  heightServo.write(heightPosition);
  Serial.print("heightPosition=");
  Serial.println(heightPosition);
  
  if (secondsUntilNext <= 0) {
    wakeUp();
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

  float percentage = (1.0 * secondsUntilNext / halfClockInSeconds);
  float position; // = percentage * servoCentre;
  if (goingHighTide) {
    position = timeMinServoReach + (percentage * servoCentre); // 0 to 90 degrees
  } else {
    position = servoCentre + (percentage * timeMaxServoReach / 2.0); // 90 to 180 degrees
  }
  position = max(timeMinServoReach, min(timeMaxServoReach, position));
  position = servoCentre + position/8;
  timeServo.write(position);

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
  oled.println(": "); // Extra space because "low" is a character less than "high"
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

  int delayTime = screenUpdateMs - powerDownMs - (millis() - startTime);
  if (delayTime < 0) delayTime = screenUpdateMs / 2;
  if (verbose) {
    Serial.print("delay: delayTime=");
    Serial.println(delayTime);
  }

  if (screenOnCountdownMs > 0) {
    screenOnCountdownMs -= screenUpdateMs;

    // Turn on the OLED screen
    oled.ssd1306WriteCmd(SSD1306_DISPLAYON);

    delay(delayTime);
    digitalWrite(ledPin, LOW);

    // Enter idle state for 500ms with the rest of peripherals turned off, except the servo.
    LowPower.powerSave(SLEEP_500MS, ADC_OFF, BOD_OFF, TIMER2_ON);
    //delay(500);
  } else {
    delay(delayTime + powerDownMs);
    digitalWrite(ledPin, LOW);

    // Turn off the OLED screen
    oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);

    // Allow wake up pin to trigger interrupt on low.
    //attachInterrupt(0, wakeUp, HIGH);

    // Power down for 60s with ADC module and BOD module off. This disables the servo movement.
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    LowPower.powerDown(SLEEP_4S, ADC_OFF, BOD_OFF);

    // Disable external pin interrupt on wake up pin.
    //detachInterrupt(0);

    // Create an empty line
    if (verbose) Serial.println();
  }
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

// Just a handler for the pin interrupt.
void wakeUp()
{
    screenOnCountdownMs = screenOnTimeMs;
}
