/*  The sketch will calculate the current tide height for
    the site (assuming ebay DS3231 real time
    clock module is set correctly) and display
    the tide height and time on a ssd1306-controller OLED
    128x64 display. Time updates every second, tide updates
    as the last significant digit changes (10-20 seconds).
*/

#define ENABLE_OLED true
#define ENABLE_SUN_RISE true
#define ENABLE_MOON_PHASE true
#define ENABLE_MOON_RISE true

// 0X3C+SA0 - 0x3C or 0x3D for oled screen on I2C bus
#define OLED_I2C_ADDRESS 0x3C

const boolean info = false;
const boolean debug = false;

#if ENABLE_OLED
//#include <Wire.h>
#include "SSD1306Ascii.h"      // https://github.com/greiman/SSD1306Ascii
#include "SSD1306AsciiWire.h"  // https://github.com/greiman/SSD1306Ascii
#endif

#include <SPI.h>     // Required for RTClib to compile properly
#include <RTClib.h>  // Originally from https://github.com/millerlp/RTClib, now using https://github.com/NeiroNx/RTCLib

// Real Time Clock setup
DS3231 RTC;  // Uncomment when using this chip

#include <Servo.h>
#include "LowPower.h"

#if ENABLE_SUN_RISE
#include <sundata.h>
#endif

#if ENABLE_MOON_RISE
//#include "Time.h"
//#include "Moon.h"
#include "moon.h"
#endif

Servo timeServo, heightServo, sunElevationServo, moonPhaseServo;  // create servo object to control a servo

// Tide calculation library setup.
// Change the library name here to predict for a different site.
//#include "TidelibSanDiegoSanDiegoBay.h"
#include "TidelibBatemansBayAustralia.h"
// Other sites available at http://github.com/millerlp/Tide_calculator
TideCalc myTideCalc;  // Create TideCalc object

#if ENABLE_OLED
SSD1306AsciiWire oled;  // create oled display object
#endif

const byte wakeUpPin = 2;
const byte ledPin = 13;
const byte timeServoPin = 11;
const byte heightServoPin = 9;
const byte sunElevationServoPin = 5;
const byte moonPhaseServoPin = 6;

// Enter the site name for display. Length of array to match
const char siteName[] = "Batemans Bay";
const int lat = -35;
const int lon = 150;
const float timezone = 10;

unsigned long oldUnixtime;                                        // keep track of update time
DateTime dateTime;                                                // define variable to hold date and time
const unsigned long invalidTime = 2000000000l;                    // fake time when value isn't valid
float height;                                                     // tide height
unsigned long future;                                             // future tide time
long secondsUntilNext;                                            // seconds until next tide
boolean goingHighTide;                                            // yes or no
const float maxTideHeight = 2.0;                                  // metres
const unsigned int halfClockInSeconds = 6 * 3600 + 12 * 60 + 30;  // how long is a tide cycle on the clock face (6 hrs 12 mins 30 secs)
const byte servoCentre = 90;
const byte timeMinServoReach = 0;
const byte timeMaxServoReach = 190;  // this is a multi-rotation servo which isn't quite accurate
const byte heightMinServoReach = 0;
const byte heightMaxServoReach = 173;                                         // some servos can't fully go to 180
const unsigned int searchIncrement = halfClockInSeconds / timeMaxServoReach;  // time accuracy (seconds) per degree of servo movement

const unsigned int powerDownMs = 500 + 250 + 60;  // how long the CPU is powered down (810 milliseconds)
const byte screenUpdateSec = 1;                   // how often to update the screen (seconds)
const byte screenOnTimeSec = 10 * screenUpdateSec;
volatile byte screenOnCountdownSec = screenOnTimeSec;  // volatile because written by interupt wakeup. Bytes are best because 8-bit system.
volatile boolean hasBeenInterupted = false;

#if ENABLE_SUN_RISE
sundata sunTimes = sundata(lat, lon, timezone);
#endif

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

  if (info || debug) {
    // For debugging output to serial monitor
    Serial.begin(115200);  // Set baud rate to 115200 in serial monitor
    Serial.print("COMPILED AT: date=");
    Serial.print(__DATE__);
    Serial.print(" time=");
    Serial.println(__TIME__);
  }

  timeServo.attach(timeServoPin);  // attaches the servo on pin 11 (Timer 2 which is usable in low-power state)
  heightServo.attach(heightServoPin);
  sunElevationServo.attach(sunElevationServoPin);
  moonPhaseServo.attach(moonPhaseServoPin);
  pinMode(ledPin, OUTPUT);
  pinMode(wakeUpPin, INPUT_PULLUP);

#if ENABLE_OLED
  // Start up the oled display
  oled.begin(&Adafruit128x64, OLED_I2C_ADDRESS);
  oled.setFont(Adafruit5x7);
  oled.clear();
#endif

  oldUnixtime = 0;
}
//------------------------------------------------------------------------------
void loop() {
  digitalWrite(ledPin, HIGH);

  unsigned long startTime = millis();

  // Get current time
  dateTime = RTC.now();

  // The main statement block will run once per second
  if (debug) {
    Serial.print("tideHeight: oldUnixtime=");
    Serial.print(oldUnixtime);
    Serial.print(" searchIncrement=");
    Serial.print(searchIncrement);
    Serial.print(" dateTime.unixtime()=");
    Serial.println(dateTime.unixtime());
  }
  if (oldUnixtime == 0 || oldUnixtime + searchIncrement < dateTime.unixtime()) {
    oldUnixtime = dateTime.unixtime();  // update oldUnixtime

    // Calculate current tide height
    height = myTideCalc.currentTide(dateTime);
    if (info) {
      Serial.println();
      Serial.print("Tide height=");
      Serial.println(height, 3);
      Serial.println();
    }

#if ENABLE_SUN_RISE
    sunTimes.time(dateTime.year(), dateTime.month(), dateTime.day(), dateTime.hour(), dateTime.minute(), dateTime.second());  //insert year, month, day, hour, minutes and seconds
    sunTimes.calculations();                                                                                                  //update calculations for last inserted time
    float el_rad = sunTimes.elevation_rad();                                                                                  //store sun's elevation in rads
    float el_deg = sunTimes.elevation_deg();                                                                                  //store sun's elevation in degrees
    float az_rad = sunTimes.azimuth_rad();                                                                                    //store sun's azimuth in rads
    float az_deg = sunTimes.azimuth_deg();                                                                                    //store sun's azimuth in degrees
    float sunrise = sunTimes.sunrise_time();                                                                                  //store sunrise time in decimal form
    float sunset = sunTimes.sunset_time();                                                                                    //store sunset time in decimal form

    // 90 degrees is sunrise/sunset. 180 is high noon. 0 is night.
    int el_servo = servoCentre + el_deg;
    if (el_servo < heightMinServoReach) el_servo = heightMinServoReach;
    if (el_servo > heightMaxServoReach) el_servo = heightMaxServoReach;
    sunElevationServo.write(el_servo);
    if (info) {
      Serial.println("Sun rise/set times:");
      Serial.print("  Elevation (degrees)=");
      Serial.println(el_deg);
      Serial.print("  Elevation (servo degrees)=");
      Serial.println(el_servo);
      Serial.print("  Azimouth (degrees)=");
      Serial.println(az_deg);
      Serial.print("  Time of sunrise (decimal form)=");
      Serial.println(sunrise);
      Serial.print("  Time of sunset (decimal form)=");
      Serial.println(sunset);
      Serial.println();
    }
#endif

#if ENABLE_MOON_PHASE
    float frac = MoonPhase(dateTime.year(), dateTime.month(), dateTime.day());

    // Go from 0 to 180 for New Moon (0) to Full Moon (90) to New Moon (180)
    int servoFrac = (1 - frac) * (servoCentre * 2);
    if (servoFrac < heightMinServoReach) servoFrac = heightMinServoReach;
    if (servoFrac > heightMaxServoReach) servoFrac = heightMaxServoReach;
    moonPhaseServo.write(servoFrac);

    if (info) {
      Serial.print("MoonPhase: Moon Phase=");
      Serial.println(frac);
      Serial.print("MoonPhase: Moon Phase Servo=");
      Serial.println(servoFrac);
      Serial.println();
    }
#endif

#if ENABLE_MOON_RISE
    riseset(lat, lon, false);
    //    setTime(dateTime.unixtime());
    //    moon_init(lat, lon); // pass it lat / lon - it uses ints for the calculation...

    if (info) {
      //    Serial.println("Moon rise/set times:");
      //    Serial.print("now=");
      //    Serial.print(now());
      //    Serial.print(" day=");
      //    Serial.println(day());
      //    Serial.print("rise azimouth in degrees=");
      //    Serial.println(Moon.riseAZ);
      //    Serial.print("set azimouth in degrees=");
      //    Serial.println(Moon.setAZ);
      Serial.print("MoonRise: Rise=");
      Serial.print(Moon.riseH);
      Serial.print(":");
      Serial.println(Moon.riseM);
      Serial.print("MoonRise: Set=");
      Serial.print(Moon.setH);
      Serial.print(":");
      Serial.println(Moon.setM);
      Serial.println();
    }
#endif
  }
  int heightPosition = heightMaxServoReach - (height / maxTideHeight * (servoCentre * 2));
  if (heightPosition < heightMinServoReach) heightPosition = heightMinServoReach;
  if (heightPosition > heightMaxServoReach) heightPosition = heightMaxServoReach;
  heightServo.write(heightPosition);
  //Serial.print("heightPosition=");
  //Serial.println(heightPosition);

  if (secondsUntilNext <= 0) {
#if ENABLE_OLED
    oled.clear();
#endif

    // Turn on the screen and move servos
    wakeUp();

    if (info) Serial.println();
    unsigned long highTide = localMax(dateTime.unixtime(), dateTime.unixtime() + halfClockInSeconds * 1.5);
    if (highTide == dateTime.unixtime())
      highTide = invalidTime;
    if (info) Serial.println();
    unsigned long lowTide = localMin(dateTime.unixtime(), dateTime.unixtime() + halfClockInSeconds * 1.5);
    if (lowTide == dateTime.unixtime())
      lowTide = invalidTime;
    if (info) {
      Serial.println();
      Serial.print("highTide=");
      Serial.print(highTide);
      Serial.print(" (");
      Serial.print(highTide - dateTime.unixtime());
      Serial.print(") lowTide=");
      Serial.print(lowTide);
      Serial.print(" (");
      Serial.print(lowTide - dateTime.unixtime());
      Serial.print(")");
    }
    if (highTide < lowTide) {
      goingHighTide = true;
      future = highTide;
      if (info) Serial.print(" goingHighTide=true");
    } else {
      goingHighTide = false;
      future = lowTide;
      if (info) Serial.print(" goingHighTide=false");
    }
    if (info) Serial.println();
  }
  secondsUntilNext = future - dateTime.unixtime();

  float percentage = (1.0 * secondsUntilNext / halfClockInSeconds);
  float position;  // = percentage * servoCentre;
  if (goingHighTide) {
    position = timeMinServoReach + (percentage * servoCentre);  // 0 to 90 degrees
  } else {
    position = servoCentre + (percentage * timeMaxServoReach / 2.0);  // 90 to 180 degrees
  }
  position = max(timeMinServoReach, min(timeMaxServoReach, position));
  position = servoCentre + position / 8;
  timeServo.write(position);

#if ENABLE_OLED
  char buf[20];  // declare a string buffer to hold the time result
  // Create a string representation of the date and time,
  // which will be put into 'buf'.
  dateTime.tostr(buf);
  // Now extract the time by making another character pointer that
  // is advanced 11 places into buf to skip over the date.
  char *timeStr = buf + 11;
  oled.home();
  oled.set2X();  // Enable large font
  oled.print(" ");
  //oled.println(siteName); // Print site name, move to next line
  oled.println(timeStr);  // print hour:min:sec
  oled.set1X();           // Enable normal font
  oled.print(siteName);   // Print site name, move to next line
  oled.println(" tide:");
  oled.print("  ");
  oled.print(height, 3);  // print tide ht. to 3 decimal places
  oled.println("m");
  oled.println();
  oled.print("Time until ");
  if (goingHighTide) {
    oled.print("high");
  } else {
    oled.print("low");
  }
  oled.println(": ");  // Extra space because "low" is a character less than "high"
  oled.set2X();        // Enable large font
  //oled.println(subbuf); // print hour:min:sec
  //secondsUntilNext = 15 * 3600l + 54 * 60l + 36;
  short hour = secondsUntilNext / 3600;
  short minute = (secondsUntilNext % 3600) / 60;
  short second = ((secondsUntilNext % 3600) % 60);
  oled.print(" ");
  if (hour < 10) oled.print("0");
  oled.print(hour);  // hours until next tide
  oled.print(":");
  if (minute < 10) oled.print("0");
  oled.print(minute);  // mins until next tide
  oled.print(":");
  if (second < 10) oled.print("0");
  oled.print(second);  // secs until next tide
#endif

  int delayTimeMs = screenUpdateSec * 1000 - powerDownMs - (millis() - startTime);
  if (delayTimeMs < 0) delayTimeMs = 0;
  if (debug) {
    Serial.print("delay: delayTimeMs=");
    Serial.println(delayTimeMs, DEC);
    Serial.print("delay: screenOnCountdownSec=");
    Serial.println(screenOnCountdownSec, DEC);
    Serial.print("delay: screenUpdateSec=");
    Serial.println(screenUpdateSec, DEC);
  }
  if (screenOnCountdownSec > 0) {
    // screenUpdateSec is a multiple of screenOnTimeSec, so will reduce neatly to zero
    screenOnCountdownSec -= screenUpdateSec;

#if ENABLE_OLED
    // Turn on the OLED screen
    oled.ssd1306WriteCmd(SSD1306_DISPLAYON);
#endif

    delay(delayTimeMs);
    digitalWrite(ledPin, LOW);
    hasBeenInterupted = false;

    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, RISING);

    // Enter idle state for 810ms (powerDownMs) with the rest of peripherals turned off, except the servo.
    if (!hasBeenInterupted) LowPower.powerSave(SLEEP_60MS, ADC_OFF, BOD_OFF, TIMER2_ON);
    if (!hasBeenInterupted) LowPower.powerSave(SLEEP_250MS, ADC_OFF, BOD_OFF, TIMER2_ON);
    if (!hasBeenInterupted) LowPower.powerSave(SLEEP_500MS, ADC_OFF, BOD_OFF, TIMER2_ON);

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(wakeUpPin));
  } else {
    delay(delayTimeMs);
    digitalWrite(ledPin, LOW);
    hasBeenInterupted = false;

#if ENABLE_OLED
    // Turn off the OLED screen
    oled.ssd1306WriteCmd(SSD1306_DISPLAYOFF);
#endif

    // Allow wake up pin to trigger interrupt on low.
    attachInterrupt(digitalPinToInterrupt(wakeUpPin), wakeUp, RISING);

    // Power down for 10mins with ADC module and BOD module off. This disables the servo movement.
    for (unsigned int i = 0; i < 10 * 60; i += 8) {  // i in seconds
      if (!hasBeenInterupted) LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    }

    // Disable external pin interrupt on wake up pin.
    detachInterrupt(digitalPinToInterrupt(wakeUpPin));

    // Create an empty line
    if (debug) Serial.println();
  }
}

// A binary search based function that returns
// index of a local minima.
unsigned long localMinUtil(unsigned long low, unsigned long high, unsigned long n) {
  // Find index of middle element
  unsigned long mid = low + (high - low) / 2; /* (low + high)/2 */
  if (info) {
    Serial.print("localMinUtil: low=");
    Serial.print(low);
    Serial.print(" mid=");
    Serial.print(mid);
    Serial.print(" high=");
    Serial.print(high);
    Serial.print(" currentTide=");
    Serial.print(myTideCalc.currentTide(mid), 5);
    Serial.println();
  }

  // If the mid is the same as the low and high, quit
  if (high - low < searchIncrement)
    return low;  //invalidTime;

  // Compare middle element with its neighbours
  // (if neighbours exist)
  if ((myTideCalc.currentTide(mid - searchIncrement) > myTideCalc.currentTide(mid)) && (myTideCalc.currentTide(mid + searchIncrement) > myTideCalc.currentTide(mid)))
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
unsigned long localMin(unsigned long beginning, unsigned long end) {
  return localMinUtil(beginning, end, end);
}

// A binary search based function that returns
// index of a local maxima.
unsigned long localMaxUtil(unsigned long low, unsigned long high, unsigned long n) {
  // Find index of middle element
  unsigned long mid = low + (high - low) / 2; /* (low + high)/2 */
  if (info) {
    Serial.print("localMaxUtil: low=");
    Serial.print(low);
    Serial.print(" mid=");
    Serial.print(mid);
    Serial.print(" high=");
    Serial.print(high);
    Serial.print(" currentTide=");
    Serial.print(myTideCalc.currentTide(mid), 5);
    Serial.println();
  }

  // If the mid is the same as the low and high, quit
  if (high - low < searchIncrement)
    return low;  //invalidTime;

  // Compare middle element with its neighbours
  // (if neighbours exist)
  if ((myTideCalc.currentTide(mid - searchIncrement) < myTideCalc.currentTide(mid)) && (myTideCalc.currentTide(mid + searchIncrement) < myTideCalc.currentTide(mid)))
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
unsigned long localMax(unsigned long beginning, unsigned long end) {
  return localMaxUtil(beginning, end, end);
}

#if ENABLE_MOON_PHASE
// Moon phase, takes three parameters: the year (4 digits), the month and the day.
// The function returns fraction full, float 0-1 (e.g. 0 for new, .25 for crescent, .5 for quarter, .75 for gibbous and 1 for full).
// Also calculates phase and age in days. THIS VERSION uses a cosine function to model the illumination fraction.
// Calculated at noon, based on new moon Jan 6, 2000 @18:00, illumination accurate to about 5%, at least for years 1900-2100.
// Modified from post at http://www.nano-reef.com/topic/217305-a-lunar-phase-function-for-the-arduino/
// S. J. Remington 11/2016
float MoonPhase(int nYear, int nMonth, int nDay)  // calculate the current phase of the moon
{
  float age, phase, frac, days_since;
  long YY, MM, K1, K2, K3, JD;
  YY = nYear - floor((12 - nMonth) / 10);
  MM = nMonth + 9;
  if (MM >= 12) {
    MM = MM - 12;
  }
  K1 = floor(365.25 * (YY + 4712));
  K2 = floor(30.6 * MM + 0.5);
  K3 = floor(floor((YY / 100) + 49) * 0.75) - 38;
  JD = K1 + K2 + nDay + 59;  //Julian day
  if (JD > 2299160)          //1582, Gregorian calendar
  {
    JD = JD - K3;
  }

  if (info) {
    Serial.print("MoonPhase: JD=");  //Julian Day, checked OK
    Serial.println(JD);
  }

  days_since = JD - 2451550L;              //since noon on Jan. 6, 2000 (new moon @18:00)
  phase = (days_since - 0.25) / 29.53059;  //0.25 = correct for 6 pm that day
  phase -= floor(phase);                   //phase in cycle
  age = phase * 29.53059;

  // calculate fraction full
  frac = (1.0 - cos(phase * 2 * PI)) * 0.5;

  if (info) {
    Serial.print("MoonPhase: Moon Age=");
    Serial.println(age);
  }

  return frac;  //phase or age or frac, as desired
}
#endif

// Just a handler for the pin interrupt.
void wakeUp() {
  hasBeenInterupted = true;
  screenOnCountdownSec = screenOnTimeSec;
}