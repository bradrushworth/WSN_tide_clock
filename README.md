# WSN_tide_clock

The sketch will calculate the current tide height for the site (assuming ebay DS3231 real time clock module is set correctly) and display the tide height and time on a ssd1306-controller OLED 128x64 display. Time updates every second, tide updates as the last significant digit changes (10-20 seconds).
