/* ESP32-GPS-msec-ts.ino
   timestamp in msec generator for thingsboard
   example by coniferconifer
   This code is provided as is , without any warranty.

   Apache License v2 LICENSE

   The code demonstrates how to make time stamp for thingsboard ,requiring time stamp in msec.

   Ex.
   ADC read task is called every 100msec and reads out GPIO_NUM_34.(0-4095 for 3.3V)
   GPS is GY-NEO6MV2 board , without 1PPS output.
   1PPS pulse (TIMEPULSE pin of NEO6M) is jumper wired from a signal line for on board LED of GY-NEO6MV2 board,
   assuming rising edge of 1PPS is anotated by NMEA following 1PPS pulse.
   See the illustration in page27 of Reference[3]

   No MQTT publish code is included in this software for simplicity.
   Please write your own MQTT publication code.
   This program works with TinyGPS++ arduino library.

    Hardware: ESP32-devkit-C
     GPIO_NUM_2 ------- LED pull down via 1KOhm (write 1 ... Light On to show GPS decipline)
     GPIO_NUM_4 ------- LED pull down via 1KOhm (write 1 ... LIght On to show TinyGPS++ is working)
     GPIO_NUM_34------Voltage monitor connected to CPU Vcc via 4.7KOhm ( ADC port as a sensor)
     GPIO_NUM_26------1PPS(TIMEPULSE) from NEO6M
     GPIO_NUM_17(TX)------NEO6M GPS RX (not used)
     GPIO_NUM_16(RX)------NEO6M GPS TX

   [1] https://thingsboard.io/
   [2] http://arduiniana.org/libraries/tinygpsplus/
   [3] https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
*/
#include <time.h>
#include <sys/time.h>

TaskHandle_t Task1, Task2, Task3, Task4, Task5;
#define GPSTIMESET 1 // how frequently synchronized with GPS PPS
#include <TinyGPS++.h>

TinyGPSPlus gps;
long GPS_PulseCount = 0;
portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED; //http://marchan.e5.valueserver.jp/cabin/comp/jbox/arc202/doc21105.html
// see interrupt processing for ESP32 at https://techtutorialsx.com/2017/09/30/esp32-arduino-external-interrupts/
#define GPS_BAUD 9600 //9600bps NEO6M by default

#include <time.h>
#include <sys/time.h>

unsigned long gpsEpochTime = 0; //in sec UNIX epoch time , second is counted up since 00:00:00 Jan 1 1970
unsigned long gpsEpochTimeMillis = 0; //in msec millis() when NMEA sentence gives UTC
unsigned long gpsPulseTimeMillis = 0; // millis() when 1PPS from GPS rises
unsigned long gpsOffsetMillis = 0; // msec between 1PPS pulse (rising edge ) to when NMEA sentence gives UTC

// interrupt service for 1PPS from NEO6M GPS
void IRAM_ATTR GPSPulseISR()
{
  //Serial.println("Interrupt serviced.");
  portENTER_CRITICAL_ISR(&mux);
  GPS_PulseCount++;
  portEXIT_CRITICAL_ISR(&mux);

}

HardwareSerial ss(2); // GPIO_NUM_16 for RX ( wired to NEO6M TX) GPIO_NUM_16 for TX (not used now)

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(GPIO_NUM_26, INPUT_PULLUP); //1PPS pulse conncted to NEO6M by jumer wire
  pinMode(GPIO_NUM_4, OUTPUT); //LED
  pinMode(GPIO_NUM_2, OUTPUT); //indicates NMEA UTC is available
  digitalWrite(GPIO_NUM_2, LOW);

  attachInterrupt(digitalPinToInterrupt(GPIO_NUM_26), GPSPulseISR, RISING);

  pinMode(GPIO_NUM_16, INPUT); //GPS NMEA output is connected here
  pinMode(GPIO_NUM_17, OUTPUT);       //to GPS NEO6M , not used now
  pinMode(GPIO_NUM_34, INPUT);
  ss.begin(GPS_BAUD);

  Serial.println(F("GPS deciplined msec time stamp for thingsboard MQTT server by coniferconifer"));

  Serial.print(F("Thanks to TinyGPS++ library v. ")); Serial.print(TinyGPSPlus::libraryVersion());
  Serial.println(F(" by Mikal Hart"));
  // data sampling example
  xTaskCreatePinnedToCore( codeForADC, "ADCTask", 4000, NULL, 1, &Task1, 1); //core 1
}

long gpsLasttime = 0; long gpsLoop = 0;
unsigned long gpsPulseTimeMicros = 0;
unsigned long gpsPulseTimeMicrosLast = 0;
void loop() {
  uint32_t t_millis;
  char c;
  if ( GPS_PulseCount > 0 ) {
    gpsPulseTimeMicros = micros();
    gpsPulseTimeMillis = millis();
    //If ISR has been serviced at least once
    portENTER_CRITICAL(&mux);
    GPS_PulseCount--;
    portEXIT_CRITICAL(&mux);

    Serial.print("GPS 1PPS pulse on time (msec) "); Serial.print(gpsPulseTimeMillis);
    Serial.print(" jitter (usec) "); Serial.println(gpsPulseTimeMicros - gpsPulseTimeMicrosLast - 1000000);
    gpsPulseTimeMicrosLast = gpsPulseTimeMicros;
  }
  if (ss.available() > 0) { //since ss.available() crashes when it is included in a Task,
    //so, only GPS task is placed in loop()
    // This sketch displays information every time a new sentence is correctly encoded.
    // , gps.encode(c) is true for about 10 times in a second.
    c = ss.read(); //
    //    Serial.write(c); // monitor for GPS application , ex. ublox u-center
    if (gps.encode(c)) { // some NMEA sentense is parsed
      t_millis = millis();
      if ( t_millis < (0xffffffff - 1000)) { // avoid millis roll over
      if ( (t_millis - gpsPulseTimeMillis) < 500) { //NMEA parsed is immediately done after PPS
          if ( t_millis - gpsLasttime > 900) { // but discard other consective NMEA sentenses after PPS
            gpsLoop++;
            digitalWrite(GPIO_NUM_4, gpsLoop % 2);
            getGPSInfo();
            gpsLasttime = millis();
          }
        }
      }
    }
    //    }
    if (millis() > 50000 && gps.charsProcessed() < 10)
    {
      Serial.println(F("No GPS detected: check wiring."));
      while (true) {
        delay(1000);
      }
    }
  }
}
//TinyGPS++ が時間を確定させるのは1PPSのパルスの後である。NEO6Mの仕様書に書いてある。
int i = 0;
void getGPSInfo()
{
  int Year = gps.date.year();
  if (!(i % GPSTIMESET)) {
    if (gps.location.isValid() && gps.time.isValid() && ( Year > 2017) )
      //check if GPS is fixed
    {

      byte Month = gps.date.month();
      byte Day = gps.date.day();
      byte Hour = gps.time.hour();
      byte Minute = gps.time.minute();
      byte Second = gps.time.second();

      struct tm tm;  // check epoch time at https://www.epochconverter.com/
      tm.tm_year = Year - 1900;
      tm.tm_mon = Month - 1 ;
      tm.tm_mday = Day;
      tm.tm_hour = Hour ;
      tm.tm_min = Minute;
      tm.tm_sec = Second;
      tm.tm_isdst = -1;  //disable summer time
      time_t t = mktime(&tm);
      gpsEpochTime = t ; //in  sec
      gpsEpochTimeMillis = millis(); //in msec
      gpsOffsetMillis = gpsEpochTimeMillis - gpsPulseTimeMillis;
      struct timeval now = { .tv_sec = t };
      settimeofday(&now, NULL); //本システムは使っていないが、とりあえずシステムの日時を設定
      Serial.printf("GPS Epochtime= %d ", gpsEpochTime); Serial.print(" offset="); Serial.print(gpsOffsetMillis);
      Serial.printf(" UTC time: %s", asctime(&tm));

      struct tm *ptm;
      t = time(NULL); ptm = localtime(&t);

      uint32_t high;
      high = gpsEpochTime + gpsOffsetMillis / 1000;
      String payload = "{";
      payload += "\"ts\":"; payload += high ; payload += int2digit3(gpsOffsetMillis % 1000);
      payload += ",\"values\":{ "; payload += "\"offset\": "; payload += gpsOffsetMillis;
      payload += "}}";
      Serial.println(payload);
      // turn on/off LED
      digitalWrite(GPIO_NUM_2, HIGH);
      delay(50);
      digitalWrite(GPIO_NUM_2, LOW);
    }
    else
    {
      Serial.println(F("GPS INVALID, wait for a second"));
    }
  }
  i++;
}
String int2digit3(uint32_t msec) {
  if (msec >= 100) return ( String(msec));
  if (msec >= 10) return ("0" + String(msec));
  return ("00" + String(msec));
}

#define ADC_CHECK_INTERVAL 100 //msec
void codeForADC(void * parameter)
{
  portTickType xLastWakeTime;
  xLastWakeTime = xTaskGetTickCount();
  Serial.println(F("Voltage check task started."));
  while (1) {
    checkVoltageLevel();
    //    vTaskDelay( ADC_CHECK_INTERVAL / portTICK_RATE_MS );
    vTaskDelayUntil(&xLastWakeTime, ADC_CHECK_INTERVAL / portTICK_RATE_MS );
  }
}

float voltage;
void checkVoltageLevel() {
  uint32_t high;
  uint32_t offsetMillis;
  offsetMillis = millis() - gpsPulseTimeMillis;
  high = gpsEpochTime + offsetMillis / 1000;
  voltage = ((float) analogRead(GPIO_NUM_34) / 4096.0) * 3.3 ;
  String payload = "{";
  payload += "\"ts\":"; payload += high ; payload += int2digit3(offsetMillis % 1000);
  payload += ",\"values\":{ "; payload += "\"voltage\": "; payload += String(voltage, 3);
  payload += "}}";
  Serial.println(payload);
}


