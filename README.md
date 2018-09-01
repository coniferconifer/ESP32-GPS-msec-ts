## ESP32-GPS-msec-ts.ino 

### msec IoT time stamper by ESP32 using GPS (GY-NEO6MV2 board)
- for Arduino IDE (FreeRTOS feature is used to demonstrate sensor access as a Task)
- 1PPS(TIMEPULSE) signal from NEO6M is connected GPIO_NUM_26 to interrupt ESP32 and deciplines ESP32's time stamp for MQTT publication.
- TinyGPS++ is used to decode NMEA sentenses.
- MQTT publication code is not included, just print out JSON format data for thingsboard , 
providing time stamp in msec.
```
serial output (115200bps)
ex. UTC=1535810503 in sec , "ts" shows UTC in msec

GPS 1PPS pulse on time (msec) 71539
{"ts":1535810503090,"values":{ "voltage": 3.299}}
{"ts":1535810503190,"values":{ "voltage": 3.299}}
GPS Epochtime= 1535810513  offset=234 UTC time: Sat Sep  1 14:01:53 2018
{"ts":1535810513234,"values":{ "offset": 234}}
{"ts":1535810513290,"values":{ "voltage": 3.299}}
{"ts":1535810513390,"values":{ "voltage": 3.299}}
{"ts":1535810513490,"values":{ "voltage": 3.299}}
{"ts":1535810513590,"values":{ "voltage": 3.299}}
{"ts":1535810513690,"values":{ "voltage": 3.299}}
{"ts":1535810513790,"values":{ "voltage": 3.299}}
{"ts":1535810513890,"values":{ "voltage": 3.299}}
{"ts":1535810513990,"values":{ "voltage": 3.299}}
GPS 1PPS pulse on time (msec) 72539
```
- GPS time decipline is done at every 10sec by default.
  (#define GPSTIMESET 10)
- GPIO_NUM_2 is used to light LED when NMEA provides UTC(in sec) to ESP32.
- (to do)This is an experimental code to observe the same phenomenon by multiple ESP32 board with sensors.  

- ![Time chart]https://github.com/coniferconifer/ESP32-GPS-msec-ts/blob/master/timechart.png
- ![1PPS signal wire jumper]https://github.com/coniferconifer/ESP32-GPS-msec-ts/blob/master/NEO6MPPS.jpg
### Hardware
- Hardware: ESP32-devkit-C
```
     GPIO_NUM_2 ------- LED pull down via 1KOhm (write 1 ... Light On to show GPS decipline) 
     GPIO_NUM_4 ------- LED pull down via 1KOhm (write 1 ... LIght On to show TinyGPS++ is working) 
     GPIO_NUM_34------Voltage monitor connected to CPU Vcc via 4.7KOhm ( ADC port as a sensor)
     GPIO_NUM_26------1PPS(TIMEPULSE) from NEO6M
     GPIO_NUM_17(TX)------NEO6M GPS RX (not used)
     GPIO_NUM_16(RX)------NEO6M GPS TX
```
### License: Apache License v2

### References
- [1 thingsboard] https://thingsboard.io/
- [2 TinyGPSplus/] http://arduiniana.org/libraries/tinygpsplus/
- [3 u-blox 6 Receiver Description Including Protocol Specification] https://www.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
- [4 thingsboard JSON data format] https://thingsboard.io/docs/reference/http-api/

### PubSubClient
- [https://github.com/knolleary/pubsubclient](https://github.com/knolleary/pubsubclient)

### MQTT server with visiualization tools 
- [https://thingsboard.io/] (https://thingsboard.io/)

