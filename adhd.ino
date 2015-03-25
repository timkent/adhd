/*

Arduino Datalogging Hardware Device (ADHD) v0.6
For datalogging Adaptronic E420C and Select ECUs
Written by Tim Kent <tim@kent.id.au>

Uses the following libraries:
* RTClib from https://github.com/adafruit/RTClib/archive/master.zip
* SD from https://github.com/adafruit/SD/archive/master.zip
* SimpleModbusMasterV2rev2 from https://code.google.com/p/simple-modbus/

Connect Adaptronic ECU to real serial port, debug output is sent using SoftwareSerial.

Tried to run this on a Uno but ran into memory problems, hopefully it is OK now I am using F() where possible.
For an older Mega (pre R3), jumper 11, 12 and 13 on shield to 51, 50 and 52 on Mega.

*/

//#define ENABLE_DEBUG_SERIAL
#define ENABLE_RTC

#ifdef ENABLE_DEBUG_SERIAL
#include <SoftwareSerial.h>
#endif

// RTC
#ifdef ENABLE_RTC
#include <Wire.h>
#include "RTClib.h"
#endif

// SD
#include <SPI.h>
#include <SD.h>

// Modbus
#include <SimpleModbusMaster.h>

#ifdef ENABLE_DEBUG_SERIAL
// Debugging output @ 19200 baud
SoftwareSerial debugSerial(7, 8); // RX, TX
#endif

// RTC
#ifdef ENABLE_RTC
RTC_DS1307 rtc;
#endif

// SD
#define CHIP_SELECT 10
File dataFile;

// Modbus
#define BAUD 57600
#define TIMEOUT 100
#define POLLING 10 // The scan rate
#define RETRY_COUNT 0
#define TX_ENABLE_PIN 2 // Not used for RS232 but needed for the library

// The total amount of available memory on the master to store data
#define TOTAL_NO_OF_REGISTERS 29

// This is the easiest way to create new packets
// Add as many as you want. TOTAL_NO_OF_PACKETS
// is automatically updated.
enum
{
  PACKET1,
  PACKET2,
  PACKET3,
  PACKET4,
  PACKET5,
  PACKET6,
  TOTAL_NO_OF_PACKETS // leave this last entry
};

// Create an array of Packets to be configured
Packet packets[TOTAL_NO_OF_PACKETS];

// Master register array
unsigned int regs[TOTAL_NO_OF_REGISTERS];

// Keeps track of last processed request
unsigned int lastSuccessfulRequest = 0;

// Specify order of logging
// regs are 4096-4116 (0-20), 4141 (21), 4152 (22), 4185 (23), 4196-4199 (24-27), 4203 (28)
// Refer to e4xxx_ECU_ModbusRegDefn.xls for details
const int logSequence[] = {0,1,7,2,3,4,5,6,8,10,11,9,20,12,16,22,23,21,24,25,26,27,28,13,14,15,17,18,19};

void setup()
{
#ifdef ENABLE_DEBUG_SERIAL
  debugSerial.begin(19200);
#endif

#ifdef ENABLE_RTC
#ifdef AVR
  Wire.begin();
#else
  Wire1.begin(); // Shield I2C pins connect to alt I2C bus on Arduino Due
#endif
  rtc.begin();

  if (!rtc.isrunning()) {
#ifdef ENABLE_DEBUG_SERIAL
    debugSerial.println(F("RTC is NOT running!"));
#endif
    // following line sets the RTC to the date & time this sketch was compiled
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }

  DateTime now = rtc.now();
#endif
  
#ifdef ENABLE_DEBUG_SERIAL
#ifdef ENABLE_RTC
  debugSerial.println(now.unixtime());
#endif
  debugSerial.println(F("Initialising SD card..."));
#endif
  // Make sure that the default chip select pin is set to
  // output, even if you don't use it:
  pinMode(SS, OUTPUT);

  // See if the card is present and can be initialised:
  if (!SD.begin(CHIP_SELECT)) {
#ifdef ENABLE_DEBUG_SERIAL
    debugSerial.println(F("Card failed, or not present."));
#endif
    // Don't do anything more
    while (1) ;
  }
#ifdef ENABLE_DEBUG_SERIAL
  debugSerial.println(F("Card initialised."));
#endif

  // Open up the file we're going to log to
  // Has to be 8.3 file name
#ifdef ENABLE_RTC
  char dataFileName[11];
  sprintf(dataFileName, "%02d%02d%02d%02d.csv", now.month(), now.day(), now.hour(), now.minute());
  dataFile = SD.open(dataFileName, FILE_WRITE);
#else
  dataFile = SD.open("datalog.csv", FILE_WRITE);
#endif
  if (!dataFile) {
#ifdef ENABLE_DEBUG_SERIAL
    debugSerial.println(F("Error opening output file."));
#endif
    // Wait forever since we can't write data
    while (1) ;
  }

#ifdef ENABLE_DEBUG_SERIAL
  debugSerial.println(F("File opened."));
#endif

  // Initialise each packet
  // regs are 4096-4116, 4141, 4152, 4185, 4196-4199, 4203
  modbus_construct(&packets[PACKET1], 1, READ_HOLDING_REGISTERS, 4096, 21, 0);
  modbus_construct(&packets[PACKET2], 1, READ_HOLDING_REGISTERS, 4141, 1, 21);
  modbus_construct(&packets[PACKET3], 1, READ_HOLDING_REGISTERS, 4152, 1, 22);
  modbus_construct(&packets[PACKET4], 1, READ_HOLDING_REGISTERS, 4185, 1, 23);
  modbus_construct(&packets[PACKET5], 1, READ_HOLDING_REGISTERS, 4196, 4, 24);
  modbus_construct(&packets[PACKET6], 1, READ_HOLDING_REGISTERS, 4203, 1, 28);

  // Initialize the Modbus finite state machine
  modbus_configure(&Serial, BAUD, SERIAL_8N1, TIMEOUT, POLLING, RETRY_COUNT, TX_ENABLE_PIN, packets, TOTAL_NO_OF_PACKETS, regs);

  // Print the header line only once a Modbus request has succeeded
  while (packets->successful_requests < 1) {
    modbus_update();
  }
#ifdef ENABLE_DEBUG_SERIAL
  debugSerial.println(F("Time (s),RPM,MAP (kPa),TPS (%),MAT (°C),WT (°C),AuxT (°C),Lambda,Knock,Idle,MVSS (km/h),SVSS (km/h),Batt (V),Trim (%),Inj 1 (ms),Ign 1 (°),Wastegate (%),Pump,Gear,ExtIn,EGT1 (°C),EGT2 (°C),EGT3 (°C),EGT4 (°C),Inj 2 (ms),Inj 3 (ms),Inj 4 (ms),Ign 2 (°),Ign 3 (°),Ign 4 (°)"));
#endif
  dataFile.println(F("Time (s),RPM,MAP (kPa),TPS (%),MAT (°C),WT (°C),AuxT (°C),Lambda,Knock,Idle,MVSS (km/h),SVSS (km/h),Batt (V),Trim (%),Inj 1 (ms),Ign 1 (°),Wastegate (%),Pump,Gear,ExtIn,EGT1 (°C),EGT2 (°C),EGT3 (°C),EGT4 (°C),Inj 2 (ms),Inj 3 (ms),Inj 4 (ms),Ign 2 (°),Ign 3 (°),Ign 4 (°)"));
}

void loop()
{
  modbus_update();

  unsigned int successfulRequests = packets->successful_requests;

  // Only run if there is a new successful request
  if (successfulRequests > lastSuccessfulRequest) {
#ifdef ENABLE_DEBUG_SERIAL
    debugSerial.print(millis()/1000.0, 3);
    debugSerial.print(",");
#endif
    dataFile.print(millis()/1000.0, 3);
    dataFile.print(",");
    
    for (int i = 0; i < TOTAL_NO_OF_REGISTERS; i++) {
      // Formatting specific to each register
      // regs are 4096-4116 (0-20), 4141 (21), 4152 (22), 4185 (23), 4196-4199 (24-27), 4203 (28)
      // Use println (and no trailing comma) on register that is last in the list of logSequence[]
      switch (logSequence[i]) {
        // AFR needs to be divided by 2570
        case 5:
#ifdef ENABLE_DEBUG_SERIAL
          // This will output in AFR instead of Lambda
          //debugSerial.print(regs[logSequence[i]]/2570.0, 1);
          debugSerial.print(regs[logSequence[i]]/2570.0/14.7, 2);
          debugSerial.print(",");
#endif
          // This will output in AFR instead of Lambda
          //dataFile.print(regs[logSequence[i]]/2570.0, 1);
          dataFile.print(regs[logSequence[i]]/2570.0/14.7, 2);
          dataFile.print(",");
          break;
        // Knock needs to be divided by 256
        case 6:
#ifdef ENABLE_DEBUG_SERIAL
          debugSerial.print(regs[logSequence[i]]/256);
          debugSerial.print(",");
#endif
          dataFile.print(regs[logSequence[i]]/256);
          dataFile.print(",");
          break;
        // Battery needs to be divided by 10
        case 9:
#ifdef ENABLE_DEBUG_SERIAL
          debugSerial.print(regs[logSequence[i]]/10.0, 1);
          debugSerial.print(",");
#endif
          dataFile.print(regs[logSequence[i]]/10.0, 1);
          dataFile.print(",");
          break;
        // Injector needs to be divided by 1500
        case 12:
        case 13:
        case 14:
        case 15:
#ifdef ENABLE_DEBUG_SERIAL
          debugSerial.print(regs[logSequence[i]]/1500.0, 3);
          debugSerial.print(",");
#endif
          dataFile.print(regs[logSequence[i]]/1500.0, 3);
          dataFile.print(",");
          break;
        // Timing needs to be divided by 5
        case 16:
        case 17:
        case 18:
#ifdef ENABLE_DEBUG_SERIAL
          debugSerial.print((int)regs[logSequence[i]]/5.0, 1);
          debugSerial.print(",");
#endif
          dataFile.print((int)regs[logSequence[i]]/5.0, 1);
          dataFile.print(",");
          break;
        // This is the last register in list of logSequence[]
        // Same formatting as 16-18, but use println() and no trailing comma
        case 19:
#ifdef ENABLE_DEBUG_SERIAL
          debugSerial.println((int)regs[logSequence[i]]/5.0, 1);
#endif
          dataFile.println((int)regs[logSequence[i]]/5.0, 1);
          break;
        // Everything else is untouched
        default:
#ifdef ENABLE_DEBUG_SERIAL
          debugSerial.print((int)regs[logSequence[i]]);
          debugSerial.print(",");
#endif
          dataFile.print((int)regs[logSequence[i]]);
          dataFile.print(",");
      }
    }

    lastSuccessfulRequest = successfulRequests;
    dataFile.flush();
  }
}
