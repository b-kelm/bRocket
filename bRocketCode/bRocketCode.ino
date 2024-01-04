// High Frequency Data Logger 
// used in the bRocket Project
// by Benjamin Kelm, 04.01.2024
// Educational Project under the [MIT License](https://github.com/git/git-scm.com/blob/main/MIT-LICENSE.txt)

// WIRING info:
// - GY-87 Sensor is wired to I2C_1 (Wire1), not I2C_0 (standard). This way the form factor is even smaller.

// LIBRARYS REQUIRED
#include "MPU9250_asukiaaa.h"
#include "Adafruit_BMP085.h"
// Instantiate Sensor Objects
Adafruit_BMP085 bme;
MPU9250_asukiaaa mySensor;

// VARIABLES
float aX, aY, aZ, gX, gY, gZ;
float T, dH_alt, h_alt_start; // Variables for Temperature & altitude measurement
int p0_ISA = 101325; // Pressure at sea-level for ISA-Atmosphere

bool DO_PRINT = false; // Print out Sensor Values?
int n_readings_Acc = 0; // Counter for readings
int N_readings_bmp085 = 10; // after 10 Acc readings, the pressure sensor is read.

elapsedMicros MicrosLoopTime = 0;
elapsedMillis MillisSinceStart = 0;      

// Logging File Frequency
uint32_t prev_time_log = 0;
/************************************************************
   SD Card Logging - adapted by Benjamin Kelm
   Demo program to log time and 9 channels of sensor data
   to the SD Card on a Teensy 4.1
   originally by MBorgerson   2/5/2021
********************************************************/
#include <ADC.h>
#include <SD.h>
#include <TimeLib.h>

const int ledpin  = 13;
#define  LEDON   digitalWriteFast(ledpin, HIGH);
#define  LEDOFF  digitalWriteFast(ledpin,  LOW);

// specify the number of channels and which pins to use
#define NUMCHANNELS 9 // 3 Acc + 3 Gyro + Height + Temp + EMPTY

// Currently Inactive, because of problems with interrupts and I2C
// Specify how fast to collect samples
// #define SAMPRATE  1000 

#include <EEPROM.h>
int address_EEPROM = 0;
byte logFileNo;
// Close file and start a new one, if time elapsed
int delta_time_ms = 5*60*1000; // in ms

// RECORDSIZE must be multiple of 4 bytes! (here 6*4 = 24 bytes)
// uint32-bit (4 bytes) time_ms: milliseconds, -> 49 days operation possible: uint32 -> max. +4294967295/(1000*60*60*24) =49
// uint16-bit (2 bytes) d_us: difference in microseconds for differentiation
// NUMCHANNELS (2 bytes) int16-bit - data channels
#define RECORDSIZE (4 + 2 + NUMCHANNELS * 2) // size in bytes of each record 

// define a new data type for the samples
// with default packing, the structure will be a multiple of 4 byte
typedef struct tSample {
  uint32_t time_ms;
  uint16_t d_us;   
  int16_t avals[NUMCHANNELS];
} sampletype;  // each record is 14 bytes long for now

// now define two buffers, each holding 1024 samples.
// For efficiency, the number of samples in each buffer
// is a multiple of 512, which means that complete sectors
// are writen to the output file each time a buffer is written.
#define SAMPLESPERBUFFER 1024
tSample buffer1[SAMPLESPERBUFFER];
tSample buffer2[SAMPLESPERBUFFER];

// Define pointers to buffers used for ADC Collection
// and SD card file writing.
tSample *adcptr = NULL;
tSample *sdptr = NULL;

static uint32_t samplecount = 0;
volatile uint32_t bufferindex = 0;
volatile uint32_t overflows = 0;

const char compileTime [] = "\n\n5-channel Rocket Data-Logger compiled on " __DATE__ " " __TIME__;

// Currently disabled because of possible problems with interrupt and I2C
// Interrupt Timer to log data consistently
// IntervalTimer CollectionTimer;

// FILE OBJECT
File adcFile;

/*******************************************************
SETUP ROUTINE
********************************************************/
void setup() {
  
  // SENSOR SETUP
  Serial.begin(115200);
  //while (!Serial);
  
  // Connect Sensors to I2C Port 1
  Wire1.begin();
  mySensor.setWire(&Wire1); //Setting Wire 1
  mySensor.beginAccel();
  mySensor.beginGyro();
  mySensor.beginMag(); // Not used, but necessary, otherwise there's an error (data does not come in)

  // BEGIN SENSOR READINGS
  bme.begin(BMP085_ULTRALOWPOWER, &Wire1); // Replace 0x76 with the actual I2C address of your sensor
  
  // SD Card Logging preparation
  Serial.printf("Size of tSample is %lu\n", sizeof(tSample));
  pinMode(ledpin, OUTPUT);
  
  // CMSI(); / Controll of SD Card, disabled for now
  Serial.print("Initializing SD card...");

// Indicate, that SD logging is not ready
  if (!StartSDCard()) {
    Serial.println("initialization failed!");
    while (1) { // Fast blink LED if SD card not ready
      LEDON; delay(100);
      LEDOFF; delay(100);
    }
  }

  // Currently disabled, because of possible problems with interrupts and I2C
  // Start the timer that controls ADC and DAC
  // CollectionTimer.begin(ADC_ISR, 1000000 / SAMPRATE);

  Serial.println("Waiting for commands.");
  setSyncProvider(getTeensy3Time); // helps put time into file directory data

  // Open the file
  if (adcFile) adcFile.close();
  adcFile = SD.open(NewFileName(), FILE_WRITE);
  samplecount = 0;
  Serial.printf("Opened file");
  bufferindex = 0;
  overflows = 0;
  adcptr = &buffer1[0];
  sdptr = NULL;


  // Get Height Base Reading
  h_alt_start = bme.readAltitude(p0_ISA);

} // End of SETUP


/*******************************************************
LOOP ROUTINE
********************************************************/
void loop() {

    // READ ACCEL
    if (mySensor.accelUpdate() == 0) {
      aX = mySensor.accelX();
      aY = mySensor.accelY();
      aZ = mySensor.accelZ();
      //aSqrt = mySensor.accelSqrt();
      
      if(DO_PRINT){
        // PRINT TIMING
        Serial.print(MillisSinceStart);
        Serial.print("\t" + String(MicrosLoopTime) + ',');
        MicrosLoopTime = 0; // Reset counter
        
        Serial.print("\taX: " + String(aX) + ",");
        Serial.print("\taY: " + String(aY)+ ",");
        Serial.print("\taZ: " + String(aZ)+ ",");
        //Serial.print("\taccelSqrt: " + String(aSqrt)+ ",");
      }
    }
    
    // READ GYRO
    if (mySensor.gyroUpdate() == 0) {
      gX = mySensor.gyroX();
      gY = mySensor.gyroY();
      gZ = mySensor.gyroZ();
      if(DO_PRINT){
        Serial.print("\tgX: " + String(gX)+ ",");
        Serial.print("\tgY: " + String(gY)+ ",");
        Serial.print("\tgZ: " + String(gZ)+ ",");
      }
    }
    
    n_readings_Acc += 1; // increment counter

    // READ BAROMETER and TEMPERATURE  
    if(n_readings_Acc >= N_readings_bmp085){ 
      dH_alt = bme.readAltitude(p0_ISA) - h_alt_start;
      T = bme.readTemperature();
      n_readings_Acc = 0; // reset couter
      if(DO_PRINT){
        Serial.print("\tdH_alt[m]: ");
        Serial.print(String(dH_alt) + ","); // this should be adjusted to your local forcase 
        Serial.print("\tT[C]: ");
        Serial.print(String(T) + ","); // this should be adjusted to your local forcase 
        Serial.println(""); // Add an empty line
      }
    }

  
  // LOGGING Register Swap
  if(millis() > (prev_time_log + delta_time_ms)){
    prev_time_log = millis(); // Update time
    bufferindex = 0;
    if (adcFile) {
      adcFile.close();
      Serial.println("File Collection halted.");
      Serial.printf("Buffer overflows = %lu\n", overflows);
    }
    //sdptr = NULL;
    //adcptr = NULL;
    //bufferindex = 0;
    Serial.println("#### New Data Record File ####");
    // Start a new sample
    adcFile = SD.open(NewFileName(), FILE_WRITE);
    samplecount = 0;
    Serial.printf("Opened new file");
    bufferindex = 0;
    overflows = 0;
    
    adcptr = &buffer1[0];
    sdptr = NULL;
  }

  // LOG All the Data
  ADC_ISR(); // Interrupt routine is not used as an interrupt here.
  CheckFileState();

}

/*******************************************************
FUNCTIONS
********************************************************/


// SD Logging
// This is the interrupt handler called by the collection interval timer
void ADC_ISR(void) {
  tSample *sptr;
  static uint32_t lastmicros;
  if (adcptr == NULL) return; // don't write unless adcptr is valid
  if (bufferindex >= SAMPLESPERBUFFER) {  // Switch buffers and signal write to SD
    if(sdptr != NULL) overflows++; // foreground didn't write buffer in time
    sdptr = adcptr; // notify foreground to write buffer to SD
    bufferindex = 0;
    if (adcptr == buffer1) {
      adcptr = buffer2;   // collect to buffer2 while buffer1 is written to SD
    } else { // use buffer 1 for collection
      adcptr = buffer1;
    }
  }
  
  // now we know that bufferindex is less than SAMPLESPERBUFFER
  // Please forgive the mix of pointers and array indices in the next line.
  sptr = (tSample *)&adcptr[bufferindex];
  // pure pointer arithmetic MIGHT be faster--depending on how well the compiler optimizes.
  sptr->time_ms = (uint32_t)millis(); // LOG milli-seconds
  //Serial.println(MillisSinceStart);
  sptr->d_us = uint16_t(micros() - lastmicros);
  lastmicros =  micros();  // we can use this later to check for sampling jitter

  // INT16_t = -32,768 to 32,767
  sptr->avals[0] = (int16_t)(aX*100); // a_max 100G
  sptr->avals[1] = (int16_t)(aY*100); 
  sptr->avals[2] = (int16_t)(aZ*100);
  sptr->avals[3] = (int16_t)(gX*10);
  sptr->avals[4] = (int16_t)(gY*10);
  sptr->avals[5] = (int16_t)(gZ*10);
  sptr->avals[6] = (int16_t)(dH_alt*10); //
  sptr->avals[7] = (int16_t)(T*10); //  
  sptr->avals[8] = (int16_t)0;
  samplecount++;
  bufferindex++;
}

// Currently not used!
void CMSI(void) {
  Serial.println();
  Serial.println(compileTime);
  if (adcFile) {
    Serial.printf("adcFile is open and contains %lu samples.\n", samplecount);
  } else {
    Serial.println("adcFile is closed.");
  }
  Serial.println("Valid commands are:");
  Serial.println("   s : Show this message");
  Serial.println("   d : Show SD Card Directory");
  Serial.println("   v : Show approximate volts");
  Serial.println("   c : Start data collection");
  Serial.println("   q : Stop data collection");
  Serial.println();
}

// Add data buffer to output file when needed
void CheckFileState(void) {

  // ADC ISR sets sdptr to a buffer point in ISR
  if (sdptr != NULL) { // write buffer to file
    if (adcFile) { // returns true when file is open
      LEDON
      adcFile.write(sdptr, SAMPLESPERBUFFER * sizeof(tSample));
      adcFile.flush();  // update directory and reduce card power
      LEDOFF
    } // end of if(adcfile)
    sdptr = NULL;  // reset pointer after file is written
  }  // end of if(sdptr != NULL)

}

/*****************************************************************************
   Read the Teensy RTC and return a time_t (Unix Seconds) value

 ******************************************************************************/
time_t getTeensy3Time() {
  return Teensy3Clock.get();
}

//------------------------------------------------------------------------------
//User provided date time callback function.
//   See SdFile::dateTimeCallback() for usage.
//
void dateTime(uint16_t* date, uint16_t* time) {
  // use the year(), month() day() etc. functions from timelib
  // return date using FAT_DATE macro to format fields
  *date = FAT_DATE(year(), month(), day());
  // return time using FAT_TIME macro to format fields
  *time = FAT_TIME(hour(), minute(), second());
}

bool StartSDCard() {
  if (!SD.begin(BUILTIN_SDCARD)) {
    Serial.println("\nSD File initialization failed.\n");
    return false;
  } else  Serial.println("initialization done.");
  // set date time callback function for file dates
  SdFile::dateTimeCallback(dateTime);
  return true;
}


// make a new file name based on time and date
char* NewFileName(void) {
  static char fname[36];
  time_t nn;
  nn = now();
  int mo = month(nn);
  int dd = day(nn);
  int hh = hour(nn);
  int mn = minute(nn);
  int ss = second(nn);
  logFileNo = EEPROM.read(address_EEPROM);
  sprintf(fname, "bR_%02d%02d%02d%02d%02d_%03d.dat", mo, dd, hh, mn, ss,logFileNo);
  EEPROM.write(address_EEPROM, logFileNo+1); // Increment the File Number
  return &fname[0];
}
