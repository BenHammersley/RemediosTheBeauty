// Remedios The Beauty


// Load the necessary libraries
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_GPS.h>
#include <Adafruit_BMP085.h>
#include <Adafruit_Sensor.h>




//////// Initialise the subsystems

////// GPS SYSTEM

// Assign the GPS chip its tx and rx pins. (tx,rx)
SoftwareSerial gpsSerial(3, 2);
Adafruit_GPS GPS(&gpsSerial);
// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#define GPSECHO  false
// this keeps track of whether we're using the interrupt
// off by default!
boolean usingInterrupt = false;
void useInterrupt(boolean); // Func prototype keeps Arduino 0023 happy

//////







////// COLOR SENSOR SYSTEM
// Initialize the Color Sensor
// ADJD-S311's I2C address, don't change
#define ADJD_S311_ADDRESS 0x74

#define RED 0
#define GREEN 1
#define BLUE 2
#define CLEAR 3

// ADJD-S311's register list
#define CTRL 0x00
#define CONFIG 0x01
#define CAP_RED 0x06
#define CAP_GREEN 0x07
#define CAP_BLUE 0x08
#define CAP_CLEAR 0x09
#define INT_RED_LO 0xA
#define INT_RED_HI 0xB
#define INT_GREEN_LO 0xC
#define INT_GREEN_HI 0xD
#define INT_BLUE_LO 0xE
#define INT_BLUE_HI 0xF
#define INT_CLEAR_LO 0x10
#define INT_CLEAR_HI 0x11
#define DATA_RED_LO 0x40
#define DATA_RED_HI 0x41
#define DATA_GREEN_LO 0x42
#define DATA_GREEN_HI 0x43
#define DATA_BLUE_LO 0x44
#define DATA_BLUE_HI 0x45
#define DATA_CLEAR_LO 0x46
#define DATA_CLEAR_HI 0x47
#define OFFSET_RED 0x48
#define OFFSET_GREEN 0x49
#define OFFSET_BLUE 0x4A
#define OFFSET_CLEAR 0x4B

// Pin definitions:
int sdaPin = A4;  // serial data, hardwired, can't change
int sclPin = A5;  // serial clock, hardwired, can't change
int ledPin = 4;  // LED light source pin, any unused pin will work

unsigned char colorCap[4] = {9, 9, 2, 5};  // values must be between 0 and 15
unsigned int colorInt[4] = {2048, 2048, 2048, 2048};  // max value for these is 4095
unsigned int colorData[4];  // This is where we store the RGB and C data values
signed char colorOffset[4];  // Stores RGB and C offset values

//////






////// ATMOSPHERIC SENSOR SYSTEM
// Initialize the Barometer Sensor
Adafruit_BMP085 bmp;

 



void setup()  
{
  
Wire.begin();
Serial.begin(115200);

delay(1);  // Wait for ADJD reset sequence on the Color Sensor, Give time to the other boards



// For the color sensor
pinMode(ledPin, OUTPUT);  // Set the sensor's LED as output
digitalWrite(ledPin, LOW);  // Initially turn LED light source on








////// First we test the systems

// Initialise the Atmospheric System
if(!bmp.begin())
  {
    /* There was a problem detecting the BMP085 ... check your connections */
    Serial.print("Atmospheric Sensor System: FAIL");
    while(1);
  }  else {
	Serial.print("Atmospheric Sensor System: OK");
  }
  

// Initialise the Color System


  initADJD_S311();  // Initialize the ADJD-S311, sets up cap and int registers
  
  /* First we'll see the initial values
  getRGBC();  // Call this to put new RGB and C values into the colorData array
  printADJD_S311Values();  // Formats and prints all important registers of ADJD-S311
  */
  
  Serial.println("\nHold up a white object in front of the sensor, then press any key to calibrate...\n");
  
  while(!Serial.available())
    ;  // Wait till a key is pressed
  Serial.flush();
  
  Serial.println("\nCalibrating...this may take a moment\n");
  calibrateColor();  // This calibrates R, G, and B int registers
  calibrateClear();  // This calibrates the C int registers
  calibrateCapacitors();  // This calibrates the RGB, and C cap registers
  getRGBC();  // After calibrating, we can get the first RGB and C data readings
  printADJD_S311Values();  // Formats and prints all important ADJD-S311 registers
  
  Serial.println("\nAll values should be under 1000. If they're not, try calibrating again, or decreasing the ambient brightness somehow. ");
  Serial.println("\nPress SPACE to read, \"c\" to calibrate, \"o\" to get offset, \"l\" to go to LED mode");
  
  
  

  
  // 9600 NMEA is the default baud rate for Adafruit MTK GPS's- some use 4800
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  // Set the update rate
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate

  // Request updates on antenna status, comment out to keep quiet
  GPS.sendCommand(PGCMD_ANTENNA);
  useInterrupt(true);
  delay(1000);
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) {
  char c = GPS.read();
  // if you want to debug, this is a good time to do it!
#ifdef UDR0
  if (GPSECHO)
    if (c) UDR0 = c;  
    // writing direct to UDR0 is much much faster than Serial.print 
    // but only one character can be written at a time. 
#endif
}

void useInterrupt(boolean v) {
  if (v) {
    // Timer0 is already used for millis() - we'll just interrupt somewhere
    // in the middle and call the "Compare A" function above
    OCR0A = 0xAF;
    TIMSK0 |= _BV(OCIE0A);
    usingInterrupt = true;
  } else {
    // do not call the interrupt function COMPA anymore
    TIMSK0 &= ~_BV(OCIE0A);
    usingInterrupt = false;
  }
}

uint32_t timer = millis();
void loop()                     // run over and over again
{
  
  while(!Serial.available())
    ;  // Wait till something's pressed
  char inKey = Serial.read();
  
  if (inKey == ' ')
  {  // If SPACE is pressed, get one reading and print it
    getRGBC();
    printADJD_S311Values();
  }
  else if (inKey == 'c')
  {  // If c is pressed, calibrate int and cap registers, then get a reading and print it
    Serial.println("\nCalibrating...\n");
    calibrateColor();
    calibrateClear();
    calibrateCapacitors();
    getRGBC();
    printADJD_S311Values();
  }
  else if (inKey == 'o')
  {  // if o is pressed, get the offset values
    getOffset();
    Serial.print("Offset: \t ");
    for (int i=0; i<4; i++)
    {
      Serial.print(colorOffset[i], DEC);
      Serial.print("\t ");
    }
    Serial.println();
  }
  else if (inKey == 'l')
  {  // if l is pressed, output color readings to an RGB LED
     // We'll assume the sensor is calibrated
    Serial.println("\nReplicating color on RGB LED, press any key to stop...\n");
    Serial.println("\t Red \t Green \t Blue");
    int averageData[3] = {0, 0, 0};  // We'll averaged the data
    while(!Serial.available())
    {  // Run continuously, until a key is pressed
      for (int i=0; i<4; i++)
      {  // Average the data four times
        getRGBC();  // Get data values
        for (int j=0; j<3; j++)
          averageData[i] += colorData[i];
      }
      for (int i=0; i<3; i++)
        averageData[i] /= 4;  // data averaging
        
      for (int i=0; i<3; i++)
      {  // print out the data, and send it to the RGB LED
        Serial.print("\t");
        Serial.print(averageData[i], DEC);
      }
      Serial.println();
    }
  }
  else
    Serial.println("\nPress SPACE to read, \"c\" to calibrate, \"o\" to get offset, \"l\" to go to LED mode");
  Serial.flush();
  
  
  
  
  
  
  
  // in case you are not using the interrupt above, you'll
  // need to 'hand query' the GPS, not suggested :(
  if (! usingInterrupt) {
    // read data from the GPS in the 'main loop'
    char c = GPS.read();
    // if you want to debug, this is a good time to do it!
    if (GPSECHO)
      if (c) Serial.print(c);
  }
  
  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) {
    // a tricky thing here is if we print the NMEA sentence, or data
    // we end up not listening and catching other sentences! 
    // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
    //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false
  
    if (!GPS.parse(GPS.lastNMEA()))   // this also sets the newNMEAreceived() flag to false
      return;  // we can fail to parse a sentence in which case we should just wait for another
  }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
   
    // Data from the Barometric sensor
    Serial.print("Temperature = ");
    Serial.print(bmp.readTemperature());
    Serial.println(" *C");
    
    Serial.print("Pressure = ");
    Serial.print(bmp.readPressure());
    Serial.println(" Pa");
    
    // Calculate altitude assuming 'standard' barometric
    // pressure of 1013.25 millibar = 101325 Pascal
    Serial.print("Altitude = ");
    Serial.print(bmp.readAltitude());
    Serial.println(" meters");

    // you can get a more precise measurement of altitude
    // if you know the current sea level pressure which will
    // vary with weather and such. If it is 1015 millibars
    // that is equal to 101500 Pascals.
    Serial.print("Real altitude = ");
    Serial.print(bmp.readAltitude(101500));
    Serial.println(" meters");  
    
    if (GPS.fix) {
      Serial.print("Location: ");
      Serial.print(GPS.latitude, 4); Serial.print(GPS.lat);
      Serial.print(", "); 
      Serial.print(GPS.longitude, 4); Serial.println(GPS.lon);
      
      Serial.print("Speed (knots): "); Serial.println(GPS.speed);
      Serial.print("Angle: "); Serial.println(GPS.angle);
      Serial.print("GPS Altitude: "); Serial.println(GPS.altitude);
      Serial.print("Satellites: "); Serial.println((int)GPS.satellites);
    }
    
   
  }
}







/* printADJD_S311Values() reads, formats, and prints all important registers
of the ADJD-S311. 
It doesn't perform any measurements, so you'll need to call getRGBC() to print
new values.
*/
void printADJD_S311Values()
{
  Serial.println("\t\t Red \t Green \t Blue \t Clear");
  Serial.print("Data: \t\t ");
  for (int i=0; i<4; i++)
  {
    Serial.print(colorData[i]);
    Serial.print("\t ");
  }
  Serial.println();
  Serial.print("Caps: \t\t ");
  for (int i=0; i<4; i++)
  {
    Serial.print(readRegister(CAP_RED+i), DEC);
    Serial.print("\t ");
  }
  Serial.println();
  Serial.print("Int: \t\t ");
  for (int i=0; i<4; i++)
  {
    Serial.print(readRegisterInt(INT_RED_LO+(i*2)), DEC);
    Serial.print("\t ");
  }
  Serial.println();
  Serial.print("Offset: \t ");
  for (int i=0; i<4; i++)
  {
    Serial.print((signed char) readRegister(OFFSET_RED+i), DEC);
    Serial.print("\t ");
  }
  Serial.println();
}




/* initADJD_S311() - This function initializes the ADJD-S311 and its
capacitor and integration registers 
The vaules for those registers are defined near the top of the code.
the colorCap[] array defines all capacitor values, colorInt[] defines
all integration values.
*/
void initADJD_S311()
{ 
  /*sensor gain registers, CAP_...
  to select number of capacitors.
  value must be <= 15 */
  writeRegister(colorCap[RED] & 0xF, CAP_RED);
  writeRegister(colorCap[GREEN] & 0xF, CAP_GREEN);
  writeRegister(colorCap[BLUE] & 0xF, CAP_BLUE);
  writeRegister(colorCap[CLEAR] & 0xF, CAP_CLEAR);

  /* Write sensor gain registers INT_...
  to select integration time 
  value must be <= 4096 */
  writeRegister((unsigned char)colorInt[RED], INT_RED_LO);
  writeRegister((unsigned char)((colorInt[RED] & 0x1FFF) >> 8), INT_RED_HI);
  writeRegister((unsigned char)colorInt[BLUE], INT_BLUE_LO);
  writeRegister((unsigned char)((colorInt[BLUE] & 0x1FFF) >> 8), INT_BLUE_HI);
  writeRegister((unsigned char)colorInt[GREEN], INT_GREEN_LO);
  writeRegister((unsigned char)((colorInt[GREEN] & 0x1FFF) >> 8), INT_GREEN_HI);
  writeRegister((unsigned char)colorInt[CLEAR], INT_CLEAR_LO);
  writeRegister((unsigned char)((colorInt[CLEAR] & 0x1FFF) >> 8), INT_CLEAR_HI);
}

/* calibrateClear() - This function calibrates the clear integration registers
of the ADJD-S311.
*/
int calibrateClear()
{
  int gainFound = 0;
  int upperBox=4096;
  int lowerBox = 0;
  int half;
  
  while (!gainFound)
  {
    half = ((upperBox-lowerBox)/2)+lowerBox;
    //no further halfing possbile
    if (half==lowerBox)
      gainFound=1;
    else 
    {
      writeInt(INT_CLEAR_LO, half);
      performMeasurement();
      int halfValue = readRegisterInt(DATA_CLEAR_LO);

      if (halfValue>1000)
        upperBox=half;
      else if (halfValue<1000)
        lowerBox=half;
      else
        gainFound=1;
    }
  }
  return half;
}

/* calibrateColor() - This function clalibrates the RG and B 
integration registers.
*/
int calibrateColor()
{
  int gainFound = 0;
  int upperBox=4096;
  int lowerBox = 0;
  int half;
  
  while (!gainFound)
  {
    half = ((upperBox-lowerBox)/2)+lowerBox;
    //no further halfing possbile
    if (half==lowerBox)
    {
      gainFound=1;
    }
    else {
      writeInt(INT_RED_LO, half);
      writeInt(INT_GREEN_LO, half);
      writeInt(INT_BLUE_LO, half);

      performMeasurement();
      int halfValue = 0;

      halfValue=max(halfValue, readRegisterInt(DATA_RED_LO));
      halfValue=max(halfValue, readRegisterInt(DATA_GREEN_LO));
      halfValue=max(halfValue, readRegisterInt(DATA_BLUE_LO));

      if (halfValue>1000) {
        upperBox=half;
      }
      else if (halfValue<1000) {
        lowerBox=half;
      }
      else {
        gainFound=1;
      }
    }
  }
  return half;
}

/* calibrateCapacitors() - This function calibrates each of the RGB and C
capacitor registers.
*/
void calibrateCapacitors()
{
  int  calibrationRed = 0;
  int  calibrationBlue = 0;
  int  calibrationGreen = 0;
  int calibrated = 0;

  //need to store detect better calibration
  int oldDiff = 5000;

  while (!calibrated)
  {
    // sensor gain setting (Avago app note 5330)
    // CAPs are 4bit (higher value will result in lower output)
    writeRegister(calibrationRed, CAP_RED);
    writeRegister(calibrationGreen, CAP_GREEN);
    writeRegister(calibrationBlue, CAP_BLUE);

    // int colorGain = _calibrateColorGain();
    int colorGain = readRegisterInt(INT_RED_LO);
    writeInt(INT_RED_LO, colorGain);
    writeInt(INT_GREEN_LO, colorGain);
    writeInt(INT_BLUE_LO, colorGain);

    int maxRead = 0;
    int minRead = 4096;
    int red   = 0;
    int green = 0;
    int blue  = 0;
    
    for (int i=0; i<4 ;i ++)
    {
      performMeasurement();
      red   += readRegisterInt(DATA_RED_LO);
      green += readRegisterInt(DATA_GREEN_LO);
      blue  += readRegisterInt(DATA_BLUE_LO);
    }
    red   /= 4;
    green /= 4;
    blue  /= 4;

    maxRead = max(maxRead, red);
    maxRead = max(maxRead, green);
    maxRead = max(maxRead, blue);

    minRead = min(minRead, red);
    minRead = min(minRead, green);
    minRead = min(minRead, blue);

    int diff = maxRead - minRead;

    if (oldDiff != diff)
    {
      if ((maxRead==red) && (calibrationRed<15))
        calibrationRed++;
      else if ((maxRead == green) && (calibrationGreen<15))
        calibrationGreen++;
      else if ((maxRead == blue) && (calibrationBlue<15))
        calibrationBlue++;
    }
    else
      calibrated = 1;
      
    oldDiff=diff;

    int rCal = calibrationRed;
    int gCal = calibrationGreen;
    int bCal = calibrationBlue;
  }
  
}

/* writeInt() - This function writes a 12-bit value
to the LO and HI integration registers */
void writeInt(int address, int gain)
{
  if (gain < 4096) 
  {
    byte msb = gain >> 8;
    byte lsb = gain;

    writeRegister(lsb, address);
    writeRegister(msb, address+1);
  }
}

/* performMeasurement() - This must be called before
reading any of the data registers. This commands the
ADJD-S311 to perform a measurement, and store the data
into the data registers.*/
void performMeasurement()
{  
  writeRegister(0x01, 0x00); // start sensing
  while(readRegister(0x00) != 0)
    ; // waiting for a result
}

/* getRGBC() - This function reads all of the ADJD-S311's
data registers and stores them into colorData[]. To get the
most up-to-date data make sure you call performMeasurement() 
before calling this function.*/
void getRGBC()
{
  performMeasurement();
  
  colorData[RED] = readRegisterInt(DATA_RED_LO);
  colorData[GREEN] = readRegisterInt(DATA_GREEN_LO);
  colorData[BLUE] = readRegisterInt(DATA_BLUE_LO);
  colorData[CLEAR] = readRegisterInt(DATA_CLEAR_LO);
}

/* getOffset() - This function performs the offset reading
and stores the offset data into the colorOffset[] array.
You can turn on data trimming by uncommenting out the 
writing 0x01 to 0x01 code.
*/
void getOffset()
{
  digitalWrite(ledPin, LOW);  // turn LED off
  delay(10);  // wait a tic
  writeRegister(0x02, 0x00); // start sensing
  while(readRegister(0x00) != 0)
    ; // waiting for a result
  //writeRegister(0x01, 0x01);  // set trim
  //delay(100);
  for (int i=0; i<4; i++)
    colorOffset[i] = (signed char) readRegister(OFFSET_RED+i);
  digitalWrite(ledPin, HIGH);
}

/* I2C functions...*/
// Write a byte of data to a specific ADJD-S311 address
void writeRegister(unsigned char data, unsigned char address)
{
  Wire.beginTransmission(ADJD_S311_ADDRESS);
  Wire.write(address);
  Wire.write(data);
  Wire.endTransmission();
}

// read a byte of data from ADJD-S311 address
unsigned char readRegister(unsigned char address)
{
  unsigned char data;
  
  Wire.beginTransmission(ADJD_S311_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(ADJD_S311_ADDRESS, 1);
  while (!Wire.available())
    ;  // wait till we can get data
  
  return Wire.read();
}

// Write two bytes of data to ADJD-S311 address and addres+1
int readRegisterInt(unsigned char address)
{
  return readRegister(address) + (readRegister(address+1)<<8);
}

