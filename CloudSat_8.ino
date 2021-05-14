
/*
   CloudSat 8 Draft

   Dustin Soileau
   Jenny Nguyen
   Jake Sonnier
   Nicholas Drozda
   Hayden Hulin
   Brian Hodge
   Timi Oleosebikan
   Josh Bowden
   Andrew DiCarlo-Craig
   Nick Stephan
*/





//Communications
#include <Wire.h>
#include <SPI.h>
#include <SD.h>


//Time
#include <math.h>


//GPS
#include <Adafruit_GPS.h> //name of GPS Library
#include <SoftwareSerial.h>


//Define TNC ports
SoftwareSerial port1(10, 11); // RX, TX on TNC


//Create GPS library Object
Adafruit_GPS GPS(&Serial3);


//BME280
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>


//Create BME Object
Adafruit_BME280 bme;


//Camera
#include <ArduCAM.h>
#include "memorysaver.h"

#if !(defined OV2640_MINI_2MP)
#error Please select the hardware platform and camera module in the ../libraries/ArduCAM/memorysaver.h file
#endif
#define SD_CS 11
const int SPI_CS = 4;
int count = 0;
#if defined (OV2640_MINI_2MP)
ArduCAM myCAM( OV2640, SPI_CS );
#endif





/*
   Initialize
*/


//Communication
byte a = (0x1);

//Communication 
byte b = (0x00);

//Create char to read the NMEA characters as they come in
char c;

//HallEffectSensor pin
int hallSensor = 42;

//Number of positive readings for hall effect sensor
int positiveReadings = 0;

//Field y/n
float field = false;

//Previous time (milliseconds)
long int previousTime;

//Time at which the last picture was taken (milliseconds)
long int lastPicTaken = 0;

//Previous altitude (feet)
float previousAltitude = 0;

//Current altitude of balloon (feet)
float currentAltitude = 0;

//The rate at which the balloon is climbing
float ascentRate = 0;

//Time when last beacon was sent (milliseconds)
float lastBeaconTime = 0;

//Current humidity (percentage)
float humidity = 0;

//Current temperature (fahrenheit)
float temperature = 0;

//Current pressure (Pascals)
float pressure = 0;

//Current latitude (decimal degrees)
float latitude = 0;

//Current longitude (decimal degrees)
float longitude = 0;

//Current number of pictures taken
float picturesTaken = 0;

//Time recorded from GPS module (UTC - 5)
String timeUTC = "";

//Has balloon been cutdown?
float cutDownStatus = false;

//Are buzzers on?
boolean buzzers = false;

//Time since launch
long int currentMillis;

//Buzzer pin
const int scream = 10;

//
long int tempTime = 0;





/*
   Setup
*/


void setup() {

  //set buzzer and LED
  pinMode(scream, OUTPUT);


  //set baud rate
  Serial.begin(9600);
  GPS.begin(9600);
  port1.begin(19200);


  //update 10 hertz and only request RMC and GGA sentences
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);

  delay(1000);

  //BME280
  if (!bme.begin()) {
    Serial.println("Couldn't find the BME280 sensor");
    while (1);
  }


  //Camera
  uint8_t vid, pid;
  uint8_t temp;
  Wire.begin();
  SD.begin();
  Serial.println(F("ArduCAM Start!"));
  //set the CS as an output:
  pinMode(SPI_CS, OUTPUT);
  digitalWrite(SPI_CS, HIGH);
  //initialze SPI:
  SPI.begin();

  //Reset the CPLD
  myCAM.write_reg(0x07, 0x80);
  delay(100);
  myCAM.write_reg(0x07, 0x00);
  delay(100);

  while (1) {
    //Check if the ArduCAM SPI bus is OK
    myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
    temp = myCAM.read_reg(ARDUCHIP_TEST1);

    if (temp != 0x55) {
      Serial.println(F("SPI interface Error!"));
      delay(1000); continue;
    } else {
      Serial.println(F("SPI interface OK.")); break;
    }
  }
  //Initialize SD Card
  while (!SD.begin(SD_CS)) {
    Serial.println(F("SD Card Error!")); delay(1000);
  }
  Serial.println(F("SD Card detected."));

#if defined (OV2640_MINI_2MP)
  while (1) {
    //Check if the camera module type is OV2640
    myCAM.wrSensorReg8_8(0xff, 0x01);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
    myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
    if ((vid != 0x26 ) && (( pid != 0x41 ) || ( pid != 0x42 ))) {
      Serial.println(F("Can't find OV2640 module!"));
      delay(1000); continue;
    }
    else {
      Serial.println(F("OV2640 detected.")); break;
    }
  }
#endif
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
#if defined (OV2640_MINI_2MP)
  myCAM.OV2640_set_JPEG_size(OV2640_640x480);
#endif
  delay(1000);
}





/*
   Main Loop
*/


void loop() {

  int tries = 5; // the amount of tries incase an error occurs

  //Humidity
  humidity = getHumidity();
  while (isnan(humidity) && tries > 0) {
    humidity = getHumidity();
    tries--;
  }
  if (isnan(humidity)) { //if we never got a good reading
    humidity = 0;
  }

  tries = 5;

  //Temperature
  temperature = getTemperature();
  while (isnan(temperature) && tries > 0) {
    temperature = getTemperature();
    tries--;
  }
  if (isnan(temperature)) {
    temperature = 0;
  }

  tries = 5;

  //Pressure
  pressure = getPressure();
  while (isnan(pressure) && tries > 0) {
    pressure = getPressure();
    tries--;
  }
  if (isnan(pressure)) {
    pressure = 0;
  }

  tries = 5;

  //Latitude
  latitude = getLatitude();
  while (latitude == 0 && tries > 0) {
    latitude = getLatitude();
    tries--;
  }

  tries = 5;

  //Longitude
  longitude = getLongitude();
  while (longitude == 0 && tries > 0) {
    longitude = getLongitude();
    tries--;
  }

  tries = 5;

  //Altitude
  currentAltitude = getAltitude();
  while (currentAltitude == 0 && tries > 0) {
    longitude = getAltitude();
    tries--;
  }

  tries = 5;

  //Time
  timeUTC = getTime();
  while (timeUTC == "0" && tries > 0) {
    timeUTC = getTime();
    tries--;
  }

  //Field
  for (int i = 0; i < 1000; i++)
  {
    if (isField())
    {
      positiveReadings++;
    }
  }

  //70% sure
  if (positiveReadings >= 700)
  {
    field = true;
  }


  //calculate ascention rate
  tempTime = millis();
  ascentRate = (currentAltitude - previousAltitude) / (tempTime - previousTime);
  previousTime = tempTime;
  previousAltitude = currentAltitude;


  //take picture every 30 seconds
  tempTime = millis();
  if (tempTime - lastPicTaken >= 30000)
  {
    takePicture();
    picturesTaken++;
    lastPicTaken = tempTime;
  }


  //cutdown after 3 hours or when altitude is 100,000
  currentMillis = millis();

  if (currentMillis >= 10800000 || currentAltitude >= 100000)
  {
    cutDown();
  }

  //turn buzzers on after 3 hours or after cutdown
  if (currentMillis >= 10800000 || cutDownStatus == true)
  {
    buzz();
  }


  //Print
  Serial.println();
  Serial.print(beacon());
  Serial.println();

  //Radio
  KEY_UP();
  delay(10000);


  //reset hall effect variables
  field = false;
  positiveReadings = 0;
}





/*
   Balloon Functions
*/


//Create payload for TNC beacon
String beacon() {
  String comma = ",";
  String beaconBuffer = "W5UL-11,CloudSat8++" + comma + String(latitude, 8) + comma + String(longitude, 8) + comma + String(currentAltitude, 9) + comma + String(ascentRate, 9) + comma + timeUTC + comma + String(picturesTaken, 1) + comma + String(humidity, 5) + comma + String(pressure, 7) + comma + String(temperature, 5) + comma + String(field, 1) + comma + String(cutDownStatus, 1) + "\0";

  return beaconBuffer;
}



//Key up and put ax25 header that will be repeated by the ISS  <-- Thats Ambitious
void KEY_UP () 
{

  char b = 0;
  
  port1.write(0xC0);//c0.. Start of frame
  port1.write(b);//00// Type packet

   
  port1.write(0x96);//K
  port1.write(0x8C);//F
  port1.write(0x6A);//5
  port1.write(0x84);//B
  port1.write(0x9C);//N
  port1.write(0x98);//L
  port1.write(0xEE);//7
  port1.write(0xAE);//w
  port1.write(0x6A);//5
  port1.write(0xAA);//u
  port1.write(0x98);//l
  port1.write(0x40);//space
  port1.write(0x40);//space
  port1.write(0x77);//11

  
  port1.write(0x03);
  port1.write(0xF0);
  
  port1.print(beacon());//Data to be returned

  port1.write(0xC0);//c0.. End of frame

  //What the fuck bro
  delay(20);
  delay(6);
  
  //Serial.print("CQ     K5Qxj Hello World TEMPI23,TEMPO-50!  ");//Test string
  
  return;
}



//Cutdown
void cutDown()
{
  pinMode(4, HIGH);
  cutDownStatus = true;
}



//Buzzer
void buzz()
{
  int numScream = 5;
  while (numScream > 0) {
    tone(scream, 1000); // Send 1KHz sound
    delay(1000); //screams for 1 second
    noTone(scream);
    delay(1000); //quiet for 1 second
    numScream--;
  }
}



//print
void printTest() {
  Serial.print("pictures taken: ");
  Serial.println(picturesTaken);
  Serial.print("field: ");
  Serial.print(field);
  Serial.print(", readings: ");
  Serial.println(positiveReadings);
  Serial.print("humidity: ");
  Serial.println(humidity);
  Serial.print("temperature: ");
  Serial.println(temperature);
  Serial.print("pressure: ");
  Serial.println(pressure);
  Serial.print("lastPicTaken: ");
  Serial.println(lastPicTaken);
  Serial.print("latitude, longitude: ");
  Serial.print(latitude);
  Serial.print(", ");
  Serial.println(longitude);
  Serial.print("altitude: ");
  Serial.println(currentAltitude);
  Serial.print("ascention rate: ");
  Serial.println(ascentRate);
  Serial.print("cutdown: ");
  Serial.println(cutDownStatus);
  Serial.println();
  delay(1000);
}





/*
   Sensor Functions
*/


//BME280

//Returns temperature (Farenheit)
float getTemperature() {
  return bme.readTemperature() * 9.0 / 5.0 + 32;
}

//Returns humidity (Percentage)
float getHumidity() {
  return bme.readHumidity();
}

//Returns Pressure (Pascals)
float getPressure() {
  return bme.readPressure();
}



//Hall Effect

//Reads high in case of magnetic field
boolean isField() {
  return digitalRead(hallSensor) == HIGH;
}



//Camera

//Takes one picture
void takePicture() {
  char str[8];
  byte buf[256];
  static int i = 0;
  static int k = 0;
  uint8_t temp = 0, temp_last = 0;
  uint32_t length = 0;
  bool is_header = false;
  File outFile;
  //Flush the FIFO
  myCAM.flush_fifo();
  //Clear the capture done flag
  myCAM.clear_fifo_flag();
  //Start capture
  myCAM.start_capture();
  Serial.println();
  Serial.println(F("start Capture"));
  while (!myCAM.get_bit(ARDUCHIP_TRIG , CAP_DONE_MASK));
  Serial.println(F("Capture Done."));
  length = myCAM.read_fifo_length();
  Serial.print(F("The fifo length is :"));
  Serial.println(length, DEC);
  if (length >= MAX_FIFO_SIZE) //384K
  {
    Serial.println(F("Over size."));
    return ;
  }
  if (length == 0 ) //0 kb
  {
    Serial.println(F("Size is 0."));
    return ;
  }
  //Construct a file name
  k = k + 1;
  itoa(k, str, 10);
  strcat(str, ".jpg");
  //Open the new file
  outFile = SD.open(str, O_WRITE | O_CREAT | O_TRUNC);
  if (!outFile) {
    Serial.println(F("File open faild"));
    return;
  }
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  while ( length-- )
  {
    temp_last = temp;
    temp =  SPI.transfer(0x00);
    //Read JPEG data from FIFO
    if ( (temp == 0xD9) && (temp_last == 0xFF) ) //If find the end ,break while,
    {
      buf[i++] = temp;  //save the last  0XD9
      //Write the remain bytes in the buffer
      myCAM.CS_HIGH();
      outFile.write(buf, i);
      //Close the file
      outFile.close();
      Serial.println(F("Image save OK."));
      is_header = false;
      i = 0;
    }
    if (is_header == true)
    {
      //Write image data to buffer if not full
      if (i < 256)
        buf[i++] = temp;
      else
      {
        //Write 256 bytes image data to file
        myCAM.CS_HIGH();
        outFile.write(buf, 256);
        i = 0;
        buf[i++] = temp;
        myCAM.CS_LOW();
        myCAM.set_fifo_burst();
      }
    }
    else if ((temp == 0xD8) & (temp_last == 0xFF))
    {
      is_header = true;
      buf[i++] = temp_last;
      buf[i++] = temp;
    }
  }
}



//GPS

//Reads in two NMEA sentences
void readGPS()
{
  //create two strings to parse the two types of NMEA Sentences
  String NMEA1;
  String NMEA2;

  //clear corrupted data
  clearGPS();

  while (!GPS.newNMEAreceived())
  {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());
  NMEA1 = GPS.lastNMEA();

  //immediately read next sentence

  while (!GPS.newNMEAreceived())
  {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());
  NMEA2 = GPS.lastNMEA();
}

//Clears NMEA strings of invalid data
void  clearGPS()
{
  //read and parse old data to ensure non-corrupt sentences are saved
  //also accounts for delays in the main loop

  while (!GPS.newNMEAreceived())
  {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  while (!GPS.newNMEAreceived())
  {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());

  while (!GPS.newNMEAreceived())
  {
    c = GPS.read();
  }

  GPS.parse(GPS.lastNMEA());
}

//Returns altitude (Feet)
int getAltitude()
{
  //read GPS
  readGPS();

  //no fix check
  if (GPS.fix == 0)
  {
    return 0;
  }

  return GPS.altitude * 3.2;
}

//Returns latitude (Decimal Degrees)
float getLatitude()
{
  //read GPS
  readGPS();

  //no fix check
  if (GPS.fix == 0)
  {
    return 0;
  }
  return GPS.latitudeDegrees;
}

//Returns longitude (Decimal Degrees)
float getLongitude()
{
  //read GPS
  readGPS();

  //no fix check
  if (GPS.fix == 0)
  {
    return 0;
  }
  return GPS.longitudeDegrees;
}

//Returns time (UTC - 5)
String getTime()
{
  //read GPS
  readGPS();

  //define variables
  String output;
  int hour = GPS.hour - 5;
  int minute = GPS.minute;
  int second = GPS.seconds;

  //define output string
  output += hour;
  output += ":";
  output += minute;
  output += ":";
  output += second;

  return output;
}
