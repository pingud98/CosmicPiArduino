//this code works pretty well. Not awesome, but well enough. 
//requires the Adafruit libraries to be installed in your /libraries directory

#include <Adafruit_BMP085_U.h>
#include <Adafruit_GPS.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_HTU21DF.h>
#include <Wire.h>
#include <SPI.h>


//definitions of stuff
//GPS serial read on Due serial 2
#define mySerial Serial1
#define GPSECHO  true
//SPI device enable pin
const int slaveAPin = 52;
//integer array for data sampling
unsigned int values[200];
//byte outputbuffer[1200];

//temperature voltage matrix. index 0, 0xFF is the minimum value - i.e. doesn't work.
int tempvoltage[50] =
{0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0x6D,0x6D,0x6D,0x6D,0x6D,0x6E,0x6F,0x70,0x71,0x72,
0x73,0x74,0x75,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,
0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

//define trigger pin
const int eventtrigger = 49;
boolean eventhappened = false;

//How many samples to readback on trigger
const int sampledepth = 10;

//how many events have happened since reset
long eventcounter = 0;

//how many measurement points have been readback in the adcoutput loop 
int readoutctr = 0;

//Index counter for the ADC buffer,
int adcloopctr = 0;

//timing second reset signal from GPS
const int timeresetpin = 50;

//define adafruit components
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_HTU21DF htu = Adafruit_HTU21DF();
Adafruit_GPS GPS(&mySerial);

//define variables
String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
boolean timereset = false; //has an event been detected?
String lastgpsread = "";
//lastgpsread.reserve(100);
int deviceid = random(1, 10000);
float temperature = 0;
double temperatureh = 0;
int localtemp = 0;
double humidity = 0;
double baroaltitude = 0;
double accelx = 0;
double accely = 0;
double accelz = 0;
double magx = 0;
double magy = 0;
double magz = 0;
int targethv = 0xFF;
int currentval = 0xFF;
long exacttime = 0;
int uptime = 0;
String outputbuffer = ""; //write all the outputs to a buffer and then fmacro it out
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

//define a reservation for the lastgpsread
//didn't work as there's already a reservation

void setup()
{
  //run the sensor initialisation routine
  initSensors();

  //init GPS on second serial port
  GPS.begin(9600);
  mySerial.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  delay(1000);
  Serial.println(F("GPS online"));
  inputString.reserve(250);
  pinMode(timeresetpin, INPUT);
  attachInterrupt(timeresetpin, timeset, RISING);
  pinMode(eventtrigger, INPUT);
  attachInterrupt(eventtrigger, triggermonkey, FALLING);
  Serial.begin(115200);                           // initialize the serial port:

  //setup ADC
  REG_ADC_MR = 0x10380180;                       // change from 10380200 to 10380180, 1 is the PREESCALER and 8 means FREERUN
  ADC -> ADC_CHER = 0x03; // enable ADC on pin A6 and A7
  eventhappened = false;
  adcloopctr = 0;
  Serial.println("ADC Alive");

//check the temperature and set the target voltage
Serial.println(F("checking temp for target voltage - temp is"));
      sensorreadout();
      localtemp = int(temperatureh);
Serial.println(localtemp);
      targethv = tempvoltage[localtemp];
Serial.println(F("Target voltage is"));
Serial.println(targethv);

//setup SPI interface and start the HV DC ramp
  pinMode (slaveAPin, OUTPUT);
  digitalWrite(slaveAPin, LOW);
  SPI.begin();
  Serial.println(F("ramping start"));
  ramp(targethv);

  
}

void loop() {

  exacttime++; //increment the sub second timer

  unsigned long t = micros();                    // init time an elapsed time, in micro seconds

// instrucction to measure from ADC in rolling mode
  while ((ADC->ADC_ISR & 0x3) == 0);          // wait for conversion
  values[adcloopctr + 100] = ADC->ADC_CDR[0];                       // read value A0
  values[adcloopctr] = ADC->ADC_CDR[1];                         // read value A0
  adcloopctr++;

//read out from the GPS if the string has finished
  if (stringComplete) {
    lastgpsread = inputString;
    lastgpsread.trim();
    //debug printout
    //Serial.println(inputString);
    // clear the string:
    inputString = "";
    stringComplete = false;
  }

//if the GPS says it's one second then restart the local clock and echo
  if (timereset) {
    Serial.println("Timereset Detection on pin 50");
    Serial.println(exacttime);
    exacttime = 0;
    timereset = false;
  }


if (eventhappened) {
    readoutctr = 10;
    sensorreadout();
    eventcounter++;
    printdatajson();
    //Serial.println(F("Event"));
    eventhappened = false;
  }

if (int(temperatureh)!=localtemp)
{
  localtemp = int(temperatureh);
  Serial.println(F("Change in temperature"));
  stepset(tempvoltage[localtemp]);
}

if (adcloopctr == sampledepth) {
    adcloopctr = 0;
  }


}

//serial event from GPS handling
void serialEvent1() {
  while (Serial1.available()) {
    // get the new byte:
    char inChar = (char)Serial1.read();
    // add it to the inputString:
    if (inChar != char(10))
    {
      inputString += inChar;
    }
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

//Serial event from GPS interrupt, launches the second routine to avoid a crash
void serialEventRun(void)
{
  if (Serial1.available())
    serialEvent1();
}

//run once to initialise the sensor systems
void initSensors()
{
  if (!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while (1);
  }
  if (!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while (1);
  }
  if (!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while (1);
  }
  if (!htu.begin()) {
    Serial.println("Couldn't find HTU21DF sensor!");
    while (1);
  }
}

//print out everything
void printdatajson()
{
  //rewritten to use a buffer
  //start the data output on in json format
  outputbuffer = "{\"gps\":\"";
  outputbuffer = outputbuffer + lastgpsread;
  outputbuffer = outputbuffer + "\",\"timing\":";
  outputbuffer = outputbuffer + exacttime;
  outputbuffer = outputbuffer +",\"energy\":{\"energy1\":[";

  for (int firsthalf = adcloopctr + 1; firsthalf < sampledepth; firsthalf++)
  {
    outputbuffer = outputbuffer + (values[firsthalf]);
    if (readoutctr != 1) outputbuffer = outputbuffer + (",");
    readoutctr--;
  }
  for (int secondhalf = 0; secondhalf < adcloopctr + 1; secondhalf++)
  {
    outputbuffer = outputbuffer + (values[secondhalf]);
    if (readoutctr != 1) outputbuffer = outputbuffer + (",");
    readoutctr--;
  }

  outputbuffer = outputbuffer + "],";
   outputbuffer = outputbuffer + "\"energy2\":[";
  readoutctr = 10;

  for (int firsthalf = adcloopctr + 1; firsthalf < sampledepth; firsthalf++)
  {
     outputbuffer = outputbuffer + values[firsthalf + 100];
    if (readoutctr != 1)  outputbuffer = outputbuffer + ",";
    readoutctr--;
  }
  for (int secondhalf = 0; secondhalf < adcloopctr + 1; secondhalf++)
  {
     outputbuffer = outputbuffer + values[secondhalf + 100];
    if (readoutctr != 1)  outputbuffer = outputbuffer + ",";
    readoutctr--;
  }
   outputbuffer = outputbuffer + "]},\"altitude\": "+ baroaltitude + ",\"humidity\": " + humidity + ",\"gravitationalOrientation\":{\"x\":" + accelx;
   outputbuffer = outputbuffer + ",\"y\": "+ accely + ",\"z\":"+ accelz+ "},";

   outputbuffer = outputbuffer + "\"magneticOrientation\":{\"x\":" + magx + ",\"y\":"+ magy+ ",\"z\":" + magz + "},";

   outputbuffer = outputbuffer + "\"temperature\":{\"value1\":" + temperatureh; //val1 is from the humidity sensor
   outputbuffer = outputbuffer + ",\"value2\":";
   outputbuffer = outputbuffer + temperature;
   outputbuffer = outputbuffer + "}, \"uptime\":";
   outputbuffer = outputbuffer + uptime;
   outputbuffer = outputbuffer + ",\"id\":";
   outputbuffer = outputbuffer + deviceid;
   outputbuffer = outputbuffer + "}";
   Serial.println((outputbuffer));
   outputbuffer = "";
}


void triggermonkey()
{
  //Serial.println("trigger");
  eventhappened = true;
}

void sensorreadout()
{
  sensors_event_t accel_event;
  sensors_event_t mag_event;
  sensors_event_t bmp_event;
  sensors_vec_t   orientation;

  /* Calculate pitch and roll from the raw accelerometer data */
  accel.getEvent(&accel_event);
  if (dof.accelGetOrientation(&accel_event, &orientation))
  {
    accelx = (accel_event.acceleration.x);
    accely = (accel_event.acceleration.y);
    accelz = (accel_event.acceleration.z);

  }

  /* Calculate the heading using the magnetometer */
  mag.getEvent(&mag_event);
  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    magx = (mag_event.magnetic.x);
    magy = (mag_event.magnetic.y);
    magz = (mag_event.magnetic.z);
  }

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    bmp.getTemperature(&temperature);
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    baroaltitude = (bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature));

    temperatureh = (htu.readTemperature());
    humidity = (htu.readHumidity());
  }

  //  energy1= energy1 + random(-100,100);
  //  energy2= energy2 + random(-100,100);
  uptime++;

}

void timeset()
{
  timereset = true;
}

void ramp(int localtarget)
{
  //wait 100ms before ramp up
  int difference = currentval - localtarget;

  while (currentval > localtarget)
  {
    digitalWrite(slaveAPin, LOW);
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    //digitalWrite(slaveAPin, LOW);
    delay(180);
    SPI.transfer(currentval);
    Serial.println(currentval);
    delay(180);
    //digitalWrite(slaveAPin, HIGH);
    SPI.endTransaction();
    digitalWrite(slaveAPin, HIGH);
    currentval = currentval - 2;
  }
}

void stepset(int localtarget)
{
    digitalWrite(slaveAPin, LOW);
    SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));
    delay(180);
    SPI.transfer(localtarget);
    Serial.println(F("temperature change, new HV setpoint"));
    Serial.println(localtarget);
    delay(180);
    SPI.endTransaction();
    digitalWrite(slaveAPin, HIGH);
  //}
}
