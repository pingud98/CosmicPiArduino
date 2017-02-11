#include <Wire.h>

//Cosmic Pi Firmware - forked version J.Devine 2017
//Bugs: Internal temperature doesn't work when ADC is in free running mode; configuration needs changing in the temp subroutine.
//This particular build is focused on being a random number generator

float ADCTempValue = 0; //buffer for the internal temperature
String SerialNumberValue = ""; //Buffer for the serial number
long PPSLength = 0; //The number of internal clock cycles in a GPS PPS
long PPSUptime = 0; //The number of PPS pulses counted since the last reboot. 
long PreviousPPS = 0; //The value of the previous PPS (to define which second we're in)
int EventsThisSecond = 0; //The number of events since the last PPS
bool dump = false; //if there's an event, dump the data. 

//SoftSPI pin assignments
#define SS_pin 42 
#define SCK_pin 44
#define MISO_pin 22
#define MOSI_pin 43

//Pinout definitions
#define Power_LED 11
#define Event_LED 12
#define Event_Input 5


//I2C Bus 0 Addresses
#define I2CPot 0x28
#define I2CPot1_PIN 35 //PA0 on the circuit diagram
#define I2CPot2_PIN 36 //PA1 on the circuit diagram
#define I2CPot3_PIN 37 //PA2 on the circuit diagram

#define AccelSA0 26
#define AccelSA1 27 // address pin for the LPS25H


//I2C Bus 1 Addresses
#define HumAddr 0x40
#define AccelAddr 0x1D  // LMS303D on the main board on i2c bus 1
#define PressureAddr 0x5C


#define AccelFullScale 2.0 // +-2g 16 bit 2's compliment
#define GravityEarth 9.80665 //The earth's gravity

const int HVSetpoints[50] = {0x8F,0x8F,0x8E,0x8D,0x8D,0x8C,0x8B,0x8B,0x8A,0x89,
                             0x89,0x88,0x87,0x87,0x86,0x86,0x85,0x84,0x84,0x83,
                             0x82,0x82,0x81,0x80,0x80,0x7F,0x7E,0x7E,0x7D,0x7D,
                             0x7C,0x7B,0x7B,0x7A,0x79,0x79,0x78,0x77,0x77,0x76,
                             0x75,0x75,0x74,0x73,0x73,0x72,0x72,0x71,0x70,0x70};
//HV setpoints, starting from 0 to 50 degrees (i.e. 0th element is 0 degrees) - add 0.5 and cast as an int for the index.

String StringEventBuf[3] = {"Output String Buffer Event 1", "Output String Buffer Event 2", "Output String Buffer Event 3"};
long EventTimestamp[3] = {0,0,0};
float Accel[3]; //Accelerometer array, 0 is X, 1 is Y and 2 is Z. 
unsigned long timeX = 0;
unsigned long oldtime = 0;


void setup() {

  //Start Wire (I2C comms)
    Wire.begin();
    Wire1.begin();
  
  //LED output pins
    pinMode(Power_LED, OUTPUT); //Power LED
    pinMode(Event_LED, OUTPUT); //Event LED
    pinMode(Event_Input, INPUT); //Event LED
  
  //SoftSPI output pins
    digitalWrite(SS, HIGH);  // Start with SS high
    pinMode(SS_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(MISO_pin, INPUT); //note this is the avalanche output from the MAX1932, but not yet used
    pinMode(MOSI_pin, OUTPUT);
  
  //I2CPot output pins
    pinMode(I2CPot1_PIN, OUTPUT);
    pinMode(I2CPot2_PIN, OUTPUT);
    pinMode(I2CPot3_PIN, OUTPUT);
  //and write them low    
    digitalWrite(I2CPot1_PIN, LOW);
    digitalWrite(I2CPot2_PIN, LOW);
    digitalWrite(I2CPot3_PIN, LOW);
  
  // Pressure sensor address setup  
    pinMode(AccelSA1, OUTPUT); //Pressure Sensor
    digitalWrite(AccelSA1, LOW);


    
    Serial.begin(9600);//we run the serial at 9600 for debugging only. 
  
    //make sure the LED's are off
    digitalWrite(Power_LED, 0);
    digitalWrite(Event_LED, 0);
    PressureSetup();
    VbiasSet(HVSetpoints[int(PressureTemp()+0.5)]);
    //VbiasSet(100);
    ThresholdSet(130,130);
    
    //ADCSetup();
    //TimerInit();
    PowerOn();
    //AccelSetup();
    SerialNumberValue = SerialNumberReadout();
    Serial.println(SerialNumberValue);
    Serial.println("finished init");
    timeX = millis();
Serial.print("analogue values ");    
int Ch1 = analogRead(A6);
int Ch2 = analogRead(A7);
Serial.print(Ch1);
Serial.print(" ");
Serial.println(Ch2);

//) + " " + int(analogRead(A7)));

}

void loop() {
  //Serial.println(String(analogRead(A6))+" "+String(analogRead(A7))+" " +int(digitalRead(Event_Input)));
 Serial.println(PressureTemp());
 
 delay(1000);
 /*  
if (digitalRead(Event_Input)) {
Serial.print("Event ");
oldtime = timeX;
timeX = millis();
//prints time since program started
Serial.println(timeX - oldtime);
EventFlashOn();
delay(50);
EventFlashOff();
}
//    Serial.println("temp");
//    Serial.println(HumReadTemp());
//delay(1000);

//    Serial.println("Hum");
//    Serial.println(HumReadHum());
//delay(1000);
//ThresholdSet(200,20);
//Serial.println("Accel");
//AccelRead();
//    Serial.println(String(Accel[0])+" "+ String(Accel[1])+" "+ String(Accel[2]));
*/
}


void TimerInit() {

        uint32_t config = 0;

  // Set up the power management controller for TC0 and TC2

        pmc_set_writeprotect(false);    // Enable write access to power management chip
        pmc_enable_periph_clk(ID_TC0);  // Turn on power for timer block 0 channel 0
        pmc_enable_periph_clk(ID_TC6);  // Turn on power for timer block 2 channel 0

  // Timer block zero channel zero is connected only to the PPS 
  // We set it up to load regester RA on each PPS and reset
  // So RA will contain the number of clock ticks between two PPS, this
  // value should be very stable +/- one tick

        config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        // Select fast clock MCK/2 = 42 MHz
                 TC_CMR_ETRGEDG_RISING |             // External trigger rising edge on TIOA0
                 TC_CMR_ABETRG |                     // Use the TIOA external input line
                 TC_CMR_LDRA_RISING;                 // Latch counter value into RA

        TC_Configure(TC0, 0, config);                // Configure channel 0 of TC0
        TC_Start(TC0, 0);                            // Start timer running

        TC0->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   // Enable the load AR channel 0 interrupt each PPS
        TC0->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   // and disable the rest of the interrupt sources
        NVIC_EnableIRQ(TC0_IRQn);                    // Enable interrupt handler for channel 0

  // Timer block 2 channel zero is connected to the OR of the PPS and the RAY event
 
        config = TC_CMR_TCCLKS_TIMER_CLOCK1 |        // Select fast clock MCK/2 = 42 MHz
                 TC_CMR_ETRGEDG_RISING |             // External trigger rising edge on TIOA1
                 TC_CMR_ABETRG |                     // Use the TIOA external input line
                 TC_CMR_LDRA_RISING;                 // Latch counter value into RA
  
  TC_Configure(TC2, 0, config);                // Configure channel 0 of TC2
  TC_Start(TC2, 0);          // Start timer running
 
  TC2->TC_CHANNEL[0].TC_IER =  TC_IER_LDRAS;   // Enable the load AR channel 0 interrupt each PPS
  TC2->TC_CHANNEL[0].TC_IDR = ~TC_IER_LDRAS;   // and disable the rest of the interrupt sources
  NVIC_EnableIRQ(TC6_IRQn);                    // Enable interrupt handler for channel 0

  // Set up the PIO controller to route input pins for TC0 and TC2

  PIO_Configure(PIOC,PIO_INPUT,
          PIO_PB25B_TIOA0,  // D2 Input for PPS
          PIO_DEFAULT);

  PIO_Configure(PIOC,PIO_INPUT,
          PIO_PC25B_TIOA6,  // D5 Input for Trigger
          PIO_DEFAULT);
}

void TC0_Handler() {
//This is called the one second event interrupt in documentation
  //when the PPS event occurs
  PPSLength = TC0->TC_CHANNEL[0].TC_RA; // Read the RA reg (PPS period)
  TC_GetStatus(TC0, 0);     // Read status and clear load bits

  PPSUptime++;        // PPS count
  EventsThisSecond=0; //reset the event counter for this second
}

void TC6_Handler() {
  Serial.println("Cosmic");
  //rega1 = TC2->TC_CHANNEL[0].TC_RA; // Read the RA on channel 1 (PPS period)
  //stsr1 =
  EventTimestamp[EventsThisSecond] =  TC0->TC_CHANNEL[0].TC_RA; //read the main clock and copy it to the event register
  EventsThisSecond++; //increment the event counter for this second
  TC_GetStatus(TC2, 0);     // Read status clear load bits, unlocking this interrupt.

}


void ADCSetup() {
  REG_ADC_MR = 0x10380080;  // Free run as fast as you can
  REG_ADC_CHER = 3;   // Channels 0 and 1
}


void PowerOn(){
digitalWrite(Power_LED, 1);
}

void EventFlashOn(){
digitalWrite(Event_LED, 1);
}

void EventFlashOff(){
digitalWrite(Event_LED, 0);
}


//This reads out the device serial number.
__attribute__ ((section (".ramfunc")))
String SerialNumberReadout() {
  unsigned int status;
  unsigned int pdwUniqueID[4]; 
 
  /* Send the Start Read unique Identifier command (STUI) 
   * by writing the Flash Command Register with the STUI command.
   */
  EFC1->EEFC_FCR = (0x5A << 24) | EFC_FCMD_STUI;
  do {
    status = EFC1->EEFC_FSR ;
  } while ((status & EEFC_FSR_FRDY) == EEFC_FSR_FRDY);

  /* The Unique Identifier is located in the first 128 bits of the 
   * Flash memory mapping. So, at the address 0x400000-0x400003. 
   */
  pdwUniqueID[0] = *(uint32_t *)IFLASH1_ADDR;
  pdwUniqueID[1] = *(uint32_t *)(IFLASH1_ADDR + 4);
  pdwUniqueID[2] = *(uint32_t *)(IFLASH1_ADDR + 8);
  pdwUniqueID[3] = *(uint32_t *)(IFLASH1_ADDR + 12);

  /* To stop the Unique Identifier mode, the user needs to send the Stop Read unique Identifier
   * command (SPUI) by writing the Flash Command Register with the SPUI command. 
   */
  EFC1->EEFC_FCR = (0x5A << 24) | EFC_FCMD_SPUI ;

  /* When the Stop read Unique Unique Identifier command (SPUI) has been performed, the
   * FRDY bit in the Flash Programming Status Register (EEFC_FSR) rises. 
   */
  do {
    status = EFC1->EEFC_FSR ;
  } while ((status & EEFC_FSR_FRDY) != EEFC_FSR_FRDY);


int uid_ok = 0;
String uidtxt;
uidtxt = String(pdwUniqueID[0]) + String(pdwUniqueID[1]) + String(pdwUniqueID[2]) + String(pdwUniqueID[3]);
return uidtxt;
}




float ADCTemp(){
//Routine uses the internal temperature sensor in the Arduino DUE
//Note this uses the ADC
float trans = 3.3/4096;
float offset = 0.8;
float factor = 0.00256;
int fixtemp = 27;
uint32_t ulValue = 0;
uint32_t ulChannel;
//BUG: The ADC needs to be reset using these register values; otherwise the values read out are WRONG.
  //REG_ADC_MR = 0x00000000;  // Void this register
  //REG_ADC_CHER = 0;   // No channels running

  
  // Enable the corresponding channel
  adc_enable_channel(ADC, ADC_TEMPERATURE_SENSOR);

  // Enable the temperature sensor
  adc_enable_ts(ADC);

  // Start the ADC
  adc_start(ADC);

  // Wait for end of conversion
  while ((adc_get_status(ADC) & ADC_ISR_DRDY) != ADC_ISR_DRDY);

  // Read the value
  ulValue = adc_get_latest_value(ADC);

  // Disable the corresponding channel
  adc_disable_channel(ADC, ADC_TEMPERATURE_SENSOR);

float treal = fixtemp + (( trans * ulValue ) - offset ) / factor;

return treal;
}

