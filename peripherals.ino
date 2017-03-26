//This contains subroutines for generic peripherals
//MAX5387 Variable resistor
//MAX1932 HV PSU

void ThresholdSet(int ThreshA, int ThreshB) {
//This kinda works
//  Serial.println("Test of threshold setting");
//  Serial.print(ThreshA);
  //Serial.println(analogRead(A1));   //internal readback of the DREF value only, not returned to main code.
  Wire.beginTransmission(I2CPot);     // transmit to device #94 (0x5F)
                                    // device address is specified in datasheet
  Wire.write(0x11);                 // sends instruction byte
  Wire.write(ThreshA);              // sends potentiometer value byte
  Wire.endTransmission();           // stop transmitting


//  Serial.println(" "+analogRead(A1));
//  Serial.println("Setting Ch2");
//  Serial.print(ThreshB);

  //Serial.println(analogRead(A2));  //internal readback of the DREF value only, not returned to main code.
  Wire.beginTransmission(I2CPot);  // transmit to device #94 (0x5F)
                                   // device address is specified in datasheet
  Wire.write(0x12);                // sends instruction byte
  Wire.write(ThreshB);             // sends potentiometer value byte
  Wire.endTransmission();          // stop transmitting
//  Serial.println(" "+ analogRead(A2));
}

void ThresholdCalc(int ThreshA[1000], int ThreshB[1000]) {
  //This isn't working right now, need to check the addressing and the way of setting A and B registers
//  Serial.println("Test of threshold setting");
//  Serial.print(ThreshA);
  //Serial.println(analogRead(A1));   //internal readback of the DREF value only, not returned to main code.
//  Wire.beginTransmission(0x28);     // transmit to device #94 (0x5F)
                                    // device address is specified in datasheet
//  Wire.write(0x11);                 // sends instruction byte
//  Wire.write(ThreshA);              // sends potentiometer value byte
//  Wire.endTransmission();           // stop transmitting


//  Serial.println(" "+analogRead(A1));
//  Serial.println("Setting Ch2");
//  Serial.print(ThreshB);

  //Serial.println(analogRead(A2));  //internal readback of the DREF value only, not returned to main code.
//  Wire.beginTransmission(I2CPot);  // transmit to device #94 (0x5F)
                                   // device address is specified in datasheet
//  Wire.write(0x12);                // sends instruction byte
//  Wire.write(ThreshB);             // sends potentiometer value byte
//  Wire.endTransmission();          // stop transmitting
//  Serial.println(" "+ analogRead(A2));
}

void VbiasSet(int Vbias) {
  byte _send = byte (Vbias);
  Serial.print("Set the Vbias to ");
  Serial.println(Vbias);
  digitalWrite(SS_pin, LOW);                        // SS low
  for (int i = 0; i < 8; i++)                       // There are 8 bits in a byte
  {
    digitalWrite(MOSI_pin, bitRead(_send, 7-i));      // Set MOSI
    digitalWrite(SCK_pin, HIGH);                    // SCK high
    //bitWrite(_receive, i, digitalRead(MISO_pin)); // Capture MISO
    digitalWrite(SCK_pin, LOW);                     // SCK low
  }
  digitalWrite(SS_pin, HIGH);                       // SS high again
}



//bus scanning routines for debugging, borrowed from code on Arduino.cc
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896

void busscan0()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C Bus 0...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}



void busscan1()
{
  byte error, address;
  int nDevices;

  Serial.println("Scanning I2C Bus 1...");

  nDevices = 0;
  for(address = 1; address < 127; address++ ) 
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire1.beginTransmission(address);
    error = Wire1.endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.print(address,HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error==4) 
    {
      Serial.print("Unknow error at address 0x");
      if (address<16) 
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");

  delay(5000);           // wait 5 seconds for next scan
}


void clearI2C(){
  // Issue 20 I2C clocks to make sure no slaves are hung in a read
  pinMode(20, OUTPUT);
  pinMode(21, OUTPUT);
  pinMode(70, OUTPUT);
  pinMode(71, OUTPUT);
  digitalWrite(20, LOW);
  digitalWrite(70, LOW);
  for (int i = 0; i < 20; i++)
  {
    digitalWrite(21, LOW);
    digitalWrite(71, LOW);
    delayMicroseconds(10);
    digitalWrite(21, HIGH);
    digitalWrite(71, HIGH);
    delayMicroseconds(10);
  }
}
