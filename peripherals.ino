//This contains subroutines for generic peripherals
//MAX5387 Variable resistor
//MAX1932 HV PSU

void ThresholdSet(int ThreshA, int ThreshB) {
//This kinda works
//  Serial.println("Test of threshold setting");
//  Serial.print(ThreshA);
  //Serial.println(analogRead(A1));   //internal readback of the DREF value only, not returned to main code.
  Wire.beginTransmission(0x28);     // transmit to device #94 (0x5F)
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
    digitalWrite(MOSI_pin, bitRead(_send, i-7));      // Set MOSI
    digitalWrite(SCK_pin, HIGH);                    // SCK high
    //bitWrite(_receive, i, digitalRead(MISO_pin)); // Capture MISO
    digitalWrite(SCK_pin, LOW);                     // SCK low
  }
  digitalWrite(SS_pin, HIGH);                       // SS high again
}




