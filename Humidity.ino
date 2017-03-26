//This file contains subroutines for the humidity sensor. 

void HumSetup(){
  //nothing happens here
}

float HumReadTemp(){
  //note this isn't working yet, no traffic on the I2C1 bus
  int HumTemp1 = 0;
  int HumTemp2 = 0;
  int HumTemp3 = 0;
  
  
  Serial.println("temp sub");
  Wire1.beginTransmission(HumAddr);
  Wire1.write(0xE3);
  delay(5);

  Wire1.requestFrom(HumAddr, 3);
  //while (!Wire1.available()) {
  //  if (htu_tmo++ > HTU_TIMEOUT) 
  //    break;
  //  else 
  //    delay(1);
  //}
  HumTemp1 = Wire1.read();
  HumTemp2 = Wire1.read();
  HumTemp3 = Wire1.read();
  //}
  
  Serial.println("read complete");
  int Stemp = (HumTemp1 << 8) | HumTemp2;
  float temp = -46.85 + (175.72 * Stemp / ((float) (unsigned int) 0x10000)); 
  return temp;
  // See data sheet for this formula
}

float HumReadHum(){
  //note this also isn't working yet, no traffic on the I2C1 bus
  int HumHum1 = 0;
  int HumHum2 = 0;
  int HumHum3 = 0;
  
  
  Serial.println("hum sub");
  Wire1.beginTransmission(HumAddr);
  Wire1.write(0xE5);
  delay(5);

  Wire1.requestFrom(HumAddr, 3);
  //while (!Wire1.available()) {
  //  if (htu_tmo++ > HTU_TIMEOUT) 
  //    break;
  //  else 
  //    delay(1);
  //}
  HumHum1 = Wire1.read();
  HumHum2 = Wire1.read();
  HumHum3 = Wire1.read();
  //}
  
  Serial.println("read complete");
  int SHumTemp = (HumHum1 << 8) | HumHum2;
  float temp = -6.0 + (125.0 * SHumTemp / ((float) (unsigned int) 0x10000));
  return temp;
  // See data sheet for this formula
}

