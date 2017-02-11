//all this stuff comes from the LHP25S data sheet

#define PressureREF_P_XL 0x08
#define PressureREF_P_L 0x09
#define PressureREF_P_H 0x0A
#define PressureWho_Am_I 0x0F
#define PressureRES_CONF 0x10
#define PressureCTRL_REG1 0x20
#define PressureCTRL_REG2 0x21
#define PressureCTRL_REG3 0x22
#define PressureCTRL_REG4 0x23
#define PressureINT_CFG 0x24
#define PressureINT_SOURCE 0x25
#define PressureSTATUS_REG 0x27
#define PressurePRESS_POUT_XL 0x28
#define PressurePRESS_OUT_L 0x29
#define PressurePRESS_OUT_H 0x2A
#define PressureTEMP_OUT_L 0x2B
#define PressureTEMP_OUT_H 0x2C
#define PressureFIFO_CTRL 0x2E
#define PressureFIFO_STATUS 0x2F
#define PressureTHS_P_L 0x30
#define PressureTHS_P_H 0x21
#define PressureRPDS_L 0x39
#define PressureRPDS_H 0x3A

void PressureDebug(){
//debug information that might be useful for this sensor

  byte PressureID = 0;

  Wire1.beginTransmission(PressureAddr);
  Wire1.write(PressureWho_Am_I);
  Wire1.endTransmission();
  Wire1.requestFrom(PressureAddr,1);
  PressureID = Wire1.read();
  Serial.println("PressureID");
  Serial.println(PressureID, HEX);

  Wire1.beginTransmission(PressureAddr);
  Wire1.write(0x27);
  Wire1.endTransmission();
  Wire1.requestFrom(PressureAddr,1);
  PressureID = Wire1.read();
  Serial.println("Status");
  Serial.println(PressureID, BIN);

}

void PressureSetup(){
//set up the pressure sensor for reading. Always do this on boot, otherwise the device will be in power down mode.
  
  byte PressureResolution = 0;
  byte PressureConfig = 0;
  byte PressureTempL = 0;
  byte PressureTempH = 0;

  //Serial.println("Setting up pressure sensor");


  //Setup Resolution
  Wire1.beginTransmission(PressureAddr);
  Wire1.write(PressureRES_CONF);
  Wire1.write(B00001111);
  Wire1.endTransmission();   
  Wire1.requestFrom(PressureAddr,1);
  PressureResolution = Wire1.read();
  Serial.println(PressureResolution, BIN);


  //Setup Control Registers
  Wire1.beginTransmission(PressureAddr);
  Wire1.write(PressureCTRL_REG1);
  Wire1.write(B10010011); //activate device, set resolution to 1hz, enable BDU
  Wire1.endTransmission();   
  Wire1.requestFrom(PressureAddr,1);  
  PressureConfig = Wire1.read();
  Serial.println(PressureConfig, BIN);
}

float PressureTemp(){
//Read the Temperature using the pressure sensor, borrowed from the LPS Library by ryantm
// go find the full thing at https://github.com/pololu/lps-arduino  
  Wire1.beginTransmission(PressureAddr);
  // assert MSB to enable register address auto-increment
  Wire1.write(PressureTEMP_OUT_L | (1 << 7));
  Wire1.endTransmission();
  Wire1.requestFrom(PressureAddr, (byte)2);

  while (Wire1.available() < 2);

  uint8_t tl = Wire1.read();
  uint8_t th = Wire1.read();

  // combine bytes
  int16_t PresTempRaw = (th << 8 | tl);
  return 42.5 + (float)PresTempRaw / 480;

}

