//This file contains subroutines for the Accelerometer and the Pressure sensor.

void AccelSetup(){

uint8_t tmp, val;

#define FIFO_EN 0x40
#define HPIS1   0x02
#define HPIS2 0x01

  val = FIFO_EN | HPIS1;
  // BusWrite(acl_ad, 0x1F, val, 1); // CTRL0

#define MXYZEN (0x7 << 5) // Enable XYZ interrupt detection
#define MIELEN 0x1    // Enable

  val = MXYZEN | MIELEN;
  // BusWrite(acl_ad, 0x12, val, 1); // INT_CTRL_M

//  val = magnat_event_threshold >> 8;  // High
  // BusWrite(acl_ad, 0x15, val, 1);    // INT_THS_H_M
//  val = magnat_event_threshold & 0xFF;  // Low
  // BusWrite(acl_ad, 0x14, val, 1);    // INT_THS_L_M

#define AXYXEN 0x7
#define A50Hz (0x5 << 4)

  val = AXYXEN | A50Hz;   // x,y,z enable and 50 Hz
  //BusWrite(AccelAddr, 0x20, val, 1); // CTRL1
Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x20);
  Wire1.write(val);
  
#define GRANGE 2.0
#define AFS2G (0x00 << 3)

  val = AFS2G;      // +/- 2g Full scale
//  BusWrite(acl_ad,0x21,val, 1); // CTRL2
Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x21);
  Wire1.write(val);
  
#define INT1_DRDY_A 0x4
#define INT1_IG1 0x20

  val = INT1_IG1;     // Inertial interrupts on INT1 enabled
//  BusWrite(acl_ad, 0x22, val, 1); // CTRL3
Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x22);
  Wire1.write(val);
#define INT2_IGM 0x10
#define INT2_DRDY_M 0x4

  val = INT2_IGM | INT2_DRDY_M;
  // BusWrite(acl_ad, 0x23, val, 1);

#define LIR1 0x1  // Latch interrupt 1
#define MODR (0x4 << 2) // 50Hz
#define MRES (0x3 << 5) // High resolution
#define TEMPEN 0x80 // Temperature enabled

  val = LIR1 | MODR | MRES | TEMPEN;    
//  BusWrite(acl_ad, 0x24, val, 1); // CTRL5

#define MFS (0x1 << 5)    // +/- 4 Gauss full scale
  
  val = MFS;
//  BusWrite(acl_ad, 0x25, val, 1); // CTRL6
Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x25);
  Wire1.write(val);

#define MD 0x0    // Continuous mode

  val = MD;
//  BusWrite(acl_ad, 0x26, val, 1); // CTRL7
Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x26);
  Wire1.write(val);
#define AI6D 0x80
#define ZHIE 0x20
#define YHIE 0x08
#define XHIE 0x02

  val = AI6D | XHIE | YHIE | ZHIE;  // Interrupt on high x,y,z
  //BusWrite(acl_ad, 0x30, val, 1);   // IG_CFG1

//  val = accelr_event_threshold & 0x7F;
  //BusWrite(acl_ad, 0x32, val, 1);   // Ineterial threshold

  val = 1;      // Interrupt duration
  //BusWrite(acl_ad, 0x33, val, 1); // IG1_DUR1
   
//  val = BusRead(acl_ad, 0x31, 1); // IG_SRC1 read and clear interrupts

  //attachInterrupt(digitalPinToInterrupt(30),Acl_ISR,RISING);
  //attachInterrupt(digitalPinToInterrupt(29),Mag_ISR,RISING);
}


void AccelRead(){


short acl_x=0, acl_y=0, acl_z=0;
float acl_fx=0.0, acl_fy=0.0, acl_fz=0.0;
uint8_t xlo,xhi,ylo,yhi,zlo,zhi;

  //xlo=BusRead(acl_ad, 0x28, acl_bus);
  Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x28);
  delay(5);
  Wire1.requestFrom(AccelAddr, 1);
  xlo = Wire1.read();
  
  //xhi=BusRead(acl_ad, 0x29, acl_bus);
  Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x29);
  delay(5);
  Wire1.requestFrom(AccelAddr, 1);
  xhi = Wire1.read();
  
  //ylo=BusRead(acl_ad, 0x2A, acl_bus);
  Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x2A);
  delay(5);
  Wire1.requestFrom(AccelAddr, 1);
  ylo = Wire1.read();
  
  //yhi=BusRead(acl_ad, 0x2B, acl_bus);
  Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x2B);
  delay(5);
  Wire1.requestFrom(AccelAddr, 1);
  yhi = Wire1.read();
  
  //zlo=BusRead(acl_ad, 0x2C, acl_bus);
  Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x2C);
  delay(5);
  Wire1.requestFrom(AccelAddr, 1);
  zlo = Wire1.read();
  
  
  //zhi=BusRead(acl_ad, 0x2D, acl_bus);
  Wire1.beginTransmission(AccelAddr);
  Wire1.write(0x2D);
  delay(5);
  Wire1.requestFrom(AccelAddr, 1);
  xhi = Wire1.read();
  
  acl_x = (xhi<<8 | xlo);
  acl_y = (yhi<<8 | ylo);
  acl_z = (zhi<<8 | zlo);

  acl_fx = (AccelFullScale * GravityEarth) * ((float) (acl_x) / (float) 0x7FFF);
  acl_fy = (AccelFullScale * GravityEarth) * ((float) (acl_y) / (float) 0x7FFF);
  acl_fz = (AccelFullScale * GravityEarth) * ((float) (acl_z) / (float) 0x7FFF);
}
