#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <IBusBM.h>
#include <CytronMotorDriver.h>

int voltage = 0;
int RA = 0;
int connection = 1;

int RA_pin = 20;
int CE_pin = 7;
int CSN_pin = 8;

int A1_pin = 10;
int A2_pin = 11;
int B1_pin = 12;
int B2_pin = 13;

int camera_switch = 4;
int mode_switch = 5;
int grip_switch = 7;

int free_offset = 10; 

const uint64_t address = 0xF0F0F0F0E1LL;
HardwareSerial& RCibus = Serial1;

struct Mydata
{
  byte voltage;
  byte radioactive;
  byte connection;
};

struct Remote
{
  int L_X;
  int L_Y;
  int R_X;
  int R_Y;
  boolean SWA;
  boolean SWB;
  int SWC;
  boolean SWD;
  int VRA;
  int VRB;
};

CytronMD M1(PWM_PWM, A1_pin, A2_pin);
CytronMD M2(PWM_PWM, B1_pin, B2_pin);
RF24 radio(CE_pin, CSN_pin); // CE, CSN
Mydata data;
Remote RC_data;

void setup()
{
  Serial.begin(115200);
  RF_setup(address, "Tx");
  pinMode(RA_pin, INPUT);
  radioactive_setup(RA_pin);
  ibusRC_setup(RCibus);
  remote_getdata();
}

void loop()
{
  voltage = 20;
  RA = radioactive_getdata();
  remote_getdata();
  Serial_test();
  nRF_send();
  delay(10);
}

void remote_getdata()
{
  RC_data.L_X = readChannel(2, -100, 100, 0);
  RC_data.L_Y = readChannel(3, -100, 100, 0);
  RC_data.R_X = readChannel(0, -100, 100, 0);
  RC_data.R_Y = readChannel(1, -100, 100, 0);
  RC_data.SWA = redSwitch(4, false);
  RC_data.SWB = redSwitch(5, false);
  RC_data.SWC = readChannel(6, 0, 2, 0);
  RC_data.SWD = redSwitch(7, false);
  RC_data.VRA = readChannel(8, 0, 100, 0);
  RC_data.VRB = readChannel(9, 0, 100, 0);
}

void nRF_send(){
  data.voltage = voltage;
  data.radioactive = RA;
  data.connection = connection;
  if (RA != -1)
  {
    radio.write(&data, sizeof(Mydata));
  }
}

void Serial_test()
{
  Serial.print("LX:");
  Serial.print(RC_data.L_X);
  Serial.print(" LY:");
  Serial.print(RC_data.L_Y);
  Serial.print(" RX:");
  Serial.print(RC_data.R_X);
  Serial.print(" RY:");
  Serial.print(RC_data.R_Y);
  Serial.print(" SA:");
  Serial.print(RC_data.SWA);
  Serial.print(" SB:");
  Serial.print(RC_data.SWB);
  Serial.print(" SC:");
  Serial.print(RC_data.SWC);
  Serial.print(" SD:");
  Serial.print(RC_data.SWD);
  Serial.print(" VA:");
  Serial.print(RC_data.VRA);
  Serial.print(" VB:");
  Serial.println(RC_data.VRB);
}   