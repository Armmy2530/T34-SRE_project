#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <IBusBM.h>
#include <CytronMotorDriver.h>

int voltage = 0;
int RA = 0;
int connection = 1;

int RA_pin = 3;
int CE_pin = 7;
int CSN_pin = 8;

int A1_pin = 11;
int A2_pin = 10;
int B1_pin = 12;
int B2_pin = 9;

int camera_switch = 4;
int mode_switch = 5;
int grip_switch = 7;

int free_offset = 10;
float kp_motor = 0.17;
float kd_motor = 0.1;
float current_L = 0, pev_L = 0, target_L = 0;
float current_R = 0, pev_R = 0, target_R = 0;
int current_time =0, pev_motor = 0, offset_motor = 50;

const uint64_t address = 0xF0F0F0F0E1LL;
HardwareSerial &RCibus = Serial1;

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

CytronMD M_R(PWM_PWM, A1_pin, A2_pin);
CytronMD M_L(PWM_PWM, B1_pin, B2_pin);
RF24 radio(CE_pin, CSN_pin); // CE, CSN
Mydata data;
Remote controller_data;

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
  current_time = millis();
  voltage = 20;
  RA = radioactive_getdata();
  remote_getdata();
  Serial_test();
  nRF_send();
  Robot_mode();
  delay(10);
}

void remote_getdata()
{
  controller_data.L_X = readChannel(3, -255, 255, 0);
  controller_data.L_Y = readChannel(2, -255, 255, 0);
  controller_data.R_X = readChannel(0, -255, 255, 0);
  controller_data.R_Y = readChannel(1, -255, 255, 0);
  controller_data.SWA = redSwitch(4, false);
  controller_data.SWB = redSwitch(5, false);
  controller_data.SWC = readChannel(6, 0, 2, 0);
  controller_data.SWD = redSwitch(7, false);
  controller_data.VRA = readChannel(8, 0, 100, 0);
  controller_data.VRB = readChannel(9, 0, 100, 0);
}

void Robot_mode()
{
  if (current_time - pev_motor >= offset_motor)
  {
  if (controller_data.SWB)
  { // drive mode
    if (controller_data.SWD)
    { // smooth mode
      current_L = pd_sum(kp_motor, kd_motor, (float(controller_data.L_Y) - current_L), pev_L, current_L);
      current_R = pd_sum(kp_motor, kd_motor, (float(controller_data.R_Y) - current_R), pev_R, current_R);
      pev_L = (float(controller_data.L_Y) - current_L);
      pev_R = (float(controller_data.R_Y) - current_R);
      M_R.setSpeed(round(current_L));
      M_L.setSpeed(round(current_R));
    }
    else
    {
      current_L = controller_data.L_Y;
      current_R = controller_data.R_Y;
      pev_L = current_L;
      pev_R = current_R;
      M_R.setSpeed(round(controller_data.L_Y));
      M_L.setSpeed(round(controller_data.R_Y));
    }
    Serial.print("TargetL:");
    Serial.print(round(controller_data.L_Y));
    Serial.print(" CurrentL:");
    Serial.print(round(current_L));
    Serial.print(" TargetR:");
    Serial.print(round(controller_data.R_Y));
    Serial.print(" CurrentR:");
    Serial.println(round(current_R));
  }
  pev_motor = millis();
  }
  else
  {
    // robot arm mode
  }
}

void nRF_send()
{
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
  // Serial.print("LX:");
  // Serial.print(controller_data.L_X);
  // Serial.print(" LY:");
  // Serial.print(controller_data.L_Y);
  // Serial.print(" RX:");
  // Serial.print(controller_data.R_X);
  // Serial.print(" RY:");
  // Serial.print(controller_data.R_Y);
  // Serial.print(" SA:");
  // Serial.print(controller_data.SWA);
  // Serial.print(" SB:");
  // Serial.print(controller_data.SWB);
  // Serial.print(" SC:");
  // Serial.print(controller_data.SWC);
  // Serial.print(" SD:");
  // Serial.print(controller_data.SWD);
  // Serial.print(" VA:");
  // Serial.print(controller_data.VRA);
  // Serial.print(" VB:");
  // Serial.println(controller_data.VRB);
}