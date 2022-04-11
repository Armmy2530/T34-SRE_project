#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <IBusBM.h>
#include <CytronMotorDriver.h>
#include <Servo.h>

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

int Servo_1 = 2;
int Servo_2 = 4;
int Servo_3 = 5;
int Servo_4 = 6;

int camera_switch = 4;
int mode_switch = 5;
int grip_switch = 7;

int free_offset = 10;
float kp_motor = 0.17;
float kd_motor = 0.1;
float current_L = 0, pev_L = 0, target_L = 0;
float current_R = 0, pev_R = 0, target_R = 0;
int current_time = 0, pev_motor = 0, offset_motor = 50;
// mode 2 setting
int min_percentage = 50;
int max_percentage = 125;

boolean first_print = true;
float freerange = 5; // percentage freerange

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

CytronMD M_L(PWM_PWM, A1_pin, A2_pin);
CytronMD M_R(PWM_PWM, B1_pin, B2_pin);
RF24 radio(CE_pin, CSN_pin); // CE, CSN
Mydata data;
Remote controller_data;
Servo grip_servo;
Servo servo1;
Servo servo2;
Servo servo3;

void setup()
{
  Serial.begin(115200);
  RF_setup(address, "Tx");
  pinMode(RA_pin, INPUT);
  radioactive_setup(RA_pin);
  ibusRC_setup(RCibus);
  remote_getdata();
  init_setup();
  grip_servo.attach(Servo_2);
  servo1.attach(Servo_1);
  servo2.attach(Servo_3);
  servo3.attach(Servo_4);
}

void loop()
{
  current_time = millis();
  voltage = 20;
  sensor_update();
  Serial_test();
  nRF_send();
  Robot_mode();
  delay(10);
}

void sensor_update()
{
  RA = radioactive_getdata();
  remote_getdata();
}

void remote_getdata()
{
  controller_data.L_X = readChannel(3, -100, 100, 0);
  controller_data.L_Y = readChannel(2, -100, 100, 0);
  controller_data.R_X = readChannel(0, -100, 100, 0);
  controller_data.R_Y = readChannel(1, -100, 100, 0);
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
        // drive mode 1
        if(isinFreerange(2,freerange) && isinFreerange(3,freerange)){
            target_L = 0;
            target_R = 0;
            current_L = pd_sum(kp_motor, kd_motor, (target_L - current_L), pev_L, current_L);
            current_R = pd_sum(kp_motor, kd_motor, (target_R - current_R), pev_R, current_R);
            pev_L = (target_L - current_L);
            pev_R = (target_R - current_R); 
        }
        else if (isinFreerange(2, freerange))
        {
            target_L = float(controller_data.L_X) * 2.55;
            target_R = 0 - float(controller_data.L_X) * 2.55;
            current_L = pd_sum(kp_motor, kd_motor, (target_L - current_L), pev_L, current_L);
            current_R = pd_sum(kp_motor, kd_motor, (target_R - current_R), pev_R, current_R);
            pev_L = (target_L - current_L);
            pev_R = (target_R - current_R);
        }
        else if (isinFreerange(3, freerange)){
            target_L = float(controller_data.L_Y) * 2.55;
            target_R = float(controller_data.L_Y) * 2.55;
            current_L = pd_sum(kp_motor, kd_motor, (target_L - current_L), pev_L, current_L);
            current_R = pd_sum(kp_motor, kd_motor, (target_R - current_R), pev_R, current_R);
            pev_L = (target_L - current_L);
            pev_R = (target_R - current_R);
        }
        else if (controller_data.L_X > 0)
        {
          target_R = (255 / 100) * (float(controller_data.L_Y) / 100) * float(map(controller_data.L_X, 0, 100, max_percentage, min_percentage));
          target_L = 255 * (float(controller_data.L_Y) / 100);
          current_L = pd_sum(kp_motor, kd_motor, (target_L - current_L), pev_L, current_L);
          current_R = pd_sum(kp_motor, kd_motor, (target_R - current_R), pev_R, current_R);
          pev_L = (target_L - current_L);
          pev_R = (target_R - current_R);
        }
        else if (controller_data.L_X < 0)
        {
          target_L = (255 / 100) * (float(controller_data.L_Y) / 100) * float(map(controller_data.L_X, -100, 0, min_percentage, max_percentage));
          target_R = (float(controller_data.L_Y) / 100) * 255;
          current_L = pd_sum(kp_motor, kd_motor, (target_L - current_L), pev_L, current_L);
          current_R = pd_sum(kp_motor, kd_motor, (target_R - current_R), pev_R, current_R);
          pev_L = (target_L - current_L);
          pev_R = (target_R - current_R);
        }

        // drive mode 2
        // target_L = float(controller_data.L_Y);
        // target_R = float(controller_data.R_Y);
        // current_L = pd_sum(kp_motor, kd_motor, (target_L - current_L), pev_L, current_L);
        // current_R = pd_sum(kp_motor, kd_motor, (target_R - current_R), pev_R, current_R);
        // pev_L = (target_L - current_L);
        // pev_R = (target_R - current_R);
      }

      //no smoother 
      else
      {
        // drive mode 1
        if(isinFreerange(2,freerange) && isinFreerange(3,freerange)){
            target_L = 0;
            target_R = 0;
            current_L = target_L;
            current_R = target_R;
            pev_L = current_L;
            pev_R = current_R;
        }
        else if (isinFreerange(2, freerange))
        {
            target_L = controller_data.L_X *2.55;
            target_R = - controller_data.L_X *2.55;
            current_L = target_L;
            current_R = target_R;
            pev_L = current_L;
            pev_R = current_R;
        }
        else if (controller_data.L_X > 0)
        {
          target_R = (255 / 100) * (float(controller_data.L_Y) / 100) * float(map(controller_data.L_X, 0, 100, max_percentage, min_percentage));
          target_L = 100 * (float(controller_data.L_Y) / 100);
          current_L = target_L;
          current_R = target_R;
          pev_L = current_L;
          pev_R = current_R;
        }
        else if (controller_data.L_X < 0)
        {
          target_L = (255 / 100) * (float(controller_data.L_Y) / 100) * float(map(controller_data.L_X, -100, 0, min_percentage, max_percentage));
          target_R = 100 * (float(controller_data.L_Y) / 100);
          current_L = target_L;
          current_R = target_R;
          pev_L = current_L;
          pev_R = current_R;
        }

        // drive mode 2
        // current_L = controller_data.L_Y;
        // current_R = controller_data.R_Y;
        // pev_L = current_L;
        // pev_R = current_R;
        // target_L = round(controller_data.L_Y);
        // target_R = round(controller_data.R_Y);
      }
      M_L.setSpeed(target_L);
      M_R.setSpeed(target_R);
      Serial.print("TargetL:");
      Serial.print(round(target_L));
      Serial.print(" CurrentL:");
      Serial.print(round(current_L));
      Serial.print(" TargetR:");
      Serial.print(round(target_R));
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

void init_setup()
{
  while (not(controller_data.SWB) || controller_data.L_Y != 0 || controller_data.R_Y != 0)
  {
    if (first_print)
    {
      Serial.println("please set rc_controller button and throttle");
      first_print = false;
    }
    RA = radioactive_getdata();
    remote_getdata();
  }
  first_print = true;
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