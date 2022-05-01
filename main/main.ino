#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <IBusBM.h>
#include <CytronMotorDriver.h>
#include <Servo.h>
#include <Ramp.h>

float voltage = 0;
int RA = 0;
int connection = 1;

#define camera_swtich 46
#define arm_yaw_switch 14

// sensor pin
#define Vb_pin A0
#define RA_pin 3

// nrf24 pin
#define CE_pin 7
#define CSN_pin 8

// motor drive
#define A1_pin 11
#define A2_pin 10
#define B1_pin 12
#define B2_pin 9

// servo motor
#define Servo_1 2
#define Servo_2 4
#define Servo_3 5
#define Servo_4 6

// parameter motor
float kp_motor = 0.12;
float kd_motor = 0.1;
float current_L = 0, pev_L = 0, target_L = 0;
float current_R = 0, pev_R = 0, target_R = 0;

// parameter servo
float servo1_target = 170, servo1_current = 170, servo1_offset = 54;
float servo2_target = 20, servo2_current = 20, servo2_offset = 6;

// IK robot arm math
float x = 7, y = 2, pev_x = 0, pev_y = 0;
float x_in = 0, y_in = 0;
float a1 = 12, a2 = 12;
float theta_1 = 0, theta_2 = 0, theta_3 = 0, theta_4 = 0;

// time loop
int current_time = 0, pev_time = 0, offset_time = 50000;
int current_arm_t = 0, pev_arm_t = 0, offset_arm_t = 100000;

// drive mode 2 setting
int min_percentage = 25;
int max_percentage = 125;

boolean first_print = true;
boolean first_cam = true;
float freerange = 7; // percentage freerange

const uint64_t address = 0xF0F0F0F0E1LL;
HardwareSerial &RCibus = Serial1;

struct Mydata
{
  float voltage;
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

class Interpolation
{
public:
  rampInt myRamp;
  int interpolationFlag = 0;
  int savedValue;

  int go(int input, int duration)
  {

    if (input != savedValue)
    { // check for new data
      interpolationFlag = 0;
    }
    savedValue = input; // bookmark the old value

    if (interpolationFlag == 0)
    {                                                  // only do it once until the flag is reset
      myRamp.go(input, duration, LINEAR, ONCEFORWARD); // start interpolation (value to go to, duration)
      interpolationFlag = 1;
    }

    int output = myRamp.update();
    return output;
  }
};

CytronMD M_L(PWM_PWM, A2_pin, A1_pin);
CytronMD M_R(PWM_PWM, B2_pin, B1_pin);
RF24 radio(CE_pin, CSN_pin); // CE, CSN
Mydata data;
Remote controller_data;
Servo grip_servo;
Servo servo1;
Servo servo2;
Servo servo3;
Servo mycam;
Interpolation interp1; // interpolation objects
Interpolation interp2;

void setup()
{
  Serial.begin(115200);
  RF_setup(address, "Tx");
  pinMode(RA_pin, INPUT);
  pinMode(arm_yaw_switch, INPUT);
  radioactive_setup(RA_pin);
  voltage_setup(Vb_pin);
  ibusRC_setup(RCibus);
  remote_getdata();
  init_setup();
  grip_servo.attach(Servo_4);
  servo1.attach(Servo_1);
  servo2.attach(Servo_2);
  servo3.attach(Servo_3);
  mycam.attach(camera_swtich);
}

void loop()
{
  current_time = micros();
  current_arm_t = micros();
  if (current_time - pev_time >= offset_time)
  {
    sensor_update();
    swtich_camera();
    Serial_test();
    nRF_send();
    Robot_mode();
    ibus_loop();
    pev_time = micros();
  }
}

void sensor_update()
{
  RA = radioactive_getdata(RA_pin);
  voltage = voltage_getdata(Vb_pin);
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
  controller_data.VRA = readChannel(8, 0, 180, 0);
  controller_data.VRB = readChannel(9, 0, 180, 0);
}

void Robot_mode()
{
  if (controller_data.SWB)
  { // drive mode
    if (controller_data.SWD)
    { // smooth mode
      // drive mode 1
      if (isinFreerange(2, freerange) && isinFreerange(3, freerange))
      {
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
      else if (isinFreerange(3, freerange))
      {
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

    // no smoother
    else
    {
      // drive mode 1
      if (isinFreerange(2, freerange) && isinFreerange(3, freerange))
      {
        target_L = 0;
        target_R = 0;
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
      }
      else if (isinFreerange(2, freerange))
      {
        target_L = float(controller_data.L_X) * 2.55;
        target_R = 0 - float(controller_data.L_X) * 2.55;
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
      }
      else if (isinFreerange(3, freerange))
      {
        target_L = float(controller_data.L_Y) * 2.55;
        target_R = float(controller_data.L_Y) * 2.55;
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
      }
      else if (controller_data.L_X > 0)
      {
        target_R = (255 / 100) * (float(controller_data.L_Y) / 100) * float(map(controller_data.L_X, 0, 100, max_percentage, min_percentage));
        target_L = 255 * (float(controller_data.L_Y) / 100);
        current_L = target_L;
        current_R = target_R;
        pev_L = current_L;
        pev_R = current_R;
      }
      else if (controller_data.L_X < 0)
      {
        target_L = (255 / 100) * (float(controller_data.L_Y) / 100) * float(map(controller_data.L_X, -100, 0, min_percentage, max_percentage));
        target_R = (float(controller_data.L_Y) / 100) * 255;
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
    M_L.setSpeed(current_L);
    M_R.setSpeed(current_R);
    Serial.print("TargetL:");
    Serial.print(round(target_L));
    Serial.print(" CurrentL:");
    Serial.print(round(current_L));
    Serial.print(" TargetR:");
    Serial.print(round(target_R));
    Serial.print(" CurrentR:");
    Serial.print(round(current_R));
  } // end drive mode
  else
  {
    // robot arm mode
    ik_cal();
    servo1_target = theta_3 + servo1_offset;
    servo2_target = servo2_offset - (theta_4 + theta_3);

    servo1_current = interp1.go(servo1_target, 1000);
    servo2_current = interp2.go(servo2_target, 1000);
    if (servo1_current != servo1_target)
    {
      servo1.write(round(servo1_current));
    }
    if (servo2_current != servo2_target)
    {
      servo2.write(round(servo2_current));
    }

    pev_time = micros();
  } // end robot arm mode
} // end robot mode

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
    remote_getdata();
    nRF_send();
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

  Serial.print(" Vb:");
  Serial.println(voltage);

  Serial.print(" SW_YAW:");
  Serial.println(digitalRead(arm_yaw_switch));
}

void swtich_camera()
{
  if (controller_data.SWA == 1 && first_cam)
  {
    mycam.write(35);
    first_cam = false;
  }
  else if (controller_data.SWA == 0 && not(first_cam))
  {
    mycam.write(90);
    first_cam = true;
  }
}

void ik_cal()
{
  arm_controller();

  float r1 = sqrt(sq(x) + sq(y));
  float phi_1 = acos((sq(a2) - sq(a1) - sq(r1)) / (-2 * a1 * r1));
  float phi_2 = atan(y / x);
  float phi_2_deg = (phi_2 * 4068) / 71;
  theta_1 = ((phi_2 - phi_1) * 4068) / 71;

  float phi_3 = acos((sq(r1) - sq(a1) - sq(a2)) / (-2 * a1 * a2));
  theta_2 = 180 - ((phi_3 * 4068) / 71);

  float pev_theta_3 = theta_3;
  float pev_theta_4 = theta_4;
  theta_3 = phi_2_deg + (phi_2_deg - theta_1);
  theta_4 = theta_1 - theta_3;

  if (isnan(theta_3) || isnan(theta_4))
  {
    x = pev_x;
    y = pev_y;
    theta_3 = pev_theta_3;
    theta_4 = pev_theta_4;
  }
}

void arm_controller()
{
  if (current_arm_t - pev_arm_t >= offset_arm_t)
  {
    if (isinFreerange(2, freerange))
    {
      pev_x = x;
      x_in = 0;
    }
    else
    {
      pev_x = x;
      x_in = float(controller_data.L_Y) / 200;
      x = x + x_in;
    }

    if (isinFreerange(1, freerange))
    {
      pev_y = y;
      y_in = 0;
    }
    else
    {
      pev_y = y;
      y_in = float(controller_data.R_Y) / 200;
      y = y + y_in;
    }
    pev_arm_t = micros();
  }
}