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
#define Servo_1 13
#define Servo_2 4
#define Servo_3 5
#define Servo_4 6

// parameter motor
float kp_motor = 0.12;
float kd_motor = 0.1;
float current_L = 0, pev_L = 0, target_L = 0;
float current_R = 0, pev_R = 0, target_R = 0;

// parameter servo
float servo1_target = 160, servo1_current = 0, servo1_offset = 54;
float servo2_target = 44, servo2_current = 0, servo2_offset = 6;
float servo3_target = 1500;
float grip_target = 180, grip_current = 0;
int min_grip = 120, max_grip = 180, close_grip = 90;

// switch parameter
boolean first_touch = true;
int count2 = 0;
int switch_count = 0;
int switch_confirm = 50;

// home parameter
boolean current_yaw_swtich = false;
boolean home_first_loop = true;
int home_current_sequean = 0;

// go parameter
boolean go_first_loop = true;
boolean triggered = false;
int go_current_sequean = 0;
int count_yaw_switch = 0;
float remeaning_deg = 0;
float remeaning_time = 0;
int confirm_go_deg = 0;

// object codinate
int pev_arm_mode = 0;
int block_count = 0;
float check_pos[] = {10.5, 2.2, 0};
float home_pos[] = {5.5, 3.8, 0};
float place1_pos[] = {11.5, 9, 200};
float place2_pos[] = {11.5, 9, 160};
float place3_pos[] = {8.2, 9, 200};
float place4_pos[] = {8.2, 9, 160};

// IK robot arm math
float x = 6, y = 4, pev_x = 6, pev_y = 4;
float x_in = 6, y_in = 4;
float a1 = 12, a2 = 12;
float theta_1 = -38, theta_2 = 145, theta_3 = 106, theta_4 = -145;

// time loop
int current_time = 0, pev_time = 0, offset_time = 5000;
int pev_arm_t = 0, offset_arm_t = 100000;
int pev_switch = 0;
int current_millis = 0, pev_homing = 0, pev_go = 0;

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
Servo servoA;
Servo servoB;
Servo servoC;
Servo mycam;
Interpolation interp1; // interpolation objects
Interpolation interp2;
Interpolation interp3;

void setup()
{
  Serial.begin(115200);
  while ((int)servo1_current != (int)servo1_target)
  {
    servo1_current = interp1.go(servo1_target, 1000);
  }
  while ((int)servo2_current != (int)servo2_target)
  {
    servo2_current = interp2.go(servo2_target, 1000);
  }
  while ((int)grip_current != (int)grip_target)
  {
    grip_current = interp3.go(grip_target, 1000);
  }
  grip_servo.attach(Servo_4);
  servoA.attach(Servo_1);
  servoB.attach(Servo_2);
  servoC.attach(Servo_3);
  servoA.write(servo1_target);
  servoB.write(servo2_target);
  servoC.write(servo3_target);
  grip_servo.write(grip_target);
  mycam.attach(camera_swtich);
  RF_setup(address, "Tx");
  pinMode(RA_pin, INPUT);
  pinMode(arm_yaw_switch, INPUT);
  radioactive_setup(RA_pin);
  voltage_setup(Vb_pin);
  ibusRC_setup(RCibus);
  remote_getdata();
  init_setup();
  Serial.println("STARTO");
  home_positon(false);
  while (true)
  {
    loop_1();
  }
}

void loop_1()
{
  current_time = micros();
  current_time = micros();
  current_millis = millis();
  if (current_time - pev_time >= offset_time)
  {
    sensor_update();
    Robot_mode();
    swtich_camera();
    nRF_send();
    Serial_test();
    pev_time = micros();
  }
}

void sensor_update()
{
  RA = radioactive_getdata(RA_pin);
  voltage = voltage_getdata(Vb_pin);
  remote_getdata();
  ibus_loop();
  switch_runtime();
  yaw_go(0,true);
  home_positon(true);
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

    robot_arm();

  } // end drive mode
  else
  {
    // robot arm mode
    base_rotate();
    robot_arm();

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
    ibus_loop();
    sensor_update();
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
  Serial.print("Target1:");
  Serial.print(servo1_target);
  Serial.print(" Current1:");
  Serial.print(servo1_current);
  Serial.print(" Target2:");
  Serial.print(servo2_target);
  Serial.print(" Current2:");
  Serial.print(servo2_current);
  Serial.print(" x:");
  Serial.print(x);
  Serial.print(" y:");
  Serial.print(y);
  Serial.print(" theta3:");
  Serial.print(theta_3);
  Serial.print(" theta4:");
  Serial.print(theta_4);
  Serial.print(" Vb:");
  Serial.print(voltage);
  Serial.print(" SW_YAW:");
  Serial.print(digitalRead(arm_yaw_switch));
  Serial.print(" SERVO3: ");
  Serial.print(servo3_target);
  Serial.print(" griptarget:");
  Serial.print(grip_target);
  Serial.print(" gripcurrent:");
  Serial.print(grip_current);
  Serial.println("");
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

void arm_controller()
{
  if (current_time - pev_arm_t >= offset_arm_t)
  {
    if (isinFreerange(0, freerange))
    {
      pev_x = x;
      x_in = 0;
    }
    else
    {
      pev_x = x;
      x_in = float(controller_data.R_X) / 200;
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

void robot_arm()
{
  ik_cal();
  servo1_target = theta_3 + servo1_offset;
  servo2_target = servo2_offset - (theta_4 + theta_3);
  servo1_current = interp1.go(servo1_target, 1000);
  servo2_current = interp2.go(servo2_target, 1000);
  grip_current = interp3.go(grip_target, 1000);
  if ((int)servo1_current != (int)servo1_target)
  {
    servoA.write(round(servo1_current));
  }
  if ((int)servo2_current != (int)servo2_target)
  {
    servoB.write(round(servo2_current));
  }
  if ((int)grip_current != (int)grip_target)
  {
    grip_servo.write(round(grip_current));
  }
}

void ik_cal()
{
  if (controller_data.SWC == 0)
  {
    if (pev_arm_mode != controller_data.SWC)
    {
      x = home_pos[0];
      y = home_pos[1];
      Serial.println("Mode 0");
    }
    grip_target = close_grip;
    arm_controller();
    pev_arm_mode = controller_data.SWC;
  }
  else if (controller_data.SWC == 1)
  {
    grip_target = map(controller_data.VRA, 0, 180, min_grip, max_grip);
    if ((int)grip_current != (int)grip_target)
    {
      grip_target = map(controller_data.VRA, 0, 180, min_grip, max_grip);
      x = pev_x;
      y = pev_y;
    }
    else if (pev_arm_mode != controller_data.SWC)
    {
      Serial.println("Mode 1");
      x = check_pos[0];
      y = check_pos[1];
      Serial.println("DOne");
      pev_arm_mode = controller_data.SWC;
    }
    else
    {
      pev_arm_mode = controller_data.SWC;
      arm_controller();
    }
  }
  else if (controller_data.SWC == 2)
  {
    if (pev_arm_mode != controller_data.SWC)
    {
      Serial.println("Mode 2");
    }
    pev_arm_mode = controller_data.SWC;
  }

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

void base_rotate()
{
  if (isinFreerange(3, freerange))
  {
    servo3_target = (1500);
  }
  else if (controller_data.L_X > 0)
  {
    servo3_target = (1470 - controller_data.L_X);
  }
  else if (controller_data.L_X < 0)
  {
    servo3_target = (1500 - controller_data.L_X);
  }
  servoC.writeMicroseconds(servo3_target);
}

void yaw_go(int go_deg, boolean runtime)
{
  // check current_heading if >180 than turn right else if<180 turn left  else if = 180 turn untill hit switch
  if (go_first_loop && not(runtime))
  {
    Serial.println("Start go sequean");
    go_first_loop = false;
    pev_go = millis();
    confirm_go_deg = go_deg;
    go_current_sequean++;
    remeaning_deg = float(go_deg > 180 ? go_deg - 180 : 180 - go_deg);
    remeaning_time = (remeaning_deg / 50) * 1000; // ms
  }
  else if (go_current_sequean == 1)
  {
    Serial.println("sequean 1");
    if (confirm_go_deg > 180)
    {
      Serial.println("CW");
      servo3_target = 1370;
    }
    else if (confirm_go_deg < 180)
    {
      Serial.println("CCW");
      servo3_target = 1600;
    }
    go_current_sequean++;
  }
  else if (go_current_sequean == 2)
  {
    Serial.print("sequean 2 ");
    Serial.println(count_yaw_switch);
    if (current_yaw_swtich && not(triggered))
    {
      count_yaw_switch++;
      triggered = true;
    }
    else if (not(current_yaw_swtich))
    {
      triggered = false;
    }

    if (count_yaw_switch == 2)
    {
      servo3_target = 1500;
      go_current_sequean++;
      pev_go = millis();
    }
  }
  else if (go_current_sequean == 3)
  {
    Serial.print("sequean 3 ");
    Serial.println(String(remeaning_deg) + " " + String(remeaning_time));
    if (confirm_go_deg > 180)
    {
      servo3_target = 1600;
    }
    else if (confirm_go_deg < 180)
    {
      servo3_target = 1370;
    }
    if (current_millis - pev_go >= remeaning_time)
    {
      servo3_target = 1500;
      go_current_sequean++;
    }
  }
  else if (go_current_sequean == 4)
  {
    Serial.println("sequean 4");
    servo3_target = 1500;
    go_first_loop = true;
    go_current_sequean = 0;
  }
  servoC.writeMicroseconds(servo3_target);
}

void home_positon(boolean runtime)
{
  // first turn left for 1s; if not touch switch --> turn right for 2s  if touch button stop turn servo

  if (home_first_loop && not(runtime) && current_yaw_swtich)
  {
    Serial.println("Start homing sequean");
    home_first_loop = false;
    pev_homing = millis();
    servo3_target = 1600;
    home_current_sequean = -1;
  }
  else if (home_first_loop && not(runtime))
  {
    Serial.println("Start homing sequean");
    home_first_loop = false;
    pev_homing = millis();
    home_current_sequean++;
  }
  else if (current_yaw_swtich && (home_current_sequean != 0))
  {
    servo3_target = 1500;
    home_current_sequean = 5;
  }
  else if (home_current_sequean == -1)
  {
    if (current_millis - pev_homing >= 300)
    {
      home_current_sequean == 1;
    }
    pev_homing = millis();
  }
  else if (home_current_sequean == 1)
  {
    Serial.println("sequean 1");
    servo3_target = 1370;
    home_current_sequean++;
  }
  else if (home_current_sequean == 2)
  {
    Serial.println("sequean 2");
    if (current_millis - pev_homing >= 300)
    {
      home_current_sequean++;
    }
  }
  else if (home_current_sequean == 3)
  {
    Serial.println("sequean 3");
    servo3_target = 1600;
    pev_homing = millis();
    home_current_sequean++;
  }
  else if (home_current_sequean == 4)
  {
    Serial.println("sequean 4");
    if (current_millis - pev_homing >= 600)
    {
      home_current_sequean++;
    }
  }
  else if (home_current_sequean == 5)
  {
    Serial.println("sequean 5");
    servo3_target = 1500;
    home_first_loop = true;
    home_current_sequean = 0;
  }
  servoC.writeMicroseconds(servo3_target);
}

void switch_runtime()
{
  if (digitalRead(arm_yaw_switch) == 0 && first_touch)
  {
    count2++;
    Serial.println((float(analogRead(A0)) / 1023) * 20);
    current_yaw_swtich = true;
    first_touch = false;
    pev_switch = millis();
  }
  if (current_time - pev_arm_t >= offset_arm_t)
  {
    if (digitalRead(arm_yaw_switch) == 1 && not(first_touch) && switch_count == switch_confirm)
    {
      first_touch = true;
      switch_count = 0;
    }
    else if (digitalRead(arm_yaw_switch) == 1 && not(first_touch))
    {
      switch_count++;
    }
    pev_arm_t = micros();
  }
  if (current_yaw_swtich)
  {
    if (current_millis - pev_switch >= 200)
    {
      current_yaw_swtich = false;
    }
  }
}