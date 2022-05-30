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
boolean drive_enable = true;

// parameter servo
float servo1_target = 160, servo1_current = 0, servo1_offset = 54;
float servo2_target = 44, servo2_current = 0, servo2_offset = 6;
float servo3_target = 1500;
float grip_target = 180, grip_current = 0;
int min_grip = 120, max_grip = 180, close_grip = 90;
int max_cw = 1320, max_ccw = 1640, stop = 1500;
float heading_target = 0, heading_current = -1;

// switch parameter
boolean current_yaw_swtich = false;
boolean first_touch = true;
int switch_count = 0, switch_confirm = 20;

// home parameter
boolean home_first_loop = true;
int home_current_sequean = 0;

// go parameter
boolean go_first_loop = true;
int go_current_sequean = 0;

boolean return_first_loop = true;
int return_current_sequean = 0;

boolean store_first_loop = true;
int store_current_sequean = 0;
boolean pev_swd = false;

boolean get_first_loop = true;
int get_current_sequean = 0;

boolean triggered = false, trigger_manula = true;
float remeaning_deg = 0;
float remeaning_time = 0;
int count_yaw_switch = 0;
int confirm_go_deg = 0;

// object codinate
float x_min = 2, x_max = 24;
float y_min = -5, y_max = 20;
int pev_arm_mode = 0;
int block_count = 0;
float check_pos[] = {8.5, 1, 0};
float home_pos[] = {5.5, 3.8, 0};
float place1_pos[] = {13.9, 10.5, 200};
float place2_pos[] = {13.9, 10.5, 165};
float place3_pos[] = {7.4, 10.5, 200};
float place4_pos[] = {7.4, 10.5, 165};

// IK robot arm math
float x = 6, y = 4, pev_x = 6, pev_y = 4;
float x_in = 6, y_in = 4;
float a1 = 12, a2 = 12;
float theta_1 = -38, theta_2 = 145, theta_3 = 106, theta_4 = -145;
boolean arm_controller_status = true;

// STORE parameter
float x_store = 0, y_store = 0, yaw_store = 0;
float x_get = 0, y_get = 0, yaw_get = 0;

// time loop
int current_millis = 0, current_time = 0;
int pev_time = 0, offset_time = 500;
int pev_arm_t = 0, offset_arm_t = 100000;
int pev_switch = 0;
int pev_homing = 0, pev_go = 0, pev_return = 0, pev_store = 0, pev_get = 0, pev_check = 0;

// drive mode 2 setting
int min_percentage = 25;
int max_percentage = 125;

boolean first_print = true;
boolean first_cam = true;
boolean first_check = true;
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
  get_runtime();
  yaw_go(0, true);
  yaw_return(true);
  home_positon(true);
  store_runtime();
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
    if (drive_enable)
    {
      M_L.setSpeed(current_L);
      M_R.setSpeed(current_R);
    }
    Serial.print("TargetL:");
    Serial.print(round(target_L));
    Serial.print(" CurrentL:");
    Serial.print(round(current_L));
    Serial.print(" TargetR:");
    Serial.print(round(target_R));
    Serial.print(" CurrentR:");
    Serial.print(round(current_R));

    robot_arm();
    servoC.writeMicroseconds(servo3_target);
  } // end drive mode
  else
  {
    // robot arm mode
    robot_arm();
    servoC.writeMicroseconds(servo3_target);
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
  Serial.print(" Block:");
  Serial.print(block_count);

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
  arm_controller_status = true;
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
      x_in = float(controller_data.R_X) / 1000;
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
      y_in = float(controller_data.R_Y) / 1000;
      y = y + y_in;
    }
    pev_arm_t = micros();
  }
}

void robot_arm()
{
  ik_cal();
  if (arm_controller_status == true)
  {
    servo1_target = theta_3 + servo1_offset;
    servo2_target = servo2_offset - (theta_4 + theta_3);
    servo1_current = servo1_target;
    servo2_current = servo2_target;
    interp1.go(servo1_target, 1000);
    interp2.go(servo2_target, 1000);
    grip_current = interp3.go(grip_target, 1000);
    servoA.write(round(servo1_current));
    servoB.write(round(servo2_current));
    if ((int)grip_current != (int)grip_target)
    {
      grip_servo.write(round(grip_current));
    }
  }
  else
  {
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
  arm_controller_status = false;
}

void ik_cal()
{
  if (controller_data.SWC == 0)
  {
    if (pev_arm_mode != controller_data.SWC)
    {
      x = home_pos[0];
      y = home_pos[1];
      pev_check = millis();
      Serial.println("Mode 0");
    }
    grip_target = close_grip;
    if (current_millis - pev_check >= 1200)
    {
      arm_controller();
    }
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
      pev_check = millis();
      pev_arm_mode = controller_data.SWC;
    }
    else if (pev_arm_mode == 1)
    {
      if (current_millis - pev_check >= 1200)
      {
        arm_controller();
      }
    }
  }
  else if (controller_data.SWC == 2)
  {
    if (pev_arm_mode != controller_data.SWC)
    {
      Serial.println("Mode 2");
      if (pev_arm_mode == 0)
      {
        // get object
        if (block_count == 1)
        {
          get_sequen(place1_pos[0], place1_pos[1], place1_pos[2]);
        }
        else if (block_count == 2)
        {
          get_sequen(place2_pos[0], place2_pos[1], place2_pos[2]);
        }
        else if (block_count == 3)
        {
          get_sequen(place3_pos[0], place3_pos[1], place3_pos[2]);
        }
        else if (block_count == 4)
        {
          get_sequen(place4_pos[0], place4_pos[1], place4_pos[2]);
        }
        block_count <= 0 ? block_count = 0 : block_count--;
      }
      else if (pev_arm_mode == 1)
      {
        // store object
        if (block_count == 0)
        {
          store_sequen(place1_pos[0], place1_pos[1], place1_pos[2]);
        }
        else if (block_count == 1)
        {
          store_sequen(place2_pos[0], place2_pos[1], place2_pos[2]);
        }
        else if (block_count == 2)
        {
          store_sequen(place3_pos[0], place3_pos[1], place3_pos[2]);
        }
        else if (block_count == 3)
        {
          store_sequen(place4_pos[0], place4_pos[1], place4_pos[2]);
        }
        block_count >= 4 ? block_count = 4 : block_count++;
      }
      pev_arm_mode = controller_data.SWC;
      pev_check = millis();
    }
    else
    {
      if (current_millis - pev_check >= 10000)
      {
        arm_controller();
      }
    }
  }

  if (controller_data.SWB == 0 && trigger_manula)
  {
    base_rotate();
  }

  if (x > x_max)
  {
    x = x_max;
  }
  else if (x < x_min)
  {
    x = x_min;
  }

  if (y > y_max)
  {
    y = y_max;
  }
  else if (y < y_min)
  {
    y = y_min;
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
    servo3_target = (stop);
  }
  else if (controller_data.L_X > 0)
  {
    servo3_target = (max_cw - (controller_data.L_X / 2) + 60);
  }
  else if (controller_data.L_X < 0)
  {
    servo3_target = (max_ccw - (controller_data.L_X / 2) - 60);
  }
}

void yaw_return(boolean runtime)
{
  // check current_heading if >180 than turn right else if<180 turn left  else if = 180 turn untill hit switch
  if (return_first_loop && not(runtime))
  {
    Serial.println("Start return sequean");
    return_first_loop = false;
    pev_return = millis();
    return_current_sequean++;
  }
  else if (return_current_sequean == 1)
  {
    Serial.println("return sequean 1");
    if (confirm_go_deg > 180)
    {
      Serial.println("return CCW");
      servo3_target = max_ccw;
    }
    else if (confirm_go_deg < 180)
    {
      Serial.println("return CW");
      servo3_target = max_cw;
    }
    return_current_sequean++;
  }
  else if (return_current_sequean == 2)
  {
    Serial.print("return sequean 2 ");
    if (current_millis - pev_return >= 500)
    {
      return_current_sequean++;
      pev_return = millis();
      pev_swd = controller_data.SWD;
    }
  }
  else if (return_current_sequean == 3)
  {
    Serial.print("return sequean 3 ");
    Serial.println(count_yaw_switch);
    if (pev_swd != controller_data.SWD)
    {
      return_current_sequean++;
    }
    if (current_yaw_swtich && not(triggered))
    {
      count_yaw_switch++;
      triggered = true;
    }
    else if (not(current_yaw_swtich))
    {
      triggered = false;
    }

    if (count_yaw_switch == 1)
    {
      servo3_target = stop;
      return_current_sequean++;
      pev_return = millis();
    }
  }
  else if (return_current_sequean == 4)
  {
    Serial.println("return sequean 4");
    servo3_target = stop;
    return_first_loop = true;
    return_current_sequean = 0;
    heading_current = 0;
    count_yaw_switch = 0;
  }
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
    Serial.println("go sequean 1");
    if (confirm_go_deg > 180)
    {
      Serial.println("go CW");
      servo3_target = max_cw;
    }
    else if (confirm_go_deg < 180)
    {
      Serial.println("go CCW");
      servo3_target = max_ccw;
    }
    go_current_sequean++;
    pev_go = millis();
  }
  else if (go_current_sequean == 2)
  {
    Serial.print("go sequean 2 ");
    if (current_millis - pev_go >= 500)
    {
      go_current_sequean++;
      pev_swd = controller_data.SWD;
      pev_go = millis();
    }
  }
  else if (go_current_sequean == 3)
  {
    Serial.print("go sequean 3");
    Serial.println(count_yaw_switch);
    if (pev_swd != controller_data.SWD)
    {
      go_current_sequean++;
    }
    if (current_yaw_swtich && not(triggered))
    {
      count_yaw_switch++;
      triggered = true;
      pev_go = millis();
    }

    else if (not(current_yaw_swtich))
    {
      triggered = false;
    }

    if (count_yaw_switch == 1)
    {
      if (confirm_go_deg < 180)
      {
        if (current_millis - pev_go >= 0)
        {
          servo3_target = stop;
          go_current_sequean++;
          pev_go = millis();
        }
      }
      else
      {
        if (current_millis - pev_go >= 90)
        {
          servo3_target = stop;
          go_current_sequean++;
          pev_go = millis();
        }
      }
    }
  }
  else if (go_current_sequean == 4)
  {
    Serial.print("go sequean 4 ");
    servo3_target = stop;
    go_current_sequean++;
    pev_go = millis();
  }
  else if (go_current_sequean == 5)
  {
    Serial.println("go sequean 5");
    servo3_target = stop;
    go_first_loop = true;
    go_current_sequean = 0;
    heading_current = confirm_go_deg;
    count_yaw_switch = 0;
  }
}

void home_positon(boolean runtime)
{
  // first turn left for 1s; if not touch switch --> turn right for 2s  if touch button stop turn servo

  if (home_first_loop && not(runtime) && current_yaw_swtich)
  {
    Serial.println("Start homing sequean");
    home_first_loop = false;
    pev_homing = millis();
    servo3_target = max_ccw;
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
    servo3_target = stop;
    home_current_sequean = 7;
  }
  else if (home_current_sequean == -1)
  {
    if (current_millis - pev_homing >= 100)
    {
      home_current_sequean = 1;
    }
    pev_homing = millis();
  }
  else if (home_current_sequean == 1)
  {
    Serial.println("homing sequean 1");
    servo3_target = max_cw;
    home_current_sequean++;
    pev_homing = millis();
  }
  else if (home_current_sequean == 2)
  {
    Serial.println("homing sequean 2");
    if (current_millis - pev_homing >= 600)
    {
      home_current_sequean++;
    }
  }
  else if (home_current_sequean == 3)
  {
    Serial.println("homing sequean 3");
    servo3_target = stop;
    pev_homing = millis();
    home_current_sequean++;
  }
  else if (home_current_sequean == 4)
  {
    Serial.println("homing sequean 4");
    if (current_millis - pev_homing >= 400)
    {
      home_current_sequean++;
    }
  }
  else if (home_current_sequean == 5)
  {
    Serial.println("homing sequean 5");
    servo3_target = max_ccw;
    pev_homing = millis();
    home_current_sequean++;
  }
  else if (home_current_sequean == 6)
  {
    Serial.println("homing sequean 6");
    if (current_millis - pev_homing >= 1200)
    {
      home_current_sequean++;
    }
  }
  else if (home_current_sequean == 7)
  {
    Serial.println("homing sequean 7");
    servo3_target = stop;
    home_first_loop = true;
    home_current_sequean = 0;
    heading_current = 0;
    count_yaw_switch = 0;
  }
}

void switch_runtime()
{
  if (digitalRead(arm_yaw_switch) == 0 && first_touch)
  {
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
    if (current_millis - pev_switch >= 60)
    {
      current_yaw_swtich = false;
    }
  }
}

void get_sequen(float x_sq, float y_sq, int go_sq)
{
  x_get = x_sq;
  y_get = y_sq;
  yaw_get = go_sq;
  get_current_sequean = 1;
  trigger_manula = false;
}
void get_runtime()
{
  if (get_current_sequean == 0)
  {
  }
  else if (get_current_sequean == 1)
  {
    yaw_go(yaw_get, false);
    x = x_get;
    y = y_get;
    grip_target = close_grip;
    get_current_sequean++;
  }
  else if (get_current_sequean == 2)
  {
    if (heading_current == yaw_get)
    {
      get_current_sequean++;
      pev_get = millis();
    }
  }
  else if (get_current_sequean == 3)
  {
    if (current_millis - pev_get >= 200)
    {
      get_current_sequean++;
    }
  }
  else if (get_current_sequean == 4)
  {
    y = y_get - 6;
    get_current_sequean++;
    pev_get = millis();
  }
  else if (get_current_sequean == 5)
  {
    if (current_millis - pev_get >= 1000)
    {
      get_current_sequean++;
    }
  }
  else if (get_current_sequean == 6)
  {
    grip_target = map(controller_data.VRA, 0, 180, min_grip, max_grip);
    get_current_sequean++;
    pev_get = millis();
  }
  else if (get_current_sequean == 7)
  {
    if (grip_current == grip_target)
    {
      get_current_sequean++;
      pev_get = millis();
    }
  }
  else if (get_current_sequean == 8)
  {
    y = y_get + 2;
    get_current_sequean++;
    pev_get = millis();
  }
  else if (get_current_sequean == 9)
  {
    if (current_millis - pev_get >= 1500)
    {
      get_current_sequean++;
    }
  }
  else if (get_current_sequean == 10)
  {
    yaw_return(false);
    get_current_sequean++;
  }
  else if (get_current_sequean == 11)
  {
    if (heading_current == 0)
    {
      get_current_sequean++;
      home_positon(false);
      pev_get = millis();
    }
  }
  else if (get_current_sequean == 12)
  {
    if (current_millis - pev_get >= 1000)
    {
      home_positon(false);
      get_current_sequean = 0;
      trigger_manula = true;
    }
  }
}

void store_sequen(float x_sq, float y_sq, int go_sq)
{
  x_store = x_sq;
  y_store = y_sq;
  yaw_store = go_sq;
  store_current_sequean = 1;
  trigger_manula = false;
}
void store_runtime()
{
  if (store_current_sequean == 0)
  {
  }
  else if (store_current_sequean == 1)
  {
    yaw_go(yaw_store, false);
    x = x_store;
    y = y_store + 2;
    store_current_sequean++;
  }
  else if (store_current_sequean == 2)
  {
    if (heading_current == yaw_store)
    {
      y = y_store - 2;
      store_current_sequean++;
      pev_store = millis();
    }
  }
  else if (store_current_sequean == 3)
  {
    if (current_millis - pev_store >= 1500)
    {
      store_current_sequean++;
      pev_swd = controller_data.SWD;
    }
  }
  else if (store_current_sequean == 4)
  {
    if (pev_swd != controller_data.SWD)
    {
      store_current_sequean++;
      drive_enable = true;
    }
    base_rotate();
    drive_enable = false;
  }
  else if (store_current_sequean == 5)
  {
    grip_target = close_grip;
    store_current_sequean++;
  }
  else if (store_current_sequean == 6)
  {
    if (grip_current == close_grip)
    {
      store_current_sequean++;
      pev_store = millis();
    }
  }
  else if (store_current_sequean == 7)
  {
    y = y_store;
    store_current_sequean++;
    pev_store = millis();
  }
  else if (store_current_sequean == 8)
  {
    if (current_millis - pev_store >= 1500)
    {
      store_current_sequean++;
    }
  }
  else if (store_current_sequean == 9)
  {
    yaw_return(false);
    store_current_sequean++;
  }
  else if (store_current_sequean == 10)
  {
    if (heading_current == 0)
    {
      store_current_sequean++;
      home_positon(false);
      pev_store = millis();
    }
  }
  else if (store_current_sequean == 11)
  {
    if (current_millis - pev_store >= 1000)
    {
      home_positon(false);
      trigger_manula = true;
      store_current_sequean = 0;
    }
  }
}
