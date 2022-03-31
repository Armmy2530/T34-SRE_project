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

const uint64_t address = 0xF0F0F0F0E1LL;
HardwareSerial &RCibus = Serial1;

struct Mydata
{
  byte voltage;
  byte radioactive;
  byte connection;
};

CytronMD M1(PWM_PWM, A1_pin, A2_pin);
CytronMD M2(PWM_PWM, B1_pin, B2_pin);
RF24 radio(CE_pin, CSN_pin); // CE, CSN
Mydata data;

void setup()
{
  Serial.begin(115200);
  RF_setup(address, "Tx");
  pinMode(RA_pin, INPUT);
  radioactive_setup(RA_pin);
  ibusRC_setup(RCibus);
}

void loop()
{
  voltage = 20;
  RA = radioactive_getdata();
  Serial_test();
  data.voltage = voltage;
  data.radioactive = RA;
  data.connection = connection;
  if (RA != -1)
  {
    radio.write(&data, sizeof(Mydata));
  }
  delay(10);
}

void remote_contorl()
{
}

void Serial_test()
{
  if (RA != -1)
  {
    Serial.print("voltage:");
    Serial.print(voltage);
    Serial.print(",");
    Serial.print("radioactive:");
    Serial.print(RA);
    Serial.print(",");
    Serial.print("connection:");
    Serial.println(connection);
  }
}
