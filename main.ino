#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <IBusBM.h>

int voltage = 0;
int RA = 0;
int connection = 1;

int RA_pin = 20;
int CE_pin = 7;
int CSN_pin = 8;

const uint64_t address = 0xF0F0F0F0E1LL;

struct Mydata
{
  byte voltage;
  byte radioactive;
  byte connection;
};

RF24 radio(CE_pin, CSN_pin); // CE, CSN
Mydata data;

void setup()
{
  Serial.begin(115200);
  RF_setup(address, "Tx");
  pinMode(RA_pin, INPUT);
  radioactive_setup(RA_pin);
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
