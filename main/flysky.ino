#include <IBusBM.h>

IBusBM ibusRc;

void ibusRC_setup(HardwareSerial& ibusRcSerial)
{
  ibusRc.begin(ibusRcSerial);
}

// Read the number of a given channel and convert to the range provided.
// If the channel is off, return the default value
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = ibusRc.readChannel(channelInput);
  if (ch < 100)
    return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
}

// Red the channel and return a boolean value
bool redSwitch(byte channelInput, bool defaultValue)
{
  int intDefaultValue = (defaultValue) ? 100 : 0;
  int ch = readChannel(channelInput, 0, 100, intDefaultValue);
  return (ch > 50);
}

bool isinFreerange(int channelInput,float freerange){
  uint16_t ch = ibusRc.readChannel(channelInput);
  uint16_t percent = round(1500 * (freerange/100));
  uint16_t min_per = 1500 - percent;
  uint16_t max_per = 1500 + percent;
  // Serial.print("percent:");
  // Serial.print(percent);
  // Serial.print(" max:");
  // Serial.print(max_per);
  // Serial.print(" min:");
  // Serial.print(min_per);
  // Serial.print(" freerange:");
  if (ch < 100)
    return false;
  else if  (ch >= min_per && ch <= max_per){
    // Serial.println("TRUE");
    return true;
  }
  else
    return false;
}

int readChannel_freerange(byte channelInput, int minLimit, int maxLimit, int defaultValue,float freerange)
{
  uint16_t ch = ibusRc.readChannel(channelInput);
  uint16_t percent = round(1500 * (freerange/100));
  uint16_t min_per = 1500 - percent;
  uint16_t max_per = 1500 + percent;
  // Serial.print("percent:");
  // Serial.print(percent);
  // Serial.print(" max:");
  // Serial.print(max_per);
  // Serial.print(" min:");
  // Serial.print(min_per);
  // Serial.print(" freerange:");
  if (ch < 100)
    return defaultValue;
  if  (ch >= min_per && ch <= max_per){
    // Serial.println("TRUE");
    return defaultValue;
  }
  // Serial.println("False");
  return map(ch, 1000, 2000, minLimit, maxLimit);
}
