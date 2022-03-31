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
