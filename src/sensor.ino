unsigned long int oldtime = 0;
unsigned long int curtime = 0;
uint16_t count = 0;
uint16_t send_count = 0;
int pin;

void radioactive_setup(int pin_in)
{
    attachInterrupt(digitalPinToInterrupt(pin_in), SignalCount, FALLING);
    pin = pin_in;
}
int radioactive_getdata()
{
    curtime = millis();
    if ((curtime - oldtime) > 1000)
    {
        detachInterrupt(digitalPinToInterrupt(pin));
        oldtime = curtime;
        send_count = count;
        count = 0;
        attachInterrupt(digitalPinToInterrupt(pin), SignalCount, FALLING);
        return send_count;
    }
    else {
        return -1;
    }
}

void SignalCount()
{
    count++;
}
