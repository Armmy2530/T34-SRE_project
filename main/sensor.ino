unsigned long int oldtime = 0;
unsigned long int curtime = 0;
uint16_t count = 0;
uint16_t send_count = 0;

void radioactive_setup(int pin_in)
{
    attachInterrupt(digitalPinToInterrupt(pin_in), SignalCount, FALLING);
}
int radioactive_getdata(int pin_in)
{
    curtime = micros();
    if ((curtime - oldtime) > 1000000)
    {
        detachInterrupt(digitalPinToInterrupt(pin_in));
        oldtime = curtime;
        send_count = count;
        count = 0;
        attachInterrupt(digitalPinToInterrupt(pin_in), SignalCount, FALLING);
        return send_count;
    }
    else {
        return -1; 
    }
}

void voltage_setup(int pin_in)
{
    pinMode(pin_in,INPUT);
}
float voltage_getdata(int pin_in)
{
    return (float(analogRead(pin_in))/1023)*20;
}

void SignalCount()
{
    count++;
}
