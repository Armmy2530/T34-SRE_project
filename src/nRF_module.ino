void RF_setup(const uint64_t address, String mode)
{
    radio.begin();
    if (mode == "Tx")
    {
        radio.openWritingPipe(address);
        radio.setPALevel(RF24_PA_MIN);
        radio.setDataRate(RF24_250KBPS);
        radio.stopListening();
    }
    else if (mode == "Rx")
    {
        radio.openReadingPipe(0, address);
        radio.setPALevel(RF24_PA_MAX);
        radio.setDataRate(RF24_250KBPS);
        radio.startListening();
    }
    else
    {
        Serial.println("Error invaild RF_mode");
    }
}
