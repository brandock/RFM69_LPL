## A cut down version of the LowPowerLabs RFM69 library with AVRDB and ATmega328 compatibility

See original and full LowPowerLabs library here: [https://github.com/LowPowerLab/RFM69](https://github.com/LowPowerLab/RFM69)

Ultimately it would be good to implement AVR-DB support within the original library, this might be relatively easy by using Arduino SPI rather than implementing SPI internally as I have done here. This partial copy has been the result of my effort to try and understand the original LowPowerLabs library implementation whilst trying to provide basic LPL packet format and tx/rx + ack support for the avr-db micro as used in the EmonTx4. This may well get replaced by a more integrated version that is closer to the original library in due time. Im not trying to do anything clever here!

### API Documentation

**bool initialize(uint8_t freqBand, uint16_t ID, uint8_t networkID=1);**<br>

**void setAddress(uint16_t addr);**<br>

**void sleep();**<br>

**bool canSend();**<br>

**void send(uint16_t toAddress, const void* buffer, uint8_t bufferSize, bool requestACK=false);**<br>

**bool sendWithRetry(uint16_t toAddress, const void* buffer, uint8_t bufferSize, uint8_t retries=2, uint8_t retryWaitTime=RFM69_ACK_TIMEOUT);**<br>

**bool receiveDone();**<br>

**bool ACKReceived(uint16_t fromNodeID);**<br>

**bool ACKRequested();**<br>

**void sendACK(const void* buffer = "", uint8_t bufferSize=0);**<br>

**uint8_t retry_count();**<br>
    
**void encrypt (const char* key);**<br>

**int16_t readRSSI(bool forceTrigger=false);**<br>
    
**uint8_t readReg (uint8_t addr);**<br>

**void writeReg (uint8_t addr, uint8_t value);**<br>
    
**void setMode (uint8_t newMode);**<br>

**void select();**<br>

**void unselect();**<br>
    
**int receive ();**<br>
