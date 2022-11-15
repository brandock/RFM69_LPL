### Minimal & partial copy of LowPowerLabs RFM69 library for AVR-DB microcontrollers

See original and full LowPowerLabs library here: [https://github.com/LowPowerLab/RFM69](https://github.com/LowPowerLab/RFM69)

Ultimately it would be good to implement AVR-DB support within the original library, this might be relatively easy by using Arduino SPI rather than implementing SPI internally as I have done here. This partial copy has been the result of my effort to try and understand the original LowPowerLabs library implementation whilst trying to provide basic LPL packet format and tx/rx + ack support for the avr-db micro as used in the EmonTx4. This may well get replaced by a more integrated version that is closer to the original library in due time. Im not trying to do anything clever here!
