
#include "Arduino.h"
#include "WireReader.h"
#include "Wire.h"
#include "Components.h"
#include "Constants.h"

uint8_t readByte(uint8_t address, uint8_t subAddress) {
    uint8_t data; // `data` will store the register data
    Wire.beginTransmission(address);         // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.endTransmission(I2C_NOSTOP); // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
//  Wire.requestFrom(address, 1);  // Read one byte from slave register address
    Wire.requestFrom(address, (size_t) 1); // Read one byte from slave register address
    data = Wire.read();                      // Fill Rx buffer with result
    return data;                         // Return data read from slave register
}
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest)
{
    Wire.beginTransmission(address);   // Initialize the Tx buffer
    Wire.write(subAddress);           // Put slave register address in Tx buffer
    Wire.endTransmission(I2C_NOSTOP); // Send the Tx buffer, but send a restart to keep connection alive
    //  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    //        Wire.requestFrom(address, count);  // Read bytes from slave register address
    Wire.requestFrom(address, (size_t) count); // Read bytes from slave register address
    while (Wire.available()) {
        dest[i++] = Wire.read();
    }         // Put read results in the Rx buffer
}
uint32_t MS5637Read(uint8_t CMD, uint8_t OSR)  // temperature data read
{
    uint8_t data[3] = { 0, 0, 0 };
    Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    Wire.write(CMD | OSR);       // Put pressure conversion command in Tx buffer
    Wire.endTransmission(I2C_NOSTOP); // Send the Tx buffer, but send a restart to keep connection alive

    switch (OSR) {
    case ADC_256:
        delay(1);
        break;  // delay for conversion to complete
    case ADC_512:
        delay(3);
        break;
    case ADC_1024:
        delay(4);
        break;
    case ADC_2048:
        delay(6);
        break;
    case ADC_4096:
        delay(10);
        break;
    case ADC_8192:
        delay(20);
        break;
    }

    Wire.beginTransmission(MS5637_ADDRESS);  // Initialize the Tx buffer
    Wire.write(0x00);                       // Put ADC read command in Tx buffer
    Wire.endTransmission(I2C_NOSTOP); // Send the Tx buffer, but send a restart to keep connection alive
    uint8_t i = 0;
    Wire.requestFrom(MS5637_ADDRESS, 3); // Read three bytes from slave PROM address
    while (Wire.available()) {
        data[i++] = Wire.read();
    }               // Put read results in the Rx buffer
    return (uint32_t)(
            ((uint32_t) data[0] << 16) | (uint32_t) data[1] << 8 | data[2]); // construct PROM data for return to main program
}
