
#pragma once

uint8_t readByte(uint8_t address, uint8_t subAddress);
void readBytes(uint8_t address, uint8_t subAddress, uint8_t count, uint8_t * dest);
uint32_t MS5637Read(uint8_t CMD, uint8_t OSR);  // temperature data read

