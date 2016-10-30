#include "../includes/MPU9250_Config.h"
#include <stdint.h>
//#include <EEPROM.h>
union float_to_byte {
  uint8_t bytes[sizeof(float)];
  float f;
};

MPU9250Config::MPU9250Config() {
  this->read_config();
}

void MPU9250Config::setMagBias(float bias[], bool save = true) {
  for(int x = 0 ; x < 3; x++) {
      magBias[x] = bias[x];
  }
  if(save) this->write_config();
}
void MPU9250Config::setMagBiasScale(float scale[], bool save = true) {
  for(int x = 0 ; x < 3; x++) {
      magBiasScale[x] = scale[x];
  }
  if(save) this->write_config();
}
void MPU9250Config::getMagBias(float arr[]) {
  for(int x = 0; x < 3; x++) {
    arr[x] = magBias[x];
  }
}
void MPU9250Config::getMagBiasScale(float arr[]) {
  for(int x = 0; x < 3; x++) {
    arr[x] = magBiasScale[x];
  }
}

void MPU9250Config::read_config() {

  // EEPROM.begin(512);
  
  uint32_t offset = 0;

  float_to_byte ftb;
  
  for(uint8_t y = 0; y < 3; y++) {
    for(uint32_t x = 0; x < sizeof(float); x++) {
      // ftb.bytes[x] = EEPROM.read(offset + x);
    }
    magBias[y] = ftb.f;
    magBias[y] = 0.0f;
    offset += sizeof(float);
  }

  for(uint8_t y = 0; y < 3; y++) {
    for(uint32_t x = 0; x < sizeof(float); x++) {
      // ftb.bytes[x] = EEPROM.read(offset + x);
    }
    magBiasScale[y] = ftb.f;
    magBiasScale[y] = 0.0f;
    offset += sizeof(float);
  }

  // EEPROM.end();
}

void MPU9250Config::write_config() {
  // EEPROM.begin(512);

  uint32_t offset = 0;

  float_to_byte ftb;

  for(uint8_t y = 0; y < 3; y++) {
    ftb.f = magBias[y];
    for(uint32_t x = 0; x < sizeof(float); x++) {
      // EEPROM.write(offset + x, ftb.bytes[x]);
    }
    offset += sizeof(float);
  }

  for(uint8_t y = 0; y < 3; y++) {
    ftb.f = magBiasScale[y];
    for(uint32_t x = 0; x < sizeof(float); x++) {
      // EEPROM.write(offset + x, ftb.bytes[x]);
    }
    offset += sizeof(float);
  }

  // EEPROM.end();
}