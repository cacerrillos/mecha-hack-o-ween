#ifndef _MPU9250_Config_H
#define _MPU9250_Config_H

class MPU9250Config {
  private:
    float magBias[3];
    float magBiasScale[3];
  public:
    MPU9250Config();
    void read_config();
    void write_config();
    void setMagBias(float[], bool);
    void setMagBiasScale(float[], bool);
    void getMagBias(float[]);
    void getMagBiasScale(float[]);

};
#endif