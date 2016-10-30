#ifdef ARDUINO
#include <ESP.h>
#include <SPI.h>
#include <Wire.h>

#endif

#ifdef __arm__

#include <bcm2835.h>
#include <unistd.h>
#endif





#define MPU9250_ADDRESS 0x68  // Device address when ADO = 0
#define AK8963_ADDRESS 0x0C   //  Address of magnetometer

#include <math.h>
#include "../includes/MPU9250.h"

//-----------------------------------------------------------------------------------------------

MPU9250::MPU9250() {
  Gscale = GFS_2000DPS;
  Ascale = AFS_16G;
  Mscale = MFS_16BITS;

  this->Mmode = MMODE_CONTINUOUS_2;
  this->magBias[0] = 0;
  this->magBias[1] = 0;
  this->magBias[2] = 0;
  this->magBiasScale[0] = 1;
  this->magBiasScale[1] = 1;
  this->magBiasScale[2] = 1;
  this->gyroBias[0] = 0;
  this->gyroBias[1] = 0;
  this->gyroBias[2] = 0;
  this->accelBias[0] = 0;
  this->accelBias[1] = 0;
  this->accelBias[2] = 0;
  this->magCalibration[0] = 0;
  this->magCalibration[1] = 0;
  this->magCalibration[2] = 0;
}

unsigned int MPU9250::WriteReg(uint8_t icAddress, uint8_t WriteAddr, uint8_t WriteData) {
#ifdef ARDUINO
  Wire.beginTransmission(icAddress);  // Initialize the Tx buffer
  Wire.write(WriteAddr);           // Put slave register address in Tx buffer
  Wire.write(WriteData);                 // Put data in Tx buffer
  Wire.endTransmission();           // Send the Tx buffer
#endif
#ifdef __arm__

  char tx[2] = {WriteAddr, WriteData};
  //unsigned char rx[2] = {0};
  bcm2835_i2c_setSlaveAddress(icAddress);
  bcm2835_i2c_write(tx, 2);

  //bcm2835_spi_transfernb((char*)tx, (char*)rx, 2);
  //SPIdev::transfer("/dev/spidev0.1", tx, rx, 2);

  //return rx[1];
#endif
  return 0;
}

//-----------------------------------------------------------------------------------------------

unsigned int  MPU9250::ReadReg(uint8_t icAddress, uint8_t WriteAddr, uint8_t WriteData = 0) {
#ifdef ARDUINO
  uint8_t data; // `data` will store the register data
  Wire.beginTransmission(icAddress);         // Initialize the Tx buffer
  Wire.write(WriteAddr);                  // Put slave register address in Tx buffer
  Wire.endTransmission(false);             // Send the Tx buffer, but send a restart to keep connection alive
  Wire.requestFrom(icAddress, (uint8_t) 1);  // Read one byte from slave register address
  data = Wire.read();                      // Fill Rx buffer with result
  return (unsigned int)data;
#endif
#ifdef __arm__
  char WriteAddr_c = WriteAddr;
  char data;
  bcm2835_i2c_setSlaveAddress(icAddress);
  //bcm2835_i2c_write(tx, 2);
  bcm2835_i2c_write_read_rs(&WriteAddr_c, 1, &data, 1);
  return data;
  //return WriteReg(MPU9250_ADDRESS, WriteAddr | READ_FLAG, WriteData);
#endif

}

//-----------------------------------------------------------------------------------------------
void MPU9250::ReadRegs(uint8_t icAddress, uint8_t ReadAddr, uint8_t *ReadBuf, unsigned int Bytes) {
#ifdef ARDUINO
  Wire.beginTransmission(icAddress);   // Initialize the Tx buffer
  Wire.write(ReadAddr);            // Put slave register address in Tx buffer
  Wire.endTransmission(false);       // Send the Tx buffer, but send a restart to keep connection alive
  uint8_t i = 0;
  Wire.requestFrom(icAddress, Bytes);  // Read bytes from slave register address
  while (Wire.available()) {
    ReadBuf[i++] = Wire.read();
  }         // Put read results in the Rx buffer
#endif
#ifdef __arm__
  bcm2835_i2c_setSlaveAddress(icAddress);
  char ReadAddr_c = ReadAddr;
  bcm2835_i2c_write_read_rs(&ReadAddr_c, 1, (char*)ReadBuf, Bytes);
  /*
  unsigned int  i = 0;

    unsigned char tx[255] = {0};
    unsigned char rx[255] = {0};

    tx[0] = ReadAddr | READ_FLAG;
    bcm2835_spi_transfernb((char*)tx, (char*)rx, Bytes + 1);
    //SPIdev::transfer("/dev/spidev0.1", tx, rx, Bytes + 1);

      for(i=0; i<Bytes; i++)
        ReadBuf[i] = rx[i + 1];

      usleep(50);
      */
#endif
}

/*-----------------------------------------------------------------------------------------------
                                TEST CONNECTION
usage: call this function to know if SPI and MPU9250 are working correctly.
returns true if mpu9250 answers
-----------------------------------------------------------------------------------------------*/

bool MPU9250::testConnection() {
  unsigned int response;

  response=WriteReg(MPU9250_ADDRESS, MPUREG_WHOAMI|READ_FLAG, 0x00);
  unsigned int mag;
  mag = AK8963_whoami();
  return (response == 0x71 && mag == 0x48);
}

/*-----------------------------------------------------------------------------------------------
                                    INITIALIZATION
usage: call this function at startup, giving the sample rate divider (raging from 0 to 255) and
low pass filter value; suitable values are:
BITS_DLPF_CFG_256HZ_NOLPF2
BITS_DLPF_CFG_188HZ
BITS_DLPF_CFG_98HZ
BITS_DLPF_CFG_42HZ
BITS_DLPF_CFG_20HZ
BITS_DLPF_CFG_10HZ
BITS_DLPF_CFG_5HZ
BITS_DLPF_CFG_2100HZ_NOLPF
returns 1 if an error occurred
-----------------------------------------------------------------------------------------------*/


bool MPU9250::self_test_accel_gyro(float* destination) {
   uint8_t rawData[6] = {0, 0, 0, 0, 0, 0};
   uint8_t selfTest[6];
   int16_t gAvg[3], aAvg[3], aSTAvg[3], gSTAvg[3];
   float factoryTrim[6];
   uint8_t FS = 0;

   WriteReg(MPU9250_ADDRESS, MPUREG_SMPLRT_DIV, 0x00);    // Set gyro sample rate to 1 kHz
   WriteReg(MPU9250_ADDRESS, MPUREG_CONFIG, 0x02);        // Set gyro sample rate to 1 kHz and DLPF to 92 Hz
   WriteReg(MPU9250_ADDRESS, MPUREG_GYRO_CONFIG, 1<<FS);  // Set full scale range for the gyro to 250 dps
   WriteReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG_2, 0x02); // Set accelerometer rate to 1 kHz and bandwidth to 92 Hz
   WriteReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG, 1<<FS); // Set full scale range for the accelerometer to 2 g

  for( int ii = 0; ii < 200; ii++) {  // get average current values of gyro and acclerometer

    ReadRegs(MPU9250_ADDRESS, MPUREG_ACCEL_XOUT_H, &rawData[0], 6);        // Read the six raw data registers into data array
    aAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    ReadRegs(MPU9250_ADDRESS, MPUREG_GYRO_XOUT_H, &rawData[0], 6);       // Read the six raw data registers sequentially into data array
    gAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average current readings
    aAvg[ii] /= 200;
    gAvg[ii] /= 200;
  }

// Configure the accelerometer for self-test
  WriteReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG, 0xE0); // Enable self test on all three axes and set accelerometer range to +/- 2 g
  WriteReg(MPU9250_ADDRESS, MPUREG_GYRO_CONFIG,  0xE0); // Enable self test on all three axes and set gyro range to +/- 250 degrees/s
  delay(25);  // Delay a while to let the device stabilize

  for( int ii = 0; ii < 200; ii++) {  // get average self-test values of gyro and acclerometer
    ReadRegs(MPU9250_ADDRESS, MPUREG_ACCEL_XOUT_H, &rawData[0],6);  // Read the six raw data registers into data array
    aSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    aSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    aSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;

    ReadRegs(MPU9250_ADDRESS, MPUREG_GYRO_XOUT_H, &rawData[0],6);  // Read the six raw data registers sequentially into data array
    gSTAvg[0] += (int16_t)(((int16_t)rawData[0] << 8) | rawData[1]) ;  // Turn the MSB and LSB into a signed 16-bit value
    gSTAvg[1] += (int16_t)(((int16_t)rawData[2] << 8) | rawData[3]) ;
    gSTAvg[2] += (int16_t)(((int16_t)rawData[4] << 8) | rawData[5]) ;
  }

  for (int ii =0; ii < 3; ii++) {  // Get average of 200 values and store as average self-test readings
    aSTAvg[ii] /= 200;
    gSTAvg[ii] /= 200;
  }

  // Configure the gyro and accelerometer for normal operation
  WriteReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG, 0x00);
  WriteReg(MPU9250_ADDRESS, MPUREG_GYRO_CONFIG,  0x00);
  delay(25);  // Delay a while to let the device stabilize

  // Retrieve accelerometer and gyro factory Self-Test Code from USR_Reg
  selfTest[0] = ReadReg(MPU9250_ADDRESS, MPUREG_SELF_TEST_X, 0x00); // X-axis accel self-test results
  selfTest[1] = ReadReg(MPU9250_ADDRESS, MPUREG_SELF_TEST_Y, 0x00); // Y-axis accel self-test results
  selfTest[2] = ReadReg(MPU9250_ADDRESS, MPUREG_SELF_TEST_Z, 0x00); // Z-axis accel self-test results
  selfTest[3] = ReadReg(MPU9250_ADDRESS, MPUREG_XG_OFFS_TC, 0x00);  // X-axis gyro self-test results
  selfTest[4] = ReadReg(MPU9250_ADDRESS, MPUREG_YG_OFFS_TC, 0x00);  // Y-axis gyro self-test results
  selfTest[5] = ReadReg(MPU9250_ADDRESS, MPUREG_ZG_OFFS_TC, 0x00);  // Z-axis gyro self-test results

  // Retrieve factory self-test value from self-test code reads
  factoryTrim[0] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[0] - 1.0) )); // FT[Xa] factory trim calculation
  factoryTrim[1] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[1] - 1.0) )); // FT[Ya] factory trim calculation
  factoryTrim[2] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[2] - 1.0) )); // FT[Za] factory trim calculation
  factoryTrim[3] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[3] - 1.0) )); // FT[Xg] factory trim calculation
  factoryTrim[4] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[4] - 1.0) )); // FT[Yg] factory trim calculation
  factoryTrim[5] = (float)(2620/1<<FS)*(pow( 1.01 , ((float)selfTest[5] - 1.0) )); // FT[Zg] factory trim calculation

  // Report results as a ratio of (STR - FT)/FT; the change from Factory Trim of the Self-Test Response
  // To get percent, must multiply by 100
  for (int i = 0; i < 3; i++) {
    destination[i]   = 100.0*((float)(aSTAvg[i] - aAvg[i]))/factoryTrim[i];   // Report percent differences
    destination[i+3] = 100.0*((float)(gSTAvg[i] - gAvg[i]))/factoryTrim[i+3]; // Report percent differences
  }
  for(int i = 0; i < 6; i++) {
#ifdef __arm__
    if((destination[i] >= 0 ? 1 : -1) * destination[i]  >= 14.0f) {
#else
    if(abs(destination[i]) >= 14.0f) {
#endif
      return false;
    }
  }
  return true;
}

void MPU9250::calibrateMPU9250(float * dest1, float * dest2) {
  uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
  uint16_t ii, packet_count, fifo_count;
  int32_t gyro_bias[3]  = {0, 0, 0}, accel_bias[3] = {0, 0, 0};

  // reset device
  WriteReg(MPU9250_ADDRESS, MPUREG_PWR_MGMT_1, 0b10000000); // Write a one to bit 7 reset bit; toggle reset device
  delay(100);

  // get stable time source; Auto select clock source to be PLL gyroscope reference if ready
  // else use the internal oscillator, bits 2:0 = 001
  WriteReg(MPU9250_ADDRESS, MPUREG_PWR_MGMT_1, 0x01);
  WriteReg(MPU9250_ADDRESS, MPUREG_PWR_MGMT_2, 0x00);
  delay(200);

  // Configure device for bias calculation
  WriteReg(MPU9250_ADDRESS, MPUREG_INT_ENABLE, 0x00);   // Disable all interrupts
  WriteReg(MPU9250_ADDRESS, MPUREG_FIFO_EN, 0x00);      // Disable FIFO
  WriteReg(MPU9250_ADDRESS, MPUREG_PWR_MGMT_1, 0x00);   // Turn on internal clock source
  WriteReg(MPU9250_ADDRESS, MPUREG_I2C_MST_CTRL, 0x00); // Disable I2C master
  WriteReg(MPU9250_ADDRESS, MPUREG_USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
  WriteReg(MPU9250_ADDRESS, MPUREG_USER_CTRL, 0x0C);    // Reset FIFO and DMP
  delay(15);

  // Configure MPU6050 gyro and accelerometer for bias calculation
  WriteReg(MPU9250_ADDRESS, MPUREG_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
  WriteReg(MPU9250_ADDRESS, MPUREG_SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
  WriteReg(MPU9250_ADDRESS, MPUREG_GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
  WriteReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

  uint16_t  gyrosensitivity  = 131;   // = 131 LSB/degrees/sec
  uint16_t  accelsensitivity = 16384;  // = 16384 LSB/g

  // Configure FIFO to capture accelerometer and gyro data for bias calculation
  WriteReg(MPU9250_ADDRESS, MPUREG_USER_CTRL, 0x40);   // Enable FIFO
  WriteReg(MPU9250_ADDRESS, MPUREG_FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
  delay(40); // accumulate 40 samples in 40 milliseconds = 480 bytes

  // At end of sample accumulation, turn off FIFO sensor read
  WriteReg(MPU9250_ADDRESS, MPUREG_FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
  ReadRegs(MPU9250_ADDRESS, MPUREG_FIFO_COUNTH, &data[0], 2); // read FIFO sample count
  fifo_count = ((uint16_t)data[0] << 8) | data[1];
  packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging

  for (ii = 0; ii < packet_count; ii++) {
    int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
    ReadRegs(MPU9250_ADDRESS, MPUREG_FIFO_R_W, &data[0], 12); // read data for averaging
    accel_temp[0] = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
    accel_temp[1] = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
    accel_temp[2] = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
    gyro_temp[0]  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
    gyro_temp[1]  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
    gyro_temp[2]  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

    accel_bias[0] += (int32_t) accel_temp[0]; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
    accel_bias[1] += (int32_t) accel_temp[1];
    accel_bias[2] += (int32_t) accel_temp[2];
    gyro_bias[0]  += (int32_t) gyro_temp[0];
    gyro_bias[1]  += (int32_t) gyro_temp[1];
    gyro_bias[2]  += (int32_t) gyro_temp[2];

  }
  accel_bias[0] /= (int32_t) packet_count; // Normalize sums to get average count biases
  accel_bias[1] /= (int32_t) packet_count;
  accel_bias[2] /= (int32_t) packet_count;
  gyro_bias[0]  /= (int32_t) packet_count;
  gyro_bias[1]  /= (int32_t) packet_count;
  gyro_bias[2]  /= (int32_t) packet_count;

  if(accel_bias[2] > 0L) {accel_bias[2] -= (int32_t) accelsensitivity;}  // Remove gravity from the z-axis accelerometer bias calculation
  else {accel_bias[2] += (int32_t) accelsensitivity;}

  // Construct the gyro biases for push to the hardware gyro bias registers, which are reset to zero upon device startup
  data[0] = (-gyro_bias[0]/4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
  data[1] = (-gyro_bias[0]/4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
  data[2] = (-gyro_bias[1]/4  >> 8) & 0xFF;
  data[3] = (-gyro_bias[1]/4)       & 0xFF;
  data[4] = (-gyro_bias[2]/4  >> 8) & 0xFF;
  data[5] = (-gyro_bias[2]/4)       & 0xFF;

  // Push gyro biases to hardware registers
  WriteReg(MPU9250_ADDRESS, MPUREG_XG_OFFS_USRH, data[0]);
  WriteReg(MPU9250_ADDRESS, MPUREG_XG_OFFS_USRL, data[1]);
  WriteReg(MPU9250_ADDRESS, MPUREG_YG_OFFS_USRH, data[2]);
  WriteReg(MPU9250_ADDRESS, MPUREG_YG_OFFS_USRL, data[3]);
  WriteReg(MPU9250_ADDRESS, MPUREG_ZG_OFFS_USRH, data[4]);
  WriteReg(MPU9250_ADDRESS, MPUREG_ZG_OFFS_USRL, data[5]);

  // Output scaled gyro biases for display in the main program
  dest1[0] = (float) gyro_bias[0]/(float) gyrosensitivity;
  dest1[1] = (float) gyro_bias[1]/(float) gyrosensitivity;
  dest1[2] = (float) gyro_bias[2]/(float) gyrosensitivity;

  // Construct the accelerometer biases for push to the hardware accelerometer bias registers. These registers contain
  // factory trim values which must be added to the calculated accelerometer biases; on boot up these registers will hold
  // non-zero values. In addition, bit 0 of the lower byte must be preserved since it is used for temperature
  // compensation calculations. Accelerometer bias registers expect bias input as 2048 LSB per g, so that
  // the accelerometer biases calculated above must be divided by 8.

  int32_t accel_bias_reg[3] = {0, 0, 0}; // A place to hold the factory accelerometer trim biases
  ReadRegs(MPU9250_ADDRESS, MPUREG_XA_OFFSET_H, &data[0], 2); // Read factory accelerometer trim values
  accel_bias_reg[0] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  ReadRegs(MPU9250_ADDRESS, MPUREG_YA_OFFSET_H, &data[0], 2);
  accel_bias_reg[1] = (int32_t) (((int16_t)data[0] << 8) | data[1]);
  ReadRegs(MPU9250_ADDRESS, MPUREG_ZA_OFFSET_H, &data[0], 2);
  accel_bias_reg[2] = (int32_t) (((int16_t)data[0] << 8) | data[1]);

  uint32_t mask = 1uL; // Define mask for temperature compensation bit 0 of lower byte of accelerometer bias registers
  uint8_t mask_bit[3] = {0, 0, 0}; // Define array to hold mask bit for each accelerometer bias axis

  for(ii = 0; ii < 3; ii++) {
  if((accel_bias_reg[ii] & mask)) mask_bit[ii] = 0x01; // If temperature compensation bit is set, record that fact in mask_bit
  }

  // Construct total accelerometer bias, including calculated average accelerometer bias from above
  accel_bias_reg[0] -= (accel_bias[0]/8); // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g (16 g full scale)
  accel_bias_reg[1] -= (accel_bias[1]/8);
  accel_bias_reg[2] -= (accel_bias[2]/8);

  data[0] = (accel_bias_reg[0] >> 8) & 0xFF;
  data[1] = (accel_bias_reg[0])      & 0xFF;
  data[1] = data[1] | mask_bit[0]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[2] = (accel_bias_reg[1] >> 8) & 0xFF;
  data[3] = (accel_bias_reg[1])      & 0xFF;
  data[3] = data[3] | mask_bit[1]; // preserve temperature compensation bit when writing back to accelerometer bias registers
  data[4] = (accel_bias_reg[2] >> 8) & 0xFF;
  data[5] = (accel_bias_reg[2])      & 0xFF;
  data[5] = data[5] | mask_bit[2]; // preserve temperature compensation bit when writing back to accelerometer bias registers

  // Apparently this is not working for the acceleration biases in the MPU-9250
  // Are we handling the temperature correction bit properly?
  // Push accelerometer biases to hardware registers
  WriteReg(MPU9250_ADDRESS, MPUREG_XA_OFFSET_H, data[0]);
  WriteReg(MPU9250_ADDRESS, MPUREG_XA_OFFSET_L, data[1]);
  WriteReg(MPU9250_ADDRESS, MPUREG_YA_OFFSET_H, data[2]);
  WriteReg(MPU9250_ADDRESS, MPUREG_YA_OFFSET_L, data[3]);
  WriteReg(MPU9250_ADDRESS, MPUREG_ZA_OFFSET_H, data[4]);
  WriteReg(MPU9250_ADDRESS, MPUREG_ZA_OFFSET_L, data[5]);

  // Output scaled accelerometer biases for display in the main program
  dest2[0] = (float)accel_bias[0]/(float)accelsensitivity;
  dest2[1] = (float)accel_bias[1]/(float)accelsensitivity;
  dest2[2] = (float)accel_bias[2]/(float)accelsensitivity;
}


void MPU9250::initMPU9250() {
  /*{0b10000000, MPUREG_PWR_MGMT_1},     // Reset Device - Disabled because it seems to corrupt initialisation of AK8963
        {0x01, MPUREG_PWR_MGMT_1, 0},     // Clock Source
        {0x00, MPUREG_PWR_MGMT_2, 0},     // Enable Acc & Gyro
        {low_pass_filter, MPUREG_CONFIG, 0},         // Use DLPF set Gyroscope bandwidth 184Hz, temperature bandwidth 188Hz
        {0x18, MPUREG_GYRO_CONFIG, 0},    // +-2000dps
        {0x08, MPUREG_ACCEL_CONFIG, 0},   // +-4G
        {0x09, MPUREG_ACCEL_CONFIG_2, 0}, // Set Acc Data Rates, Enable Acc LPF , Bandwidth 184Hz
        {0x30, MPUREG_INT_PIN_CFG, 0},    //
        {0x20, MPUREG_USER_CTRL, 0},       // I2C Master mode
        {0x0D, MPUREG_I2C_MST_CTRL, 0}, //  I2C configuration multi-master  IIC 400KHz
        {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR, 0},  //Set the I2C slave addres of AK8963 and set for write.
        {AK8963_CNTL2, MPUREG_I2C_SLV0_REG, 0}, //I2C slave 0 register address from where to begin data transfer
        {0x01, MPUREG_I2C_SLV0_DO, 0}, // Reset AK8963
        {0x81, MPUREG_I2C_SLV0_CTRL, 0},  //Enable I2C and set 1 byte
        {AK8963_CNTL1, MPUREG_I2C_SLV0_REG, 0}, //I2C slave 0 register address from where to begin data transfer
        {0x12, MPUREG_I2C_SLV0_DO, 0}, // Register value to continuous measurement in 16bit
        {0x81, MPUREG_I2C_SLV0_CTRL, 0}  //Enable I2C and set 1 byte */

  //WriteReg(MPU9250_ADDRESS, MPUREG_PWR_MGMT_1, 0x00); // Clear sleep mode bit (6), enable all sensors
  ////usleep(100000); // Wait for all registers to reset

  // get stable time source
  WriteReg(MPU9250_ADDRESS, MPUREG_PWR_MGMT_1, 0x00);
  delay(100);
  WriteReg(MPU9250_ADDRESS, MPUREG_PWR_MGMT_1, 0x01);  // Auto select clock source to be PLL gyroscope reference if ready else
  //usleep(200000);
  delay(200);

  // Configure Gyro and Thermometer
  // Disable FSYNC and set thermometer and gyro bandwidth to 41 and 42 Hz, respectively;
  // minimum delay time for this setting is 5.9 ms, which means sensor fusion update rates cannot
  // be higher than 1 / 0.0059 = 170 Hz
  // DLPF_CFG = bits 2:0 = 011; this limits the sample rate to 1000 Hz for both
  // With the MPU9250, it is possible to get gyro sample rates of 32 kHz (!), 8 kHz, or 1 kHz
  WriteReg(MPU9250_ADDRESS, MPUREG_CONFIG, 0b00000011);//011

  // Set sample rate = gyroscope output rate/(1 + SMPLRT_DIV)
  WriteReg(MPU9250_ADDRESS, MPUREG_SMPLRT_DIV, 0x04);  // Use a 200 Hz rate; a rate consistent with the filter update rate
                // determined inset in CONFIG above

  // Set gyroscope full scale range
  // Range selects FS_SEL and AFS_SEL are 0 - 3, so 2-bit values are left-shifted into positions 4:3
  uint8_t c = ReadReg(MPU9250_ADDRESS, MPUREG_GYRO_CONFIG, 0x00);
  //  writeRegister(GYRO_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  c = c & ~0x02; // Clear Fchoice bits [1:0]
  c = c & ~0x18; // Clear AFS bits [4:3]
  c = c | Gscale << 3; // Set full scale range for the gyro
  WriteReg(MPU9250_ADDRESS, MPUREG_GYRO_CONFIG, c); // Set full scale range for the gyro
  // writeRegister(GYRO_CONFIG, c | 0x00); // Set Fchoice for the gyro to 11 by writing its inverse to bits 1:0 of GYRO_CONFIG

  // Set accelerometer full-scale range configuration
  c = ReadReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG, 0x00);
  c = c & ~0x18;  // Clear AFS bits [4:3]
  c = c | Ascale << 3; // Set full scale range for the accelerometer
  //  writeRegister(ACCEL_CONFIG, c & ~0xE0); // Clear self-test bits [7:5]
  WriteReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG, c); // Set full scale range for the accelerometer

  // Set accelerometer sample rate configuration
  // It is possible to get a 4 kHz sample rate from the accelerometer by choosing 1 for
  // accel_fchoice_b bit [3]; in this case the bandwidth is 1.13 kHz
  c = ReadReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG_2, 0x00);
  c = c & ~0x0F; // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])
  c = c | 0x03;  // Set accelerometer rate to 1 kHz and bandwidth to 41 Hz
  WriteReg(MPU9250_ADDRESS, MPUREG_ACCEL_CONFIG_2, c); // Clear accel_fchoice_b (bit 3) and A_DLPFG (bits [2:0])

  // The accelerometer, gyro, and thermometer are set to 1 kHz sample rates,
  // but all these rates are further reduced by a factor of 5 to 200 Hz because of the SMPLRT_DIV setting

  // Configure Interrupts and Bypass Enable
  // Set interrupt pin active high, push-pull, hold interrupt pin level HIGH until interrupt cleared,
  // clear on read of INT_STATUS, and enable I2C_BYPASS_EN so additional chips
  // can join the I2C bus and all can be controlled by the Arduino as master
  //WriteReg(MPU9250_ADDRESS, MPUREG_INT_PIN_CFG, 0x22);
  //WriteReg(MPU9250_ADDRESS, MPUREG_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  //usleep(100000);

  //WriteReg(AK8963_ADDRESS, AK8963_CNTL2, 0x01);
  //I forgot, does this break init?
  //delay(100);
  WriteReg(MPU9250_ADDRESS, MPUREG_INT_PIN_CFG, 0x22);
  WriteReg(MPU9250_ADDRESS, MPUREG_INT_ENABLE, 0x01);  // Enable data ready (bit 0) interrupt
  //WriteReg(AK8963_ADDRESS, AK8963_CNTL1, 0b00010010);
  delay(100);
  /*
  {0x30, MPUREG_INT_PIN_CFG, 0},    //
          {0x20, MPUREG_USER_CTRL, 0},       // I2C Master mode
          {0x0D, MPUREG_I2C_MST_CTRL, 0}, //  I2C configuration multi-master  IIC 400KHz
          {AK8963_I2C_ADDR, MPUREG_I2C_SLV0_ADDR, 0},  //Set the I2C slave addres of AK8963 and set for write.
          {AK8963_CNTL2, MPUREG_I2C_SLV0_REG, 0}, //I2C slave 0 register address from where to begin data transfer
          {0x01, MPUREG_I2C_SLV0_DO, 0}, // Reset AK8963
          {0x81, MPUREG_I2C_SLV0_CTRL, 0},  //Enable I2C and set 1 byte
          {AK8963_CNTL1, MPUREG_I2C_SLV0_REG, 0}, //I2C slave 0 register address from where to begin data transfer
          {0x12, MPUREG_I2C_SLV0_DO, 0}, // Register value to continuous measurement in 16bit
          {0x81, MPUREG_I2C_SLV0_CTRL, 0}  //Enable I2C and set 1 byte */
}
/*-----------------------------------------------------------------------------------------------
                                ACCELEROMETER SCALE
usage: call this function at startup, after initialization, to set the right range for the
accelerometers. Suitable ranges are:
BITS_FS_2G
BITS_FS_4G
BITS_FS_8G
BITS_FS_16G
returns the range set (2,4,8 or 16)
-----------------------------------------------------------------------------------------------*/
void MPU9250::getMres() {
  switch (Mscale)
  {
  // Possible magnetometer scales (and their register bit settings) are:
  // 14 bit resolution (0) and 16 bit resolution (1)
    case MFS_14BITS:
          mRes = 10.*4912./8190.; // Proper scale to return milliGauss
          break;
    case MFS_16BITS:
          mRes = 10.*4912./32760.0; // Proper scale to return milliGauss
          break;
  }
}

void MPU9250::getGres() {
  switch (Gscale)
  {
  // Possible gyro scales (and their register bit settings) are:
  // 250 DPS (00), 500 DPS (01), 1000 DPS (10), and 2000 DPS  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case GFS_250DPS:
          gRes = 250.0/32768.0;
          break;
    case GFS_500DPS:
          gRes = 500.0/32768.0;
          break;
    case GFS_1000DPS:
          gRes = 1000.0/32768.0;
          break;
    case GFS_2000DPS:
          gRes = 2000.0/32768.0;
          break;
  }
}

void MPU9250::getAres() {
  switch (Ascale)
  {
  // Possible accelerometer scales (and their register bit settings) are:
  // 2 Gs (00), 4 Gs (01), 8 Gs (10), and 16 Gs  (11).
        // Here's a bit of an algorith to calculate DPS/(ADC tick) based on that 2-bit value:
    case AFS_2G:
          aRes = 2.0/32768.0;
          break;
    case AFS_4G:
          aRes = 4.0/32768.0;
          break;
    case AFS_8G:
          aRes = 8.0/32768.0;
          break;
    case AFS_16G:
          aRes = 16.0/32768.0;
          break;
  }
}
/*-----------------------------------------------------------------------------------------------
                                WHO AM I?
usage: call this function to know if SPI is working correctly. It checks the I2C address of the
mpu9250 which should be 104 when in SPI mode.
returns the I2C address (104)
-----------------------------------------------------------------------------------------------*/

unsigned int MPU9250::whoami() {
    unsigned int response;

    response = ReadReg(MPU9250_ADDRESS, MPUREG_WHOAMI);
    return response;
}


/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/

void MPU9250::read_acc()
{
  getAres();
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPU9250_ADDRESS, MPUREG_ACCEL_XOUT_H,response,6);

    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        accelerometer_data_raw[i] = data;
        accelerometer_data[i] = data * aRes;
    }

}

/*-----------------------------------------------------------------------------------------------
                                READ GYROSCOPE
usage: call this function to read gyroscope data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
-----------------------------------------------------------------------------------------------*/

void MPU9250::read_gyro()
{
  getGres();
    uint8_t response[6];
    int16_t bit_data;
    float data;
    int i;
    ReadRegs(MPU9250_ADDRESS, MPUREG_GYRO_XOUT_H,response,6);
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
        data=(float)bit_data;
        gyroscope_data_raw[i]=data;
        gyroscope_data[i] = data *gRes;
    }

}

/*-----------------------------------------------------------------------------------------------
                                READ TEMPERATURE
usage: call this function to read temperature data.
returns the value in C
-----------------------------------------------------------------------------------------------*/

void MPU9250::read_temp()
{
    uint8_t response[2];
    int16_t bit_data;
    float data;
    ReadRegs(MPU9250_ADDRESS, MPUREG_TEMP_OUT_H,response,2);

    bit_data=((int16_t)response[0]<<8)|response[1];
    data=(float)bit_data;
    //tempout - (roomtemp-offset)/sensitivity + 21
    //temperature=(data/340)+36.53;
    temperature=(data/333.87f) + 21.0;
}

/*-----------------------------------------------------------------------------------------------
                                READ ACCELEROMETER CALIBRATION
usage: call this function to read accelerometer data. Axis represents selected axis:
0 -> X axis
1 -> Y axis
2 -> Z axis
returns Factory Trim value
-----------------------------------------------------------------------------------------------*/

//-----------------------------------------------------------------------------------------------

uint8_t MPU9250::AK8963_whoami(){
    uint8_t response;

    response = ReadReg(AK8963_ADDRESS, WHO_AM_I_AK8963);

    return response;
}

//-----------------------------------------------------------------------------------------------
void MPU9250::closeAK8963() {
  WriteReg(AK8963_ADDRESS, AK8963_CNTL1, MMODE_OFF);
}
void MPU9250::initAK8963(float * destination) {
  // First extract the factory calibration for each magnetometer axis
  uint8_t rawData[3];  // x/y/z gyro calibration data stored here

  WriteReg(AK8963_ADDRESS, AK8963_CNTL1, MMODE_OFF);
  delay(10);
  WriteReg(AK8963_ADDRESS, AK8963_CNTL1, MMODE_FUSE_ROM_ACCESS);
  delay(10);
  ReadRegs(AK8963_ADDRESS, AK8963_ASAX, rawData, 3);

  //response=WriteReg(MPU9250_ADDRESS, MPUREG_I2C_SLV0_DO, 0x00);    //Read I2C
  for(int i=0; i<3; i++) {
    destination[i] = (float)(rawData[i] - 128)/256. + 1.;
    //magnetometer_ASA[i]=((data-128)/256+1)*Magnetometer_Sensitivity_Scale_Factor;
  }

  WriteReg(AK8963_ADDRESS, AK8963_CNTL1, MMODE_OFF);
  delay(10);

  WriteReg(AK8963_ADDRESS, AK8963_CNTL1, Mscale << 4 | this->Mmode);
  delay(10);
}
void MPU9250::my_test() {

}
void MPU9250::magcalMPU9250(float * dest1, float * dest2, int sample_count, float ** sample_out) {
  uint16_t ii = 0;
  int32_t mag_bias[3] = {0, 0, 0}, mag_scale[3] = {0, 0, 0};
  int16_t mag_max[3] = {0, 0, 0};
  int16_t mag_min[3] = {0x7FFF, 0x7FFF, 0x7FFF};
  int16_t mag_temp[3] = {0, 0, 0};

  //Serial.println("Mag Calibration: Wave device in a figure eight until done!\n");
  //usleep(3000000);

  for(ii = 0; ii < sample_count; ii++) {
  read_mag();
  //mag_temp = imu->magnetometer_data;  // Read the mag data
  mag_temp[0] = magnetometer_data_raw[0];
  mag_temp[1] = magnetometer_data_raw[1];
  mag_temp[2] = magnetometer_data_raw[2];
   sample_out[ii][0] = magnetometer_data_raw[0];
  sample_out[ii][1] = magnetometer_data_raw[1];
  sample_out[ii][2] = magnetometer_data_raw[2];
  //Serial.println("%f %f %f\n", imu->magnetometer_data_raw[0], imu->magnetometer_data_raw[1], imu->magnetometer_data_raw[2]);
  //Serial.println(ii, DEC);
  for (int jj = 0; jj < 3; jj++) {
    if(mag_temp[jj] > mag_max[jj]) mag_max[jj] = mag_temp[jj];
    if(mag_temp[jj] < mag_min[jj]) mag_min[jj] = mag_temp[jj];
    }
    delay(135);
  //usleep(135000);  // at 8 Hz ODR, new mag data is available every 125 ms
  }

  // Get hard iron correction
  mag_bias[0]  = (mag_max[0] + mag_min[0])/2;  // get average x mag bias in counts
  mag_bias[1]  = (mag_max[1] + mag_min[1])/2;  // get average y mag bias in counts
  mag_bias[2]  = (mag_max[2] + mag_min[2])/2;  // get average z mag bias in counts

  dest1[0] = (float) mag_bias[0]*mRes*magCalibration[0];  // save mag biases in G for main program
  dest1[1] = (float) mag_bias[1]*mRes*magCalibration[1];
  dest1[2] = (float) mag_bias[2]*mRes*magCalibration[2];

  // Get soft iron correction estimate
  mag_scale[0]  = (mag_max[0] - mag_min[0])/2;  // get average x axis max chord length in counts
  mag_scale[1]  = (mag_max[1] - mag_min[1])/2;  // get average y axis max chord length in counts
  mag_scale[2]  = (mag_max[2] - mag_min[2])/2;  // get average z axis max chord length in counts

  float avg_rad = mag_scale[0] + mag_scale[1] + mag_scale[2];
  avg_rad /= 3.0;

  dest2[0] = avg_rad/((float)mag_scale[0]);
  dest2[1] = avg_rad/((float)mag_scale[1]);
  dest2[2] = avg_rad/((float)mag_scale[2]);

  //Serial.println("Mag Calibration done!");
 }
//-----------------------------------------------------------------------------------------------

void MPU9250::read_mag(){
  getMres();

  /*
  magBias[0] = +470.;  // User environmental x-axis correction in milliGauss, should be automatically calculated
  magBias[1] = +120.;  // User environmental x-axis correction in milliGauss
  magBias[2] = +125.;  // User environmental x-axis correction in milliGauss
  */
    uint8_t response[7];
    int16_t bit_data;
    float data;
    int i;

    ReadRegs(AK8963_ADDRESS, AK8963_HXL, response, 7);
    //must start your read from AK8963A register 0x03 and read seven bytes so that upon read of ST2 register 0x09 the AK8963A will unlatch the data registers for the next measurement.
    for(i=0; i<3; i++) {
        bit_data=((int16_t)response[i*2+1]<<8)|response[i*2];
        data=(float)bit_data;
        magnetometer_data_raw[i] = data;
        magnetometer_data_unscaled[i] = ((float)data * mRes * magCalibration[i]);
        magnetometer_data[i] = ((float)data * mRes * magCalibration[i] - magBias[i])*magBiasScale[i];
        //magnetometer_data[i] = data * mRes - magBias[i];
        //magnetometer_data[i]=data*magnetometer_ASA[i];
    }
    magnetometer_status = response[6];
}

//-----------------------------------------------------------------------------------------------

void MPU9250::read_all() {
  getAres();
  getGres();
  getMres();

  uint8_t response[14];
  uint8_t response_mag[7];
  int16_t bit_data;
  float data;
  int i;


  ReadRegs(MPU9250_ADDRESS, MPUREG_ACCEL_XOUT_H,response,14);
  //Get accelerometer value
  for(i=0; i<3; i++) {
      bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
      data=(float)bit_data;
      accelerometer_data[i] = data * aRes;
      accelerometer_data_raw[i] = data;

  }
  //Get temperature
  bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
  data=(float)bit_data;
  temperature=((data-21)/333.87)+21;
  //Get gyroscope value
  for(i=4; i<7; i++) {
      bit_data=((int16_t)response[i*2]<<8)|response[i*2+1];
      data=(float)bit_data;
      gyroscope_data[i - 4] = data *gRes;
      gyroscope_data_raw[i - 4]=data;

  }

  ReadRegs(AK8963_ADDRESS, AK8963_HXL, response_mag, 7);

  this->magnetometer_status = response_mag[6];
  //Get Magnetometer value
  for(i=0; i<3; i++) {
      bit_data=((int16_t)response_mag[i*2+1]<<8)|response_mag[i*2];
      data=(float)bit_data;
      //magnetometer_data[i-7]=data*magnetometer_ASA[i-7];
      magnetometer_data[i] = ((float)data * mRes * magCalibration[i] - magBias[i])*magBiasScale[i];
      magnetometer_data_unscaled[i] = ((float)data * mRes * magCalibration[i]);

      magnetometer_data_raw[i] = data;

  }
}

/*-----------------------------------------------------------------------------------------------
                                         GET VALUES
usage: call this functions to read and get values
returns accel, gyro and mag values
-----------------------------------------------------------------------------------------------*/

void MPU9250::getMotion9(float *ax, float *ay, float *az, float *gx, float *gy, float *gz, float *mx, float *my, float *mz)
{
    read_all();
    *ax = accelerometer_data[0];
    *ay = accelerometer_data[1];
    *az = accelerometer_data[2];
    *gx = gyroscope_data[0];
    *gy = gyroscope_data[1];
    *gz = gyroscope_data[2];
    *mx = magnetometer_data[0];
    *my = magnetometer_data[1];
    *mz = magnetometer_data[2];
}

//-----------------------------------------------------------------------------------------------

void MPU9250::getMotion6(float *ax, float *ay, float *az, float *gx, float *gy, float *gz)
{
    read_acc();
    read_gyro();
    *ax = accelerometer_data[0];
    *ay = accelerometer_data[1];
    *az = accelerometer_data[2];
    *gx = gyroscope_data[0];
    *gy = gyroscope_data[1];
    *gz = gyroscope_data[2];
}

//------------------------------------------------------------------------------------------------
