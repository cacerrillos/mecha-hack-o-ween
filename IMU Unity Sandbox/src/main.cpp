/*
 * main.cpp
 *
 *  Created on: Oct 6, 2016
 *      Author: Carlos
 */

#include <iostream>
#include <stdio.h>
#include <bcm2835.h>
#include "boost/asio.hpp"
#include <string>
#include <sstream>
#include <fstream>
#include <signal.h>

#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/json_parser.hpp>

#include <boost/lexical_cast.hpp>

#include "../includes/MPU9250.h"
#include "../includes/MPU9250_Config.h"

using namespace std;
using namespace boost::asio;

namespace pt = boost::property_tree;

void setup_imu(bool calibrate_mag);
void send_udp(string s);
void open_udp();
void close_udp();

#define MPU9250_POWER_PIN RPI_V2_GPIO_P1_07

int flag = 0;

MPU9250 *imu;
MPU9250Config* config;

//#define DEBUG

InterfaceEnum interface = SPI;

void my_function(int sig){ // can be called asynchronously
  flag = 1; // set flag
}

ofstream myfile;

void endcon() {
  //myfile.close();
  close_udp();
  imu->closeAK8963();
  bcm2835_spi_end();
  bcm2835_close();
}

int main() {
  signal(SIGINT, my_function);
  config = new MPU9250Config();
#ifdef DEBUG
  bcm2835_set_debug(1);
  printf("Running BCM2835 in DEBUG mode!\n");
#endif
  
  if (!bcm2835_init()) {
    printf("Failed to init BCM2835!\n");
    return 1;
  }
  bcm2835_gpio_fsel(MPU9250_POWER_PIN, BCM2835_GPIO_FSEL_OUTP);
#ifdef __arm__
  cout << "ARM" << endl;
#else
  cout << "NOT ARM" << endl;
#endif
  if(interface == I2C) {
    if(!bcm2835_i2c_begin()){
      printf("bcm2835_i2c_begin failed. Are you running as root??\n");
      return 1;
    }
    bcm2835_i2c_setClockDivider(BCM2835_I2C_CLOCK_DIVIDER_2500);
    //BCM2835_I2C_CLOCK_DIVIDER_2500 100 khz i2c
  } else if(interface == SPI) {
    if(!bcm2835_spi_begin()){
      printf("bcm2835_spi_begin failed. Are you running as root??\n");
      return 1;
    }
    bcm2835_spi_setBitOrder(BCM2835_SPI_BIT_ORDER_MSBFIRST);
    bcm2835_spi_setDataMode(BCM2835_SPI_MODE3);
    bcm2835_spi_setClockDivider(BCM2835_SPI_CLOCK_DIVIDER_16);
    bcm2835_spi_chipSelect(BCM2835_SPI_CS0);
    bcm2835_spi_setChipSelectPolarity(BCM2835_SPI_CS0, LOW);
  }


  setup_imu(false);
  unsigned long long last_d;
  last_d = bcm2835_st_read();
  unsigned int counter = 0;
  unsigned long long this_t;
  //myfile.open("data.csv");
  open_udp();
  while(true) {
    imu->read_accel_gyro();
    this_t = bcm2835_st_read();
    // myfile <<
    //                   imu->accelerometer_data[0] << ", " <<
    //                   imu->accelerometer_data[1] << ", " <<
    //                   imu->accelerometer_data[2] << ", " <<
    //                   imu->gyroscope_data[0] << ", " <<
    //                   imu->gyroscope_data[1] << ", " <<
    //                   imu->gyroscope_data[2] << endl;
    // pt::ptree root;
    // root.put("t", this_t);
    // root.put("a.x", imu->accelerometer_data[0]);
    // root.put("a.y", imu->accelerometer_data[1]);
    // root.put("a.z", imu->accelerometer_data[2]);
    // root.put("g.x", imu->gyroscope_data[0]);
    // root.put("g.y", imu->gyroscope_data[1]);
    // root.put("g.z", imu->gyroscope_data[2]);
    // std::stringstream ss;
    // pt::write_json(ss, root);
    send_udp(boost::lexical_cast<string>(this_t) + ", "
             + boost::lexical_cast<string>(imu->accelerometer_data[0]) + ", "
             + boost::lexical_cast<string>(imu->accelerometer_data[1]) + ", "
             + boost::lexical_cast<string>(imu->accelerometer_data[2]));
    usleep(100);
    counter++;
    
    if(this_t - last_d >= 1000000) {
      printf("%f Hz\n", (double)counter / (this_t - last_d) * 1000000);
      counter = 0;
      last_d = bcm2835_st_read();
    }
    //printf("Accel: %f %f %f\n", imu->accelerometer_data[0], imu->accelerometer_data[1], imu->accelerometer_data[2]);

    //send_udp(ss.str());
    //send_udp(boost::lexical_cast<string>(imu->accelerometer_data[0]) + " " + boost::lexical_cast<string>(imu->accelerometer_data[1]) + " " + boost::lexical_cast<string>(imu->accelerometer_data[2]));
    //bcm2835_delay(500);
    if(flag){
      printf("Quitting!\n");
      endcon();
      return 0;
    }
  }

  while(true) {
    bcm2835_gpio_write(MPU9250_POWER_PIN , LOW);
    usleep(1000000);

    bcm2835_gpio_write(MPU9250_POWER_PIN, HIGH);
    usleep(1000000);
  }
  return 0;
  

}

void setup_imu(bool calibrate_mag) {
  bcm2835_gpio_write(MPU9250_POWER_PIN, LOW);
  usleep(100000);

  bcm2835_gpio_write(MPU9250_POWER_PIN, HIGH);

  usleep(250000);
  delete imu;
  imu = new MPU9250(SPI);
  printf("Starting IMU!\n");

  uint8_t mpu = imu->whoami();
  printf("\nMPU: %02x\n", mpu);
  if(mpu != 0x71) {
    usleep(1000000);
    setup_imu(calibrate_mag);
    return;
  }
  printf("Running Self Test on MPU9250!\n");
  printf("Self Test: %s \n",(imu->self_test_accel_gyro(imu->SelfTest) ? "PASSED" : "FAILED"));


  printf("Acceleration trim: %f %f %f\n", imu->SelfTest[0], imu->SelfTest[1], imu->SelfTest[2]);

  printf("Gyration trim: %f %f %f\n", imu->SelfTest[3], imu->SelfTest[4], imu->SelfTest[5]);

  printf("Attempting to Calibrate MPU9250!\n");

  imu->calibrateMPU9250(imu->gyroBias, imu->accelBias); // Calibrate gyro and accelerometers, load biases in bias registers

  printf("Acceleration bias (mg): %d %d %d\n", (int)(1000*imu->accelBias[0]), (int)(1000*imu->accelBias[1]),(int)(1000*imu->accelBias[2]));

  printf("Gyration bias (o/s): %f %f %f\n", imu->gyroBias[0], imu->gyroBias[1], imu->gyroBias[2]);

  printf("Attempting to initialize MPU9250!\n");

  imu->initMPU9250();
  printf("Initialized MPU9250!\n");

  uint8_t mag = imu->AK8963_whoami();
  
  printf("MAGN: %02x\n", mag);
  if(mag != 0x48) {
    setup_imu(calibrate_mag);
    return;
  }

  imu->initAK8963(imu->magCalibration);//init mag

  printf("Mag sensitivity adjustment values: %f %f %f\n", imu->magCalibration[0], imu->magCalibration[1], imu->magCalibration[2]);

  if(calibrate_mag) {
    uint32_t sample_count = 512;
    float bias[3];
    float scale[3];
    float ** pre_calib = new float*[sample_count];
    for(uint32_t i = 0; i < sample_count; i++) {
      pre_calib[i] = new float[3];
    }
    imu->magcalMPU9250(bias, scale, sample_count, pre_calib);
    imu->my_test();
    config->setMagBias(bias, false);
    config->setMagBiasScale(bias, false);

    config->write_config();

    for(uint32_t i = 0; i < sample_count; i++) {
      delete[] pre_calib[i];
    }
    delete[] pre_calib;
  }
  float magBias[3];
  config->getMagBias(magBias);

  printf("Bias: %f %f %f\n", magBias[0], magBias[1], magBias[2]);

  float magBiasScale[3];
  config->getMagBiasScale(magBiasScale);

  printf("Scale: %f %f %f\n", magBiasScale[0], magBiasScale[1], magBiasScale[2]);

  config->getMagBias(imu->magBias);
  config->getMagBiasScale(imu->magBiasScale);


// imu->magBias[0] = 10.788576f;
// imu->magBias[1] = 71.923843f;
// imu->magBias[2] = -194.580933f;
// imu->magBiasScale[0] = 1.067725f;
// imu->magBiasScale[1] = 1.006986f;
//     imu->magBiasScale[2] = 0.934259f;

    /*

    Bias: 206.7810363770, 188.8000793457, -141.2003326416
Scale: 1.1039844751, 0.9709401727, 0.9396194816

*/
}
io_service *io_s;
ip::udp::socket *sock;
ip::udp::endpoint remote_endpoint;

void open_udp() {
  io_s = new io_service();
  sock = new ip::udp::socket(*io_s);
  

  sock->open(ip::udp::v4());

  remote_endpoint = ip::udp::endpoint(ip::address::from_string("192.168.0.42"), 7777);

}

void close_udp() {
  sock->close();
  delete sock;
  delete io_s;
}

void send_udp(string s) {
  
  

  boost::system::error_code err;
  sock->send_to(buffer(s, s.length()), remote_endpoint, 0, err);

  

}
