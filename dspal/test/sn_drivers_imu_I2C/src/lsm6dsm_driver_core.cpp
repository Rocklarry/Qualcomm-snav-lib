/*==============================================================================
 Copyright (c) 2016 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

//WARNING: the I2C ADDRESS will depend on state of address pins on IMU chip
//last bit of address can be 1 if SDO/SA0 pin is high
#define IMU_I2C_ADDRESS 0b1101010

#define LSM6DSM_I2C_ADDRESS_WHOAMI_ADDR 0x0F
#define LSM6DSM_I2C_ADDRESS_WHOAMI_VAL  0b01101010

#define M_PI  3.14159265358979323846


static int32_t lsm6dsm_config()
{
  snav_info_print("%s: configuring LSM6DSM sensor",DRIVER_NAME);

  //enable accelerometer, ODR = 416Hz, 16G range, LPF = ODR / 4
  int ret = i2c_write_check_reg(file_des, 0x10, 0b01100110); if (ret) return 0x10;

  //no additional accelerometer LPF
  ret = i2c_write_check_reg(file_des, 0x17, 0b00000000); if (ret) return 0x17;

  //enable gyro, ODR = 833Hz
  ret = i2c_write_check_reg(file_des, 0x11, 0b01111100); if (ret) return 0x11;

  //gyro LPF = 245Hz
  ret = i2c_write_check_reg(file_des, 0x15, 0b00000000); if (ret) return 0x15;

  usleep(10000);

  return 0;
}


int32_t lsm6dsm_detect()
{
  snav_info_print("%s: initializing LSM6DSM i2c driver",DRIVER_NAME);

  uint8_t reg;
  int ret = i2c_read_reg(file_des,LSM6DSM_I2C_ADDRESS_WHOAMI_ADDR, &reg, 1); //read the WHOAMI register (identifies the chip type)

  if (ret)
  {
    snav_error_print("%s: could not read WHOAMI register from LSM6DSM",DRIVER_NAME);
    return -1;
  }

  uint8_t whoami_exp = LSM6DSM_I2C_ADDRESS_WHOAMI_VAL;

  snav_info_print("%s: %d register is 0x%02X (expected 0x%02X)",
                  DRIVER_NAME, LSM6DSM_I2C_ADDRESS_WHOAMI_ADDR, reg, whoami_exp);

  if (reg == whoami_exp) return lsm6dsm_config();
  else return -1;
}

int32_t lsm6dsm_init()
{
  imu_slave_addr = IMU_I2C_ADDRESS;

  //set the slave config
  if (i2c_slave_config(file_des, imu_slave_addr, imu_bit_rate, imu_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,imu_bit_rate);
    return -1;
  }

  int ret;
  int ntry    = 3;
  int success = 0;

  while (ntry>0)
  {
    ntry--;

    ret = lsm6dsm_detect();
    if (ret == 0)
    {
      success = 1;
      break;
    }
  }

  if (success == 1) return 0;
  else              return -1;
}

int32_t lsm6dsm_read_data()
{
  uint8_t imu_data[14];

  //set the slave config
  if (i2c_slave_config(file_des, imu_slave_addr, imu_bit_rate, imu_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,imu_bit_rate);
    return -1;
  }

  uint64_t time_read_start  = snav_get_time_1us();

  if (i2c_read_reg(file_des,0x20, imu_data, 14) != 0)
  {
    snav_error_print("%s: could not read data from mpu using i2c",DRIVER_NAME);
    return -1;
  }

  int16_t tr,axr,ayr,azr,wxr,wyr,wzr;

  tr  = imu_data[1];  tr  <<= 8;  tr  |=imu_data[0];  //raw temperature
  wxr = imu_data[3];  wxr <<= 8;  wxr |=imu_data[2];  //raw gyro values
  wyr = imu_data[5];  wyr <<= 8;  wyr |=imu_data[4];
  wzr = imu_data[7];  wzr <<= 8;  wzr |=imu_data[6];
  axr = imu_data[9];  axr <<= 8;  axr |=imu_data[8];  //raw acceleration values
  ayr = imu_data[11]; ayr <<= 8;  ayr |=imu_data[10];
  azr = imu_data[13]; azr <<= 8;  azr |=imu_data[12];



  const float gyro_scale  = (2000.0/180*M_PI) / 32768.0; //2000 degrees max range
  const float accel_scale = 16.0 / 32768.0;              //16G max range

  float axf = axr * accel_scale;
  float ayf = ayr * accel_scale;
  float azf = azr * accel_scale;
  float wxf = wxr * gyro_scale;
  float wyf = wyr * gyro_scale;
  float wzf = wzr * gyro_scale;
  float tempf = 0;

  tempf = 25.0 + (tr) / 256.0;

  pthread_mutex_lock(&imu_lock);

  imu_read_counter++;
  imu_read_time  = time_read_start;
  imu_fresh_data = 1;

  ax     = axf;
  ay     = ayf;
  az     = azf;
  wx     = wxf;
  wy     = wyf;
  wz     = wzf;
  temp   = tempf;

  pthread_mutex_unlock(&imu_lock);


  //snav_info_print("%+5d  %+5d %+5d %+5d   %+5d %+5d %+5d",tr,axr,ayr,azr,wxr,wyr,wzr);
  //snav_info_print("%+4.2f  %+4.2f %+4.2f %+4.2f   %+4.2f %+4.2f %+4.2f",temp,ax,ay,az,wx,wy,wz);

  return 0;
}

int imu_read_data()
{
  return lsm6dsm_read_data();
}
