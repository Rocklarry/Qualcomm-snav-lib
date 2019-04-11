/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#define HMC5883L_I2C_ADDRESS 0b0011110//0x1E


static int hmc5883l_verify_id()
{
  uint8_t buf[3] = {255,255,255};

  if (i2c_read_reg(file_des,10,buf,3) != 0) { snav_error_print("%s: HMC5883L failed to read ID!!!",DRIVER_NAME); return -1; }

  snav_info_print("%s: HMC5883L: verify id: read %d %d %d, expected %d %d %d", DRIVER_NAME,
             buf[0],buf[1],buf[2],0b01001000,0b00110100,0b00110011);
  if ((buf[0] == 0b01001000) && (buf[1] == 0b00110100) && (buf[2] == 0b00110011))//0x48 0x34 0x33
  {
    return 0;
  }

  return -2;
}


static int hmc5883l_configure_sensor()
{
  int ret1 = i2c_write_check_reg(file_des, 0x00, 0b01111000); //8 sample average, 75hz  0x78
  int ret2 = i2c_write_check_reg(file_des, 0x01, 0x00);       //largest gain
  int ret3 = i2c_write_check_reg(file_des, 0x02, 0x00);       //continuos operation

  if (ret1 || ret2 || ret3) return -1;
  else                      return  0;
}


static int hmc5883l_init()
{
  mag_slave_addr = HMC5883L_I2C_ADDRESS;
  mag_data_scale = (1.0 / 1370.0) * 100.0;    //1370 LSb/Gauss, 1Gauss = 100uT  HMC5883L

  snav_info_print("%s: Setting i2c slave address 0x%02X, baud rate %lu",DRIVER_NAME,mag_slave_addr, mag_bit_rate);

  //set the slave config
  if (i2c_slave_config(file_des, mag_slave_addr, mag_bit_rate, mag_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,mag_bit_rate);
    return -1;
  }

  if (hmc5883l_verify_id()!=0)
  {
    snav_error_print("%s: HMC5883L verify ID failed!", DRIVER_NAME);
    return -2;
  }

  if (hmc5883l_configure_sensor()!=0)
  {
    snav_error_print("%s: HMC5883L config failed!", DRIVER_NAME);
    return -3;
  }

  return 0;
}



static int hmc5883l_read_data()
{
  //set the slave config each time, just in case
  if (i2c_slave_config(file_des, mag_slave_addr, mag_bit_rate, mag_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,mag_bit_rate);
    return -1;
  }

  uint64_t time_read_start  = snav_get_time_1us();
  uint8_t buf[6];
  int ret = i2c_read_reg(file_des,0x03, buf, 6);
 // IIC_Read_Device(MEMSIC_MAG_ADDRESS,6,MMC5883MA_REG_DATA,buffer);
 /*

IIC_Read_Device(MEMSIC_MAG_ADDRESS,6,0x00,buffer);

	mxyz[0] = (u16)(buffer[1] << 8 | buffer[0]);	
	mxyz[1] = (u16)(buffer[3] << 8 | buffer[2]);			
	mxyz[2] = (u16)(buffer[5] << 8 | buffer[4]); 
 
 */

  if (ret != 0)
  {
    snav_error_print("%s: HMC5883L: read failed", DRIVER_NAME);
    return -2;
  }

  int16_t mx = ((uint16_t)buf[0]) << 8 | buf[1];
  int16_t mz = ((uint16_t)buf[2]) << 8 | buf[3]; //ordering of y and z axes is swapped
  int16_t my = ((uint16_t)buf[4]) << 8 | buf[5];

  float mx_scaled = mx * mag_data_scale;
  float my_scaled = my * mag_data_scale;
  float mz_scaled = mz * mag_data_scale;

  pthread_mutex_lock(&mag_lock);

  mag_read_counter++;
  mag_fresh_data = 1;
  mag_read_time  = time_read_start;
  mag_data_x     = mx_scaled;
  mag_data_y     = my_scaled;
  mag_data_z     = mz_scaled;

  pthread_mutex_unlock(&mag_lock);

  //snav_info_print("%s: mag %5d %5d %5d",DRIVER_NAME, mx,my,mz);
  //snav_info_print("%s: mag %5d %5d %5d   %3d %3d %3d %3d %3d %3d", DRIVER_NAME ,mx,my,mz, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);

  return 0;
}
