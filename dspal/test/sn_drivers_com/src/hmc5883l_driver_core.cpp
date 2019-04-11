/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#define HMC5883L_I2C_ADDRESS 0b0011110
#define MMX5883L_I2C_ADDRESS 0b00110000	// 0x30


static int hmc5883l_verify_id()
{
  uint8_t buf[3] = {255,255,255};

  if (i2c_read_reg(file_des,10,buf,3) != 0) { snav_error_print("%s: HMC5883L failed to read ID!!!",DRIVER_NAME); return -1; }

  snav_info_print("%s: HMC5883L: verify id: read %d %d %d, expected %d %d %d", DRIVER_NAME,
             buf[0],buf[1],buf[2],0b01001000,0b00110100,0b00110011);
  if ((buf[0] == 0b01001000) && (buf[1] == 0b00110100) && (buf[2] == 0b00110011))
  {
    return 0;
  }

  return -2;
}


static int mmx5883l_verify_id()
{
  uint8_t buf[3] = {255,255,255};

  if (i2c_read_reg(file_des,0b0101111,buf,1) != 0) { snav_error_print("%s: HMC5883L failed to read ID!!!",DRIVER_NAME); return -1; }

  snav_info_print("%s: ======rrd===== MMC HMC5883: verify id: read %d %d %d, expected %d %d %d", DRIVER_NAME,
             //buf[0],buf[1],buf[2],0b01001000,0b00110100,0b00110011);
             buf[0],0b0001100);
 // if ((buf[0] == 0b01001000) && (buf[1] == 0b00110100) && (buf[2] == 0b00110011))//0x48 0x34 0x33
 if (buf[0] == 0b0001100)//
  {
    return 0;
  }

  return -2;
}


static int hmc5883l_configure_sensor()
{

  int ret1 = i2c_write_check_reg(file_des, 0x00, 0b01111000); //8 sample average, 75hz
  snav_info_print("%s   hmc5883l_configure_sensor   ret1 = %d",DRIVER_NAME,ret1);
  int ret2 = i2c_write_check_reg(file_des, 0x01, 0x00);       //largest gain
    snav_info_print("%s   hmc5883l_configure_sensor   ret2 = %d",DRIVER_NAME,ret2);
  int ret3 = i2c_write_check_reg(file_des, 0x02, 0x00);       //continuos operation
    snav_info_print("%s   hmc5883l_configure_sensor   ret3 = %d",DRIVER_NAME,ret3);

  if (ret1 || ret2 || ret3) return -1;
  else                      return  0;
}


static int mmx_5883l_configure_sensor()
{
	int ret1 = i2c_write_single_reg(file_des, 0x09, 0x00);

//  int ret1 = i2c_write_check_reg(file_des, 0x09, 0x00); //set 100hz
  snav_info_print("%s   hmc5883l_configure_sensor   ret1 = %d",DRIVER_NAME,ret1);
  int ret2 = i2c_write_single_reg(file_des, 0x08, 0x08); //set/reset  (0x08,0x08)
    snav_info_print("%s   hmc5883l_configure_sensor   ret2 = %d",DRIVER_NAME,ret2);
	usleep(10000);
  int ret3 = i2c_write_single_reg(file_des, 0x08, 0x01);       //continuos operation
    snav_info_print("%s   hmc5883l_configure_sensor   ret3 = %d",DRIVER_NAME,ret3);
	usleep(10000);
  if (ret1 || ret2 || ret3) return -1;
  else                      return  0;
}


static int hmc5883l_init()
{
  uint32_t mag_slave_addr_hmc = HMC5883L_I2C_ADDRESS;
  uint32_t mag_slave_addr_mmx = MMX5883L_I2C_ADDRESS;
  mag_data_scale = (1.0 / 1370.0) * 100.0;    //1370 LSb/Gauss, 1Gauss = 100uT  HMC5883L

  snav_info_print("%s: Setting i2c slave address hmc=0x%02X  or mmx=0x%02X, baud rate %lu",DRIVER_NAME,mag_slave_addr_hmc,mag_slave_addr_mmx, mag_bit_rate);

  //set the slave config
  if (i2c_slave_config(file_des, mag_slave_addr_hmc, mag_bit_rate, mag_trx_timeout_us) == 0){
	snav_info_print("%s: i2c_slave_config  file_des = %d mag_slave_addr_hmc = 0x%02X mag_bit_rate = %lu mag_trx_timeout_us = %lu ",DRIVER_NAME,mag_slave_addr_hmc, mag_bit_rate);
  
  }else if(i2c_slave_config(file_des, mag_slave_addr_mmx, mag_bit_rate, mag_trx_timeout_us) == 0)	  {
	snav_info_print("%s: i2c_slave_config  file_des = %d mag_slave_addr_mmx = 0x%02X mag_bit_rate = %lu mag_trx_timeout_us = %lu ",DRIVER_NAME,mag_slave_addr_mmx, mag_bit_rate);
  
  }else{
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,mag_bit_rate);
    return -1;
  }


  if (hmc5883l_verify_id()==0){
	  snav_info_print("%s: HMC5883L verify ID OK!", DRIVER_NAME);
	
	if (hmc5883l_configure_sensor()!=0)  {
    snav_error_print("%s: HMC5883L config failed!", DRIVER_NAME);
    return -3;
  }
	return 0;
  }else if(mmx5883l_verify_id()==0){
	   snav_info_print("%s: MMX5883L verify ID OK!", DRIVER_NAME);

	if (mmx_5883l_configure_sensor()!=0)  {
    snav_error_print("%s: HMC5883L config failed!", DRIVER_NAME);
    return -3;
  }
	return 0;
  }else{
    snav_error_print("%s: HMC5883L verify ID failed!", DRIVER_NAME);
    return -2;
  }

	snav_info_print("%s: fhmc5883l_verify_id ",DRIVER_NAME);


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

  if (ret != 0)
  {
    snav_error_print("%s: HMC5883L: read failed", DRIVER_NAME);
    return -2;
  }

// -8  -371  -289
// -8  -371  -289   255 248 254 223 254 141"
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

  //snav_info_print("%s  rrd : mag %5d %5d %5d",DRIVER_NAME, mx,my,mz);
  //snav_info_print("%s  rrd: mag %5d %5d %5d   %3d %3d %3d %3d %3d %3d", DRIVER_NAME ,mx,my,mz, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);

  return 0;
}

static int mmx5883l_read_data()
{
  //set the slave config each time, just in case
  if (i2c_slave_config(file_des, mag_slave_addr, mag_bit_rate, mag_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,mag_bit_rate);
    return -1;
  }

  uint64_t time_read_start  = snav_get_time_1us();
  uint8_t buf[6];
  uint8_t mt_read[6];

   usleep(10000);
	 

  
  int ret = i2c_read_reg(file_des,0x00, buf, 6);
  if (ret != 0)
  {
    snav_error_print("%s: HMC5883L: read failed", DRIVER_NAME);
    return -2;
  }
  uint16_t mx = (uint16_t)(buf[1] << 8 | buf[0]);
  uint16_t my = (uint16_t)(buf[3] << 8 | buf[2]); //ordering of y and z axes is swapped
  uint16_t mz = (uint16_t)(buf[5] << 8 | buf[4]);

/*
	float mx_scaled = mx * mag_data_scale;
	float my_scaled = my * mag_data_scale;
	float mz_scaled = mz * mag_data_scale;*/


  float mx_scaled = (float)(mx-32768)/4096 * 100;
  float my_scaled = (float)(my-32768)/4096 * 100;
  float mz_scaled = (float)(mz-32768)/4096 * 100;

  pthread_mutex_lock(&mag_lock);

  mag_read_counter++;
  mag_fresh_data = 1;
  mag_read_time  = time_read_start;
  mag_data_x     =  mx_scaled;
  mag_data_y     =  my_scaled;
  mag_data_z     =  mz_scaled;

  i2c_write_single_reg(file_des, 0x08, 0x08);
	usleep(1000);

	int rets = i2c_write_single_reg(file_des, 0x08, 0x01); //TM_M

  pthread_mutex_unlock(&mag_lock);

  //snav_info_print("%s  rrd : mag %5d %5d %5d",DRIVER_NAME, mx,my,mz);
  //snav_info_print("%s  rrd : mag %f %f %f",DRIVER_NAME, mag_data_x,mag_data_y,mag_data_z);
  //snav_info_print("%s  rrd: mag %5d %5d %5d   %3d %3d %3d %3d %3d %3d", DRIVER_NAME ,mx,my,mz, buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);

  return 0;
}
