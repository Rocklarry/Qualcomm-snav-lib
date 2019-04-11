/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#define HMC5883L_I2C_ADDRESS 0b00110000	// 0x30


static int hmc5883l_verify_id()
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
  mag_slave_addr = HMC5883L_I2C_ADDRESS;
  mag_data_scale = (1.0 / 1370.0) * 100.0; //1370 LSb/Gauss, 1Gauss = 100uT  HMC5883L

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
    snav_error_print("%s: rrd HMC5883L config failed!", DRIVER_NAME);
    return -3;
  }

  return 0;
}


#define FILTERNUM 24
float fAccX[FILTERNUM];
float fAccY[FILTERNUM];
float fAccZ[FILTERNUM];

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
  uint8_t mt_read[6];

	 


  //i2c_read_reg(file_des, 0x07, mt_read,1);

 // while(mt_read[0] && 0x01 != 0x01 ){
   usleep(10000);
  //  i2c_read_reg(file_des, 0x07, mt_read,1);
 // }

	//snav_info_print("%s   **************  ret2 = %d",DRIVER_NAME,mt_read[0]);
	 

  
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
		float AccsumX = 0;
        float AccsumY = 0;
        float AccsumZ = 0;

		int i;
         for( i=0;i<FILTERNUM-1;i++)
         {
                   fAccX[i] = fAccX[i+1];
                   fAccY[i] = fAccY[i+1];
                   fAccZ[i] = fAccZ[i+1];
         }

         fAccX[FILTERNUM-1] = mx_scaled;
         fAccY[FILTERNUM-1] = my_scaled;
         fAccZ[FILTERNUM-1] = mz_scaled;

         for( i=0;i<FILTERNUM;i++)
         {
                   AccsumX += fAccX[i];
                   AccsumY += fAccY[i];
                   AccsumZ += fAccZ[i];
         }
         mx_scaled = AccsumX / FILTERNUM;
         my_scaled = AccsumY / FILTERNUM;
         mz_scaled = AccsumZ / FILTERNUM;
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
