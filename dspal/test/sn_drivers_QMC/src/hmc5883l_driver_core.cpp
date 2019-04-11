/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#define HMC5883L_I2C_ADDRESS	 0x0d//7bit

#define HMC5883L_I2C_ID	 0x21  //读取id的地址


static int hmc5883l_verify_id()
{

 int ret1 = i2c_write_check_reg(file_des, 0x0a, 0x80); //8 sample average, 75hz
  snav_info_print("%s   hmc5883l_configure_sensor   ret1 = %d",DRIVER_NAME,ret1);


  uint8_t buf[3] = {255,255,255};
  if (i2c_read_reg(file_des,HMC5883L_I2C_ID,buf,1) != 0) { snav_error_print("%s: HMC5883L failed to read ID!!!",DRIVER_NAME); return -1; }

  snav_info_print("%s: HMC5883L: verify id: read %d", DRIVER_NAME,buf[0]);
  if (buf[0] == 0x03)
  {
    return 0;
  }
  return -2;
}

static int hmc5883l_configure_sensor()
{
/*
// 设置Mode寄存器 
		QMC5883L_WriteByte(0x0B, 0x01);	
		QMC5883L_WriteByte(0x20, 0x40);	
		QMC5883L_WriteByte(0x21, 0x01);	
		QMC5883L_WriteByte(0x09, 0x1D);	
*/
/*
 uint8_t buf[3] = {255,255,255};
  if (i2c_read_reg(file_des,HMC5883L_I2C_ID,buf,1) != 0) { snav_error_print("%s: HMC5883L failed to read ID!!!",DRIVER_NAME); }

  snav_info_print("%s: HMC5883L: verify id: read %d", DRIVER_NAME,buf[0]);*/

  int ret1 = i2c_write_check_reg(file_des, 0x0B, 0x01); //8 sample average, 75hz
  snav_info_print("%s   hmc5883l_configure_sensor   ret1 = %d",DRIVER_NAME,ret1);
/*
 uint8_t buf_t[3] = {255,255,255};
  if (i2c_read_reg(file_des,HMC5883L_I2C_ID,buf_t,1) != 0) { snav_error_print("%s: HMC5883L failed to read ID!!!",DRIVER_NAME); return -1; }

  snav_info_print("%s: HMC5883L: verify id teset : read %d", DRIVER_NAME,buf_t[0]);*/

  int ret2 = i2c_write_check_reg(file_des, 0x20, 0x40);       //largest gain
    snav_info_print("%s   hmc5883l_configure_sensor   ret2 = %d",DRIVER_NAME,ret2);
  int ret3 = i2c_write_check_reg(file_des, 0x21, 0x01);       //continuos operation
    snav_info_print("%s   hmc5883l_configure_sensor   ret3 = %d",DRIVER_NAME,ret3);
  int ret4 = i2c_write_check_reg(file_des, 0x09, 0x19);       //continuos operation
    snav_info_print("%s   hmc5883l_configure_sensor   ret4 = %d",DRIVER_NAME,ret4);

  if (ret1 || ret2 || ret3 || ret4) return -1;
  else                      return  0;
}


static int hmc5883l_init()
{
  mag_slave_addr = HMC5883L_I2C_ADDRESS;
  mag_data_scale = (1.0 / 3000.0) * 100.0;    //1370 LSb/Gauss, 1Gauss = 100uT  HMC5883L

  snav_info_print("%s: Setting i2c slave address = 0x%02X baud rate = %lu  mag_bit_rate = %lu",DRIVER_NAME,mag_slave_addr, mag_bit_rate);


  //set the slave config
  if (i2c_slave_config(file_des, mag_slave_addr, mag_bit_rate, mag_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,mag_bit_rate);
    return -1;
  }
//"SNAV_MAG0_HMC5883L_DRIVER: i2c_slave_config  file_des = 24 mag_slave_addr = 0x61A80 mag_bit_rate = 0 mag_trx_timeout_us = 0 "
	snav_info_print("%s: i2c_slave_config  file_des = %d mag_slave_addr = 0x%02X mag_bit_rate = %lu mag_trx_timeout_us = %lu ",DRIVER_NAME,mag_slave_addr, mag_bit_rate);



  if (hmc5883l_verify_id()!=0)
  {
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

/*
#define FILTER_N 12
float filter_buf_x[FILTER_N + 1];
float filter_x( float data) 
{
int i;
float filter_sum = 0;
filter_buf_x[FILTER_N] = data;
for(i = 0; i < FILTER_N; i++) 
{
filter_buf_x[i] = filter_buf_x[i + 1];
filter_sum += filter_buf_x[i];
}
return (float)(filter_sum / FILTER_N);
}

float filter_buf_y[FILTER_N + 1];
float filter_y( float data) 
{
int i;
float filter_sum = 0;
filter_buf_y[FILTER_N] = data;
for(i = 0; i < FILTER_N; i++) 
{
filter_buf_y[i] = filter_buf_y[i + 1];
filter_sum += filter_buf_y[i];
}
return (float)(filter_sum / FILTER_N);
}

float filter_buf_z[FILTER_N + 1];
float filter_z( float data) 
{
int i;
float filter_sum = 0;
filter_buf_z[FILTER_N] = data;
for(i = 0; i < FILTER_N; i++) 
{
filter_buf_z[i] = filter_buf_z[i + 1];
filter_sum += filter_buf_z[i];
}
return (float)(filter_sum / FILTER_N);
}

*/


#define FILTERNUM 36
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

  int ret = i2c_read_reg(file_des,0x00, buf, 6);

  if (ret != 0)
  {
    snav_error_print("%s: HMC5883L: read failed", DRIVER_NAME);
    return -2;
  }

  int16_t mx = (int16_t)(buf[1] << 8 | buf[0]);
  int16_t my = (int16_t)(buf[3] << 8 | buf[2]); //ordering of y and z axes is swapped
  int16_t mz = (int16_t)(buf[5] << 8 | buf[4]);

  //float mx_scaled = filter(mx * mag_data_scale);
  //float my_scaled = filter(my * mag_data_scale);
  //float mz_scaled = filter(mz * mag_data_scale);
  

  float mx_scaled = mx * mag_data_scale;
  float my_scaled = my * mag_data_scale;
  float mz_scaled = mz * mag_data_scale;
 
  pthread_mutex_lock(&mag_lock);

  mag_read_counter++;
  mag_fresh_data = 1;
  mag_read_time  = time_read_start;
/*
  mag_data_x     = filter_x(mx_scaled);
  mag_data_y     = filter_y(my_scaled);
  mag_data_z     = filter_z(mz_scaled);
*/

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
  
  mag_data_x     = mx_scaled;
  mag_data_y     = my_scaled;
  mag_data_z     = mz_scaled;

  pthread_mutex_unlock(&mag_lock);

  //snav_info_print("%s  rrd: mag %5d   %5d   %5d",DRIVER_NAME, mx,my,mz);
  //snav_info_print("%s  rrd: mag %5d   %5d   %5d", DRIVER_NAME ,mag_data_x,mag_data_y,mag_data_z);
  //snav_info_print("%s  rrd: mag %5d   %5d   %5d  ******* %3d   %3d   %3d   %3d   %3d   %3d", DRIVER_NAME ,mag_data_x,mag_data_y,mag_data_z,buf[0],buf[1],buf[2],buf[3],buf[4],buf[5]);

  return 0;
}
