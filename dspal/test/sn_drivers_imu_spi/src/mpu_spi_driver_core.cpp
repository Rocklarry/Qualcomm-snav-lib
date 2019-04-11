/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
//#define SPI_DEV_PATH   "/dev/spi-1"
//#define SPI_INT_GPIO   65  // GPIO device for MPU data ready interrupt

#include <mpu9x50.h>
//#include "LowPassFilter2p.h"

#include <stdio.h>    
#include <sys/time.h> 

//static float tmp_z=0;



long getCurrentTime()    
{    
   struct timeval tv;    
   gettimeofday(&tv,NULL);    
   return tv.tv_sec * 1000 + tv.tv_usec / 1000;    
}   



int mpu_driver_init(struct mpu9x50_config * config)
{
  snav_info_print("%s: Attempting to initialize MPU driver.",DRIVER_NAME);
  int mpu_init_ret = mpu9x50_initialize(config);
  if (mpu_init_ret != 0)
  {
    snav_error_print("%s: MPU failed to initialize[%d]",DRIVER_NAME,mpu_init_ret);
    return -1;
  }

  snav_info_print("%s: MPU SPI initialized successfully",DRIVER_NAME);
  return 0;
}

int mpu_driver_close()
{
  int close_ret = mpu9x50_close();
  snav_info_print("%s: mpu close result: %d",DRIVER_NAME,close_ret);
  return close_ret;
}

int mpu_spi_read_data()
{
  //TODO: add reading / reporting MAG1

  struct mpu9x50_data sensor_data;
  int mpu_ret = mpu9x50_get_data(&sensor_data);

  int bad_mpu_data = (sensor_data.accel_raw[0] == 0) && (sensor_data.accel_raw[1] == 0) && (sensor_data.accel_raw[2] == 0) &&
                     (sensor_data.gyro_raw[0]  == 0) && (sensor_data.gyro_raw[1]  == 0) && (sensor_data.gyro_raw[2]  == 0) ;

  if( (mpu_ret!=0) || (bad_mpu_data))
  {
    //FIXME: don't print too often
    //snav_error_print("%s: Failed to read mpu data!",DRIVER_NAME);
    return -1;
  }

  float axf   = sensor_data.accel_raw[0]*sensor_data.accel_scaling/9.81;
  float ayf   = sensor_data.accel_raw[1]*sensor_data.accel_scaling/9.81;
  float azf   = sensor_data.accel_raw[2]*sensor_data.accel_scaling/9.81;
  float wxf   = sensor_data.gyro_raw[0]*sensor_data.gyro_scaling;
  float wyf   = sensor_data.gyro_raw[1]*sensor_data.gyro_scaling;
  float wzf   = sensor_data.gyro_raw[2]*sensor_data.gyro_scaling;

  pthread_mutex_lock(&imu_lock);

  imu_read_counter++;
  //imu_read_time  = time_read_start;
  imu_read_time  = sensor_data.timestamp;
  imu_fresh_data = 1;


	//LowPassFilter2p data_op(500,250);

	snav_info_print("%s: ===idea===  data_z = %lf time= %ld",DRIVER_NAME,azf,getCurrentTime());

	//tmp_z = data_op.apply(azf);

		axf = axf*0.8;
		ayf = ayf*0.8;


	if(azf>1)
		{
		azf = (azf-1)*0.8+1;
	}/*else if(azf<-1)
	{
		azf = (azf+1)*0.8-1;
	}*/

  ax   = axf;
  ay   = ayf;
  az   = azf;
  wx   = wxf;
  wy   = wyf;
  wz   = wzf;

   snav_info_print("%s: ***idea***  data_z = %lf time= %ld",DRIVER_NAME,az,getCurrentTime());
  temp = sensor_data.temperature;

  pthread_mutex_unlock(&imu_lock);

  //snav_info_print("===rrd=== IMU0_SPI: %+4.2f  %+4.2f %+4.2f %+4.2f   %+4.2f %+4.2f %+4.2f",temp,ax,ay,az,wx,wy,wz);

  return 0;
}
