/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
static SnavDriverPortTypeEnum  imu_port_type = SNAV_DRIVER_PORT_TYPE_SPI;

static int32_t  imu_port           = 1;
static int32_t  imu_bit_rate       = 10000000;

static uint32_t imu_read_counter   = 0;
static uint32_t imu_fresh_data     = 0;
static uint64_t imu_read_time      = 0;
static int32_t  imu_blocking_read  = 0;

static float ax=0, ay =0, az=0, wx=0, wy=0, wz=0, temp = 0;

static pthread_mutex_t imu_lock    = PTHREAD_MUTEX_INITIALIZER;

static SnavDriverImuData imu_data_out;
static SnavDriverMagData mag_data_out;

int imu0_driver_set(int handle, SnavDriverSetEnum type, void *  data_ptr, uint32_t data_size)
{
  if (type == SNAV_DRIVER_SET_PORT_NUMBER)
  {
    imu_port = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated imu0_port param to %d",DRIVER_NAME, imu_port);
    return 0;
  }
  else if (type == SNAV_DRIVER_SET_BIT_RATE)
  {
    imu_bit_rate = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated imu0_bit_rate param to %d",DRIVER_NAME, imu_bit_rate);
    return 0;
  }
  else if (type == SNAV_DRIVER_SET_BLOCKING_IO)
  {
    imu_blocking_read = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated imu0 to use blocking IO=%d",DRIVER_NAME, imu_blocking_read);
    return 0;
  }
  return -1;
}


int imu0_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size)
{
  if (type == SNAV_DRIVER_GET_API_VERSION)
  {
    int str_len    = strlen(SNAV_DRIVERS_API_VERSION) + 1;
    if (str_len > data_size)
      str_len = data_size;
    memcpy(data_ptr,SNAV_DRIVERS_API_VERSION,str_len);
    return 0;
  }
  else if (type == SNAV_DRIVER_GET_DRIVER_INFO)
  {
    int str_len    = strlen(DRIVER_NAME) + 1;
    if (str_len > data_size)
      str_len = data_size;
    memcpy(data_ptr,DRIVER_NAME,str_len);
    return 0;
  }
  else if (type == SNAV_DRIVER_GET_ACCEL_GYRO0)
  {
    if (imu_blocking_read)
    {
      mpu_spi_read_data();
    }

    if (imu_fresh_data == 1)
    {
      pthread_mutex_lock(&imu_lock);

      imu_data_out.time        = imu_read_time;
      imu_data_out.cntr        = imu_read_counter;
      imu_data_out.accel_x     = ax;
      imu_data_out.accel_y     = ay;
      imu_data_out.accel_z     = az;
      imu_data_out.gyro_x      = wx;
      imu_data_out.gyro_y      = wy;
      imu_data_out.gyro_z      = wz;
      imu_data_out.temperature = temp;

      imu_fresh_data    = 0;

      pthread_mutex_unlock(&imu_lock);
    }

    int out_size = sizeof(imu_data_out);
    if (out_size > data_size)
      return -2;

    memcpy(data_ptr,&imu_data_out,sizeof(imu_data_out));
    return 0;
  }
  else if (type == SNAV_DRIVER_GET_MAG1)
  {
    int out_size = sizeof(mag_data_out);
    if (out_size > data_size)
      return -2;

    memcpy(data_ptr,&mag_data_out,sizeof(mag_data_out));
    return 0;
  }

  return -1;
}
