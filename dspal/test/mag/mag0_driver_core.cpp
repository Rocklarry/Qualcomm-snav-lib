/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
static SnavDriverPortTypeEnum  mag_port_type = SNAV_DRIVER_PORT_TYPE_I2C;

static int32_t  mag_port           = 8;
static int32_t  mag_bit_rate       = 400000;
static uint32_t mag_trx_timeout_us = 1000;
static uint32_t mag_slave_addr     = 0;

static uint32_t mag_read_counter   = 0;
static uint32_t mag_fresh_data     = 0;
static uint64_t mag_read_time      = 0;
static float    mag_data_x         = 0;
static float    mag_data_y         = 0;
static float    mag_data_z         = 0;
static float    mag_data_temp      = -273.15;
static float    mag_data_scale     = 0;

static pthread_mutex_t mag_lock = PTHREAD_MUTEX_INITIALIZER;

static SnavDriverMagData mag_data_out;


static int mag0_snav_driver_set(int handle, SnavDriverSetEnum type, void *  data_ptr, uint32_t data_size)
{
  if (type == SNAV_DRIVER_SET_PORT_NUMBER)
  {
    mag_port = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated mag0_port param to %d",DRIVER_NAME, mag_port);
    return 0;
  }
  else if (type == SNAV_DRIVER_SET_BIT_RATE)
  {
    mag_bit_rate = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated mag0_bit_rate param to %d",DRIVER_NAME, mag_bit_rate);
    return 0;
  }
  return -1;
}


static int mag0_snav_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size)
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
  else if (type == SNAV_DRIVER_GET_MAG0)
  {
    if (mag_fresh_data == 1)
    {
      pthread_mutex_lock(&mag_lock);

      mag_data_out.time        = mag_read_time;
      mag_data_out.cntr        = mag_read_counter;
      mag_data_out.field_x     = mag_data_x;
      mag_data_out.field_y     = mag_data_y;
      mag_data_out.field_z     = mag_data_z;
      mag_data_out.temperature = mag_data_temp;

      mag_fresh_data    = 0;

      pthread_mutex_unlock(&mag_lock);
    }

    int out_size = sizeof(mag_data_out);
    if (out_size > data_size)
      return -2;

    memcpy(data_ptr,&mag_data_out,sizeof(mag_data_out));
    return 0;
  }

  return -1;
}
