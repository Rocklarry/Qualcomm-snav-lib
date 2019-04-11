/*==============================================================================
 Copyright (c) 2016 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

static SnavDriverPortTypeEnum  baro_port_type = SNAV_DRIVER_PORT_TYPE_SPI;

static int32_t  baro_port           = 3;
static int32_t  baro_bit_rate       = 400000;
//static uint32_t baro_trx_timeout_us = 1000;
//static uint32_t baro_slave_addr     = 0;

static uint32_t baro_read_counter   = 0;
static uint32_t baro_fresh_data     = 0;
static uint64_t baro_read_time      = 0;
static float    baro_pressure_pa    = 0;
static float    baro_temperature_c  = -273.15;

static pthread_mutex_t baro_lock    = PTHREAD_MUTEX_INITIALIZER;

static SnavDriverBaroData baro_data_out;

static int baro0_snav_driver_set(int handle, SnavDriverSetEnum type, void *  data_ptr, uint32_t data_size)
{
  if (type == SNAV_DRIVER_SET_PORT_NUMBER)
  {
    baro_port = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated baro0_port param to %d",DRIVER_NAME, baro_port);
    return 0;
  }
  else if (type == SNAV_DRIVER_SET_BIT_RATE)
  {
    baro_bit_rate = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated baro0_bit_rate param to %d",DRIVER_NAME, baro_bit_rate);
    return 0;
  }
  return -1;
}


static int baro0_snav_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size)
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
  else if (type == SNAV_DRIVER_GET_BARO0)
  {
    if (baro_fresh_data == 1)
    {
      pthread_mutex_lock(&baro_lock);

      baro_data_out.time        = baro_read_time;
      baro_data_out.cntr        = baro_read_counter;
      baro_data_out.pressure    = baro_pressure_pa;
      baro_data_out.temperature = baro_temperature_c;

      baro_fresh_data    = 0;

      pthread_mutex_unlock(&baro_lock);
    }

    int out_size = sizeof(baro_data_out);
    if (out_size > data_size)
      return -2;

    memcpy(data_ptr,&baro_data_out,sizeof(baro_data_out));
    return 0;
  }

  return -1;
}
