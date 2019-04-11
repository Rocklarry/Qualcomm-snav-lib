/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
static SnavDriverPortTypeEnum  sonar_port_type = SNAV_DRIVER_PORT_TYPE_UART;

static int32_t  sonar_port           = 4;
static int32_t  sonar_bit_rate       = 9600;

static uint32_t sonar_read_counter   = 0;
static uint32_t sonar_fresh_data     = 0;
static uint64_t sonar_read_time      = 0;
static float    sonar_range          = 0;  //meters
static float    sonar_temperature    = -273.15;

static pthread_mutex_t sonar_lock    = PTHREAD_MUTEX_INITIALIZER;

static SnavDriverSonarData sonar_data_out;

static int sonar0_update_data();


static int snav_driver_set(int handle, SnavDriverSetEnum type, void *  data_ptr, uint32_t data_size)
{
  if (type == SNAV_DRIVER_SET_PORT_NUMBER)
  {
    sonar_port = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated sonar0_port param to %d",DRIVER_NAME, sonar_port);
    return 0;
  }
  else if (type == SNAV_DRIVER_SET_BIT_RATE)
  {
    sonar_bit_rate = *(int32_t*)data_ptr;
    snav_info_print("%s: Updated sonar0_bit_rate param to %d",DRIVER_NAME, sonar_bit_rate);
    return 0;
  }
  return -1;
}


static int snav_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size)
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
  else if (type == SNAV_DRIVER_GET_SONAR0)
  {
    sonar0_update_data();

    if (sonar_fresh_data == 1)
    {
      //no need to lock mutex because sonar0_update_data uses mutex
      sonar_data_out.time        = sonar_read_time;
      sonar_data_out.cntr        = sonar_read_counter;
      sonar_data_out.range       = sonar_range;
      sonar_data_out.temperature = sonar_temperature;

      sonar_fresh_data    = 0;
    }

    int out_size = sizeof(sonar_data_out);
    if (out_size > data_size)
      return -2;

    memcpy(data_ptr,&sonar_data_out,sizeof(sonar_data_out));
    return 0;
  }

  return -1;
}
