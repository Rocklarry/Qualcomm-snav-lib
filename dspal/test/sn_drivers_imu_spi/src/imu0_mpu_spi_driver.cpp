/*==============================================================================
 Copyright (c) 2016 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

#define DRIVER_NAME "SNAV_IMU0_MPU_SPI_DRIVER"

#include "snav_drivers.h"
#include "snav_drivers_io.h"

static pthread_t thread;
static uint32_t  thread_running = 0;
static int       file_des       = -1;

extern "C"
{
void *     get_driver_interface(void);

static int snav_driver_open(void);
static int snav_driver_init(int handle);
static int snav_driver_close(int handle);
static int snav_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size);
static int snav_driver_set(int handle, SnavDriverSetEnum type, void * data_ptr, uint32_t data_size);

static int start_driver_thread(void * params);
static int main_driver_thread(void * params);
}

int mpu_spi_read_data();

#include "imu0_driver_core.cpp"
#include "mpu_spi_driver_core.cpp"

static int main_driver_thread(void * params)
{
  char * spi_device_format = (char*)"/dev/spi-%d";
  char spi_device[32];
  snprintf(spi_device, 32, spi_device_format , imu_port);

  snav_info_print("%s: Starting thread, opening spi device %s",DRIVER_NAME,spi_device);


  struct mpu9x50_config config = {
    .gyro_lpf            = MPU9X50_GYRO_LPF_184HZ,
    .acc_lpf             = MPU9X50_ACC_LPF_184HZ,
    .gyro_fsr            = MPU9X50_GYRO_FSR_2000DPS,
    .acc_fsr             = MPU9X50_ACC_FSR_16G,
    .gyro_sample_rate    = MPU9x50_SAMPLE_RATE_1000HZ,
    .compass_enabled     = false,
    .compass_sample_rate = MPU9x50_COMPASS_SAMPLE_RATE_100HZ,
    .spi_dev_path        = spi_device,
  };

  if (0)  //todo: use a param
  {
    config.compass_enabled = true;
  }


  if (mpu_driver_init(&config))
  {
    snav_error_print("%s: unable to initialize spi imu",DRIVER_NAME);
    close(file_des);
    file_des = -1;
    return -2;
  }

  snav_info_print("%s: initialization successful!", DRIVER_NAME);

  if (imu_blocking_read == 1)
  {
    snav_info_print("%s: Exiting thread because blocking read has been requested", DRIVER_NAME);
    return 0;
  }

  while(1)
  {
    mpu_spi_read_data();

    usleep(4000);
  }

  return 0;
}

static void *driver_thread_trampoline(void *params)
{
  int ret = main_driver_thread(params);
  snav_info_print("%s: Exiting main thread with code (%d)",DRIVER_NAME,ret);
  pthread_exit(NULL);
  return NULL;
}

static int start_driver_thread(void * params)
{
  if (thread_running)
    return 0;

  // initialize required pthreads
  pthread_attr_t   thread_attr;
  size_t           thread_stack_size = 4 * 1024; // allocate 4KB for the stack

  if (pthread_attr_init(&thread_attr) != 0)
  {
    snav_error_print("%s: pthread_read_attr_init returned error",DRIVER_NAME);
    return -1;
  }
  if (pthread_attr_setstacksize(&thread_attr, thread_stack_size) != 0)
  {
    snav_error_print("%s: pthread_attr_setstacksize returned error",DRIVER_NAME);
    return -1;
  }
  if (pthread_create(&thread, &thread_attr, driver_thread_trampoline, params) != 0)
  {
    snav_error_print("%s: thread_create returned error",DRIVER_NAME);
    return -1;
  }
  else
  {
    snav_info_print("%s: thread creation Successful",DRIVER_NAME);
  }

  thread_running = 1;

  return 0;
}






static int snav_driver_set(int handle, SnavDriverSetEnum type, void *  data_ptr, uint32_t data_size)
{
  return imu0_driver_set(handle,type,data_ptr,data_size);
}

static int snav_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size)
{
  return imu0_driver_get(handle,type,data_ptr,data_size);
}


static snav_driver_interface_t interface;

void * get_driver_interface(void)
{
  interface.header   = SNAV_DRIVER_HANDLE_HEADER;
  interface.version  = 1;
  interface.size     = sizeof(interface);
  interface.open     = snav_driver_open;
  interface.init     = snav_driver_init;
  interface.close    = snav_driver_close;
  interface.get      = snav_driver_get;
  interface.set      = snav_driver_set;
  interface.footer   = SNAV_DRIVER_HANDLE_FOOTER;

  return (void*)&interface;
}

static int snav_driver_open()
{
  return 0;
}

static int snav_driver_init(int handle)
{
  return start_driver_thread(NULL);
}

static int snav_driver_close(int handle)
{
  //TODO
  return 0;
}





