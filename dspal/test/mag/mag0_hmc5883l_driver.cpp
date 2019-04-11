/*==============================================================================
 Copyright (c) 2015 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

#define DRIVER_NAME "SNAV_MAG0_HMC5883L_DRIVER"

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

#include "mag0_driver_core.cpp"
#include "hmc5883l_driver_core.cpp"

static int main_driver_thread(void * params)
{
  char * i2c_device_format      = (char*)"/dev/iic-%d";
  char i2c_device[32];
  snprintf(i2c_device, 32, i2c_device_format , mag_port);

  snav_info_print("%s: Starting thread, opening device %s",DRIVER_NAME,i2c_device);

  file_des = open(i2c_device, O_RDWR);
  if (file_des == -1)
  {
    snav_error_print("%s: Failed to open mag i2c port %s. Exiting.",DRIVER_NAME,i2c_device);
    return -1;
  }

  int hmc5883l_init_ret = hmc5883l_init();
  if (hmc5883l_init_ret !=0 )
    return hmc5883l_init_ret;

  snav_info_print("%s: HMC5883 initialization successful!", DRIVER_NAME);

  while(1)
  {
    hmc5883l_read_data();

    usleep(19500); //TODO: implement better timer
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
  return mag0_snav_driver_set(handle,type,data_ptr,data_size);
}

static int snav_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size)
{
  return mag0_snav_driver_get(handle,type,data_ptr,data_size);
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


