/*==============================================================================
 Copyright (c) 2015 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

#define DRIVER_NAME "SNAV_SONAR0_MAXBOTIX_DRIVER"

#include <dev_fs_lib_serial.h>

#include "snav_drivers.h"
#include "snav_drivers_io.h"

//static pthread_t thread;
//static uint32_t  thread_running = 0;
static int       file_des       = -1;

extern "C"
{
void *     get_driver_interface(void);

static int snav_driver_open(void);
static int snav_driver_init(int handle);
static int snav_driver_close(int handle);
static int snav_driver_get(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size);
static int snav_driver_set(int handle, SnavDriverSetEnum type, void * data_ptr, uint32_t data_size);

}

#include "sonar0_driver_core.cpp"
#include "maxbotix_driver_core.cpp"

int maxbotix_driver_init()
{
  char * uart_device_format      = (char*)"/dev/tty-%d";
  char uart_device[32];
  snprintf(uart_device, 32, uart_device_format, sonar_port);

  snav_info_print("%s: ====== rrd========  Starting thread, opening device %s at baud rate of %lu",DRIVER_NAME,uart_device,sonar_bit_rate);
  file_des = open(uart_device, O_RDWR);

  if (file_des == -1)
  {
    snav_error_print("%s: Failed to open uart port %s. Exiting.",DRIVER_NAME,uart_device);
    return -1;
  }

  dspal_serial_ioctl_data_rate ioctl_baud_rate;

  switch (sonar_bit_rate)
  {
    case 9600:   ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_9600;   break;
    case 38400:  ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_38400;  break;
    case 57600:  ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_57600;  break;
    case 115200: ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_115200; break;
    case 230400: ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_230400; break;
    case 460800: ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_460800; break;
    case 921600: ioctl_baud_rate.bit_rate = DSPAL_SIO_BITRATE_921600; break;
    default: snav_error_print("%s: ===rrd====  unsupported baud rate %lu",DRIVER_NAME,sonar_bit_rate); return -2;
  }

  snav_info_print("%s:  ===rrd===  Setting baud rate to %d   %d",DRIVER_NAME,sonar_bit_rate,SERIAL_IOCTL_SET_DATA_RATE);
  if (ioctl(file_des, SERIAL_IOCTL_SET_DATA_RATE, &ioctl_baud_rate) != 0)
  {
    snav_error_print("%s:  ===rrd===  Failed to set baud rate. Exiting.",DRIVER_NAME);
    close(file_des);
    file_des=-1;
    return -3;
  }

  snav_info_print("%s:  ===rrd===  Setting 1111 ",DRIVER_NAME);

  struct dspal_serial_ioctl_receive_data_callback  maxbotix_receive_callback_config;
  maxbotix_receive_callback_config.context = NULL;
  maxbotix_receive_callback_config.rx_data_callback_func_ptr = multi_port_read_callback;  // maxbotix_read_callback;//接受数据

  int result = ioctl(file_des, SERIAL_IOCTL_SET_RECEIVE_DATA_CALLBACK, (void *)&maxbotix_receive_callback_config);
  if (result < 0)
  {
    snav_error_print("%s: ===rrd===   failed to set callback",DRIVER_NAME);
    close(file_des);
    file_des=-1;
    return -4;
  }

  snav_info_print("%s:  ===rrd===  Setting 2222 ",DRIVER_NAME);

  sonar_read_counter = 0;

  return 0;
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
  return maxbotix_driver_init();
}

static int snav_driver_close(int handle)
{
  //TODO
  return 0;
}
