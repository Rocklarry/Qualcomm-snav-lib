/*==============================================================================
 Copyright (c) 2016 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

#include <dev_fs_lib_i2c.h>
#include <dev_fs_lib_spi.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <dspal_time.h>

#include "snav_drivers.h"
#include "snav_drivers_io.h"

#define MAX_LEN_TRANSMIT_BUFFER_IN_BYTES  32

//int snav_print_ext(FlightMsgPriority input_priority, const char* filename_ptr, int line_num, const char* fmt,...) { return 0; }

int64_t snav_get_time_1us()
{
  // grab device time, convert to microseconds
  struct timespec time_struct;
  clock_gettime(0,&time_struct);
  uint64_t time_1us = (time_struct.tv_nsec)/1000 + ((int64_t) time_struct.tv_sec)*1000000;
  return time_1us;
}

int i2c_slave_config(int file_des, uint8_t addr_7bit, uint32_t bit_rate, uint32_t timeout_us)
{
  struct dspal_i2c_ioctl_slave_config slave_config;
  memset(&slave_config, 0, sizeof(slave_config));
  slave_config.slave_address                 = addr_7bit;
  slave_config.bus_frequency_in_khz          = bit_rate/1000;
  slave_config.byte_transer_timeout_in_usecs = timeout_us;
  if (ioctl(file_des, I2C_IOCTL_SLAVE, &slave_config) != 0)
  {
    return -1;
  }
  return 0;
}


int spi_read_reg(int file_des, uint8_t address,uint8_t* out_buffer, int length)
{
  struct dspal_spi_ioctl_read_write  read_write;
  uint8_t write_buffer[1];

  // Save the address of the register to read from in the write buffer for the combined write.
  write_buffer[0]                = address;
  read_write.write_buffer     = write_buffer;
  read_write.write_buffer_length = 1;
  read_write.read_buffer      = out_buffer;
  read_write.read_buffer_length  = length;

  int bytes_written = ioctl(file_des, SPI_IOCTL_RDWR, &read_write);
  if (bytes_written != length)
  {
    snav_error_print("spi_read_reg: Read register reports a read of %d bytes, but attempted to read %d bytes",bytes_written,length);
    return -1;
  }
  return 0;
}



int i2c_read_reg(int file_des, uint8_t address,uint8_t* out_buffer, int length)
{
  struct dspal_i2c_ioctl_combined_write_read ioctl_write_read;
  uint8_t write_buffer[1];

  // Save the address of the register to read from in the write buffer for the combined write.
  write_buffer[0]                = address;
  ioctl_write_read.write_buf     = write_buffer;
  ioctl_write_read.write_buf_len = 1;
  ioctl_write_read.read_buf      = out_buffer;
  ioctl_write_read.read_buf_len  = length;

  int bytes_written = ioctl(file_des, I2C_IOCTL_RDWR, &ioctl_write_read);
  if (bytes_written != length)
  {
    snav_error_print("i2c_read_reg: Read register reports a read of %d bytes, but attempted to read %d bytes",bytes_written,length);
    return -1;
  }
  return 0;
}

int i2c_write_single_reg(int file_des, uint8_t address, uint8_t val)
{
  return i2c_write_reg(file_des,address,&val,1);
}

int i2c_write_reg(int file_des, uint8_t address, uint8_t *in_buffer, int length)
{
  if (length + 1 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES)
  {
    snav_error_print("i2c_write_reg: Caller's buffer size (%d) exceeds size of local buffer (%d)",length,MAX_LEN_TRANSMIT_BUFFER_IN_BYTES);
    return -1;
  }

  uint8_t write_buffer[MAX_LEN_TRANSMIT_BUFFER_IN_BYTES];

  write_buffer[0] = address;
  memcpy(&write_buffer[1], in_buffer, length);
  int bytes_written = write(file_des, (char *)write_buffer, length + 1);
  if (bytes_written != length+1)
  {
    snav_error_print("i2c_write_reg: i2c write failed. Reported %d bytes written", bytes_written);
    return -1;
  }

  return 0;
}



int spi_write_reg(int file_des, /*uint8_t address,*/ uint8_t *in_buffer, int length)
{
  if (length + 1 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES)
  {
    snav_error_print("spi_write_reg: Caller's buffer size (%d) exceeds size of local buffer (%d)",length,MAX_LEN_TRANSMIT_BUFFER_IN_BYTES);
    return -1;
  }

  uint8_t write_buffer[MAX_LEN_TRANSMIT_BUFFER_IN_BYTES];

  //write_buffer[0] = address;
  memcpy(&write_buffer[0], in_buffer, length);
  int bytes_written = write(file_des, (char *)write_buffer, length);
  if (bytes_written != length)
  {
    snav_error_print("spi_write_reg: i2c write failed. Reported %d bytes written", bytes_written);
    return -1;
  }

  return 0;
}


int i2c_write_reg_nddr(int file_des, /*uint8_t address,*/ uint8_t *in_buffer, int length)
{
  if (length + 1 > MAX_LEN_TRANSMIT_BUFFER_IN_BYTES)
  {
    snav_error_print("i2c_write_reg: Caller's buffer size (%d) exceeds size of local buffer (%d)",length,MAX_LEN_TRANSMIT_BUFFER_IN_BYTES);
    return -1;
  }

  uint8_t write_buffer[MAX_LEN_TRANSMIT_BUFFER_IN_BYTES];

  //write_buffer[0] = address;
  memcpy(&write_buffer[0], in_buffer, length);
  int bytes_written = write(file_des, (char *)write_buffer, length);
  if (bytes_written != length)
  {
    snav_error_print("i2c_write_reg: i2c write failed. Reported %d bytes written", bytes_written);
    return -1;
  }

  return 0;
}



int i2c_write_check_reg(int file_des, uint8_t reg, uint8_t val)
{
  uint8_t val2 = 0;
  int ret1 = i2c_write_reg(file_des, reg, &val, 1);
  int ret2 = i2c_read_reg(file_des, reg, &val2, 1);
  int ret3 = val == val2 ? 0 : 1;

  //snav_info_print("i2c_write_check_reg: %X %X", val, val2);

  return (ret1 || ret2 || ret3);
}



