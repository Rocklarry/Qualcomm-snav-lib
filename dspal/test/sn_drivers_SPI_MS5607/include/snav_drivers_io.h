/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#ifndef SNAV_DRIVERS_IO_H
#define SNAV_DRIVERS_IO_H

#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

typedef enum
{
  SN_DEBUG, SN_INFO, SN_WARN, SN_ERROR
} FlightMsgPriority;


#ifdef __cplusplus
extern "C" {
#endif

extern int snav_print_ext(FlightMsgPriority input_priority, const char* filename_ptr, int line_num, const char* fmt,...);

int64_t snav_get_time_1us();

//read a certain number of bytes, starting from the specified address
int i2c_read_reg(int file_des, uint8_t address, uint8_t* out_buffer, int length);

//write a certain number of bytes, starting from the specified address
int i2c_write_reg(int file_des, uint8_t address, uint8_t *in_buffer, int length);

int i2c_write_reg_nddr(int file_des, uint8_t *in_buffer, int length);

//write one byte and verify by reading it back
int i2c_write_check_reg(int file_des, uint8_t reg, uint8_t val);

//write a single byte to a specified register address
int i2c_write_single_reg(int file_des, uint8_t address, uint8_t val);

//configure the I2C bus to new slave address and bit rate
int i2c_slave_config(int file_des, uint8_t addr_7bit, uint32_t bit_rate, uint32_t timeout_us);



int spi_read_reg(int file_des, uint8_t address, uint8_t* out_buffer, int length);
int spi_write_reg(int file_des, uint8_t *in_buffer, int length);

#ifdef __cplusplus
}
#endif

#define snav_debug_print(fmt, ...) { snav_print_ext(SN_DEBUG,"",0,fmt,##__VA_ARGS__); }
#define snav_info_print(fmt, ...)  { snav_print_ext(SN_INFO,"",0,fmt,##__VA_ARGS__);  }
#define snav_warn_print(fmt, ...)  { snav_print_ext(SN_WARN,"",0,fmt,##__VA_ARGS__);  }
#define snav_error_print(fmt, ...) { snav_print_ext(SN_ERROR,"",0,fmt,##__VA_ARGS__); }

#endif //SNAV_DRIVERS_IO_H
