/*
 * Copyright (c) 2015-2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */

#ifndef SNAV_DRIVERS_H
#define SNAV_DRIVERS_H

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef __cplusplus
extern "C"
{
#endif

static const char SNAV_DRIVERS_API_VERSION[] = "SNAV_DRIVERS_API_VERSION_0.2";


typedef enum
{
  SNAV_DRIVER_SET_PORT_TYPE = 0,
  SNAV_DRIVER_SET_PORT_NUMBER,
  SNAV_DRIVER_SET_BIT_RATE,
  SNAV_DRIVER_SET_LED0_RGB,
  SNAV_DRIVER_SET_BLOCKING_IO,
  SNAV_DRIVER_SET_MAX_TYPES
} SnavDriverSetEnum;

typedef enum
{
  SNAV_DRIVER_GET_API_VERSION = 0,
  SNAV_DRIVER_GET_DRIVER_INFO,
  SNAV_DRIVER_GET_ACCEL_GYRO0,
  SNAV_DRIVER_GET_ACCEL_GYRO1,
  SNAV_DRIVER_GET_BARO0,
  SNAV_DRIVER_GET_BARO1,
  SNAV_DRIVER_GET_MAG0,
  SNAV_DRIVER_GET_MAG1,
  SNAV_DRIVER_GET_SONAR0,
  SNAV_DRIVER_GET_SONAR1,
  SNAV_DRIVER_GET_VOLTAGE0,
  SNAV_DRIVER_GET_CURRENT0,
  SNAV_DRIVER_GET_ACCEL_GYRO2,
  SNAV_DRIVER_GET_MAX_TYPES
} SnavDriverGetEnum;

typedef enum
{
  SNAV_DRIVER_PORT_TYPE_I2C = 0,
  SNAV_DRIVER_PORT_TYPE_UART,
  SNAV_DRIVER_PORT_TYPE_SPI,
  SNAV_DRIVER_PORT_TYPE_MAX_TYPES
} SnavDriverPortTypeEnum;


//typedefs for function pointers to be returned in the handle by the driver
typedef int (*open_func_ptr_t)(void);
typedef int (*init_func_ptr_t)(int handle);
typedef int (*shutdown_func_ptr_t)(int handle);
typedef int (*set_func_ptr_t)(int handle, SnavDriverSetEnum type, void * data_ptr, uint32_t data_size);
typedef int (*get_func_ptr_t)(int handle, SnavDriverGetEnum type, void * data_ptr, uint32_t data_size);

#define SNAV_DRIVER_HANDLE_HEADER 123456789
#define SNAV_DRIVER_HANDLE_FOOTER 987654321

typedef struct
{
  uint32_t            header;
  uint32_t            version;
  uint32_t            size;
  open_func_ptr_t     open;
  init_func_ptr_t     init;
  shutdown_func_ptr_t close;
  get_func_ptr_t      get;
  set_func_ptr_t      set;
  uint32_t            footer;
} snav_driver_interface_t;


typedef struct
{
  uint64_t time;         //data timestamp in microceconds
  uint32_t cntr;         //number of data read by the driver
  float    accel_x;      //accelerometer reading in sensor's x axis
  float    accel_y;      //accelerometer reading in sensor's y axis
  float    accel_z;      //accelerometer reading in sensor's z axis
  float    gyro_x;       //accelerometer reading in sensor's x axis
  float    gyro_y;       //accelerometer reading in sensor's y axis
  float    gyro_z;       //accelerometer reading in sensor's z axis
  float    temperature;  //temperature of the mag sensor in degrees C, if available
} SnavDriverImuData;


typedef struct
{
  uint64_t time;         //data timestamp in microceconds
  uint32_t cntr;         //number of data read by the driver
  float    pressure;     //pressure reading in Pascals
  float    temperature;  //temperature of the pressure sensor in degrees C
} SnavDriverBaroData;


typedef struct
{
  uint64_t time;         //data timestamp in microceconds
  uint32_t cntr;         //number of data read by the driver
  float    field_x;      //field reading in sensor's x axis
  float    field_y;      //field reading in sensor's y axis
  float    field_z;      //field reading in sensor's z axis
  float    temperature;  //temperature of the mag sensor in degrees C, if available
} SnavDriverMagData;

typedef struct
{
  uint64_t time;         //data timestamp in microceconds
  uint32_t cntr;         //number of data read by the driver
  float    range;        //range in meters
  float    temperature;  //temperature of the sonar sensor in degrees C
} SnavDriverSonarData;

typedef struct
{
  uint64_t time;         //data timestamp in microceconds
  uint32_t cntr;         //number of data read by the driver
  float    voltage;      //battery voltage in Volts
} SnavDriversVoltageData;

typedef struct
{
  uint64_t time;         //data timestamp in microceconds
  uint32_t cntr;         //number of data read by the driver
  float    current;      //battery current in Amps
} SnavDriversCurrentData;

typedef struct
{
  uint64_t time;             //data timestamp in microceconds
  uint32_t cntr;             //number of data read by the driver
  float    voltage;          //total battery voltage
  float    voltage_cells[6]; //voltage of individual cells
  float    voltage_aux;      //voltage at the aux input
  float    current;          //total battery current
  float    current_fc;       //current going to the flight controller
  float    temperatures[4];  //board-specific tempearture measurements
} SnavDriversPowerData;

#ifdef __cplusplus
}
#endif


#endif //SNAV_DRIVERS_H
