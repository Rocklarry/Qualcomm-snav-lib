#pragma once
#ifdef __cplusplus
extern "C" {
#endif

/*==============================================================================
 Copyright (c) 2015 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

/**
 * @mainpage
 *
 * @par Overview
 * MPU-9250 driver is a C dynamic library to operate InvenSense MPU-9250 IMU
 * chipset. MPU-9250 library provides a set of APIs to confiugre mpu9250 and
 * read the IMU measurementfrom the chipset. This MPU-9250 driver uses SPI
 * serial interface to communicate with the host.  The MPU-9250 driver supports
 * up to 8KHz gyro sample rate and 4KHz accelerometer sample rate.
 *
 * @paragraph <driver_modes>
 * @par Driver Modes
 * MPU-9250 driver currently supports the following three modes. The following
 * are the overview of the three modes.
 * - <b>Poll Mode</b>
 *   \n
 *   In Poll mode, you can use mpu9x50_get_data() to poll the sensor
 *   measurements <i><b>at any time</b></i>. mpu9x50_get_data() reads the sensor
 *   measurements from the sensor registers. The results are parsed and stored
 *   in the struct mpu9x50_data data type. Poll mode supports up to 1KHz gyro
 *   sample rate. Users can specify all the sensor settings in struct
 *   mpu9x50_config data type while initializing the driver. Gyro, temperature
 *   and accelerometer are always enabled.  Compass can be enabled or disabled
 *   at driver intialization time. \n
 *   Generally, users need to set up a timer to periodcially poll the new
 *   measurements.  See @ref sample_mpu9x50.
 *   \n\n
 * - <b>Data Ready Interrupt (DRI) Mode</b>
 *   \n
 *   Data Ready Interrupt (DRI) mode is similar to poll mode in that users can
 *   use mpu9x50_get_data() to read the sensor measurements from sensor registers.
 *   In DRI mode, mpu9250 generated a data ready interrupt when new
 *   measurements are availabe. The DRI signal is propagated to the INT
 *   GPIO pin. Users can register a custom Interrupt Service Routine (ISR) to
 *   handle the GPIO interrupt. The DRI fires at the specified sample rate. When
 *   DRI fires, the users can mpu9x50_get_data() to read out the new sample.
 *   As a common practice in embedded program, the ISR shall not contain
 *   intensive operation. <i><b>Due to the SPI transfer overhead, it is recommended
 *   NOT to call mpu9x50_get_data() inside ISR</b></i>. Instead, the ISR should wake up
 *   a separate measurement thread, which carries out the actual SPI transfer.
 *   See @ref sample_mpu9x50.
 *   \n\n
 * - <b>FIFO Mode</b>
 *   \n
 *   Users can set struct mpu9x50_config <i><b>fifo_enabled</b></i> and
 *   <i><b>fifo_en_mask</b></i> fields
 *   to enable FIFO mode operation on gyro, acclerometer and temperature sensors.
 *   In FIFO mode, MPU-9250 writes the sensor measurements to the onchip FIFO
 *   buffer at the specified sample rate. Users can issue mpu9x50_read_fifo()
 *   to read out a collection of samples from the FIFO buffer in burst. This
 *   function reads out all samples over SPI, parse the bytes, and return
 *   a list of struct mpu9x50_data instances to caller.
 *   In FIFO mode, each sample contains up to 14 bytes, i.e. acceleratometer
 *   (6 bytes) + temperature (2 bytes) + gyro (6 bytes).
 *   FIFO mode is designed specifically for applications requiring high sample
 *   frequency.  In FIFO mode, gyro and temperature sensors sample at 8KHz.
 *   Acceleratometer samples at 1KHz. Thus, the accelerometer measurement
 *   remains the same for 8 samples. Compass is disabled in FIFO mode.
 *   See @ref sample_mpu9x50.
 *
 * @par Selecting Proper Mode for Your Application
 * - Use Poll or DRI mode for gyro/accel sample rate no more than 1KHz. In these
 *   two modes, you can sample compass at up to 100Hz. Due to the extra overhead
 *   to communicate with FIFO buffer, it is more efficient to read measurements
 *   from sensor registers directly when sample rate is low.
 * - FIFO mode is required to achieve 8KHz gyro sampling.
 *
 * @par How to Use MPU-9250 Driver
 * - <b>To use MPU-9250 driver in your program:</b>
 *   - include mpu9x50.h in your source
 *   - link the pre-compiled libmpu9x50.so to your program\n\n
 * - <b>A High Level Steps for Using MPU-9250:</b>
 *   - Create a struct mpu9x50_config instance with proper configurations
 *   - Call mpu9x50_initialize() to open the SPI device and initialize the
 *     mpu-9250 device with specified configurations.
 *   - For DRI mode, register proper ISR to handle the data ready GPIO interrupt.
 *     For FIFO mode, call mpu9x50_start_fifo() to start the FIFO operation.
 *   - Call mpu9x50_get_data() to read the IMU measurements from the sensor
 *     registers. For FIFO mode, call mpu9x50_read_fifo() to read the IMU
 *     measurements availabe in FIFO buffer in a burst. Both APIs return the
 *     parsed results of struct mpu9x50_data format. See @ref driver_modes for
 *   - Call mpu9x50_close() to close the mpu9250 device. For DRI mode, this
 *     function unregisters the ISR automatically.
 *
 * @par MPU-9250 Configurations
 * MPU-9250 driver allows configuring the following sensor attributes,
 * For details, see struct mpu9x50_config definition in mpu9x50.h and
 * InvenSense technical spec and register map document.
 * - Gyro Low Pass Filter
 * - Accelerometer Low Pass Filter
 * - Gyro Full Scale Range
 * - Accelerometer Full Scale Range
 * - Gyro Sample Rate:  100Hz, 200Hz, 500Hz, 1000Hz, 8000Hz
 * - Compass Enable/Disable
 * - Compass Sample Rate: 100Hz
 * - FIFO Mode Enable/Disable
 * - FIFO Enable Bitmask for specific sensor: temperature, gyro, accelerometer
 * - SPI device path associated with MPU-9250, e.g. /dev/spi-1
 *
 * @par App Note
 * - In MPU-9250, DMP and FIFO share 4KB memory space. Since we do not use DMP
 *   in the driver, we allocate the whole 4KB for FIFO buffer use.
 * - At 8KHz sample rate, with gyro, temperature and acceleratometer all
 *   enabled, i.e. 14 bytes per sample, it takes around 35ms to fill the FIFO
 *   buffer. Generally the flight control loop runs below 1KHz (1ms). Thus,
 *   conducting FIFO read every 1ms is frequent enough to consume the FIFO data.
 *   In practice, there is no chance for FIFO overflow to happen.
 * - It is developer's responsibility to determine the frequency of FIFO read,
 *   i.e. how many samples to collect for each FIFO read, denoted by
 *   <i><b>fifo_wait_cycles</b></i>. This primarily depends on the frequency of
 *   the control loop. It is important to ensure that the fifo_wait_cycles does
 *   not exceed the time to fill half of FIFO buffer size, in order to avoid
 *   FIFO overflow. In our driver, it takes about 18ms to fill half of FIFO
 *   buffer, i.e. 2KB. Waiting more than 18ms before FIFO ready may result in
 *   FIFO overflow and corrupted measurement readings.
 *   In our example codes, we read 8 samples every 1ms.
 * - int mpu9x50_read_fifo(struct mpu9x50_data *data_list, uint16_t size) reads
 *   up to <i><b>size</b></i> samples.  It is recommended to set
 *   size = 2 x <i><b>fifo_wait_cycles</b></i>. This is critical in particular
 *   when the system is under load and FIFO read cannot happen at accurate
 *   scheduled time. In this case there might be more than fifo_wait_cycles
 *   samples and we need to read all available samples. Otherwise, the residual
 *   samples will be accumulated in the FIFO and eventually results in FIFO
 *   overflow.  E.g. with 8KHz sample rate, I schedule a 1ms timer to do FIFO
 *   read and expect to get on average 8 samples every 1ms. However, I allocate
 *   a 16 sample buffer to accommodate extra samples if any.
 * - Default pthread stack size is 1KB. If you want to store the FIFO samples
 *   on stack, you may need to use pthread_attr_setstacksize() to set larger
 *   stack size. See mpu_tester_test_mpu9x50_fifo_poll_data() in
 *   @ref sample_mpu9x50.
 *
 * @subpage sample_mpu9x50
 *
 * @page sample_mpu9x50 mpu9x50 driver library example codes
 * The following is a set of examples showing how to use mpu9x50 driver to
 * obtain IMU measurements in Poll, DRI and FIFO mode.
 * @include mpu9x50_test.c
 */

 /**
 * @brief
 * Utility macro functions to do unit conversion
 */
#define DEG_TO_RAD(x) ((x) * (M_PI_F) / 180.0)
#define G_TO_MS2(x)   ((x) * 9.80665)

/**
 * MPU9250 Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_REG_ADDR {
   MPU9250_REG_SMPLRT_DIV     = 25,
   MPU9250_REG_CONFIG         = 26,
   MPU9250_REG_GYRO_CONFIG    = 27,
   MPU9250_REG_ACCEL_CONFIG   = 28,
   MPU9250_REG_ACCEL_CONFIG2  = 29,
   MPU9250_REG_FIFO_ENABLE    = 35,
   MPU9250_REG_I2C_MST_CTRL   = 36,
   MPU9250_REG_I2C_SLV0_ADDR  = 37,
   MPU9250_REG_I2C_SLV0_REG   = 38,
   MPU9250_REG_I2C_SLV0_CTRL  = 39,
   MPU9250_REG_I2C_SLV1_ADDR  = 40,
   MPU9250_REG_I2C_SLV1_REG   = 41,
   MPU9250_REG_I2C_SLV1_CTRL  = 42,
   MPU9250_REG_I2C_SLV4_ADDR  = 49,
   MPU9250_REG_I2C_SLV4_REG   = 50,
   MPU9250_REG_I2C_SLV4_DO    = 51,
   MPU9250_REG_I2C_SLV4_CTRL  = 52,
   MPU9250_REG_I2C_SLV4_DI    = 53,
   MPU9250_REG_I2C_MST_STATUS = 54,
   MPU9250_REG_INT_BYPASS     = 55,
   MPU9250_REG_INT_EN         = 56,
   MPU9250_REG_INT_STATUS     = 58,
   MPU9250_REG_I2C_SLV1_DO    = 100,
   MPU9250_REG_I2C_MST_DELAY_CTRL = 103,
   MPU9250_REG_USER_CTRL      = 106,
   MPU9250_REG_PWR_MGMT1      = 107,
   MPU9250_REG_PWR_MGMT2      = 108,
   MPU9250_REG_FIFO_COUNTH    = 114,
   MPU9250_REG_FIFO_COUNTL    = 115,
   MPU9250_REG_FIFO_RW        = 116,
   MPU9250_REG_WHOAMI         = 117,
};

/**
 * Cached register configurations
 * This is to keep track of all the registers which have been set
 */
struct reg_cfg_s {
   uint8_t smplrt_div;
   uint8_t config;
   uint8_t gyro_config;
   uint8_t accel_config;
   uint8_t accel_config2;
   uint8_t fifo_enable;
   uint8_t i2c_mst_ctrl;
   uint8_t int_en;
   uint8_t i2c_mst_delay_ctrl;
   uint8_t user_ctrl;
   uint8_t pwr_mgmt1;
   uint8_t pwr_mgmt2;
   uint8_t fifo_counth;
   uint8_t fifo_countl;
};

/**
 * MPU9250 Compass Register Addresses.
 * Here defines only the register addresses used in mpu9x50 driver.
 * See the spec for the full register list.
 */
enum MPU9250_COMPASS_REG_ADDR {
   MPU9250_COMP_REG_WIA       = 0x00,
   MPU9250_COMP_REG_CNTL1     = 0x0a,
   MPU9250_COMP_REG_ASAX      = 0x10,
   MPU9250_COMP_REG_ASAY      = 0x11,
   MPU9250_COMP_REG_ASAZ      = 0x12,
};

/**
 * Gyro Low Pass Filter Enum
 * MPU9X50_GYRO_LPF_250HZ and MPU9X50_GYRO_LPF_3600HZ_NOLPF is only applicable
 * for 8KHz sample rate. All other LPF values are only applicable for 1KHz
 * internal sample rate.
 */
enum gyro_lpf_e {
   MPU9X50_GYRO_LPF_250HZ = 0,  /**< 250Hz low pass filter for 8KHz gyro sampling*/
   MPU9X50_GYRO_LPF_184HZ,
   MPU9X50_GYRO_LPF_92HZ,
   MPU9X50_GYRO_LPF_41HZ,
   MPU9X50_GYRO_LPF_20HZ,
   MPU9X50_GYRO_LPF_10HZ,
   MPU9X50_GYRO_LPF_5HZ,
   MPU9X50_GYRO_LPF_3600HZ_NOLPF, /**< 3600Hz low pass filter for 8KHz gyro sampling*/
   NUM_MPU9X50_GYRO_LPF
};

/**
 * Accelerometer Low Pass Filter Enum.
 */
enum acc_lpf_e {
   MPU9X50_ACC_LPF_460HZ = 0,
   MPU9X50_ACC_LPF_184HZ,
   MPU9X50_ACC_LPF_92HZ,
   MPU9X50_ACC_LPF_41HZ,
   MPU9X50_ACC_LPF_20HZ,
   MPU9X50_ACC_LPF_10HZ,
   MPU9X50_ACC_LPF_5HZ,
   MPU9X50_ACC_LPF_460HZ_NOLPF,
   NUM_MPU9X50_ACC_LPF
};

/**
 * Gyro Full Scale Range Enum
 */
enum gyro_fsr_e {
   MPU9X50_GYRO_FSR_250DPS = 0,
   MPU9X50_GYRO_FSR_500DPS,
   MPU9X50_GYRO_FSR_1000DPS,
   MPU9X50_GYRO_FSR_2000DPS,
   NUM_MPU9X50_GYRO_FSR
};

/**
 * Accelerometor Full Scale Range Enum
 */
enum acc_fsr_e {
   MPU9X50_ACC_FSR_2G = 0,
   MPU9X50_ACC_FSR_4G,
   MPU9X50_ACC_FSR_8G,
   MPU9X50_ACC_FSR_16G,
   NUM_MPU9X50_ACC_FSR
};

#define MPU9X50_INTERNAL_SAMPLE_RATE_HZ    1000

/**
 * Supported Sample rate for gyro.
 * If gyro sample rate is set to 8KHz, accelerometer sample rate is 1KHz.
 * If other sample rate is selected, the same sample rate is set for gyro and
 * accelerometer.
 */
enum gyro_sample_rate_e {
   MPU9x50_SAMPLE_RATE_100HZ = 0,
   MPU9x50_SAMPLE_RATE_200HZ,
   MPU9x50_SAMPLE_RATE_500HZ,
   MPU9x50_SAMPLE_RATE_1000HZ,
   MPU9x50_SAMPLE_RATE_8000HZ,
   NUM_MPU9X50_SAMPLE_RATE
};

#define MPU9X50_COMPASS_MAX_SAMPLE_RATE_HZ   100
/**
 * Sample rate for compass.
 * NOTE: only 100Hz compass sampling rate is supported in current driver.
 */
enum compass_sample_rate_e {
   MPU9x50_COMPASS_SAMPLE_RATE_100HZ = 0,
   NUM_MPU9X50_COMPASS_SAMPLE_RATE
};

/**
 * Full Scale Range of the magnetometer chip AK89xx in MPU9250
 */
#define MPU9250_AK89xx_FSR  4915 // from invensense doc

#define MPU9250_AKM_DEV_ID  0x48 // compass device ID

/**
 * Structure used to store the MPU9x50 configuration.
 */
struct mpu9x50_config {
   enum gyro_lpf_e            gyro_lpf;
   enum acc_lpf_e             acc_lpf;
   enum gyro_fsr_e            gyro_fsr;
   enum acc_fsr_e             acc_fsr;
   enum gyro_sample_rate_e    gyro_sample_rate;
   bool                       compass_enabled;
   enum compass_sample_rate_e compass_sample_rate;
   bool                       fifo_enabled;
   uint8_t                    fifo_en_mask;  /**< FIFO enable bitmask. See TEMP_FIFO_EN_BIT, GYRO_FIFO_EN_BIT and ACCEL_FIFO_EN_BIT */
   const char*                spi_dev_path;
};

struct mpu9x50_data {
   uint64_t timestamp;        /**< system clock time in us */
   int16_t  temperature_raw;
   float    temperature;
   int16_t  accel_raw[3];
   float    accel_scaling;
   float    accel_range_m_s2;
   int16_t  gyro_raw[3];
   float    gyro_range_rad_s;
   float    gyro_scaling;
   bool     mag_data_ready;
   int16_t  mag_raw[3];
   float    mag_range_ga;
   float    mag_scaling;
};

#define BIT_FIFO_SIZE_1024  (0x40)
#define BIT_FIFO_SIZE_2048  (0x80)
#define BIT_FIFO_SIZE_4096  (0xC0)
#define BIT_I2C_IF_DIS      (0x10)
#define BIT_FIFO_RST        (0x04)
#define BIT_I2C_MST_RST     (0x02)
#define BIT_DMP_RST         (0x08)
#define BIT_RAW_RDY_EN      (0x01)
#define BIT_WAIT_FOR_ES     (0x40)
#define BIT_I2C_MST_EN      (0x20)
#define BIT_DELAY_ES_SHADOW (0x80)
#define BIT_SLV4_DLY_EN     (0x10)
#define BIT_SLV3_DLY_EN     (0x08)
#define BIT_SLV2_DLY_EN     (0x04)
#define BIT_SLV1_DLY_EN     (0x02)
#define BIT_SLV0_DLY_EN     (0x01)
#define BIT_FIFO_EN         (0x40)
#define BIT_TEMP_FIFO_EN    (0x80)
#define BIT_GYRO_FIFO_EN    (0x70)
#define BIT_ACCEL_FIFO_EN   (0x08)
#define BIT_FIFO_OVERFLOW_INT  (0x10)

#define MPU9250_FIFO_MAX_SIZE  (4000)
#define MPU9250_FIFO_DEFAULT_SIZE  (512)

/**
 * In practice, we shall not read too many bytes from FIFO at once.
 * Reading too many bytes at once result in chip hang.
 */
#define MPU9250_FIFO_SINGLE_READ_MAX_BYTES 256

/**
 * @brief
 * Opens the mpu9250 device and configure the sensor using specified config
 * settings. The device path is part of the mpu9x50_config structure.
 * On success, this function returns a file descriptor for the mpu device.
 * The file descriptor should be used for following driver operations.
 *
 * @param   config[in]   struct mpu9x50_config which stores the mpu9x50
 *                       configurations
 * @return  0 on success, negaitve integer value on error
 */
int mpu9x50_initialize(struct mpu9x50_config* config);

/**
 * @brief
 * Close the mpu9250 device.  This must be called after a successful call of
 * mpu9x50_initialize()
 *
 * @return  0 on success, negaitve integer value on error
 */
int mpu9x50_close();

/**
 * @brief
 * Validate the configuration.
 *
 * @param   config[in]   struct mpu9x50_config object which needs to be
 *                       validated
 * @return  0 if validation pass, negaitve integer value on error
 */
int mpu9x50_validate_configuration(struct mpu9x50_config* config);

/**
 * @brief
 * Detect the presence of mpu9x50 gyro/accelerometer by reading the whoami
 * register. This must be called after a successful mpu9x50_initialize() call.
 *
 * @return  0 on success, negative integer on error
 */
int mpu9x50_detect_gyro();

/**
 * @brief
 * Register Data Ready Interrupt in the following mode
 * active high, push-pull, pulse, FSYNC does not create INT.
 * The timing of interrupts is determined by the highest sensor sample rate.
 * E.g. if gyro sample rate is set to 1KHz, compass sample rate is set to 100Hz,
 * then the interrupt will be invoked at 8KHz
 *
 * @param   gpio_id[in]         gpio id used for data ready interrupt pin
 * @param   data_ready_isr[in]  function pointer to be called when interrupt
 *                              fires. context argument is passed to
 *                              data_ready_isr.
 * @param   context[in]         address of the context data provided by user
 * @return  0 on success, negative integer on error
 */
int mpu9x50_register_interrupt(int gpio_id,
      void *(*data_ready_isr)(void *context), void* context);

/**
 * @brief
 * Unregister the Data Ready Interrupt. This can ben called explicitly to
 * disable interrupt. Alternatively, if mpu9x50_close() is called when the
 * interrupt is still enabled, this will be called implicitly.
 *
 * @return  0 on success or mpu device is not yet open,
 *          negative integer on error
 */
int mpu9x50_unregister_interrupt();

/**
 * Get Gyro LPF value in HZ, and store the value in gyro_lpf
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_gyro_lpf(int* gyro_lpf);

/**
 * Get Accel LPF value in HZ, and store the value in accel_lpf
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_accel_lpf(int* accel_lpf);

/**
 * Get Gyro sample rate in Hz, and store the value in sample_Rate
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_gyro_sample_rate(int* sample_rate);

/**
 * Get Accel sample rate in Hz, and store the value  in sample_Rate
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_accel_sample_rate(int* sample_rate);

/**
 * Get compass sample rate in Hz, and store the value in compass_sample_Rate
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_compass_sample_rate(int* compass_sample_rate);

/**
 * Get Gyro FSR, and store the value in gyro_fsr in DPS
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_gyro_fsr(int* gyro_fsr);

/**
 * Get Accel FSR in G, and store in accel_fsr
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_accel_fsr(int* accel_fsr);

/**
 * Get compass FSR, and store in compass_fsr
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_compass_fsr(int* compass_fsr);


/**
 * Get INT status, and store in int_status
 * @return             0 on success, -1 on error
 */
int mpu9x50_get_int_status(uint8_t* int_status);

/**
 * Read sensor readings from measurement registers. The results are parsed
 * and stored in the data pointer passed to the function. The compass data is
 * not available in data if compass is disabled.
 *
 * @param[in/out]  data
 * Non-NULL address where the parsed sensor data is stored at.
 * @return
 * 0 on success, -1 on error
 */
int mpu9x50_get_data(struct mpu9x50_data *data);

/**
 * Read measurement data from mpu9250 FIFO buffer and write the parsed
 * results to data_list
 *
 * @param[out]  data_list
 * address of the list of mpu9x50_data structure. The parsed results
 * are saved from this address.
 * @param[in]   size
 * size of data_list. Caller must ensure that enough memory space is
 * allocated for data_list. Besides, caller must ensure size is large
 * enough to acommondate all samples collected in FIFO buffer. Or else FIFO
 * buffer overflow would occur eventually.
 *
 * @return
 * - number of samples read and parsed from FIFO.
 * - -1 if fifo mode is not enabled
 * - -2 on error
 */
int mpu9x50_read_fifo(struct mpu9x50_data *data_list, uint16_t size);

/**
 * @brief
 * Convert the gyro_sample_rate_e enum to actual integer rate in HZ
 * @return   integer rate value in HZ, -1 if rate is not supported
 */
int gyro_sample_rate_enum_to_hz(enum gyro_sample_rate_e rate);

/**
 * @brief
 * Convert the compass_sample_rate_e enum to actual integer rate in HZ
 * @return   integer rate value in HZ, -1 if rate is not supported
 */
int compass_sample_rate_enum_to_hz(enum compass_sample_rate_e rate);

/**
 * @brief
 * Start the FIFO operation. mpu9x50 chipset will not write sensor measurements
 * to FIFO buffer until this function is called.
 *
 * @return   0 on success, -1 on error.
 */
int mpu9x50_start_fifo();

/**
 * @brief
 * Stop the FIFO operation.This would NOT clear the FIFO enable bitmasks.
 *
 * @return   0 on success, -1 on error.
 */
int mpu9x50_stop_fifo();

/**
 * @brief
 * Obtain the FIFO size in bytes. This is regardless of FIFO being enabled
 * or not.
 *
 * @return
 * FIFO size in bytes.  The FIFO can be configured as 1KB, 2KB and 4KB.
 * The default FIFO size is 512B.
 */
int mpu9x50_get_fifo_size();

#ifdef __cplusplus
}
#endif