/*==============================================================================
 Copyright (c) 2016 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

//TODO: support both slave addresses
#define BMP280_I2C_ADDRESS 0b1110110

static int32_t  t_fine_;

typedef struct {
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;
} bmp280_sensor_calibration;

static bmp280_sensor_calibration calib_;


static uint32_t bmp280_compensate_P_int64(int32_t adc_P)
{
  int64_t var1, var2, p;
  var1 = ((int64_t)t_fine_) - 128000;
  var2 = var1 * var1 * (int64_t)calib_.dig_P6;
  var2 = var2 + ((var1*(int64_t)calib_.dig_P5)<<17);
  var2 = var2 + (((int64_t)calib_.dig_P4)<<35);
  var1 = ((var1 * var1 * (int64_t)calib_.dig_P3)>>8) + ((var1 * (int64_t)calib_.dig_P2)<<12);
  var1 = (((((int64_t)1)<<47)+var1))*((int64_t)calib_.dig_P1)>>33;
  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }

  p = 1048576-adc_P;
  p = (((p<<31)-var2)*3125)/var1;
  var1 = (((int64_t)calib_.dig_P9) * (p>>13) * (p>>13)) >> 25;
  var2 = (((int64_t)calib_.dig_P8) * p) >> 19;
  p = ((p + var1 + var2) >> 8) + (((int64_t)calib_.dig_P7)<<4);

  return (uint32_t)p;
}

static int32_t  bmp280_compensate_T_int32(int32_t adc_T)
{
  int32_t var1, var2, T;
  var1 = ((((adc_T>>3) - ((int32_t)calib_.dig_T1<<1))) * ((int32_t)calib_.dig_T2)) >> 11;
  var2 = (((((adc_T>>4) - ((int32_t)calib_.dig_T1)) * ((adc_T>>4) - ((int32_t)calib_.dig_T1))) >> 12) * ((int32_t)calib_.dig_T3)) >> 14;
  t_fine_ = var1 + var2;
  T = (t_fine_ * 5 + 128) >> 8;

  return T;
}


static int32_t  bmp280_verify_id()
{
  uint8_t buf[1]      = {255};
  uint8_t expected_id = 0x88;
  int ret = i2c_read_reg(file_des,0xD0,buf,1);

  if (ret != 0) { snav_error_print("%s: BMP280 failed to read ID!!!",DRIVER_NAME); return -1; }

  if (buf[0] == expected_id)
  {
    snav_info_print("%s: BMP280 verify id: read %d, expected %d: SUCCESS!",DRIVER_NAME,buf[0],expected_id);
    return -2;
  }

  snav_info_print("%s: BMP280 verify id: read %d, expected %d: FAILURE!",DRIVER_NAME,buf[0],expected_id);
  return 0;
}

static int bmp280_configure_sensor()
{
  int ret;

  ret = i2c_read_reg(file_des,0x88, (uint8_t*)&calib_, 26);
  if (ret != 0) { snav_error_print("%s: BMP280 failed to read calibration!!!",DRIVER_NAME); return -1; }

  snav_info_print("%s: BMP280 params %d %d %d    %d %d %d %d %d %d %d %d %d",DRIVER_NAME,
              calib_.dig_T1, calib_.dig_T2, calib_.dig_T3,
              calib_.dig_P1, calib_.dig_P2, calib_.dig_P3, calib_.dig_P4, calib_.dig_P5,
              calib_.dig_P6, calib_.dig_P7, calib_.dig_P8, calib_.dig_P9);


  //uint8_t ctrl_meas[2] = {0xF4, 0b01010011}; //power on, oversampling 2 for temp and 8 for pressure
  uint8_t ctrl_meas[2] =   {0xF4, 0b01010111}; // 0x57  0x53 精度模式
  ret = i2c_write_reg(file_des,ctrl_meas[0],&ctrl_meas[1],1);

   //ret = i2c_read_reg(file_des,0x88, (uint8_t*)&calib_, 26);

  if (ret == 0) { snav_info_print("%s: BMP280 sensor config1 ok!",DRIVER_NAME);}
  else          { snav_error_print("%s: BMP280 sensor config1 failed!!!",DRIVER_NAME); return -2; }

  uint8_t config[2] = {0xF5,0b00000000};
  ret = i2c_write_reg(file_des,config[0],&config[1],1);

  if (ret == 0) {  snav_info_print("%s: BMP280 sensor config2 ok!",DRIVER_NAME); }
  else          { snav_error_print("%s: BMP280 sensor config2 failed!!!",DRIVER_NAME); return -3; }

  return 0;
}


static int bmp280_init()
{
  baro_slave_addr = BMP280_I2C_ADDRESS;

  snav_info_print("%s: Setting i2c slave address 0x%02X, baud rate %lu",DRIVER_NAME,baro_slave_addr, baro_bit_rate);

  //set the slave config
  if (i2c_slave_config(file_des, baro_slave_addr, baro_bit_rate, baro_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,baro_bit_rate);
    return -2;
  }

  if (bmp280_verify_id()!=0)
  {
    snav_error_print("%s: BMP280 verify ID failed!", DRIVER_NAME);
    return -3;
  }

  if (bmp280_configure_sensor()!=0)
  {
    snav_error_print("%s: BMP280 config failed!", DRIVER_NAME);
    return -4;
  }

  return 0;
}

#define FILTERNUM 32
float fAccX[FILTERNUM];


/*
float interval_value( float old_value ,float new_value)
{
	if(30 <old_value - new_value< 30)
		return  old_value;
	else
		return new_value;

}
*/

static float tmp;
static int i = 0;
static int count = 0;

float bmp280_range_value_old = 0;
float bmp280_range_value_new_old = 0;

static int bmp280_read_data()
{
  //set the slave config each time, just in case
  if (i2c_slave_config(file_des, baro_slave_addr, baro_bit_rate, baro_trx_timeout_us) != 0)
  {
    snav_error_print("%s: Unable to set i2c slave config: baud rate %lu",DRIVER_NAME,baro_bit_rate);
    return -1;
  }

  uint8_t  baro_raw_data[6] = {0,0,0,0,0,0};
  uint64_t time_read_start  = snav_get_time_1us();
  int      baro_read_ret    = i2c_read_reg(file_des,0xF7, baro_raw_data, 6);  //baro time is about 300us


  if (baro_read_ret == 0)
  {
    //convert the raw data to pascals and degrees C
    uint32_t pressure = 0;
    pressure |= baro_raw_data[0]; pressure <<= 8;
    pressure |= baro_raw_data[1]; pressure <<= 8;
    pressure |= baro_raw_data[2] & 0b11110000; pressure >>=4;

    uint32_t temp = 0;
    temp |= baro_raw_data[3]; temp <<= 8;
    temp |= baro_raw_data[4]; temp <<= 8;
    temp |= baro_raw_data[5] & 0b11110000; temp >>=4;

    int32_t  t_comp = bmp280_compensate_T_int32(temp);
    uint32_t p_comp = bmp280_compensate_P_int64(pressure);

    float new_value   = p_comp / 256.0;
    float temperature_c = t_comp / 100.0;
	float new_pressure_pa;
	float pressure_pa;


	//snav_info_print("%s:  baro:   new = %f ",DRIVER_NAME, new_value);

	//new_pressure_pa = new_value;

	/*if((new_value - tmp)<3 ||(tmp - new_value)<3){
		new_pressure_pa = tmp;
	}*/

//////////////////////////////////////////////////////////////////////////////
	float AccsumX = 0;

		int i;
         for( i=0;i<FILTERNUM-1;i++)
         {
                   fAccX[i] = fAccX[i+1];

         }

         fAccX[FILTERNUM-1] = new_value;


         for( i=0;i<FILTERNUM;i++)
         {
                   AccsumX += fAccX[i];

         }
         new_value = AccsumX / FILTERNUM;


/////////////////////////////////////////////////////////////////////////////////
	/*
	if((new_value - tmp)> 3 ||(tmp - new_value)> 3)
	  {
		count ++;
		if(count < 50)
		{
		new_pressure_pa = tmp;
		//snav_info_print("%s: 111 baro: %f  new = %f  diff =  %f  count = %d",DRIVER_NAME, pressure_pa, new_value ,new_value - tmp,count);
		}
		else
		{
		count = 0;
		new_pressure_pa = new_value;
		//snav_info_print("%s: 222 baro: %f     %f  count=%d",DRIVER_NAME, pressure_pa, new_value - tmp,count);
		}
	  }
	  else
		{
		new_pressure_pa = tmp + (new_value - tmp) * 0.000123;
		//snav_info_print("%s: 333 baro: %f     %f",DRIVER_NAME, pressure_pa, new_value - tmp);
		}

	i++;
  if( i < 3)
	  {
	tmp = new_value;
	//snav_info_print("===rrd=== 111 i=%d  tmp=%f new=%f ",i,tmp,new_value);
	  }
	else
	  {
	//snav_info_print("===rrd=== 222i=%d  tmp=%f new=%f ",i,tmp,new_value);
	tmp = new_pressure_pa;
	  }

	  //tmp = pressure_pa;
	  */

/*
	float    offset_v = new_pressure_pa - bmp280_range_value_old;
	snav_info_print("===rrd===  offset_v=%f ",offset_v);

				if(offset_v>9 || offset_v <-9)
				{
					//if(offset_v>9)
					//{
						pressure_pa = new_pressure_pa;
					//}
				}
				else if(offset_v<1 && offset_v >-1)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.08;
				}

				else if(offset_v<2 && offset_v >-2)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.06;
				}
				else if(offset_v<3 && offset_v >-3)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.05;
				}
				else if(offset_v<4 && offset_v >-4)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.04;
				}
				else if(offset_v<5 && offset_v >-5)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.03;
				}
				else if(offset_v<6 && offset_v >-6)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.02;
				}
				else if(offset_v<7 && offset_v >-7)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.015;
				}
				else if(offset_v<8 && offset_v >-8)
				{
					pressure_pa = bmp280_range_value_old + offset_v*0.01;
				}
		 		else*/
				
					//pressure_pa = new_pressure_pa;

				
				
				//bmp280_range_value_old = pressure_pa;
				//bmp280_range_value_new_old = new_pressure_pa;




    //update data for external use
    pthread_mutex_lock(&baro_lock);

		 

    baro_read_counter++;
    baro_fresh_data    = 1;
    baro_read_time     = time_read_start;
    baro_pressure_pa   = new_value;
    baro_temperature_c = temperature_c;

    pthread_mutex_unlock(&baro_lock);

	//snav_info_print("%s:  pressure_pa:   new = %f ",DRIVER_NAME, pressure_pa);

    //snav_info_print("%s: baro: [%lld] %f %f",DRIVER_NAME, time_read_start, pressure_pa, temperature_c);
  }
  else
  {
    snav_error_print("%s: BMP280 read failed",DRIVER_NAME);
    return -2;
  }

  return 0;
}
