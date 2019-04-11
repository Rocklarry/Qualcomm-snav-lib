/*==============================================================================
 Copyright (c) 2016 Qualcomm Technologies, Inc.
 All rights reserved. Qualcomm Proprietary and Confidential.
 ==============================================================================*/

//TODO: support both slave addresses
#define MS5607_I2C_ADDRESS 0b1110111

#include <math.h>

//static int32_t  t_fine_;

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
} ms5607_sensor_calibration;

static ms5607_sensor_calibration calib_;




static int32_t  ms5607_verify_id()
{
  uint8_t buf[1]      = {255};
  uint8_t expected_id = 0x10;//¡Ÿ ±÷µ
  int ret = i2c_read_reg(file_des,0xA0,buf,1);//prom A0

  if (ret != 0) { snav_error_print("%s: ms5607 failed to read ID  buf=0x%x!!!",DRIVER_NAME,buf[0]); return -1; }

  if (buf[0] == expected_id)
  {
    snav_info_print("%s: ms5607 verify id: read %d, expected 0x%x: SUCCESS!",DRIVER_NAME,buf[0],expected_id);
    return -2;
  }

  snav_info_print("%s: ms5607 verify id: read 0x%x, expected %d: FAILURE!",DRIVER_NAME,buf[0],expected_id);
  return 0;
}


#define MS5607_ADDR_W		   0xEE // Module address write mode
#define MS5607_ADDR_R		   0xEF // Module address read mode

#define MS5607_CMD_RESET	   0x1E // ADC reset command
#define MS5607_CMD_ADC_READ    0x00 // ADC read command
#define MS5607_CMD_ADC_CONV    0x40 // ADC conversion command

#define MS5607_CMD_ADC_D1      0x00 // ADC D1 conversion
#define MS5607_CMD_ADC_D2      0x10 // ADC D2 conversion

#define MS5607_CMD_ADC_256     0x00 // ADC OSR=256
#define MS5607_CMD_ADC_512     0x02 // ADC OSR=512
#define MS5607_CMD_ADC_1024	   0x04 // ADC OSR=1024
#define MS5607_CMD_ADC_2048    0x06 // ADC OSR=2048
#define MS5607_CMD_ADC_4096    0x08 // ADC OSR=4096

#define MS5607_CMD_PROM_RD	   0xA0 // Prom read command

#define uchar unsigned char
#define uint unsigned int
#define ulong unsigned long


ulong  MS5607_D1; // ADC value of the pressure conversion
ulong  MS5607_D2; // ADC value of the temperature conversion

double MS5607_P; // compensated pressure value
double MS5607_T; // compensated temperature value
double MS5607_dT; // difference between actual and measured temperature
double MS5607_OFF; // offset at actual temperature
double MS5607_SENS; // sensitivity at actual temperature

double T2;
double OFF2;
double SENS2;


uint   MS5607_C[8];

void ms5607_cmd_reset()
{
	//uint8_t config[2] = {MS5607_CMD_RESET,0xff};

	uint8_t config[2] = {0xee,MS5607_CMD_RESET};


	//int ret = i2c_write_reg(file_des,config[0],&config[1],1);
	int ret = spi_write_reg(file_des,&config[1],1);
	snav_info_print("%s ********rest******  ret = %d",DRIVER_NAME,ret);

	// int ret2 = i2c_write_single_reg(file_des, MS5607_CMD_RESET, MS5607_CMD_RESET); //set/reset  (0x08,0x08)
   // snav_info_print("%s   ********rest******   ret2 = %d",DRIVER_NAME,ret2);


	 

}


void  MS5607_cmd_prom()
{

	uint8_t   data0[2] = {255,255};
	uint8_t   data1[2] = {255,255};
	uint8_t   data2[2] = {255,255};
	uint8_t   data3[2] = {255,255};
	uint8_t   data4[2] = {255,255};
	uint8_t   data5[2] = {255,255};
	uint8_t   data6[2] = {255,255};
	uint8_t   data7[2] = {255,255};


int ret0 = spi_read_reg(file_des,0xA0, data0, 2);
int ret1 = spi_read_reg(file_des,0xA2, data1, 2);
int ret2 = spi_read_reg(file_des,0xA4, data2, 2);
int ret3 = spi_read_reg(file_des,0xA6, data3, 2);
int ret4 = spi_read_reg(file_des,0xA8, data4, 2);
int ret5 = spi_read_reg(file_des,0xAA, data5, 2);
int ret6 = spi_read_reg(file_des,0xAC, data6, 2);
int ret7 = spi_read_reg(file_des,0xAE, data7, 2);

 usleep(1000);

//snav_error_print(" data0[0]=0x%x data0[1]=0x%x data1[0]=0x%x data1[1]=0x%x data2[0]=0x%x data2[1]=0x%x data3[0]=0x%x  data3[1]=0x%x ",	
//data0[0],data0[1],data1[0],data1[1] ,data2[0],data2[1],data3[0],data3[1]);


//snav_error_print("data4[0]=0x%x data4[1]=0x%x data5[0]=0x%x data5[1]=0x%x data6[0]=0x%x data6[1]=0x%x data7[0]=0x%x data7[1]=0x%x",
//	data4[0],data4[1],data5[0],data5[1],data6[0],data6[1],data7[0],data7[1]);


MS5607_C[0] = (data0[0]<<8)|data0[1];
MS5607_C[1] = (data1[0]<<8)|data1[1];
MS5607_C[2] = (data2[0]<<8)|data2[1];
MS5607_C[3] = (data3[0]<<8)|data3[1];
MS5607_C[4] = (data4[0]<<8)|data4[1];
MS5607_C[5] = (data5[0]<<8)|data5[1];
MS5607_C[6] = (data6[0]<<8)|data6[1];
MS5607_C[7] = (data7[0]<<8)|data7[1];

snav_error_print("%d   %d   %d   %d   %d   %d   %d   %d",MS5607_C[0],MS5607_C[1],MS5607_C[2],MS5607_C[3],MS5607_C[4],MS5607_C[5],MS5607_C[6],MS5607_C[7]);

	}


static uint ms5607_prom(int8_t coef_num)
{
    uint8_t rxbuf[2] = { 0, 0 };

  //  ms5611ReadCommand(busdev, MS5607_CMD_PROM_RD + coef_num * 2, 2, rxbuf); // send PROM READ command

	int ret = spi_read_reg(file_des,MS5607_CMD_PROM_RD+coef_num * 2, rxbuf, 2);

	snav_error_print("0x%x 0x%x ",rxbuf[0],rxbuf[1]);

    return rxbuf[0] << 8 | rxbuf[1];
}



uchar MS5607_crc4(uint n_prom[])
{
	int cnt; // simple counter
	uint n_rem; // crc reminder
	uint crc_read; // original value of the crc
	uchar n_bit;
	n_rem = 0x00;
	crc_read=n_prom[7]; //save read CRC
	n_prom[7]=(0xFF00 & (n_prom[7])); //CRC byte is replaced by 0
	for (cnt = 0; cnt < 16; cnt++) // operation is performed on bytes
	{ // choose LSB or MSB
		if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
		else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
		for (n_bit = 8; n_bit > 0; n_bit--)
		{
			if (n_rem & (0x8000))
			{
				n_rem = (n_rem << 1) ^ 0x3000;
			}
			else
			{
				n_rem = (n_rem << 1);
			}
		}
	}
	n_rem= (0x000F & (n_rem >> 12)); // // final 4-bit reminder is CRC code
	n_prom[7]=crc_read; // restore the crc_read to its original place
	return (n_rem ^ 0x00);
}




int   ms5611_crc(uint *prom)
{
    int32_t i, j;
    uint32_t res = 0;
    uint8_t crc = prom[7] & 0xF;
    prom[7] &= 0xFF00;

    bool blankEeprom = true;

    for (i = 0; i < 16; i++) {
        if (prom[i >> 1]) {
            blankEeprom = false;
        }
        if (i & 1)
            res ^= ((prom[i >> 1]) & 0x00FF);
        else
            res ^= (prom[i >> 1] >> 8);
        for (j = 8; j > 0; j--) {
            if (res & 0x8000)
                res ^= 0x1800;
            res <<= 1;
        }
    }
    prom[7] |= crc;
    if (!blankEeprom && crc == ((res >> 12) & 0xF))
        return 0;

    return -1;
}



int  MS5607_Init_CRC(void)
   {
	uchar i,n_crc,ci;

	//ms5607_cmd_reset();		// reset the module after power up
	
	MS5607_cmd_prom();
	/*for (i=0;i<8;i++)
	{
		MS5607_C[i]=ms5607_prom(i);
		usleep(10000);
	}
*/
	ci=MS5607_C[7]&0x000f;		//Low 4bit
	n_crc=MS5607_crc4(MS5607_C);

	usleep(10000);

	if(n_crc==ci)
	   {
		snav_error_print("%s: ****************MS5607_Init_CRC   Successful");
	return 0;
	   }
	else
	   {
	snav_error_print("%s: ****************MS5607_Init_CRC  Fail ");
	return 1;
	   }

	}


static int ms5607_init()
{
  int result = 1;
	usleep(13000);



   /*
    * Initialize the write buffers in preparation for a read/write sequence.
    */
  /* write_data_buffer[SPI_LOOPBACK_TEST_TRANSMIT_BUFFER_LENGTH-1] = 0;
   init_write_buffer(write_data_buffer, MAX_LEN_TRANSMIT_BUFFER_IN_BYTES-1);


    result = ioctl(file_des, SPI_IOCTL_LOOPBACK_TEST, &loopback);
   if (result < 1)
   {
      return -1;
   }*/



	uint8_t   data[1] = {255};
	int ret = spi_read_reg(file_des,0xA0, data, 1);
	snav_info_print("%s  read a0 0x%x  ret = %d",DRIVER_NAME,data[0],ret);


  ms5607_cmd_reset();
	
	while(1){
		if(!MS5607_Init_CRC())
			break;

	}

//snav_error_print("%s: ****************MS5607_Init_CRC = %d",DRIVER_NAME, MS5607_Init_CRC());

  return 0;
}



static int ms5607_read_data()
{


  uint64_t time_read_start  = snav_get_time_1us();



	uint8_t data_d1[3]={255,255,255};
	uint8_t data_d2[3]={255,255,255};
	
	usleep(13000);
		

	uint8_t config[2] = {0xee,0x58};
	int retw1 = spi_write_reg(file_des,&config[1],1);
	if(retw1!=0)
	snav_info_print("%s  D2  ret = %d",DRIVER_NAME,retw1);

	usleep(13000);
	
	int ret2 = spi_read_reg(file_des,0x00, data_d2, 3);
	// snav_info_print(" D2 0x%x 0x%x 0x%x   ret=%d",data_d2[0],data_d2[1],data_d2[2],ret2);


	uint8_t configd1[2] = {0xee,0x48};
	int retd1 = spi_write_reg(file_des,&configd1[1],1);
	if(retw1!=0)
	snav_info_print("%s D1  ret = %d",DRIVER_NAME,retd1);

	usleep(13000);
	int ret1 =  spi_read_reg(file_des,0x00, data_d1, 3);
	// snav_info_print(" D1 0x%x 0x%x 0x%x   ret=%d",data_d1[0],data_d1[1],data_d1[2],ret1);

	MS5607_D1 = (data_d1[0]<<16)|(data_d1[1]<<8)|data_d1[2];
	MS5607_D2 = (data_d2[0]<<16)|(data_d2[1]<<8)|data_d2[2];




 	MS5607_dT=MS5607_D2-MS5607_C[5]*pow(2,8);
	MS5607_OFF=MS5607_C[2]*pow(2,17)+MS5607_dT*MS5607_C[4]/pow(2,6);
	MS5607_SENS=MS5607_C[1]*pow(2,16)+MS5607_dT*MS5607_C[3]/pow(2,7);
	MS5607_T=2000+(MS5607_dT*MS5607_C[6])/pow(2,23);
	
		if(MS5607_T <2000){
			T2 = pow(MS5607_dT,2)/pow(2,31);
			OFF2 = 61*pow((MS5607_T - 2000),2)/pow(2,4);
			SENS2 = 2*pow((MS5607_T-2000),2);

			if(MS5607_T < -1500){
				OFF2 = OFF2+15*pow((MS5607_T+1500),2);
				SENS2 = SENS2+8*pow((MS5607_T+1500),2);
			}

		}else{
			T2 = 0;
			OFF2 = 0;
			SENS2 = 0;
		}
		MS5607_T = MS5607_T-T2;
		MS5607_OFF = MS5607_OFF-OFF2;
		MS5607_SENS = MS5607_SENS - SENS2;

	MS5607_P=(((MS5607_D1*MS5607_SENS)/pow(2,21)-MS5607_OFF)/pow(2,15));



    pthread_mutex_lock(&baro_lock);

    baro_read_counter++;
    baro_fresh_data    = 1;
    baro_read_time     = time_read_start;
    baro_pressure_pa   = MS5607_P;
    baro_temperature_c = MS5607_T/100;

    pthread_mutex_unlock(&baro_lock);

	snav_error_print(" ms5607 P=%f   T=%f",baro_pressure_pa ,baro_temperature_c);

  return 0;
}
