/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */


#define SONAR_BUF_SIZE 128 //maximum size of data in the rx callback

#include <string.h>
static uint32_t maxbotix_num_bytes;
static uint8_t  maxbotix_read_buf[SONAR_BUF_SIZE];
static uint32_t maxbotix_packet_length = 0;
static uint32_t maxbotix_fresh_data    = 0;
static uint32_t maxbotix_read_counter  = 0;
static uint32_t maxbotix_read_time     = 0;



#define SERIAL_SIZE_OF_DATA_BUFFER 128

void multi_port_read_callback(void *context, char *buffer, size_t num_bytes)                                                                                                                                                             
 {
     int rx_dev_id = (int)context;
     //char rx_buffer[SERIAL_SIZE_OF_DATA_BUFFER];
	 pthread_mutex_lock(&sonar_lock);
 
     if (num_bytes > 0) {
	 maxbotix_read_time = snav_get_time_1us();
         memcpy(maxbotix_read_buf, buffer, num_bytes);
         maxbotix_read_buf[num_bytes] = 0;
         snav_info_print("===rrd===  /dev/tty-%d read callback received bytes =[%d]:  rx_buffer =%s",
              rx_dev_id, num_bytes, maxbotix_read_buf);
			
		  maxbotix_packet_length = num_bytes;
		  maxbotix_fresh_data = 1;
		  maxbotix_read_counter++;
 
     } else {
         snav_info_print("===rrd=== error: read callback with no data in the buffer");
     }

	 maxbotix_num_bytes += num_bytes;
	 pthread_mutex_unlock(&sonar_lock);
 }

/*
void maxbotix_read_callback(void *context, char *buffer, size_t num_bytes)
{
	snav_info_print("===rrd===  start maxbotix read_callback %d , %s",num_bytes,buffer);
	//  ===rrd===  start maxbotix read_callback 5 , R024 

  pthread_mutex_lock(&sonar_lock);
  if (((num_bytes==5) || (num_bytes==6)) && (buffer[0] == 'R'))
  {
    maxbotix_read_time = snav_get_time_1us();
	maxbotix_read_buf[num_bytes] = 0;
    memcpy(maxbotix_read_buf,buffer,num_bytes);

snav_info_print("===rrd=== read:num=%d , buffer=%s , read_buf=%s",num_bytes,buffer,maxbotix_read_buf);
    maxbotix_packet_length = num_bytes;
    maxbotix_fresh_data = 1;
    maxbotix_read_counter++;
  }

  snav_info_print("===rrd===  Failed maxbotix  %d , %s , %d",num_bytes,buffer,maxbotix_fresh_data);
  maxbotix_num_bytes += num_bytes;
  pthread_mutex_unlock(&sonar_lock);
}
*/



uint8_t maxbotix_read_buf_tmp[SERIAL_SIZE_OF_DATA_BUFFER];

static int sonar0_update_data()
{


  	pthread_mutex_lock(&sonar_lock);

  	if (maxbotix_fresh_data != 0)
  	{
	snav_info_print("===rrd=== ******111******* maxbotix_fresh_data %d maxbotix_read_buf=%s",maxbotix_fresh_data,maxbotix_read_buf);
	 maxbotix_read_buf_tmp[maxbotix_packet_length] = 0;

	memcpy(maxbotix_read_buf_tmp, maxbotix_read_buf+10, 4);
	snav_info_print("===rrd=== ******333******* maxbotix_fresh_data %d maxbotix_read_buf          =%s",maxbotix_fresh_data,maxbotix_read_buf_tmp);

    maxbotix_fresh_data = 0;
	uint32_t distance_reading_raw = atoi((const char*)(maxbotix_read_buf_tmp));

	//snav_info_print("===rrd===  maxbotix packet %d range, %f",maxbotix_packet_length,distance_reading_raw);

	sonar_range                   = distance_reading_raw * 0.01;
	sonar_read_time               = maxbotix_read_time;
	sonar_read_counter            = maxbotix_read_counter;
	sonar_fresh_data              = 1;

	snav_info_print("===rrd===  maxbotix packet %d range, %f",maxbotix_packet_length,sonar_range);
	}

  	pthread_mutex_unlock(&sonar_lock);

  	return 0;
}
