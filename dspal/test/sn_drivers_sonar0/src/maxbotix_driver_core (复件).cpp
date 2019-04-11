/*
 * Copyright (c) 2016 Qualcomm Technologies, Inc.  All Rights Reserved.
 * Qualcomm Technologies Proprietary and Confidential.
 */
#define SONAR_BUF_SIZE 128 //maximum size of data in the rx callback

static uint32_t maxbotix_num_bytes;
static uint8_t  maxbotix_read_buf[SONAR_BUF_SIZE];
static uint32_t maxbotix_packet_length = 0;
static uint32_t maxbotix_fresh_data    = 0;
static uint32_t maxbotix_read_counter  = 0;
static uint32_t maxbotix_read_time     = 0;

void maxbotix_read_callback(void *context, char *buffer, size_t num_bytes)
{
  pthread_mutex_lock(&sonar_lock);
  if (((num_bytes==5) || (num_bytes==6)) && (buffer[0] == 'R'))
  {
    maxbotix_read_time = snav_get_time_1us();
    memcpy(maxbotix_read_buf,buffer,num_bytes);

//snav_info_print("===rrd===  maxbotix read_callback %d , %s",num_bytes,buffer);
    maxbotix_packet_length = num_bytes;
    maxbotix_fresh_data = 1;
    maxbotix_read_counter++;
  }
  maxbotix_num_bytes += num_bytes;
  pthread_mutex_unlock(&sonar_lock);
}

#define a   0.2            // ÂË²¨ÏµÊýa£¨0-1£© 
float sonar_range_value_old = 0;
float sonar_range_value_new_old = 0;
static int sonar0_update_data()
{
  pthread_mutex_lock(&sonar_lock);

  if (maxbotix_fresh_data != 0)
  {
    maxbotix_fresh_data = 0;

    if (maxbotix_read_buf[0] == 'R')
    {
      if (maxbotix_packet_length==5)
      {
        maxbotix_read_buf[4]          = 13; //terminate with endline just in case
	int i ;
		//for(i=0;i<8;i++)
		//snav_info_print("===rrd===  maxbotix packet %d",maxbotix_read_buf[i]);
        uint32_t distance_reading_raw = atoi((const char*)(maxbotix_read_buf+1));
		 float    sonar_range_new = distance_reading_raw * 0.01;
		//snav_info_print("===rrd===  maxbotix packet %d rgggg, %f",maxbotix_packet_length,sonar_range_new);
		if((sonar_range_new-sonar_range_value_old)>0.2 || (sonar_range_new-sonar_range_value_old)<0.2)
		{
			if(sonar_range_new == sonar_range_value_new_old)
			{
				sonar_range = sonar_range_new;
			}
			else
				sonar_range = sonar_range_value_old;
		}	
		else if(sonar_range_new >2.31)
			 sonar_range = 2.31;//7.56;
		 else
		 	//sonar_range = a*sonar_range_value+(1-a)*sonar_range_new;
		 sonar_range = sonar_range_new;

		sonar_range_value_old = sonar_range;
		sonar_range_value_new_old = sonar_range_new;
		//sonar_range                   = distance_reading_raw * 0.01;
        sonar_read_time               = maxbotix_read_time;
        sonar_read_counter            = maxbotix_read_counter;
        sonar_fresh_data              = 1;
		//sonar_range_value = sonar_range;
      }
      else if (maxbotix_packet_length==6)
      {
        maxbotix_read_buf[5]          = 13; //terminate with endline just in case
        uint32_t distance_reading_raw = atoi((const char*)(maxbotix_read_buf+1));
        sonar_range                   = distance_reading_raw * 0.001;
        sonar_read_time               = maxbotix_read_time;
        sonar_read_counter            = maxbotix_read_counter;
        sonar_fresh_data              = 1;
      }
      else
      {
        //some data error
      }

      //info_print_user("===rrd===  maxbotix packet %d range,%f",maxbotix_packet_length,sonar_range);
      //snav_info_print("===rrd===  maxbotix packet %d range, %f",maxbotix_packet_length,sonar_range);
    }
  }

  pthread_mutex_unlock(&sonar_lock);

  return 0;
}
