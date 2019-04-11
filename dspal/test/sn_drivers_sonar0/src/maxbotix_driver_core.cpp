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


    float           _cutoff_freq;
	float           _a1;
	float           _a2;
	float           _b0;
	float           _b1;
	float           _b2;
	float           _delay_element_1;        // buffered sample -1
	float           _delay_element_2;        // buffered sample -2

#define C_PI   3.14159265358979323846    /* PI */



#include <cmath>

void set_cutoff_frequency(float sample_freq, float cutoff_freq)
{
	_cutoff_freq = cutoff_freq;

	if (_cutoff_freq <= 0.0f) {
		// no filtering
		return;
	}

	float fr = sample_freq / _cutoff_freq;
	float ohm = tanf(C_PI / fr);
	float c = 1.0f + 2.0f * cosf(C_PI / 4.0f) * ohm + ohm * ohm;
	_b0 = ohm * ohm / c;
	_b1 = 2.0f * _b0;
	_b2 = _b0;
	_a1 = 2.0f * (ohm * ohm - 1.0f) / c;
	_a2 = (1.0f - 2.0f * cosf(C_PI / 4.0f) * ohm + ohm * ohm) / c;

	snav_info_print("===rrd===  %f %f %f %f %f %f %f  %f  ",fr,ohm,c,_b0,_b1,_b2,_a1,_a2);
}

float apply(float sample)
{
	if (_cutoff_freq <= 0.0f) {
		// no filtering
		return sample;
	}

	// do the filtering
	float delay_element_0 = sample - _delay_element_1 * _a1 - _delay_element_2 * _a2;

	if (!isfinite(delay_element_0)) {
		// don't allow bad values to propagate via the filter
		delay_element_0 = sample;
	}

	float output = delay_element_0 * _b0 + _delay_element_1 * _b1 + _delay_element_2 * _b2;

	_delay_element_2 = _delay_element_1;
	_delay_element_1 = delay_element_0;

	// return the value.  Should be no need to check limits
	return output;
}

float reset(float sample)
{
	float dval = sample / (_b0 + _b1 + _b2);
	_delay_element_1 = dval;
	_delay_element_2 = dval;

	snav_info_print("===rrd===  %f ",dval);
	return apply(sample);
}











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
      			if (maxbotix_packet_length==5 || maxbotix_packet_length==6)
      			{
				if(maxbotix_packet_length==5 )
        				maxbotix_read_buf[4]          = 13; //terminate with endline just in case
				else if(maxbotix_packet_length==6 )
					maxbotix_read_buf[5]          = 13; 
		
        			uint32_t distance_reading_raw = atoi((const char*)(maxbotix_read_buf+1));
		 		float    sonar_range_new = distance_reading_raw * 0.01;
                 		float    offset_v = sonar_range_new-sonar_range_value_old;

				if(offset_v>0.5 || offset_v <-0.5)
				{
					if(offset_v>0.5)
					{
						sonar_range = sonar_range_value_old + 0.5;
					}
					else
						sonar_range = sonar_range_value_old - 0.5;
				}
				else if(offset_v<0.05 && offset_v >-0.05)
				{
					sonar_range = sonar_range_value_old + offset_v;
				}
				else if(offset_v<0.10 && offset_v >-0.10)
				{
					sonar_range = sonar_range_value_old + offset_v*0.5;
				}/*
				else if(offset_v<0.20 && offset_v >-0.20)
				{
					sonar_range = sonar_range_value_old + offset_v*0.25;
				}
				else if(offset_v<0.30 && offset_v >-0.30)
				{
					sonar_range = sonar_range_value_old + offset_v*0.17;
				}
				else if(offset_v<0.40 && offset_v >-0.40)
				{
					sonar_range = sonar_range_value_old + offset_v*0.13;
				}
				else if(offset_v<0.50 && offset_v >-0.50)
				{
					sonar_range = sonar_range_value_old + offset_v*0.1;
				}*/
		 		else
					sonar_range = sonar_range_new;

				
				
				sonar_range_value_old = sonar_range;
				sonar_range_value_new_old = sonar_range_new;

				//sonar_range                   = distance_reading_raw * 0.01;
        			sonar_read_time               = maxbotix_read_time;
        			sonar_read_counter            = maxbotix_read_counter;
        			sonar_fresh_data              = 1;
		//sonar_range_value = sonar_range;
      			}
     // else if (maxbotix_packet_length==6)
     // {
      //  maxbotix_read_buf[5]          = 13; //terminate with endline just in case
     //   uint32_t distance_reading_raw = atoi((const char*)(maxbotix_read_buf+1));
     //   sonar_range                   = distance_reading_raw * 0.01;
     //   sonar_read_time               = maxbotix_read_time;
     //   sonar_read_counter            = maxbotix_read_counter;
     //   sonar_fresh_data              = 1;
      	//	}
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
