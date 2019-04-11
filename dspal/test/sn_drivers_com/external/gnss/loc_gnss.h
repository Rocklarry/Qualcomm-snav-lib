/* Copyright (c) 2016 The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*     * Redistributions of source code must retain the above copyright
*       notice, this list of conditions and the following disclaimer.
*     * Redistributions in binary form must reproduce the above
*       copyright notice, this list of conditions and the following
*       disclaimer in the documentation and/or other materials provided
*       with the distribution.
*     * Neither the name of The Linux Foundation nor the names of its
*       contributors may be used to endorse or promote products derived
*       from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef LE_INCLUDE_HARDWARE_GNSS_H
#define LE_INCLUDE_HARDWARE_GNSS_H

#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>
#include <stdbool.h>
#include <sys/types.h>

#define MAX_CONSTELLATIONS  6

typedef uint32_t     location_flags_t;

#define LOCATION_IS_LATITUDE_VALID      (1 << 0)
#define LOCATION_IS_LONGITUDE_VALID     (1 << 1)
#define LOCATION_IS_ALTITUDE_VALID      (1 << 2)
#define LOCATION_IS_ACCURACY_VALID      (1 << 3)
#define LOCATION_IS_SPEED_VALID         (1 << 4)
#define LOCATION_IS_SPEED_UNC_VALID     (1 << 5)
#define LOCATION_IS_HEADING_VALID       (1 << 6)
#define LOCATION_IS_HEADING_UNC_VALID   (1 << 7)
#define LOCATION_IS_VEL_EAST_VALID      (1 << 8)
#define LOCATION_IS_VEL_EAST_UNC_VALID  (1 << 9)
#define LOCATION_IS_VEL_NORTH_VALID     (1 << 10)
#define LOCATION_IS_VEL_NORTH_UNC_VALID (1 << 11)
#define LOCATION_IS_VEL_UP_VALID        (1 << 12)
#define LOCATION_IS_VEL_UP_UNC_VALID    (1 << 13)
#define LOCATION_IS_PDOP_VALID          (1 << 14)
#define LOCATION_IS_HDOP_VALID          (1 << 15)
#define LOCATION_IS_VDOP_VALID          (1 << 16)
#define LOCATION_IS_USED_IN_FIX_VALID   (1 << 17)
#define LOCATION_IS_TIMESTAMP_VALID     (1 << 18)
#define LOCATION_IS_GPS_TIME_VALID      (1 << 19)
#define LOCATION_IS_GPS_TIME_UNC_VALID  (1 << 20)
#define LOCATION_IS_2D_FIX_VALID        (1 << 21)
#define LOCATION_IS_ALTITUDE_MSL_VALID  (1 << 22)
#define LOCATION_IS_HOR_REL_VALID       (1 << 23)
#define LOCATION_IS_VER_REL_VALID       (1 << 24)
#define LOCATION_IS_HW_CLOCK_VALID      (1 << 25)
#define LOCATION_IS_ALTITUDE_UNC_VALID  (1 << 26)

typedef uint8_t     location_rel_t;

#define LOCATION_RELIABILITY_NOT_SET    0
#define LOCATION_RELIABILITY_VERY_LOW   1
#define LOCATION_RELIABILITY_LOW        2
#define LOCATION_RELIABILITY_MEDIUM     3
#define LOCATION_RELIABILITY_HIGH       4

typedef struct {
    uint16_t    gps_week;
    uint32_t    gps_tow_sec;
    uint32_t    gps_tow_ns;
} gps_time_t;

typedef struct {
    uint16_t            size;
    double              latitude_deg;
    double              longitude_deg;
    double              altitude_m;     /* altitude (WGS84 ellipsoid) */
    double              altitude_msl_m; /* altitude (mean sea level) */
    double              altitude_unc_m;
    double              accuracy_m;     /* expected accuracy in meters */
    double              speed_mps;
    double              speed_unc_mps;
    double              heading_deg;
    double              heading_unc_deg;
    double              vel_east_mps;
    double              vel_east_unc_mps;
    double              vel_north_mps;
    double              vel_north_unc_mps;
    double              vel_up_mps;
    double              vel_up_unc_mps;
    double              pdop;
    double              hdop;
    double              vdop;
    uint64_t            used_in_fix[MAX_CONSTELLATIONS]; /* for each constellation the used_in_fix has one bit per sv
                                                            (1 if the svid was used in computing the fix, 0 otherwise) */
    uint64_t            timestamp_ms;       /* timestamp of the fix in ms (UTC) */
    gps_time_t          gps_time;           /* GPS time in weeks, seconds, nanoseconds */
    double              gps_time_unc_ns;    /* GPS time uncertainty in nanoseconds */
    uint8_t             two_D_fix;          /* non-zero if the fix is 2D, zero if the fix is 3D */
    location_rel_t      horizontal_reliability;
    location_rel_t      vertical_reliability;
    location_flags_t    location_flags;
    uint64_t            hw_clock_count_ms;
    uint8_t             version;
} Location;

#define GNSS_CONSTELLATION_UNKNOWN  0
#define GNSS_CONSTELLATION_GPS      1
#define GNSS_CONSTELLATION_SBAS     2
#define GNSS_CONSTELLATION_GLONASS  3
#define GNSS_CONSTELLATION_QZSS     4
#define GNSS_CONSTELLATION_BEIDOU   5
#define GNSS_CONSTELLATION_GALILEO  6

typedef uint16_t    svid_t;
typedef uint8_t     sv_info_flags_t;
typedef uint8_t     gnss_constellation_t;

#define SV_INFO_IS_CN0_VALID        (1 << 0)
#define SV_INFO_IS_ELEVATION_VALID  (1 << 1)
#define SV_INFO_IS_AZIMUTH_VALID    (1 << 2)
#define SV_INFO_HAS_EPHEMERIS       (1 << 3)
#define SV_INFO_HAS_ALMANAC         (1 << 4)
#define SV_INFO_IS_BEING_TRACKED    (1 << 5)
#define SV_INFO_IS_TIMESTAMP_VALID  (1 << 6)

typedef struct {
    uint16_t                size;
    svid_t                  svid;
    gnss_constellation_t    constellation;
    float                   cN0_dbHz;
    float                   elevation_deg;
    float                   azimuth_deg;
    sv_info_flags_t         sv_info_flags;
    uint64_t                timestamp_ms;   /* timestamp in ms (UTC) */
} SV_Info;

typedef struct {
    void(*location_cb)(const Location* location);
    void(*sv_info_cb)(const SV_Info* sv_info, uint32_t num_of_svs);
    void(*get_XTRA_data_cb)();
} LocationCallbacks;

typedef enum {
    ERROR_NO_ERROR = 0,
    ERROR_SESSION_IN_PROGRESS = -1,
    ERROR_CANNOT_INITIALIZE = -2,
} ErrorCodes_t;

int gnss_start(LocationCallbacks* loc_cbs, uint32_t fix_rate_ms);
int gnss_stop();
int gnss_inject_XTRA_data(const char* XTRA_data, uint32_t XTRA_length);
int gnss_delete_aiding_data(uint32_t delete_aiding_mask);
int gnss_inject_information(uint8_t is_on_ground);
int gnss_unload();

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif /* LE_INCLUDE_HARDWARE_GNSS_H */

