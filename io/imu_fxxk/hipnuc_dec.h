/*
 * Copyright (c) 2006-2024, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef IO__HIPNUC_IMU__HIPNUC_DEC_H_
#define IO__HIPNUC_IMU__HIPNUC_DEC_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#define HIPNUC_MAX_RAW_SIZE (512)

typedef struct __attribute__((__packed__))
{
  uint8_t tag;
  uint16_t main_status;
  int8_t temp;
  float air_pressure;
  uint32_t system_time;
  float acc[3];
  float gyr[3];
  float mag[3];
  float roll;
  float pitch;
  float yaw;
  float quat[4];
} hi91_t;

typedef struct __attribute__((__packed__))
{
  uint8_t tag;
  uint16_t main_status;
  uint8_t ins_status;
  uint16_t gpst_wn;
  uint32_t gpst_tow;
  uint16_t reserved;
  int16_t gyr_b[3];
  int16_t acc_b[3];
  int16_t mag_b[3];
  int16_t air_pressure;
  int16_t reserved1;
  int8_t temperature;
  uint8_t utc_year;
  uint8_t utc_month;
  uint8_t utc_day;
  uint8_t utc_hour;
  uint8_t utc_min;
  uint16_t utc_msec;
  int16_t roll;
  int16_t pitch;
  uint16_t yaw;
  int16_t quat[4];
  int32_t ins_lon;
  int32_t ins_lat;
  int32_t ins_msl;
  uint8_t pdop;
  uint8_t hdop;
  uint8_t solq_pos;
  uint8_t nv_pos;
  uint8_t solq_heading;
  uint8_t nv_heading;
  uint8_t diff_age;
  int16_t undulation;
  uint8_t ant_status;
  int16_t vel_enu[3];
  int16_t acc_enu[3];
  int32_t gnss_lon;
  int32_t gnss_lat;
  int32_t gnss_msl;
  uint8_t reserved2[2];
} hi81_t;

#define HI83_BMAP_ACC_B (1u << 0)
#define HI83_BMAP_GYR_B (1u << 1)
#define HI83_BMAP_MAG_B (1u << 2)
#define HI83_BMAP_RPY (1u << 3)
#define HI83_BMAP_QUAT (1u << 4)
#define HI83_BMAP_SYSTEM_TIME (1u << 5)
#define HI83_BMAP_UTC (1u << 6)
#define HI83_BMAP_AIR_PRESSURE (1u << 7)
#define HI83_BMAP_TEMPERATURE (1u << 8)
#define HI83_BMAP_INCLINATION (1u << 9)
#define HI83_BMAP_HSS (1u << 10)
#define HI83_BMAP_HSS_FRQ (1u << 11)
#define HI83_BMAP_VEL_ENU (1u << 12)
#define HI83_BMAP_ACC_ENU (1u << 13)
#define HI83_BMAP_INS_LON_LAT_MSL (1u << 14)
#define HI83_BMAP_GNSS_QUALITY_NV (1u << 15)
#define HI83_BMAP_OD_SPEED (1u << 16)
#define HI83_BMAP_UNDULATION (1u << 17)
#define HI83_BMAP_DIFF_AGE (1u << 18)
#define HI83_BMAP_NODE_ID (1u << 19)
#define HI83_BMAP_GNSS_LON_LAT_MSL (1u << 30)
#define HI83_BMAP_GNSS_VEL (1u << 31)

typedef struct __attribute__((__packed__))
{
  uint8_t tag;
  uint16_t main_status;
  uint8_t ins_status;
  uint32_t data_bitmap;
  float acc_b[3];
  float gyr_b[3];
  float mag_b[3];
  float rpy[3];
  float quat[4];
  uint64_t system_time_us;
  struct __attribute__((__packed__))
  {
    uint8_t year;
    uint8_t month;
    uint8_t day;
    uint8_t hour;
    uint8_t min;
    uint16_t sec_ms;
    uint8_t rev;
  } utc;
  float air_pressure;
  float temperature;
  float inclination[3];
  float hss[3];
  float hss_frq[3];
  float vel_enu[3];
  float acc_enu[3];
  double ins_lon_lat_msl[3];
  uint8_t solq_pos;
  uint8_t nv_pos;
  uint8_t solq_heading;
  uint8_t nv_heading;
  float od_speed;
  float undulation;
  float diff_age;
  struct __attribute__((__packed__))
  {
    uint8_t node_id;
    uint8_t reserved[3];
  } node;
  double gnss_lon_lat_msl[3];
  float gnss_vel[3];
} hi83_t;

typedef struct
{
  int nbyte;
  int len;
  uint8_t buf[HIPNUC_MAX_RAW_SIZE];
  hi91_t hi91;
  hi81_t hi81;
  hi83_t hi83;
} hipnuc_raw_t;

int hipnuc_input(hipnuc_raw_t * raw, uint8_t data);
int hipnuc_dump_packet(hipnuc_raw_t * raw, char * buf, size_t buf_size);

#ifdef __cplusplus
}
#endif

#endif
