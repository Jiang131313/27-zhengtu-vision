/*
 * Copyright (c) 2006-2024, HiPNUC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "hipnuc_dec.h"

#define CHSYNC1 (0x5A)
#define CHSYNC2 (0xA5)
#define CH_HDR_SIZE (0x06)

#define HIPNUC_ID_HI91 (0x91)
#define HIPNUC_ID_HI81 (0x81)
#define HIPNUC_ID_HI83 (0x83)

#ifndef D2R
#define D2R (0.0174532925199433F)
#endif

#ifndef R2D
#define R2D (57.2957795130823F)
#endif

#ifndef GRAVITY
#define GRAVITY (9.8F)
#endif

#define I2(p) (*((int16_t *)(p)))

static void hipnuc_crc16(uint16_t * inital, const uint8_t * buf, uint32_t len);

static uint16_t U2(uint8_t * p)
{
  uint16_t u;
  memcpy(&u, p, 2);
  return u;
}

static float R4(uint8_t * p)
{
  float r;
  memcpy(&r, p, 4);
  return r;
}

static uint32_t U4(uint8_t * p)
{
  uint32_t u;
  memcpy(&u, p, 4);
  return u;
}

static double D8(uint8_t * p)
{
  double d;
  memcpy(&d, p, 8);
  return d;
}

static uint64_t U8(uint8_t * p)
{
  uint64_t u;
  memcpy(&u, p, 8);
  return u;
}

static int parse_data(hipnuc_raw_t * raw)
{
  int ofs = 0;
  uint8_t * p = &raw->buf[CH_HDR_SIZE];

  raw->hi91.tag = 0;
  raw->hi81.tag = 0;
  raw->hi83.tag = 0;

  while (ofs < raw->len) {
    switch (p[ofs]) {
      case HIPNUC_ID_HI91:
        memcpy(&raw->hi91, p + ofs, sizeof(hi91_t));
        ofs += sizeof(hi91_t);
        break;
      case HIPNUC_ID_HI81:
        memcpy(&raw->hi81, p + ofs, sizeof(hi81_t));
        ofs += sizeof(hi81_t);
        break;
      case HIPNUC_ID_HI83:
      {
        int idx = ofs + 8;
        uint32_t bm = 0;

        raw->hi83.tag = HIPNUC_ID_HI83;
        raw->hi83.main_status = U2(p + ofs + 1);
        raw->hi83.ins_status = p[ofs + 3];
        raw->hi83.data_bitmap = U4(p + ofs + 4);
        bm = raw->hi83.data_bitmap;

        if (bm & HI83_BMAP_ACC_B) {
          raw->hi83.acc_b[0] = R4(p + idx + 0);
          raw->hi83.acc_b[1] = R4(p + idx + 4);
          raw->hi83.acc_b[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_GYR_B) {
          raw->hi83.gyr_b[0] = R4(p + idx + 0);
          raw->hi83.gyr_b[1] = R4(p + idx + 4);
          raw->hi83.gyr_b[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_MAG_B) {
          raw->hi83.mag_b[0] = R4(p + idx + 0);
          raw->hi83.mag_b[1] = R4(p + idx + 4);
          raw->hi83.mag_b[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_RPY) {
          raw->hi83.rpy[0] = R4(p + idx + 0);
          raw->hi83.rpy[1] = R4(p + idx + 4);
          raw->hi83.rpy[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_QUAT) {
          raw->hi83.quat[0] = R4(p + idx + 0);
          raw->hi83.quat[1] = R4(p + idx + 4);
          raw->hi83.quat[2] = R4(p + idx + 8);
          raw->hi83.quat[3] = R4(p + idx + 12);
          idx += 16;
        }
        if (bm & HI83_BMAP_SYSTEM_TIME) {
          raw->hi83.system_time_us = U8(p + idx);
          idx += 8;
        }
        if (bm & HI83_BMAP_UTC) {
          raw->hi83.utc.year = p[idx + 0];
          raw->hi83.utc.month = p[idx + 1];
          raw->hi83.utc.day = p[idx + 2];
          raw->hi83.utc.hour = p[idx + 3];
          raw->hi83.utc.min = p[idx + 4];
          raw->hi83.utc.sec_ms = U2(p + idx + 5);
          raw->hi83.utc.rev = p[idx + 7];
          idx += 8;
        }
        if (bm & HI83_BMAP_AIR_PRESSURE) {
          raw->hi83.air_pressure = R4(p + idx);
          idx += 4;
        }
        if (bm & HI83_BMAP_TEMPERATURE) {
          raw->hi83.temperature = R4(p + idx);
          idx += 4;
        }
        if (bm & HI83_BMAP_INCLINATION) {
          raw->hi83.inclination[0] = R4(p + idx + 0);
          raw->hi83.inclination[1] = R4(p + idx + 4);
          raw->hi83.inclination[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_HSS) {
          raw->hi83.hss[0] = R4(p + idx + 0);
          raw->hi83.hss[1] = R4(p + idx + 4);
          raw->hi83.hss[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_HSS_FRQ) {
          raw->hi83.hss_frq[0] = R4(p + idx + 0);
          raw->hi83.hss_frq[1] = R4(p + idx + 4);
          raw->hi83.hss_frq[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_VEL_ENU) {
          raw->hi83.vel_enu[0] = R4(p + idx + 0);
          raw->hi83.vel_enu[1] = R4(p + idx + 4);
          raw->hi83.vel_enu[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_ACC_ENU) {
          raw->hi83.acc_enu[0] = R4(p + idx + 0);
          raw->hi83.acc_enu[1] = R4(p + idx + 4);
          raw->hi83.acc_enu[2] = R4(p + idx + 8);
          idx += 12;
        }
        if (bm & HI83_BMAP_INS_LON_LAT_MSL) {
          raw->hi83.ins_lon_lat_msl[0] = D8(p + idx + 0);
          raw->hi83.ins_lon_lat_msl[1] = D8(p + idx + 8);
          raw->hi83.ins_lon_lat_msl[2] = D8(p + idx + 16);
          idx += 24;
        }
        if (bm & HI83_BMAP_GNSS_QUALITY_NV) {
          raw->hi83.solq_pos = p[idx + 0];
          raw->hi83.nv_pos = p[idx + 1];
          raw->hi83.solq_heading = p[idx + 2];
          raw->hi83.nv_heading = p[idx + 3];
          idx += 4;
        }
        if (bm & HI83_BMAP_OD_SPEED) {
          raw->hi83.od_speed = R4(p + idx);
          idx += 4;
        }
        if (bm & HI83_BMAP_UNDULATION) {
          raw->hi83.undulation = R4(p + idx);
          idx += 4;
        }
        if (bm & HI83_BMAP_DIFF_AGE) {
          raw->hi83.diff_age = R4(p + idx);
          idx += 4;
        }
        if (bm & HI83_BMAP_NODE_ID) {
          raw->hi83.node.node_id = p[idx + 0];
          raw->hi83.node.reserved[0] = p[idx + 1];
          raw->hi83.node.reserved[1] = p[idx + 2];
          raw->hi83.node.reserved[2] = p[idx + 3];
          idx += 4;
        }
        if (bm & HI83_BMAP_GNSS_LON_LAT_MSL) {
          raw->hi83.gnss_lon_lat_msl[0] = D8(p + idx + 0);
          raw->hi83.gnss_lon_lat_msl[1] = D8(p + idx + 8);
          raw->hi83.gnss_lon_lat_msl[2] = D8(p + idx + 16);
          idx += 24;
        }
        if (bm & HI83_BMAP_GNSS_VEL) {
          raw->hi83.gnss_vel[0] = R4(p + idx + 0);
          raw->hi83.gnss_vel[1] = R4(p + idx + 4);
          raw->hi83.gnss_vel[2] = R4(p + idx + 8);
          idx += 12;
        }

        ofs = idx;
        break;
      }
      default:
        ofs++;
        break;
    }
  }

  return 1;
}

static int decode_hipnuc(hipnuc_raw_t * raw)
{
  uint16_t crc = 0;

  hipnuc_crc16(&crc, raw->buf, CH_HDR_SIZE - 2);
  hipnuc_crc16(&crc, raw->buf + CH_HDR_SIZE, raw->len);
  if (crc != U2(raw->buf + (CH_HDR_SIZE - 2))) return -1;

  return parse_data(raw);
}

static int sync_hipnuc(uint8_t * buf, uint8_t data)
{
  buf[0] = buf[1];
  buf[1] = data;
  return buf[0] == CHSYNC1 && buf[1] == CHSYNC2;
}

int hipnuc_input(hipnuc_raw_t * raw, uint8_t data)
{
  if (raw->nbyte == 0) {
    if (!sync_hipnuc(raw->buf, data)) return 0;
    raw->nbyte = 2;
    return 0;
  }

  raw->buf[raw->nbyte++] = data;

  if (raw->nbyte == CH_HDR_SIZE) {
    raw->len = U2(raw->buf + 2);
    if (raw->len > (HIPNUC_MAX_RAW_SIZE - CH_HDR_SIZE)) {
      raw->nbyte = 0;
      return -1;
    }
  }

  if (raw->nbyte < CH_HDR_SIZE || raw->nbyte < (raw->len + CH_HDR_SIZE)) return 0;

  raw->nbyte = 0;
  return decode_hipnuc(raw);
}

int hipnuc_dump_packet(hipnuc_raw_t * raw, char * buf, size_t buf_size)
{
  int ret = 0;
  int written = 0;

  if (raw->hi91.tag == HIPNUC_ID_HI91) {
    ret = snprintf(
      buf + written,
      buf_size - written,
      "{\n"
      "  \"type\": \"HI91\",\n"
      "  \"main_status\": [0x%X],\n"
      "  \"system_time\": %d,\n"
      "  \"acc\": [%.3f, %.3f, %.3f],\n"
      "  \"gyr\": [%.3f, %.3f, %.3f],\n"
      "  \"mag\": [%.3f, %.3f, %.3f],\n"
      "  \"pitch\": %.2f,\n"
      "  \"roll\": %.2f,\n"
      "  \"yaw\": %.2f,\n"
      "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
      "  \"air_pressure\": %.1f\n"
      "}\n",
      raw->hi91.main_status,
      raw->hi91.system_time,
      raw->hi91.acc[0] * GRAVITY,
      raw->hi91.acc[1] * GRAVITY,
      raw->hi91.acc[2] * GRAVITY,
      raw->hi91.gyr[0],
      raw->hi91.gyr[1],
      raw->hi91.gyr[2],
      raw->hi91.mag[0],
      raw->hi91.mag[1],
      raw->hi91.mag[2],
      raw->hi91.pitch,
      raw->hi91.roll,
      raw->hi91.yaw,
      raw->hi91.quat[0],
      raw->hi91.quat[1],
      raw->hi91.quat[2],
      raw->hi91.quat[3],
      raw->hi91.air_pressure);
  } else if (raw->hi81.tag == HIPNUC_ID_HI81) {
    ret = snprintf(
      buf + written,
      buf_size - written,
      "{\n"
      "  \"type\": \"HI81\",\n"
      "  \"main_status\": %d,\n"
      "  \"ins_status\": %d,\n"
      "  \"gpst_wn\": %d,\n"
      "  \"gpst_tow\": %d,\n"
      "  \"gyr\": [%.3f, %.3f, %.3f],\n"
      "  \"acc\": [%.3f, %.3f, %.3f],\n"
      "  \"mag\": [%.3f, %.3f, %.3f],\n"
      "  \"air_pressure\": %.1f,\n"
      "  \"temperature\": %d,\n"
      "  \"utc\": \"20%02d-%02d-%02d %02d:%02d:%02d.%03d\",\n"
      "  \"pitch\": %.2f,\n"
      "  \"roll\": %.2f,\n"
      "  \"yaw\": %.2f,\n"
      "  \"quat\": [%.3f, %.3f, %.3f, %.3f],\n"
      "  \"ins_lat\": %.7f,\n"
      "  \"ins_lon\": %.7f,\n"
      "  \"ins_msl\": %.2f,\n"
      "  \"pdop\": %.1f,\n"
      "  \"hdop\": %.1f,\n"
      "  \"solq_pos\": %d,\n"
      "  \"nv_pos\": %d,\n"
      "  \"solq_heading\": %d,\n"
      "  \"nv_heading\": %d,\n"
      "  \"diff_age\": %d,\n"
      "  \"undulation\": %.2f,\n"
      "  \"vel_enu\": [%.2f, %.2f, %.2f],\n"
      "  \"acc_enu\": [%.2f, %.2f, %.2f]\n"
      "}\n",
      raw->hi81.main_status,
      raw->hi81.ins_status,
      raw->hi81.gpst_wn,
      raw->hi81.gpst_tow,
      raw->hi81.gyr_b[0] * (0.001 * R2D),
      raw->hi81.gyr_b[1] * (0.001 * R2D),
      raw->hi81.gyr_b[2] * (0.001 * R2D),
      raw->hi81.acc_b[0] * 0.0048828,
      raw->hi81.acc_b[1] * 0.0048828,
      raw->hi81.acc_b[2] * 0.0048828,
      raw->hi81.mag_b[0] * 0.030517,
      raw->hi81.mag_b[1] * 0.030517,
      raw->hi81.mag_b[2] * 0.030517,
      (float)raw->hi81.air_pressure,
      raw->hi81.temperature,
      raw->hi81.utc_year,
      raw->hi81.utc_month,
      raw->hi81.utc_day,
      raw->hi81.utc_hour,
      raw->hi81.utc_min,
      raw->hi81.utc_msec / 1000,
      raw->hi81.utc_msec % 1000,
      raw->hi81.pitch * 0.01,
      raw->hi81.roll * 0.01,
      raw->hi81.yaw * 0.01,
      raw->hi81.quat[0] * 0.0001,
      raw->hi81.quat[1] * 0.0001,
      raw->hi81.quat[2] * 0.0001,
      raw->hi81.quat[3] * 0.0001,
      raw->hi81.ins_lat * 1e-7,
      raw->hi81.ins_lon * 1e-7,
      raw->hi81.ins_msl * 1e-3,
      raw->hi81.pdop * 0.1,
      raw->hi81.hdop * 0.1,
      raw->hi81.solq_pos,
      raw->hi81.nv_pos,
      raw->hi81.solq_heading,
      raw->hi81.nv_heading,
      raw->hi81.diff_age,
      raw->hi81.undulation * 0.01,
      raw->hi81.vel_enu[0] * 0.01,
      raw->hi81.vel_enu[1] * 0.01,
      raw->hi81.vel_enu[2] * 0.01,
      raw->hi81.acc_enu[0] * 0.0048828,
      raw->hi81.acc_enu[1] * 0.0048828,
      raw->hi81.acc_enu[2] * 0.0048828);
  } else if (raw->hi83.tag == HIPNUC_ID_HI83) {
    ret = snprintf(
      buf + written,
      buf_size - written,
      "{\n"
      "  \"type\": \"HI83\",\n"
      "  \"main_status\": %d,\n"
      "  \"ins_status\": %u,\n"
      "  \"data_bitmap\": %u\n",
      raw->hi83.main_status,
      (unsigned)raw->hi83.ins_status,
      (unsigned)raw->hi83.data_bitmap);
    if (ret > 0) written += ret;

    if (raw->hi83.data_bitmap & HI83_BMAP_ACC_B) {
      ret = snprintf(
        buf + written, buf_size - written, "  ,\"acc\": [%.3f, %.3f, %.3f]\n",
        raw->hi83.acc_b[0], raw->hi83.acc_b[1], raw->hi83.acc_b[2]);
      if (ret > 0) written += ret;
    }
    if (raw->hi83.data_bitmap & HI83_BMAP_GYR_B) {
      ret = snprintf(
        buf + written, buf_size - written, "  ,\"gyr\": [%.3f, %.3f, %.3f]\n",
        raw->hi83.gyr_b[0], raw->hi83.gyr_b[1], raw->hi83.gyr_b[2]);
      if (ret > 0) written += ret;
    }
    if (raw->hi83.data_bitmap & HI83_BMAP_MAG_B) {
      ret = snprintf(
        buf + written, buf_size - written, "  ,\"mag\": [%.3f, %.3f, %.3f]\n",
        raw->hi83.mag_b[0], raw->hi83.mag_b[1], raw->hi83.mag_b[2]);
      if (ret > 0) written += ret;
    }
    if (raw->hi83.data_bitmap & HI83_BMAP_RPY) {
      ret = snprintf(
        buf + written, buf_size - written,
        "  ,\"pitch\": %.2f\n  ,\"roll\": %.2f\n  ,\"yaw\": %.2f\n",
        raw->hi83.rpy[1], raw->hi83.rpy[0], raw->hi83.rpy[2]);
      if (ret > 0) written += ret;
    }
    if (raw->hi83.data_bitmap & HI83_BMAP_QUAT) {
      ret = snprintf(
        buf + written, buf_size - written, "  ,\"quat\": [%.3f, %.3f, %.3f, %.3f]\n",
        raw->hi83.quat[0], raw->hi83.quat[1], raw->hi83.quat[2], raw->hi83.quat[3]);
      if (ret > 0) written += ret;
    }
    if (raw->hi83.data_bitmap & HI83_BMAP_SYSTEM_TIME) {
      ret = snprintf(
        buf + written, buf_size - written, "  ,\"system_time_us\": %llu\n",
        (unsigned long long)raw->hi83.system_time_us);
      if (ret > 0) written += ret;
    }
    if (raw->hi83.data_bitmap & HI83_BMAP_AIR_PRESSURE) {
      ret = snprintf(
        buf + written, buf_size - written, "  ,\"air_pressure\": %.1f\n",
        raw->hi83.air_pressure);
      if (ret > 0) written += ret;
    }
    if (raw->hi83.data_bitmap & HI83_BMAP_TEMPERATURE) {
      ret = snprintf(
        buf + written, buf_size - written, "  ,\"temperature\": %.2f\n",
        raw->hi83.temperature);
      if (ret > 0) written += ret;
    }
    ret = snprintf(buf + written, buf_size - written, "}\n");
  }

  if (ret > 0) written += ret;
  return written;
}

static void hipnuc_crc16(uint16_t * inital, const uint8_t * buf, uint32_t len)
{
  uint32_t crc = *inital;
  uint32_t j = 0;
  for (j = 0; j < len; ++j) {
    uint32_t i = 0;
    uint32_t byte = buf[j];
    crc ^= byte << 8;
    for (i = 0; i < 8; ++i) {
      uint32_t temp = crc << 1;
      if (crc & 0x8000) temp ^= 0x1021;
      crc = temp;
    }
  }
  *inital = crc;
}
