/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  jd9365.c: Jadard JD9365TX touch controller driver.
  This file is licensed under GPL V3.
*/

/*
  Jadard ICs report touch events in V1 or V2 formats:
  V1: fixed slot, empty slot reported as 0xFFFF
  V2: compact slot, use bitmap to identify each point
  Here we only supports V1 format.

  The panel I used for testing is very unstable :(
  Slot 1 and Slot 4/5 swap their coordinates randomly.
  Could be bad calibration in factory.
*/


#include "u2hts_core.h"

static bool jd9365_setup(U2HTS_BUS_TYPES bus_type);
static bool jd9365_coord_fetch();
static void jd9365_get_config(u2hts_touch_controller_config* cfg);
static u2hts_touch_controller_operations jd9365_ops = {
    .setup = &jd9365_setup,
    .fetch = &jd9365_coord_fetch,
    .get_config = &jd9365_get_config};

static u2hts_touch_controller jd9365 = {
    .name = "jd9365",
    .irq_type = IRQ_TYPE_EDGE_FALLING,
    .report_mode = UTC_REPORT_MODE_CONTINOUS,
    .i2c_config = {.primary_addr = 0x68, .speed_hz = 400 * 1000 /*400 KHz*/},
    .operations = &jd9365_ops};

U2HTS_TOUCH_CONTROLLER(jd9365);

#define JD9365_I2C_ADDR jd9365.i2c_config.primary_addr
#define JD9365_CHIP_ID_REG 0x40008076
#define JD9365_ERAM_BASE 0x20011000
#define JD9365_COORD_ADDR_REG JD9365_ERAM_BASE + 0xD8
#define JD9365_COORD_CONFIG_ADDR JD9365_ERAM_BASE + 0x0C

#define JD9365_MAX_TPS 10

static uint32_t jd9365_coord_reg = 0x20011120;

typedef struct __packed {
  uint16_t x_be;
  uint16_t y_be;
  uint8_t w;
} jd9365_tp;

typedef struct __packed {
  uint8_t tp_num;
  uint8_t frame;
  uint8_t event;
  jd9365_tp tps[10];
  uint8_t state[5];
  uint8_t rsvd[6];
  uint8_t stylus[14];
} jd9365_touch_data;

static void jd9365_enter_backdoor() {
  uint8_t payload[] = {0xF2, 0xAA, 0xF0, 0x0F, 0x55, 0x68};
  u2hts_i2c_write(JD9365_I2C_ADDR, payload, sizeof(payload), true);
}

inline static void jd9365_backdoor_write(uint32_t reg, void* buf, size_t len) {
  uint8_t payload[6 + len];
  payload[0] = 0xF2;
  u2hts_write_unaligned_u32(payload + 1, U2HTS_SWAP32(reg));
  payload[5] = 0x03;
  memcpy(payload + 6, buf, len);
  u2hts_i2c_write(JD9365_I2C_ADDR, payload, sizeof(payload), true);
}

inline static void jd9365_backdoor_read(uint32_t reg, void* buf, size_t len) {
  uint8_t payload[6] = {0};
  payload[0] = 0xF3;
  u2hts_write_unaligned_u32(payload + 1, U2HTS_SWAP32(reg));
  payload[5] = 0x03;
  u2hts_i2c_write(JD9365_I2C_ADDR, payload, sizeof(payload), false);
  u2hts_i2c_read(JD9365_I2C_ADDR, buf, len);
}

inline static uint16_t jd9365_read_id() {
  uint16_t id = 0;
  jd9365_backdoor_read(JD9365_CHIP_ID_REG, &id, sizeof(id));
  return id;
}

static bool jd9365_setup(U2HTS_BUS_TYPES bus_type) {
  u2hts_tprst_set(false);
  u2hts_delay_ms(50);
  u2hts_tprst_set(true);
  u2hts_delay_ms(100);
  U2HTS_DETECT_TOUCH_CONTROLLER(jd9365);
  jd9365_enter_backdoor();

  jd9365_backdoor_read(JD9365_COORD_ADDR_REG, &jd9365_coord_reg,
                       sizeof(jd9365_coord_reg));
  U2HTS_LOG_INFO("JD9365 id = %x, coord reg = %x", jd9365_read_id(),
                 jd9365_coord_reg);

  return true;
}

static bool jd9365_coord_fetch() {
  jd9365_touch_data data = {0};
  jd9365_backdoor_read(jd9365_coord_reg, &data, sizeof(data));

  uint8_t tp_index = 0;
  for (uint8_t i = 0; i < JD9365_MAX_TPS; i++) {
    if (data.tps[i].x_be == 0xFFFF || data.tps[i].y_be == 0xFFFF)
      continue;
    else
      u2hts_set_tp(tp_index++, true, i, U2HTS_SWAP16(data.tps[i].x_be),
                   U2HTS_SWAP16(data.tps[i].y_be), 0, 0, data.tps[i].w);
  }
  U2HTS_SET_TP_COUNT_SAFE(tp_index);
  return true;
}

static void jd9365_get_config(u2hts_touch_controller_config* cfg) {
  uint32_t cfg_addr = 0;
  struct {
    uint16_t x_max;
    uint16_t y_max;
    uint16_t channels;
  } jd9365_coord_config;

  jd9365_backdoor_read(JD9365_COORD_CONFIG_ADDR, &cfg_addr, sizeof(cfg_addr));

  jd9365_backdoor_read(cfg_addr, &jd9365_coord_config,
                       sizeof(jd9365_coord_config));

  cfg->max_tps = JD9365_MAX_TPS;
  cfg->x_max = jd9365_coord_config.x_max - 1;
  cfg->y_max = jd9365_coord_config.y_max - 1;
}