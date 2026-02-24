/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  ft54x6.c: ft54x6 driver
  This file is licensed under GPL V3.
  All rights reserved.
*/

#include "u2hts_core.h"
static bool ft54x6_setup(U2HTS_BUS_TYPES bus_type);
static bool ft54x6_coord_fetch();

static u2hts_touch_controller_operations ft54x6_ops = {
    .setup = &ft54x6_setup, .fetch = &ft54x6_coord_fetch};

static u2hts_touch_controller ft54x6 = {
    .name = "ft54x6",
    .irq_type = IRQ_TYPE_EDGE_FALLING,
    .report_mode = UTC_REPORT_MODE_CONTINOUS,
    .i2c_config =
        {
            .addr = 0x38,
            .speed_hz = 100 * 1000,
        },
    .operations = &ft54x6_ops};

U2HTS_TOUCH_CONTROLLER(ft54x6);

typedef struct {
  uint8_t x_h;
  uint8_t x_l;
  uint8_t y_h;
  uint8_t y_l;
  uint8_t : 8;
  uint8_t : 8;
} ft54x6_tp_data;

typedef struct {
  uint8_t fwver_h;
  uint8_t fwver_l;
  uint8_t vendor_id;
  uint8_t irq_status;
  uint8_t pcm;
  uint8_t fw_id;
} ft54x6_product_info;

#define FT54X6_PRODUCT_INFO_START_REG 0xA1

#define FT54X6_TP_COUNT_REG 0x02

#define FT54X6_TP_DATA_START_REG 0x03

inline static void ft54x6_i2c_read(uint8_t reg, void* data, size_t data_size) {
  u2hts_i2c_mem_read(ft54x6.i2c_config.addr, reg, sizeof(reg), data, data_size);
}

inline static uint8_t ft54x6_read_byte(uint8_t reg) {
  uint8_t var = 0;
  ft54x6_i2c_read(reg, &var, sizeof(var));
  return var;
}

inline static bool ft54x6_setup(U2HTS_BUS_TYPES bus_type) {
  U2HTS_UNUSED(bus_type);
  u2hts_tprst_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(200);
  bool ret = u2hts_i2c_detect_slave(ft54x6.i2c_config.addr);
  if (!ret) return ret;
  ft54x6_product_info info = {0};
  ft54x6_i2c_read(FT54X6_PRODUCT_INFO_START_REG, &info, sizeof(info));
  U2HTS_LOG_INFO(
      "fwver_h = 0x%x, fwver_l = 0x%x, vendor_id = 0x%x, fw_id = 0x%x",
      info.fwver_h, info.fwver_l, info.vendor_id, info.fw_id);
  return ret;
}

inline static bool ft54x6_coord_fetch() {
  uint8_t tp_count = ft54x6_read_byte(FT54X6_TP_COUNT_REG);
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  ft54x6_tp_data tp[tp_count];
  ft54x6_i2c_read(FT54X6_TP_DATA_START_REG, &tp, sizeof(tp));
  for (uint8_t i = 0; i < tp_count; i++)
    u2hts_set_tp(i, (tp[i].x_h >> 6 == 0x02), tp[i].y_h >> 4,
                 (tp[i].x_h & 0xF) << 8 | tp[i].x_l,
                 (tp[i].y_h & 0xF) << 8 | tp[i].y_l, 0, 0, 0);

  return true;
}