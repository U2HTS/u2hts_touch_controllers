/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  cst8xx.c: cst8xx driver
  This file is licensed under GPL V3.
*/

#include "u2hts_core.h"

#define LOG_TAG cst8xx.name
static bool cst8xx_setup(u2hts_bus_config buscfg);
static bool cst8xx_coord_fetch();

const static u2hts_touch_controller_operations cst8xx_ops = {
    .setup = &cst8xx_setup, .fetch = &cst8xx_coord_fetch};

const static u2hts_touch_controller cst8xx = {
    .name = "cst8xx",
    .int_flag = INT_FLAG_EDGE_FALLING,
    .report_mode = UTC_REPORT_MODE_CONTINOUS,
    .i2c_config =
        {
            .primary_addr = 0x15,
            .speed_hz = 100 * 1000,
        },
    .operations = &cst8xx_ops};

U2HTS_TOUCH_CONTROLLER(cst8xx);

static uint8_t cst8xx_i2c_addr = 0x0;

#define CST8XX_I2C_ADDR cst8xx_i2c_addr
#define CST8XX_FINGER_NUM_REG 0x02
#define CST8XX_TP_DATA_START_REG 0x03
#define CST8XX_PRODUCT_INFO_START_REG 0xA7

typedef struct {
  uint8_t x_h;
  uint8_t x_l;
  uint8_t y_h;
  uint8_t y_l;
} cst8xx_tp_data;

typedef struct {
  uint8_t chip_id;
  uint8_t proj_id;
  uint8_t fw_ver;
  uint8_t vendor_id;
} cst8xx_product_info;

inline static void cst8xx_i2c_read(uint8_t reg, void* data, size_t data_size) {
  u2hts_i2c_mem_read(CST8XX_I2C_ADDR, reg, sizeof(reg), data,
                     data_size);
}

inline static uint8_t cst8xx_read_byte(uint8_t reg) {
  uint8_t var = 0;
  cst8xx_i2c_read(reg, &var, sizeof(var));
  return var;
}

static bool cst8xx_setup(u2hts_bus_config buscfg) {
  cst8xx_i2c_addr = buscfg.i2c_addr;
  u2hts_tprst_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(50);
  U2HTS_I2C_DETECT_TOUCH_CONTROLLER(cst8xx, cst8xx_i2c_addr);
  cst8xx_product_info info = {0};
  cst8xx_i2c_read(CST8XX_PRODUCT_INFO_START_REG, &info, sizeof(info));
  ULOGI(LOG_TAG, 
      "chip_id = 0x%x, ProjID = 0x%x, fw_ver = 0x%x, vendor_id = 0x%x",
      info.chip_id, info.proj_id, info.fw_ver, info.vendor_id);
  return true;
}

static bool cst8xx_coord_fetch() {
  if (!cst8xx_read_byte(CST8XX_FINGER_NUM_REG)) return false;
  U2HTS_SET_TP_COUNT_SAFE(1);
  cst8xx_tp_data tp = {0};
  cst8xx_i2c_read(CST8XX_TP_DATA_START_REG, &tp, sizeof(tp));
  u2hts_set_tp(0, true, 0, (tp.x_h & 0xF) << 8 | tp.x_l,
               (tp.y_h & 0xF) << 8 | tp.y_l, 0, 0, 0);
  return true;
}