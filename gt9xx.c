/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  gt9xx.c: touch driver for gt9xx touch controllers.
  This file is licensed under GPL V3.
  All rights reserved.
*/

#include "u2hts_core.h"

static bool gt9xx_setup(U2HTS_BUS_TYPES bus_type);
static bool gt9xx_coord_fetch();
static void gt9xx_get_config(u2hts_touch_controller_config* cfg);

static u2hts_touch_controller_operations gt9xx_ops = {
    .setup = &gt9xx_setup,
    .fetch = &gt9xx_coord_fetch,
    .get_config = &gt9xx_get_config};

static u2hts_touch_controller gt9xx = {
    .name = "gt9xx",
    .irq_type = IRQ_TYPE_EDGE_FALLING,
    .report_mode = UTC_REPORT_MODE_CONTINOUS,
    .i2c_config =
        {
            .primary_addr = 0x5d,
            .alt_addrs = (uint8_t[]){0x14, 0},
            .speed_hz = 400 * 1000,
        },
    .operations = &gt9xx_ops};

U2HTS_TOUCH_CONTROLLER(gt9xx);

#define GT9XX_I2C_ADDR gt9xx.i2c_config.primary_addr
#define GT9XX_GT1X_CONFIG_START_REG 0x8050
#define GT9XX_GT9X_CONFIG_START_REG 0x8047
#define GT9XX_PRODUCT_INFO_START_REG 0x8140
#define GT9XX_STATUS_REG 0x814E
#define GT9XX_TP_DATA_START_REG 0x814F

static char gt9xx_product_id[5] = {0};
static uint32_t gt9xx_fetch_retry_timeout_us = 0;
typedef struct __packed {
  uint8_t track_id;
  uint16_t x_coord;
  uint16_t y_coord;
  uint8_t point_size_w;
  uint8_t point_size_h;
  uint8_t reserved;
} gt9xx_tp_data;

typedef struct __packed {
  // too many config entries, for now we only concern about these 6 items...
  uint8_t cfgver;
  uint16_t x_max;
  uint16_t y_max;
  uint8_t max_tps;
} gt9xx_config;

inline static void gt9xx_i2c_read(uint16_t reg, void* data, size_t data_size) {
  u2hts_i2c_mem_read(GT9XX_I2C_ADDR, reg, sizeof(reg), data, data_size);
}

inline static void gt9xx_i2c_write(uint16_t reg, void* data, size_t data_size) {
  u2hts_i2c_mem_write(GT9XX_I2C_ADDR, reg, sizeof(reg), data, data_size);
}

inline static uint8_t gt9xx_read_byte(uint16_t reg) {
  uint8_t var = 0;
  gt9xx_i2c_read(reg, &var, sizeof(var));
  return var;
}

inline static void gt9xx_write_byte(uint16_t reg, uint8_t data) {
  gt9xx_i2c_write(reg, &data, sizeof(data));
}

static void gt9xx_get_config(u2hts_touch_controller_config* cfg) {
  gt9xx_config gt_cfg = {0};
  // Try GT1X reg addr first, then GT9X
  gt9xx_i2c_read(GT9XX_GT1X_CONFIG_START_REG, &gt_cfg, sizeof(gt_cfg));
  if (gt_cfg.x_max > U2HTS_LOGICAL_MAX || gt_cfg.y_max > U2HTS_LOGICAL_MAX ||
      gt_cfg.max_tps > U2HTS_MAX_TPS)
    gt9xx_i2c_read(GT9XX_GT9X_CONFIG_START_REG, &gt_cfg, sizeof(gt_cfg));
  cfg->x_max = gt_cfg.x_max - 1;
  cfg->y_max = gt_cfg.y_max - 1;
  cfg->max_tps = gt_cfg.max_tps;
}

inline static void gt9xx_clear_irq() { gt9xx_write_byte(GT9XX_STATUS_REG, 0); }

static bool gt9xx_coord_fetch() {
  uint16_t retry = 0;
  uint8_t buffer_status = 0;

  do {
    buffer_status = gt9xx_read_byte(GT9XX_STATUS_REG);
    if (U2HTS_CHECK_BIT(buffer_status, 7)) break;
    u2hts_delay_us(1);
    retry++;
  } while (retry < gt9xx_fetch_retry_timeout_us);

  if (!U2HTS_CHECK_BIT(buffer_status, 7)) return false;

  uint8_t tp_count = buffer_status & 0xF;
  gt9xx_clear_irq();
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  gt9xx_tp_data tp_data[tp_count];
  gt9xx_i2c_read(GT9XX_TP_DATA_START_REG, tp_data, sizeof(tp_data));
  for (uint8_t i = 0; i < tp_count; i++)
    u2hts_set_tp(i, true, tp_data[i].track_id & 0xF, tp_data[i].x_coord,
                 tp_data[i].y_coord, tp_data[i].point_size_w,
                 tp_data[i].point_size_h, 0);
  return true;
}

static bool gt9xx_setup(U2HTS_BUS_TYPES bus_type) {
  gt9xx_fetch_retry_timeout_us =
      u2hts_get_custom_config_u32("gt9xx.fetch_retry_timeout_us", 5000);
  // GT9xx only supports I2C bus.
  U2HTS_UNUSED(bus_type);
  u2hts_tprst_set(false);
  u2hts_delay_ms(20);
  u2hts_tpint_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(5);

  // i2c addr should be 0x5d now.
  U2HTS_DETECT_TOUCH_CONTROLLER(gt9xx);

  gt9xx_i2c_read(GT9XX_PRODUCT_INFO_START_REG, gt9xx_product_id,
                 sizeof(gt9xx_product_id));
  U2HTS_LOG_INFO("gt9xx i2c addr: 0x%x, product ID: %s", GT9XX_I2C_ADDR,
                 gt9xx_product_id);

  u2hts_delay_ms(100);
  gt9xx_clear_irq();
  return true;
}