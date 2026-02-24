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

static u2hts_touch_controller gt9xx = {.name = "gt9xx",
                                       .irq_type = IRQ_TYPE_EDGE_FALLING,
                                       .report_mode = UTC_REPORT_MODE_CONTINOUS,
                                       .i2c_config =
                                           {
                                               .addr = 0x5d,
                                               .speed_hz = 400 * 1000,
                                           },
                                       .alt_i2c_addr = 0x14,
                                       .operations = &gt9xx_ops};

U2HTS_TOUCH_CONTROLLER(gt9xx);

#define GT9XX_GT1X_CONFIG_START_REG 0x8050
#define GT9XX_GT9X_CONFIG_START_REG 0x8047
#define GT9XX_PRODUCT_INFO_START_REG 0x8140
#define GT9XX_STATUS_REG 0x814E
#define GT9XX_TP_DATA_START_REG 0x814F

static char gt9xx_product_id[5] = {0};

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

// ref linux/drivers/input/touchscreen/goodix.c
static const char* gt1x_products[] = {"1151", "1158", "5663", "5688",
                                      "917S", "9286", NULL};

static const char* gt9x_products[] = {"911", "9271", "9110", "9111",
                                      "927", "928",  "912",  "9147",
                                      "967", "615",  NULL};

inline static void gt9xx_i2c_read(uint16_t reg, void* data, size_t data_size) {
  u2hts_i2c_mem_read(gt9xx.i2c_config.addr, reg, sizeof(reg), data, data_size);
}

inline static void gt9xx_i2c_write(uint16_t reg, void* data, size_t data_size) {
  u2hts_i2c_mem_write(gt9xx.i2c_config.addr, reg, sizeof(reg), data, data_size);
}

inline static uint8_t gt9xx_read_byte(uint16_t reg) {
  uint8_t var = 0;
  gt9xx_i2c_read(reg, &var, sizeof(var));
  return var;
}

inline static void gt9xx_write_byte(uint16_t reg, uint8_t data) {
  gt9xx_i2c_write(reg, &data, sizeof(data));
}

static uint16_t gt9xx_get_config_start_addr(const char* product_id) {
  for (uint8_t i = 0; gt1x_products[i]; i++)
    if (!strcmp(product_id, gt1x_products[i]))
      return GT9XX_GT1X_CONFIG_START_REG;
  for (uint8_t i = 0; gt9x_products[i]; i++)
    if (!strcmp(product_id, gt9x_products[i]))
      return GT9XX_GT9X_CONFIG_START_REG;
  return GT9XX_GT9X_CONFIG_START_REG;
}

static void gt9xx_get_config(u2hts_touch_controller_config* cfg) {
  gt9xx_config gt_cfg = {0};
  gt9xx_i2c_read(gt9xx_get_config_start_addr(gt9xx_product_id), &gt_cfg,
                 sizeof(gt_cfg));
  cfg->x_max = gt_cfg.x_max - 1;
  cfg->y_max = gt_cfg.y_max - 1;
  cfg->max_tps = gt_cfg.max_tps;
}

inline static void gt9xx_clear_irq() { gt9xx_write_byte(GT9XX_STATUS_REG, 0); }

static bool gt9xx_coord_fetch() {
  uint8_t tp_count = gt9xx_read_byte(GT9XX_STATUS_REG) & 0xF;
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
  // GT9xx only supports I2C bus.
  U2HTS_UNUSED(bus_type);
  u2hts_tprst_set(false);
  u2hts_delay_ms(20);
  u2hts_tpint_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(5);

  // i2c addr should be 0x5d now.
  if (!u2hts_i2c_detect_slave(gt9xx.i2c_config.addr)) {
    if (u2hts_i2c_detect_slave(gt9xx.alt_i2c_addr))
      gt9xx.i2c_config.addr = gt9xx.alt_i2c_addr;
    else
      return false;
  }

  gt9xx_i2c_read(GT9XX_PRODUCT_INFO_START_REG, gt9xx_product_id,
                 sizeof(gt9xx_product_id));
  U2HTS_LOG_INFO("gt9xx i2c addr: 0x%x, product ID: %s", gt9xx.i2c_config.addr,
                 gt9xx_product_id);

  u2hts_delay_ms(100);
  gt9xx_clear_irq();
  return true;
}