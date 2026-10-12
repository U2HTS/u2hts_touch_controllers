/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  rmi_f11.c: Synaptics F11 driver based on RMI4-I2C.
  Tested on:
    - Synaptics S7300B
  This file is licensed under GPL V3.
*/

#include "rmi_common.h"
static bool rmi_f11_setup(u2hts_bus_config buscfg);
static bool rmi_f11_coord_fetch();
static void rmi_f11_get_config(u2hts_touch_controller_coord_config* cfg);

const static u2hts_touch_controller_operations rmi_ops = {
    .setup = &rmi_f11_setup,
    .fetch = &rmi_f11_coord_fetch,
    .get_config = &rmi_f11_get_config};

const static u2hts_touch_controller rmi_f11 = {
    .name = "rmi_f11",
    .int_flag = INT_FLAG_LEVEL_LOW,
    .report_mode = UTC_REPORT_MODE_CONTINOUS,
    .i2c_config =
        {
            .primary_addr = 0x2c,
            .alt_addrs = (uint8_t[]){0x20, 0x70, 0},
            .speed_hz = 100 * 1000,
        },
    .operations = &rmi_ops};

U2HTS_TOUCH_CONTROLLER(rmi_f11);

static uint8_t rmi_f11_i2c_addr = 0x0;

#define RMI_F11_I2C_ADDR rmi_f11_i2c_addr

typedef struct {
  uint8_t x_high;
  uint8_t y_high;
  uint8_t xy_low;
  uint8_t wxy;
  uint8_t z;
} rmi_f11_tp_data;

#define RMI_FUNC_F11 0x11

static rmi_pdt f11;
static uint8_t rmi_f11_max_tps = 0;

inline static uint8_t rmi_f11_ctrl_read(uint16_t offset) {
  return rmi_ctrl_read(RMI_F11_I2C_ADDR, &f11, offset);
}

inline static uint8_t rmi_f11_data_read(uint16_t offset) {
  return rmi_data_read(RMI_F11_I2C_ADDR, &f11, offset);
}

inline static uint8_t rmi_f11_cmd_read(uint16_t offset) {
  return rmi_cmd_read(RMI_F11_I2C_ADDR, &f11, offset);
}

inline static uint8_t rmi_f11_query_read(uint16_t offset) {
  return rmi_query_read(RMI_F11_I2C_ADDR, &f11, offset);
}

inline static void rmi_f11_ctrl_write(uint16_t offset, uint8_t value) {
  rmi_ctrl_write(RMI_F11_I2C_ADDR, &f11, offset, value);
}

inline static void rmi_f11_cmd_write(uint16_t offset, uint8_t value) {
  rmi_cmd_write(RMI_F11_I2C_ADDR, &f11, offset, value);
}

static bool rmi_f11_coord_fetch() {
  // read irq reg to clear irq
  rmi_clear_irq(RMI_F11_I2C_ADDR);
  rmi_f11_tp_data f11_data[rmi_f11_max_tps];
  uint8_t fsd_size = (rmi_f11_max_tps + 3) / 4;
  uint32_t fsd = 0x0;  // finger status data
  rmi_i2c_read(RMI_F11_I2C_ADDR, f11.data_base, &fsd, fsd_size);
  rmi_i2c_read(RMI_F11_I2C_ADDR, f11.data_base + fsd_size, f11_data,
               sizeof(f11_data));
  uint8_t tp_count = 0;
  for (uint8_t i = 0; i < rmi_f11_max_tps; i++) {
    if ((fsd & (3 << i * 2))) {
      u2hts_set_tp(tp_count, true, i,
                   (f11_data[i].xy_low & 0xF) | (f11_data[i].x_high << 4),
                   (f11_data[i].xy_low & 0xF0) >> 4 | (f11_data[i].y_high << 4),
                   f11_data[i].wxy & 0xF, (f11_data[i].wxy & 0xF0) >> 4,
                   f11_data[i].z);
      tp_count++;
    }
  }
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  return tp_count;
}

static void rmi_f11_get_config(u2hts_touch_controller_coord_config* cfg) {
  uint8_t tps = rmi_f11_query_read(1) & 0x7;
  cfg->max_tps = (tps <= 4) ? tps + 1 : 10;
  rmi_f11_max_tps = cfg->max_tps;
  rmi_i2c_read(RMI_F11_I2C_ADDR, f11.ctrl_base + 6, &cfg->x_max,
               sizeof(cfg->x_max));
  rmi_i2c_read(RMI_F11_I2C_ADDR, f11.ctrl_base + 8, &cfg->y_max,
               sizeof(cfg->y_max));
}

static bool rmi_f11_setup(u2hts_bus_config buscfg) {
  rmi_f11_i2c_addr = buscfg.i2c_addr;
  u2hts_tprst_set(true);
  u2hts_delay_ms(50);
  u2hts_tprst_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(200);

  U2HTS_I2C_DETECT_TOUCH_CONTROLLER(rmi_f11, rmi_f11_i2c_addr);

  int8_t f11_index = rmi_fetch_pdt(RMI_F11_I2C_ADDR, RMI_FUNC_F11, &f11);
  if (f11_index < 0) {
    U2HTS_LOG_ERROR("Failed to fetch F01/F11 PDT from device");
    return false;
  }

  rmi_f01_setup(RMI_F11_I2C_ADDR);

  uint8_t sensor_count_reg = rmi_f11_query_read(0);
  if ((sensor_count_reg & 0x7) != 0x00)  // lower 3 bits
    U2HTS_LOG_WARN("F11 2D Sensors > 1 devices are not supported");

  uint8_t generic_sensor_info = rmi_f11_query_read(1);
  if (U2HTS_CHECK_BIT(generic_sensor_info, 3))
    U2HTS_LOG_WARN(
        "Device contain relative reporting mode which is not supported");
  if (U2HTS_CHECK_BIT(generic_sensor_info, 4))
    U2HTS_LOG_INFO("Device support absolule reporting mode");

  // we need to put ReportingMode to '000', aka "Continous reporting mode".
  uint8_t general_control = rmi_f11_ctrl_read(0);
  if (general_control & 0x7) {
    U2HTS_LOG_WARN("Reporting mode is 0x%x which is not 0, changing mode.",
                   general_control);
    U2HTS_SET_BIT(general_control, 0, 0);
    U2HTS_SET_BIT(general_control, 1, 0);
    U2HTS_SET_BIT(general_control, 2, 0);
  }

  rmi_f11_ctrl_write(0, general_control);

  rmi_enable_irq(RMI_F11_I2C_ADDR, f11_index);
  return true;
}