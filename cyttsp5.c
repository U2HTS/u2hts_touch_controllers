/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  cyttsp5.c: Parade TrueTouch(TM) Standard Product V5 driver.
  This file is licensed under GPL V3.
*/

#include "u2hts_core.h"
static bool cyttsp5_setup(U2HTS_BUS_TYPES bus_type);
static bool cyttsp5_coord_fetch();

static void cyttsp5_get_config(u2hts_touch_controller_config* cfg);

static u2hts_touch_controller_operations cyttsp5_ops = {
    .setup = &cyttsp5_setup,
    .fetch = &cyttsp5_coord_fetch,
    .get_config = &cyttsp5_get_config};

static u2hts_touch_controller cyttsp5 = {.name = "cyttsp5",
                                         .i2c_config =
                                             {
                                                 .primary_addr = 0x24,
                                                 .speed_hz = 100 * 1000,
                                             },
                                         .irq_type = IRQ_TYPE_EDGE_FALLING,
                                         .report_mode = UTC_REPORT_MODE_EVENT,
                                         .operations = &cyttsp5_ops};

U2HTS_TOUCH_CONTROLLER(cyttsp5);

#define CYTTSP5_I2C_ADDR cyttsp5.i2c_config.primary_addr

#define CYTTSP5_HID_DESC_REG 0x01
#define CYTTSP5_HID_VERSION 0x0100
#define CYTTSP5_HID_APP_REPORT_ID 0xF7
#define CYTTSP5_HID_BL_REPORT_ID 0xFF

#define CYTTSP5_HID_CMD_TYPE_APP 0
#define CYTTSP5_HID_CMD_TYPE_BL 1

#define CYTTSP5_HID_APP_CMD_REPORT_ID 0x2F
#define CYTTSP5_HID_BL_CMD_REPORT_ID 0x40

#define CYTTSP5_HID_CMD_CODE_BL_LAUNCH_APP 0x3B
#define CYTTSP5_HID_CMD_GET_SYSINFO 0x2

#define HID_APP_RESPONSE_REPORT_ID 0x1F
#define HID_APP_OUTPUT_REPORT_ID 0x2F
#define HID_BL_RESPONSE_REPORT_ID 0x30
#define HID_BL_OUTPUT_REPORT_ID 0x40
#define HID_RESPONSE_REPORT_ID 0xF0

#define HID_OUTPUT_BL_SOP 0x1
#define HID_OUTPUT_BL_EOP 0x17
#define HID_OUTPUT_BL_LAUNCH_APP 0x3B
#define HID_OUTPUT_BL_LAUNCH_APP_SIZE 11
#define HID_OUTPUT_GET_SYSINFO 0x2
#define HID_OUTPUT_GET_SYSINFO_SIZE 5
#define HID_OUTPUT_MAX_CMD_SIZE 12

#define CYTTSP5_HID_CMD_BL_SOP 0x1
#define CYTTSP5_HID_CMD_BL_EOP 0x17

// these values should read from HID report descriptor.

#define CYTTSP5_TMA568_REPORT_OFFSET 7
#define CYTTSP5_TMA568_TOUCH_REPORT_SIZE 10
#define CYTTSP5_TMA568_TCH_SIZE 10

typedef struct __packed {
  uint16_t hid_desc_len;
  uint8_t packet_id;
  uint8_t reserved_byte;
  uint16_t bcd_version;
  uint16_t report_desc_len;
  uint16_t report_desc_register;
  uint16_t input_register;
  uint16_t max_input_len;
  uint16_t output_register;
  uint16_t max_output_len;
  uint16_t command_register;
  uint16_t data_register;
  uint16_t vendor_id;
  uint16_t product_id;
  uint16_t version_id;
  uint32_t reserved;
} cyttsp5_hid_desc;

typedef struct {
  uint8_t cmd_type;
  uint16_t length;
  uint8_t command_code;
  uint16_t write_length;
  uint8_t* write_buf;
  uint8_t novalidate;
  uint8_t reset_expected;
  uint16_t timeout_ms;
} cyttsp5_hid_cmd;

typedef struct __packed {
  uint8_t electrodes_x;
  uint8_t electrodes_y;
  uint16_t len_x;
  uint16_t len_y;
  uint16_t res_x;
  uint16_t res_y;
  uint16_t max_z;
  uint8_t origin_x;
  uint8_t origin_y;
  uint8_t panel_id;
  uint8_t btn;
  uint8_t scan_mode;
  uint8_t max_num_of_tch_per_refresh_cycle;
} cyttsp5_sensing_conf_data;

typedef struct __packed {
  uint8_t unknown0;
  uint8_t tip_id;
  uint16_t x;
  uint16_t y;
  uint8_t p;
  uint8_t wx;
  uint8_t wy;
  uint8_t unknown1;
} cyttsp5_hid_report;

static cyttsp5_sensing_conf_data scd = {0};

static cyttsp5_hid_desc hid_desc = {0};

inline static void cyttsp5_read(void* data, size_t len) {
  u2hts_i2c_read(CYTTSP5_I2C_ADDR, data, len);
}

inline static void cyttsp5_write_raw(uint8_t* data, size_t len) {
  u2hts_i2c_write(CYTTSP5_I2C_ADDR, data, len, true);
}

inline static void cyttsp5_write_reg(uint16_t reg) {
  u2hts_i2c_write(CYTTSP5_I2C_ADDR, &reg, sizeof(reg), true);
}

inline static void cyttsp5_wait_irq() { while (u2hts_tpint_get()); }

inline static void cyttsp5_dummy_read() {
  uint8_t buf[16] = {0};
  cyttsp5_read(buf, 16);
}

static const uint16_t crc_table[16] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50a5, 0x60c6, 0x70e7,
    0x8108, 0x9129, 0xa14a, 0xb16b, 0xc18c, 0xd1ad, 0xe1ce, 0xf1ef,
};

inline static uint16_t cyttsp5_get_crc(uint8_t* buf, size_t size) {
  uint16_t remainder = 0xFFFF;
  uint16_t xor_mask = 0x0000;
  uint32_t byte_value = 0;
  uint32_t table_index;
  uint32_t crc_bit_width = sizeof(uint16_t) * 8;

  /* Divide the message by polynomial, via the table. */
  for (uint32_t index = 0; index < size; index++) {
    byte_value = buf[index];
    table_index =
        ((byte_value >> 4) & 0x0F) ^ (remainder >> (crc_bit_width - 4));
    remainder = crc_table[table_index] ^ (remainder << 4);
    table_index = (byte_value & 0x0F) ^ (remainder >> (crc_bit_width - 4));
    remainder = crc_table[table_index] ^ (remainder << 4);
  }

  /* Perform the final remainder CRC. */
  return remainder ^ xor_mask;
}

inline static void cyttsp5_write_hid_cmd(cyttsp5_hid_cmd* cmd) {
  uint8_t report_id = CYTTSP5_HID_APP_CMD_REPORT_ID;
  uint16_t len = 5;
  uint8_t buf[64] = {0};
  uint8_t buf_offset = 0;
  if (cmd->cmd_type) {
    report_id = CYTTSP5_HID_BL_CMD_REPORT_ID;
    len = 11;
  }
  len += cmd->write_length;
  memcpy(buf, &hid_desc.output_register, sizeof(hid_desc.output_register));
  buf_offset += sizeof(hid_desc.output_register);  // 2

  memcpy(&buf[buf_offset], &len, sizeof(len));
  buf_offset += sizeof(len);  // 2

  buf[buf_offset] = report_id;

  buf_offset++;  // 1
  buf[buf_offset] = 0;

  if (cmd->cmd_type) {
    buf_offset++;  // 1
    buf[buf_offset] = CYTTSP5_HID_CMD_BL_SOP;
  }

  buf_offset++;  // 1
  buf[buf_offset] = cmd->command_code;

  buf_offset++;  // 1
  if (cmd->cmd_type) {
    memcpy(&buf[buf_offset], &cmd->write_length, sizeof(cmd->write_length));
    buf_offset += sizeof(cmd->write_length);  // 2
  }

  if (cmd->write_length && cmd->write_buf) {
    memcpy(&buf[buf_offset], cmd->write_buf, cmd->write_length);
    buf_offset += cmd->write_length;
  }

  if (cmd->cmd_type) {
    uint16_t crc = cyttsp5_get_crc(&buf[6], cmd->write_length + 4);
    memcpy(&buf[buf_offset], &crc, sizeof(uint16_t));
    buf_offset += 2;                           // 2
    buf[buf_offset] = CYTTSP5_HID_CMD_BL_EOP;  // 1
  }
  cyttsp5_write_raw(buf, len + 2);
}

inline static bool cyttsp5_validate_cmd_response(uint8_t cmd_code) {
  uint16_t len = 0;
  cyttsp5_read(&len, sizeof(len));
  if (len) {
    U2HTS_LOG_DEBUG("%s: len true", __func__);
    return true;
  }
  U2HTS_LOG_DEBUG("%s: len = %d false", __func__, len);
  uint8_t response_buf[len];
  cyttsp5_read(response_buf, len);
  uint8_t command_code = 0;
  uint16_t report_id = response_buf[2];
  uint16_t val = 0;
  U2HTS_LOG_DEBUG("%s: report_id = %d", __func__, report_id);

  switch (report_id) {
    case HID_BL_RESPONSE_REPORT_ID:
      if (response_buf[4] != HID_OUTPUT_BL_SOP) {
        U2HTS_LOG_ERROR("HID output response, wrong SOP\n");
        return false;
      }

      if (response_buf[len - 1] != HID_OUTPUT_BL_EOP) {
        U2HTS_LOG_ERROR("HID output response, wrong EOP\n");
        return false;
      }

      uint16_t crc = cyttsp5_get_crc(&response_buf[4], len - 7);
      memcpy(&val, &response_buf[len - 3], sizeof(val));
      if (val != crc) {
        U2HTS_LOG_ERROR("HID output response, wrong CRC 0x%X\n", crc);
        return false;
      }

      uint8_t status = response_buf[5];
      if (status) {
        U2HTS_LOG_ERROR("HID output response, ERROR:%d\n", status);
        return false;
      }
      break;

    case HID_APP_RESPONSE_REPORT_ID:
      command_code = response_buf[4] & 0x7F;
      if (command_code != cmd_code) {
        U2HTS_LOG_ERROR("HID output response, wrong command_code:%X\n",
                        command_code);
        return false;
      }
      break;
  }
  return true;
}

inline static void cyttsp5_enter_app_mode() {
  cyttsp5_hid_cmd enter_app_cmd = {
      .cmd_type = CYTTSP5_HID_CMD_TYPE_BL,
      .command_code = CYTTSP5_HID_CMD_CODE_BL_LAUNCH_APP,
  };
  cyttsp5_write_hid_cmd(&enter_app_cmd);
  u2hts_delay_ms(500);
}

inline static bool cyttsp5_get_sysinfo(cyttsp5_sensing_conf_data* data) {
  cyttsp5_hid_cmd get_sys_info_cmd = {
      .cmd_type = CYTTSP5_HID_CMD_TYPE_APP,
      .command_code = CYTTSP5_HID_CMD_GET_SYSINFO,
  };
  cyttsp5_write_hid_cmd(&get_sys_info_cmd);
  u2hts_delay_ms(100);
  uint16_t len = 0;
  cyttsp5_read(&len, sizeof(len));
  if (!len) return false;
  uint8_t buf[len];
  cyttsp5_read(buf, sizeof(buf));
  memcpy(data, &buf[33], sizeof(cyttsp5_sensing_conf_data));
  return true;
}

inline static void cyttsp5_print_sysinfo(cyttsp5_sensing_conf_data* data) {
  U2HTS_LOG_INFO(
      "electrodes_x=%u, electrodes_y=%u, len_x=%u, len_y=%u, res_x=%u, "
      "res_y=%u, max_z=%u, origin_x=%u, origin_y=%u, panel_id=%u, btn=%u, "
      "scan_mode=%u, max_num_of_tch_per_refresh_cycle=%u",
      data->electrodes_x, data->electrodes_y, data->len_x, data->len_y,
      data->res_x, data->res_y, data->max_z, data->origin_x, data->origin_y,
      data->panel_id, data->btn, data->scan_mode,
      data->max_num_of_tch_per_refresh_cycle);
}

static bool cyttsp5_get_hid_desc(cyttsp5_hid_desc* desc) {
  cyttsp5_write_reg(CYTTSP5_HID_DESC_REG);
  uint16_t len = 0;
  do {
    cyttsp5_wait_irq();
    cyttsp5_read(&len, sizeof(len));
    u2hts_delay_ms(16);
  } while (len != sizeof(cyttsp5_hid_desc));

  do {
    cyttsp5_wait_irq();
    cyttsp5_read(desc, sizeof(hid_desc));
    u2hts_delay_ms(16);
  } while (desc->bcd_version != CYTTSP5_HID_VERSION);

  if (len != sizeof(cyttsp5_hid_desc)) {
    U2HTS_LOG_ERROR("HID descriptor length error: %d", len);
    return false;
  }

  cyttsp5_read(desc, sizeof(hid_desc));
  U2HTS_LOG_DEBUG("HID length = %d, HID version = %d", desc->hid_desc_len,
                  desc->bcd_version);
  if (desc->bcd_version != CYTTSP5_HID_VERSION) {
    U2HTS_LOG_ERROR("Unsupported HID version %d", desc->bcd_version);
    return false;
  }
  return true;
}

inline static bool cyttsp5_coord_fetch() {
  /* HID-over-I2C 输入报告：直接从 Input Register 纯读，无需先写寄存器地址 */
  uint16_t len = 0;
  cyttsp5_read(&len, sizeof(len));
  /* 空缓冲：0x0000(复位完成) / 0x0002(PIP1.7 前) / 0xFFXX(PIP1.7+) */
  if (!len || len == 2 || len > 0xFF00) return false;
  if (len > CYTTSP5_TMA568_REPORT_OFFSET +
                 CYTTSP5_TMA568_TCH_SIZE * U2HTS_MAX_TPS)
    return false;
  uint8_t buf[len];
  cyttsp5_read(buf, sizeof(buf));
  if (buf[2] != 0x01) return false; /* 非触摸报告 */

  /* 触点数：以头部声明为准，并受实际帧长度约束，防止越界 */
  uint8_t tp_count = buf[5];
  uint8_t len_tp = (len - CYTTSP5_TMA568_REPORT_OFFSET) /
                   CYTTSP5_TMA568_TCH_SIZE;
  if (tp_count > len_tp) tp_count = len_tp;
  if (tp_count > U2HTS_MAX_TPS) tp_count = U2HTS_MAX_TPS;

  /* EVENT 模式：抬起触点(bit7=0)也要显式上报 contact=false，
   * 供上层产生抬起事件（框架在 EVENT 模式下不会自动补 released） */
  for (uint8_t i = 0; i < tp_count; i++) {
    cyttsp5_hid_report* r =
        (cyttsp5_hid_report*)&buf[CYTTSP5_TMA568_REPORT_OFFSET +
                                  i * CYTTSP5_TMA568_TCH_SIZE];
    u2hts_set_tp(i, U2HTS_CHECK_BIT(r->tip_id, 7), r->tip_id & 0x1F,
                 r->x, r->y, r->wx, r->wy, r->p);
  }
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  return true;
}

static bool cyttsp5_setup(U2HTS_BUS_TYPES bus_type) {
  U2HTS_UNUSED(bus_type);
  u2hts_tprst_set(false);
  u2hts_delay_ms(40);
  u2hts_tprst_set(true);
  u2hts_delay_ms(100);
  U2HTS_DETECT_TOUCH_CONTROLLER(cyttsp5);
  // input pullup mode
  u2hts_tpint_set_mode(false, true);

  cyttsp5_dummy_read();

  bool ret = cyttsp5_get_hid_desc(&hid_desc);
  if (!ret) {
    U2HTS_LOG_DEBUG("Failed to get HID descriptor");
    return ret;
  }

  if (hid_desc.packet_id == CYTTSP5_HID_BL_REPORT_ID) {
    U2HTS_LOG_INFO("Device in bootloader mode, enter app mode");
    cyttsp5_enter_app_mode();
    // wait 50ms for device enter app mode
    u2hts_delay_ms(50);

    cyttsp5_dummy_read();
    // re-fetch hid descriptor in app mode.
    memset(&hid_desc, 0x00, sizeof(hid_desc));
    ret = cyttsp5_get_hid_desc(&hid_desc);
    if (!ret) {
      U2HTS_LOG_DEBUG("Failed to get HID descriptor");
      return ret;
    }
  }

  if (hid_desc.packet_id == CYTTSP5_HID_APP_REPORT_ID)
    U2HTS_LOG_INFO("Device is in app mode");
  else {
    U2HTS_LOG_ERROR("Failed to set device in app mode");
    return false;
  }

  cyttsp5_write_reg(hid_desc.report_desc_register);
  u2hts_delay_ms(200);
  uint8_t report_buf[hid_desc.report_desc_len];
  cyttsp5_read(report_buf, sizeof(report_buf));

  ret = cyttsp5_get_sysinfo(&scd);
  if (!ret) {
    U2HTS_LOG_ERROR("Failed to get sysinfo");
    return ret;
  }
  cyttsp5_print_sysinfo(&scd);
  return ret;
}

inline static void cyttsp5_get_config(u2hts_touch_controller_config* cfg) {
  cfg->x_max = scd.res_x;
  cfg->y_max = scd.res_y;
  cfg->max_tps = scd.max_num_of_tch_per_refresh_cycle;
}