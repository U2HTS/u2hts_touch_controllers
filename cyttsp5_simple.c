/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  cyttsp5.c: Parade TrueTouch(TM) Standard Product V5 driver.
  This file is licensed under GPL V3.
*/

#include "u2hts_core.h"
static bool cyttsp5_setup(U2HTS_BUS_TYPES bus_type);
static bool cyttsp5_coord_fetch();

static u2hts_touch_controller_operations cyttsp5_ops = {
    .setup = &cyttsp5_setup, .fetch = &cyttsp5_coord_fetch};

static u2hts_touch_controller cyttsp5 = {.name = "cyttsp5",
                                         .irq_type = IRQ_TYPE_LEVEL_LOW,
                                         .i2c_config =
                                             {
                                                 .primary_addr = 0x24,
                                                 .speed_hz = 100 * 1000,
                                             },
                                         .report_mode = UTC_REPORT_MODE_EVENT,
                                         .operations = &cyttsp5_ops};

U2HTS_TOUCH_CONTROLLER(cyttsp5);

#define CYTTSP5_I2C_ADDR cyttsp5.i2c_config.primary_addr
#define CYTTSP5_HID_APP_CMD_REPORT_ID 0x2F
#define CYTTSP5_HID_BL_CMD_REPORT_ID 0x40
#define CYTTSP5_HID_CMD_BL_SOP 0x1
#define CYTTSP5_HID_CMD_BL_EOP 0x17

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

typedef struct __packed {
  uint16_t len;
  uint8_t report_id;
  uint16_t scan_time;
  uint8_t tp_count;
  uint8_t unknown;
} cyttsp5_data_header;

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

static uint8_t cyttsp5_report_buf[sizeof(cyttsp5_hid_report) * 10 + 7] = {0};

inline static void cyttsp5_read(void* data, size_t len) {
  u2hts_i2c_read(CYTTSP5_I2C_ADDR, data, len);
}

inline static void cyttsp5_write_raw(uint8_t* data, size_t len) {
  u2hts_i2c_write(CYTTSP5_I2C_ADDR, data, len, true);
}

static const uint8_t cyttsp5_enter_app_mode_cmd[] = {
    0x04, 0x00, 0x0B, 0x00, 0x40, 0x00, 0x01,
    0x3B, 0x00, 0x00, 0x20, 0xC7, 0x17};

static bool cyttsp5_setup(U2HTS_BUS_TYPES bus_type) {
  U2HTS_UNUSED(bus_type);
  u2hts_tprst_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(200);
  U2HTS_DETECT_TOUCH_CONTROLLER(cyttsp5);
  cyttsp5_write_raw((uint8_t*)cyttsp5_enter_app_mode_cmd,
                    sizeof(cyttsp5_enter_app_mode_cmd));
  /* BL_LAUNCH_APP 会让设备复位并切入 app 模式，需等待其就绪 */
  u2hts_delay_ms(500);
  return true;
}

static bool cyttsp5_coord_fetch() {
  uint16_t packet_len = 0;
  cyttsp5_read(&packet_len, sizeof(packet_len));
  U2HTS_LOG_DEBUG("packet_len = %d", packet_len);
  if (packet_len == 0 || packet_len > sizeof(cyttsp5_report_buf)) return false;

  if (packet_len == 2 /* empty buffer */) {
    U2HTS_LOG_DEBUG("empty buffer");
    return false;
  } else {
    memset(cyttsp5_report_buf, 0x00, sizeof(cyttsp5_report_buf));
    cyttsp5_read(cyttsp5_report_buf, packet_len);
  }

  cyttsp5_data_header* header = (cyttsp5_data_header*)cyttsp5_report_buf;

#if U2HTS_LOG_LEVEL >= U2HTS_LOG_LEVEL_DEBUG
  printf("raw data: ");
  for (uint8_t i = 0; i < packet_len; i++) printf("%x ", cyttsp5_report_buf[i]);
  printf("\n");
#endif

  uint8_t tp_count = (packet_len - 7) / 10;
  if (header->report_id != 0x01 /* touch */ || header->tp_count != tp_count ||
      !tp_count)
    return false;
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  for (uint8_t i = 0; i < tp_count; i++) {
    cyttsp5_hid_report* cy_report =
        (cyttsp5_hid_report*)(cyttsp5_report_buf + 7 +
                              i * sizeof(cyttsp5_hid_report));
    u2hts_set_tp(i, U2HTS_CHECK_BIT(cy_report->tip_id, 7),
                 cy_report->tip_id & 0x1F, cy_report->x, cy_report->y,
                 cy_report->wx, cy_report->wy, cy_report->p);
  }
  return true;
}