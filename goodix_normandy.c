/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".
  goodix_normandy.c: touch driver for Goodix Normandy series touch
  controllers. Tested on:
    - GT9886
  This file is licensed under GPL V3.
  All rights reserved.
*/

#include "u2hts_core.h"
static bool goodix_normandy_setup(U2HTS_BUS_TYPES bus_type);
static bool goodix_normandy_coord_fetch();

static u2hts_touch_controller_operations goodix_normandy_ops = {
    .setup = &goodix_normandy_setup, .fetch = &goodix_normandy_coord_fetch};

static u2hts_touch_controller goodix_normandy = {
    .name = "goodix_normandy",
    .irq_type = IRQ_TYPE_EDGE_FALLING,
    .report_mode = UTC_REPORT_MODE_CONTINOUS,
    .i2c_config =
        {
            .addr = 0x5d,
            .speed_hz = 100 * 1000, // 100 KHz
        },
    .operations = &goodix_normandy_ops};
U2HTS_TOUCH_CONTROLLER(goodix_normandy);

#define GOODIX_NORMANDY_I2C_ADDR goodix_normandy.i2c_config.addr
#define GOODIX_NORMANDY_TOUCH_DATA_ADDR 0x4100
#define GOODIX_NORMANDY_VERSION_ADDR_NORMANDY 0x4535
// this is from linux/drivers/input/touchscreen/goodix_gtx8.h
typedef struct __packed {
  /* 4 digits IC number */
  uint8_t product_id[4];
  /* Most likely unused */
  uint32_t __unknown;
  /* Four components version number */
  uint32_t fw_version;
} goodix_normandy_firmware_version;

typedef struct {
  uint8_t status;
  uint8_t tp_count;
} goodix_normandy_touch_header;

typedef struct __packed {
  uint8_t id;
  uint16_t x;
  uint16_t y;
  uint8_t w;
  uint8_t unknown_0;
  uint8_t unknown_1;
} goodix_normandy_touch_data;

inline static void goodix_normandy_read_reg(uint16_t addr, void* buf,
                                            size_t len) {
  u2hts_i2c_mem_read(GOODIX_NORMANDY_I2C_ADDR, addr, sizeof(addr), buf, len);
}

inline static void goodix_normandy_write_reg(uint16_t addr, uint8_t var) {
  u2hts_i2c_mem_write(GOODIX_NORMANDY_I2C_ADDR, addr, sizeof(addr), &var,
                      sizeof(var));
}

inline static void goodix_normandy_clear_irq() {
  goodix_normandy_write_reg(GOODIX_NORMANDY_TOUCH_DATA_ADDR, 0);
}

static bool goodix_normandy_setup(U2HTS_BUS_TYPES bus_type) {
  U2HTS_UNUSED(bus_type);
  u2hts_tpint_set(false);
  u2hts_tprst_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(200);
  u2hts_tpint_set(true);
  bool ret = u2hts_i2c_detect_slave(GOODIX_NORMANDY_I2C_ADDR);
  if (!ret) return ret;
  goodix_normandy_firmware_version fwver = {0};
  goodix_normandy_read_reg(GOODIX_NORMANDY_VERSION_ADDR_NORMANDY, &fwver,
                           sizeof(fwver));
  U2HTS_LOG_INFO("goodix_normandy product id = %s, fwver = %d",
                 fwver.product_id, fwver.fw_version);
  goodix_normandy_clear_irq();
  return ret;
}

static bool goodix_normandy_coord_fetch() {
  uint8_t buf[sizeof(goodix_normandy_touch_header) +
              sizeof(goodix_normandy_touch_data) * 10 + 2 /*crc*/];
  goodix_normandy_read_reg(GOODIX_NORMANDY_TOUCH_DATA_ADDR, buf, sizeof(buf));
  goodix_normandy_touch_header* hdr = (goodix_normandy_touch_header*)buf;
  U2HTS_LOG_DEBUG("status = %d, tp_count = %d", hdr->status, hdr->tp_count);
  goodix_normandy_clear_irq();
  if (hdr->status == 0 || hdr->tp_count == 0 || hdr->tp_count == 0x80)
    return false;
  U2HTS_SET_TP_COUNT_SAFE(hdr->tp_count);
  for (uint8_t i = 0; i < hdr->tp_count; i++) {
    goodix_normandy_touch_data* tp =
        (goodix_normandy_touch_data*)(buf +
                                      sizeof(goodix_normandy_touch_header) +
                                      i * sizeof(goodix_normandy_touch_data));
    u2hts_set_tp(i, true, tp->id, tp->x, tp->y, tp->w, 0, 0);
  }
  return true;
}