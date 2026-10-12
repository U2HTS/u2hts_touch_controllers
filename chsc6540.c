/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  chsc6540.c: chsc6540 driver
  This file is licensed under GPL V3.
*/

/*
  Known issues
  - Setting ATTN pin to input-pullup mode sometimes generate
  irq without actual touch. Possible hardware glitch.
  - Read data filled with 0xFF.
*/

#include "u2hts_core.h"

#define CHSC6540_I2C_ADDR chsc6540_i2c_addr

static bool chsc6540_setup(u2hts_bus_config buscfg);
static bool chsc6540_coord_fetch();

const static u2hts_touch_controller_operations chsc6540_ops = {
    .setup = &chsc6540_setup, .fetch = &chsc6540_coord_fetch};

const static u2hts_touch_controller chsc6540 = {
    .name = "chsc6540",
    .int_flag = INT_FLAG_EDGE_FALLING,
    .i2c_config =
        {
            .primary_addr = 0x2e, .speed_hz = 100 * 1000 /* 100 KHz*/
        },
    .operations = &chsc6540_ops};

U2HTS_TOUCH_CONTROLLER(chsc6540);

static uint8_t chsc6540_i2c_addr = 0x0;

typedef struct __packed {
  uint8_t dummy_0;
  uint8_t dummy_1;
  uint8_t fingers;
  uint8_t x_h;
  uint8_t x_l;
  uint8_t y_h;
  uint8_t y_l;
  uint8_t reg7_0x0d;
  uint8_t reg8_0x10;
  uint16_t x_pos_f1_be;
  uint16_t y_pos_f1_be;
} chsc6540_registers;

// workaround for reading 0xFF data
static chsc6540_registers previous_regs;

inline static void chsc6540_read(void* buf, size_t len) {
  u2hts_i2c_read(CHSC6540_I2C_ADDR, buf, len);
}

inline static void chsc6540_read_reg(uint8_t reg, void* buf, size_t len) {
  u2hts_i2c_mem_read(CHSC6540_I2C_ADDR, reg, sizeof(reg), buf, len);
}

inline static void chsc6540_write(void* buf, size_t len) {
  u2hts_i2c_write(CHSC6540_I2C_ADDR, buf, len, true);
}

static bool chsc6540_setup(u2hts_bus_config buscfg) {
  chsc6540_i2c_addr = buscfg.i2c_addr;
  u2hts_tprst_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(100);
  U2HTS_I2C_DETECT_TOUCH_CONTROLLER(chsc6540, chsc6540_i2c_addr);
  return true;
}

static bool chsc6540_coord_fetch() {
  chsc6540_registers regs = {0};
  chsc6540_read_reg(0x02, &regs, sizeof(regs));
  U2HTS_LOG_DEBUG("chsc6540 tp_count = %d", regs.fingers);

  if (regs.reg7_0x0d != 0x0d) regs = previous_regs;
  U2HTS_SET_TP_COUNT_SAFE(regs.fingers);
  if (regs.fingers == 1) {
    u2hts_set_tp(0, (regs.x_h >> 6 == 0x02), regs.y_h >> 4,
                 (regs.x_h & 0xF) << 8 | regs.x_l,
                 (regs.y_h & 0xF) << 8 | regs.y_l, 0, 0, 0);
    previous_regs = regs;
  } else if (regs.fingers == 2) {
    u2hts_set_tp(0, (regs.x_h >> 6 == 0x02), regs.y_h >> 4,
                 (regs.x_h & 0xF) << 8 | regs.x_l,
                 (regs.y_h & 0xF) << 8 | regs.y_l, 0, 0, 0);
    u2hts_set_tp(1, true, 1, U2HTS_SWAP16(regs.x_pos_f1_be) & 0xFFF,
                 U2HTS_SWAP16(regs.y_pos_f1_be) & 0xFFF, 0, 0, 0);
    previous_regs = regs;
  }

  return true;
}