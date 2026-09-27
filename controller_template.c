/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  contoller_template.c: template for custom controller.
  contoller_template.c: 自定义控制器模板。
  This file is licensed under GPL V3.
*/

#include "u2hts_core.h"
static bool mycontroller_setup(U2HTS_BUS_TYPES bus_type);
static bool mycontroller_coord_fetch();
static void mycontroller_get_config(u2hts_touch_controller_config* cfg);

static u2hts_touch_controller_operations mycontroller_ops = {
    .setup = &mycontroller_setup,
    .fetch = &mycontroller_coord_fetch,
    /*
      if your controller does not supports read config, keep `get_config` empty
      如果你的控制器不支持读取配置，请将下面这条回调留空
    */
    .get_config = &mycontroller_get_config};

static u2hts_touch_controller mycontroller = {
    .name = "mycontroller",                    // controller name 控制器名称
    .irq_type = IRQ_TYPE_EDGE_FALLING,         // irq type 中断类型
    .report_mode = UTC_REPORT_MODE_CONTINOUS,  // report mode 回报模式
    // I2C
    .i2c_config =
        {
            .primary_addr = 0xFF,    // I2C slave addr I2C从机地址
            .speed_hz = 400 * 1000,  // I2C speed in Hz I2C速度，单位为Hz
            .alt_addrs =
                (uint8_t[]){0xFE, 0}  // Alternative I2C addrs, must end with 0
                                      // 其它I2C地址列表，结尾必须为 0
        },
    // SPI
    .spi_config =
        {
            .cpha = false,
            .cpol = false,
            .speed_hz = 1000 * 1000,
        },
    .operations = &mycontroller_ops};

static U2HTS_BUS_TYPES mycontroller_bus_type = UB_I2C;

// register controller
// 注册控制器
U2HTS_TOUCH_CONTROLLER(mycontroller);

#define MYCONTROLLER_I2C_ADDR mycontroller.i2c_config.primary_addr
// Touch point count register
// 读取触摸点数量的寄存器
#define MYCONTROLLER_TP_COUNT_REG 0x0001
// Touch point data start register
// 触摸数据开始的寄存器
#define MYCONTROLLER_TP_DATA_START_REG 0x0002
// config register
// 配置寄存器
#define MYCONTROLLER_CONFIG_START_REG 0x0100

// rw byte on SPI mode
// SPI模式下的收发控制字节
#define MYCONTROLLER_SPI_CMD_WRITE 0x00
#define MYCONTROLLER_SPI_CMD_READ 0x01

// Touch point data structure
// 触摸数据结构布局
typedef struct __packed {
  uint8_t id;
  uint16_t x;
  uint16_t y;
  uint8_t width;
  uint8_t height;
} mycontroller_tp_data;

// example config layout
// 示例配置布局
typedef struct __packed {
  uint16_t x_max;
  uint16_t y_max;
  uint8_t max_tps;
} mycontroller_config;

inline static void mycontroller_read(uint16_t reg, void* data,
                                     size_t data_size) {
  switch (mycontroller_bus_type) {
    case UB_I2C:
      u2hts_i2c_mem_read(MYCONTROLLER_I2C_ADDR, reg, sizeof(reg), data,
                         data_size);
      break;
    case UB_SPI:
      uint8_t buf[1 + data_size + sizeof(reg)];
      memset(buf, 0x00, sizeof(buf));
      *(uint8_t*)buf = MYCONTROLLER_SPI_CMD_READ;
      u2hts_write_unaligned_u16(buf + 1, reg);
      u2hts_spi_transfer(buf, sizeof(buf));
      memcpy(data, buf + 1 + sizeof(uint16_t), data_size);
      break;
  }
}

inline static void mycontroller_write(uint16_t reg, void* data,
                                      size_t data_size) {
  switch (mycontroller_bus_type) {
    case UB_I2C:
      u2hts_i2c_mem_write(mycontroller.i2c_config.primary_addr, reg,
                          sizeof(reg), data, data_size);
      break;
    case UB_SPI:
      uint8_t buf[1 + data_size + sizeof(reg)];
      *(uint8_t*)buf = MYCONTROLLER_SPI_CMD_WRITE;
      u2hts_write_unaligned_u16(buf + 1, reg);
      memcpy(buf + 1 + sizeof(uint16_t), data, data_size);
      u2hts_spi_transfer(buf, sizeof(buf));
      break;
  }
}

inline static uint8_t mycontroller_read_byte(uint16_t reg) {
  uint8_t var = 0;
  mycontroller_read(reg, &var, sizeof(var));
  return var;
}

inline static void mycontroller_write_byte(uint16_t reg, uint8_t data) {
  mycontroller_write(reg, &data, sizeof(data));
}

static bool mycontroller_setup(U2HTS_BUS_TYPES bus_type) {
  /*
    sometimes controller have extra configurable options (e.g. scan rate)
    declare as custom config here so user can override them
    某些控制器拥有额外的可配置选项（比如扫描频率）
    在这里声明为自定义配置，用户就可以配置它们。
  */
  uint32_t custom_config_value = u2hts_get_custom_config_u32(
      "mycontroller.scan_rate", /* default 默认值 */ 60);

  // do hardware reset
  // 进行硬件复位
  u2hts_tprst_set(false);
  u2hts_delay_ms(100);
  u2hts_tprst_set(true);
  u2hts_delay_ms(50);

  // save bus_type for further use
  // 保存bus_type的值以供后用
  mycontroller_bus_type = bus_type;

  if (bus_type == UB_I2C) {  // detect controller
    // 检测控制器
    U2HTS_DETECT_TOUCH_CONTROLLER(mycontroller);
  }

  // if controller needs more steps to fully initialise, do it here.
  // 如果控制器还需要一些初始化才能正常工作，请在这里完成。

  return true;
}

static bool mycontroller_coord_fetch() {
  /*
    this callback will be invoked immediately after touch interrupt (ATTN)
    triggered. some controller require clear it's internal interrupt flag after
    ATTN generated. here we write 0x00 to TP_COUNT_REG to clear irq flag.

    这个回调会在触摸中断(ATTN)产生后立即执行。有一些控制器需要在触发中断后清除内置的中断标志。
    在这个示例中，我们向TP_COUNT_REG写入0x00来清除中断标志。
  */
  uint8_t tp_count = mycontroller_read_byte(MYCONTROLLER_TP_COUNT_REG);
  // clear irq
  // 清中断标志
  mycontroller_write_byte(MYCONTROLLER_TP_COUNT_REG, 0x00);
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  // prepare tp data
  // 准备触摸数据
  mycontroller_tp_data tp[tp_count];
  mycontroller_read(MYCONTROLLER_TP_DATA_START_REG, &tp, sizeof(tp));
  for (uint8_t i = 0; i < tp_count; i++)
    u2hts_set_tp(i, true, tp[i].id, tp[i].x, tp[i].y, tp[i].width, tp[i].height,
                 0);
  return true;
}

static void mycontroller_get_config(u2hts_touch_controller_config* cfg) {
  mycontroller_config mycfg = {0};
  mycontroller_read(MYCONTROLLER_CONFIG_START_REG, &mycfg, sizeof(mycfg));
  cfg->x_max = mycfg.x_max;
  cfg->y_max = mycfg.y_max;
  cfg->max_tps = mycfg.max_tps;
}