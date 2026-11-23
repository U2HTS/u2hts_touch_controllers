/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".
  linux-multitouch.c: bridge a Linux evdev multitouch input device into U2HTS.
  This file is licensed under GPL V3.
  **THIS DRIVER IS AI-GENERATED**
*/

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "u2hts_core.h"

#define LOG_TAG "linux-mt"

#define MT_INPUT_DEV_MAX 32
#define MT_CUSTOM_CONFIG_DEV "linux-mt.dev"

typedef struct {
  int32_t tracking_id;
  int32_t x;
  int32_t y;
} mt_slot;

static int mt_fd = -1;
static int mt_x_max = U2HTS_LOGICAL_MAX;
static int mt_y_max = U2HTS_LOGICAL_MAX;
static uint8_t mt_max_tps = U2HTS_MAX_TPS;

static mt_slot mt_slots[U2HTS_MAX_TPS];
static uint8_t mt_current_slot = 0;

static bool mt_setup(u2hts_bus_config buscfg);
static bool mt_fetch();
static void mt_get_config(u2hts_touch_controller_coord_config* cfg);

const static u2hts_touch_controller_operations mt_ops = {
    .setup = &mt_setup,
    .fetch = &mt_fetch,
    .get_config = &mt_get_config,
};

const static u2hts_touch_controller mt = {
    .name = "linux-multitouch",
    .int_flag = 0xFF,
    .report_mode = UTC_REPORT_MODE_CONTINOUS,
    // bypass u2hts_init bus type detection
    .i2c_config = {.primary_addr = 0xFF, .speed_hz = 100 * 1000},
    .operations = &mt_ops};

U2HTS_TOUCH_CONTROLLER(mt);

static bool mt_test_bit(const unsigned long* bits, int bit) {
  return (bits[bit / (8 * sizeof(unsigned long))] >>
          (bit % (8 * sizeof(unsigned long)))) &
         1;
}

static bool mt_is_multitouch(int fd) {
  unsigned long abs_bits[ABS_CNT / (8 * sizeof(unsigned long)) + 1] = {0};
  if (ioctl(fd, EVIOCGBIT(EV_ABS, sizeof(abs_bits)), abs_bits) < 0)
    return false;
  return mt_test_bit(abs_bits, ABS_MT_POSITION_X) &&
         mt_test_bit(abs_bits, ABS_MT_POSITION_Y);
}

static int mt_open_device(const char* path) {
  int fd = open(path, O_RDONLY | O_NONBLOCK);
  if (fd < 0) return -1;
  if (!mt_is_multitouch(fd)) {
    close(fd);
    return -1;
  }
  return fd;
}

static bool mt_setup(u2hts_bus_config buscfg) {
  U2HTS_UNUSED(buscfg);

  char path[U2HTS_CUSTOM_CONFIG_STR_MAX_KEY_LENGTH] = {0};
  if (u2hts_get_custom_config(MT_CUSTOM_CONFIG_DEV, (uint8_t*)path,
                              sizeof(path))) {
    mt_fd = mt_open_device(path);
    if (mt_fd < 0) {
      ULOGE(LOG_TAG, "failed to open %s: %s", path, strerror(errno));
      return false;
    }
  } else {
    for (int i = 0; i < MT_INPUT_DEV_MAX && mt_fd < 0; i++) {
      snprintf(path, sizeof(path), "/dev/input/event%d", i);
      mt_fd = mt_open_device(path);
    }
    if (mt_fd < 0) {
      ULOGE(LOG_TAG, "no multitouch input device found");
      return false;
    }
  }

  struct input_absinfo x_info = {0}, y_info = {0}, slot_info = {0};
  if (ioctl(mt_fd, EVIOCGABS(ABS_MT_POSITION_X), &x_info) < 0 ||
      ioctl(mt_fd, EVIOCGABS(ABS_MT_POSITION_Y), &y_info) < 0) {
    ULOGE(LOG_TAG, "failed to get abs info: %s", strerror(errno));
    close(mt_fd);
    mt_fd = -1;
    return false;
  }
  ioctl(mt_fd, EVIOCGABS(ABS_MT_SLOT), &slot_info);

  mt_x_max = x_info.maximum > 0 ? x_info.maximum : U2HTS_LOGICAL_MAX;
  mt_y_max = y_info.maximum > 0 ? y_info.maximum : U2HTS_LOGICAL_MAX;
  if (mt_x_max > U2HTS_LOGICAL_MAX) mt_x_max = U2HTS_LOGICAL_MAX;
  if (mt_y_max > U2HTS_LOGICAL_MAX) mt_y_max = U2HTS_LOGICAL_MAX;

  int max_tps = slot_info.maximum > 0 ? slot_info.maximum + 1 : 1;
  if (max_tps > U2HTS_MAX_TPS) max_tps = U2HTS_MAX_TPS;
  mt_max_tps = (uint8_t)max_tps;

  for (uint8_t i = 0; i < U2HTS_MAX_TPS; i++) mt_slots[i].tracking_id = -1;

  ULOGI(LOG_TAG, "using %s, x_max = %d, y_max = %d, max_tps = %d", path,
        mt_x_max, mt_y_max, mt_max_tps);
  return true;
}

static void mt_get_config(u2hts_touch_controller_coord_config* cfg) {
  cfg->max_tps = mt_max_tps;
  cfg->x_max = mt_x_max;
  cfg->y_max = mt_y_max;
}

static uint16_t mt_clamp_coord(int32_t value, int max) {
  if (value < 0) return 0;
  if (value > max) return (uint16_t)max;
  return (uint16_t)value;
}

static uint8_t mt_submit() {
  uint8_t count = 0;
  for (uint8_t i = 0; i < mt_max_tps; i++) {
    if (mt_slots[i].tracking_id < 0) continue;
    u2hts_set_tp(count++, true, (uint8_t)mt_slots[i].tracking_id,
                 mt_clamp_coord(mt_slots[i].x, mt_x_max),
                 mt_clamp_coord(mt_slots[i].y, mt_y_max), 0, 0, 0);
  }
  u2hts_set_tp_count(count);
  return count;
}

static bool mt_fetch() {
  if (mt_fd < 0) return false;

  struct input_event ev;
  ssize_t n;
  while ((n = read(mt_fd, &ev, sizeof(ev))) == (ssize_t)sizeof(ev)) {
    switch (ev.type) {
      case EV_ABS:
        switch (ev.code) {
          case ABS_MT_SLOT:
            if (ev.value >= 0 && ev.value < mt_max_tps)
              mt_current_slot = (uint8_t)ev.value;
            break;
          case ABS_MT_TRACKING_ID:
            mt_slots[mt_current_slot].tracking_id =
                ev.value < 0 ? -1 : (ev.value & 0x7F);
            break;
          case ABS_MT_POSITION_X:
            mt_slots[mt_current_slot].x = ev.value;
            break;
          case ABS_MT_POSITION_Y:
            mt_slots[mt_current_slot].y = ev.value;
            break;
        }
        break;
      case EV_SYN:
        if (ev.code == SYN_DROPPED) {
          for (uint8_t i = 0; i < U2HTS_MAX_TPS; i++)
            mt_slots[i].tracking_id = -1;
        }
        break;
    }
  }

  if (n == 0) return false;
  if (n < 0 && errno != EAGAIN && errno != EINTR) {
    ULOGE(LOG_TAG, "read failed: %s", strerror(errno));
    return false;
  }

  return mt_submit() > 0;
}
