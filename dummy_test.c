/*
  Copyright (C) CNflysky.
  U2HTS stands for "USB to HID TouchScreen".

  dummy_test.c: generate random coordinates report to host.
  WARNING: THIS DRIVER IS INTENDED FOR TOUCH REPORT RATE MEASUREMENT,
  IT GENERATES LOT OF RANDOM TOUCHS MAY CAUSE UNEXPECTED BEHAVIOUR ON YOUR HOST!
  To monitor input device report rate:
    - Build the "getevent" tool (originally from Android, you can find many
  Linux ports on GitHub):
    - Run `./getevent -r`.

  To use this driver:
    - Enable polling mode.
    - set controller_name to "dummy".
  See `CMakeLists.txt` for build.

  This file is licensed under GPL V3.
  All rights reserved.
*/
#include <stdlib.h>

#include "u2hts_core.h"
static bool dummy_setup(U2HTS_BUS_TYPES bus_type) {
  U2HTS_UNUSED(bus_type);
  int32_t rand_seed = 0;
  u2hts_get_custom_config_i32("dummy.rand_seed");
  if (rand_seed < 0) rand_seed = u2hts_get_timestamp();
  srand(rand_seed);
  return true;
}

static bool dummy_coord_fetch() {
  uint8_t tp_count = rand() % 10;
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  for (uint8_t i = 0; i < tp_count; i++)
    u2hts_set_tp(i, true, i, rand() % 4096, rand() % 4096, rand() % 256,
                 rand() % 256, rand() % 256);
  return true;
}

static void dummy_get_config(u2hts_touch_controller_config* cfg) {
  cfg->max_tps = 10;
  cfg->x_max = 4095;
  cfg->y_max = 4095;
}

static u2hts_touch_controller_operations dummy_ops = {
    .setup = &dummy_setup,
    .fetch = &dummy_coord_fetch,
    .get_config = &dummy_get_config};

static u2hts_touch_controller dummy = {.name = "dummy",
                                       .irq_type = 0xFF,
                                       .report_mode = UTC_REPORT_MODE_CONTINOUS,
                                       .operations = &dummy_ops};
U2HTS_TOUCH_CONTROLLER(dummy);