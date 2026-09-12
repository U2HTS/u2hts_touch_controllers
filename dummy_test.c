/*
  Copyright (C) U2HTS Developers. All rights reserved.
  U2HTS stands for "USB to HID TouchScreen".

  dummy_test.c: report rate measurement driver.
  WARNING: RANDOM TOUCHS MAY CAUSE UNEXPECTED BEHAVIOUR ON YOUR HOST!

  To get input device report rate:
    - Build the "getevent" tool (originally from Android, you can find many
  Linux ports on GitHub):
    - Run `./getevent -r`.

  To use this driver:
    - Enable polling mode.
    - set controller_name to "dummy".
    - Add this file to your build system.

  This file is licensed under GPL V3.
*/
#include <stdlib.h>

#include "u2hts_core.h"

static unsigned long int u2hts_rand_state = 1;

__weak_symbol void srand(unsigned int seed)
{
  u2hts_rand_state = seed;
}

__weak_symbol int rand(void)
{
  u2hts_rand_state = u2hts_rand_state * 1103515245UL + 12345UL;
  return (int)((u2hts_rand_state >> 16) & 0x7FFF);
}

static bool dummy_setup(U2HTS_BUS_TYPES bus_type)
{
  U2HTS_UNUSED(bus_type);
  srand(u2hts_get_custom_config_u32("dummy.rand_seed", u2hts_get_timestamp()));
  return true;
}

static bool dummy_coord_fetch()
{
  uint8_t tp_count = rand() % 10;
  U2HTS_SET_TP_COUNT_SAFE(tp_count);
  for (uint8_t i = 0; i < tp_count; i++)
    u2hts_set_tp(i, true, i, rand() % 4096, rand() % 4096, rand() % 256, rand() % 256, rand() % 256);
  return true;
}

static void dummy_get_config(u2hts_touch_controller_config *cfg)
{
  cfg->max_tps = U2HTS_MAX_TPS;
  cfg->x_max = U2HTS_LOGICAL_MAX;
  cfg->y_max = U2HTS_LOGICAL_MAX;
}

static u2hts_touch_controller_operations dummy_ops = {
    .setup = &dummy_setup, .fetch = &dummy_coord_fetch, .get_config = &dummy_get_config};

static u2hts_touch_controller dummy = {.name = "dummy",
                                       .irq_type = 0xFF,
                                       .report_mode = UTC_REPORT_MODE_CONTINOUS,
                                       // bypass u2hts_init bus type detection
                                       .i2c_config = {.primary_addr = 0xFF, .speed_hz = 100 * 1000},
                                       .operations = &dummy_ops};
U2HTS_TOUCH_CONTROLLER(dummy);