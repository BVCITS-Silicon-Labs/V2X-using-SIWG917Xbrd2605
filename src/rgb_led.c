/***************************************************************************//**
 * @file rgb_led.c
 * @brief RGB LED control functions for SiWG917
 ******************************************************************************/

#include "rgb_led.h"

#include "sl_si91x_rgb_led.h"
#include "sl_si91x_rgb_led_instances.h"

#ifndef RED
#define RED led_red
#endif

#ifndef GREEN
#define GREEN led_green
#endif

#ifndef BLUE
#define BLUE led_blue
#endif

void turn_on_green(void)
{
  sl_si91x_rgb_led_on(&GREEN);
}

void turn_on_red(void)
{
  sl_si91x_rgb_led_on(&RED);
}

void turn_on_blue(void)
{
  sl_si91x_rgb_led_on(&BLUE);
}

void turn_off_green(void)
{
  sl_si91x_rgb_led_off(&GREEN);
}

void turn_off_red(void)
{
  sl_si91x_rgb_led_off(&RED);
}

void turn_off_blue(void)
{
  sl_si91x_rgb_led_off(&BLUE);
}

void turn_off_all_leds(void)
{
  sl_si91x_rgb_led_off(&GREEN);
  sl_si91x_rgb_led_off(&RED);
  sl_si91x_rgb_led_off(&BLUE);
}
