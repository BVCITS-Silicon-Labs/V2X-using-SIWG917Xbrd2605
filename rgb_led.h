/***************************************************************************//**
 * @file rgb_led.h
 * @brief Visual status indicator interface for V2X platform
 ******************************************************************************/

#ifndef RGB_LED_H
#define RGB_LED_H

#ifdef __cplusplus
extern "C" {
#endif

void turn_on_green(void);
void turn_on_red(void);
void turn_on_blue(void);

void turn_off_green(void);
void turn_off_red(void);
void turn_off_blue(void);

void turn_off_all_leds(void);

#ifdef __cplusplus
}
#endif

#endif /* RGB_LED_H */