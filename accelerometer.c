/***************************************************************************//**
 * @file accelerometer.c
 * @brief ICM40627 IMU driver and Float32 sampling buffer
 ******************************************************************************/

#include "accelerometer.h"
#include "constants.h"
#include "sl_si91x_icm40627.h"
#include "sl_si91x_driver_gpio.h"
#include "sl_si91x_clock_manager.h"
#include "sl_si91x_ssi.h"
#include "rsi_rom_clks.h"
#include "sl_sleeptimer.h"
#include <stdio.h>
#include <string.h>

#if defined(SL_COMPONENT_CATALOG_PRESENT)
#include "sl_component_catalog.h"
#endif

/*******************************************************************************
 * Defines
 ******************************************************************************/
#define IMU_BUFFER_CAPACITY               100

#define SSI_MASTER_BIT_WIDTH              8
#define SSI_MASTER_BAUDRATE               10000000
#define SSI_MASTER_RECEIVE_SAMPLE_DELAY   0

#ifndef SENSOR_ENABLE_GPIO_PIN
#define SENSOR_ENABLE_GPIO_PIN            1
#endif

/*******************************************************************************
 * Local Variables
 ******************************************************************************/
static sl_ssi_handle_t s_ssi_handle = NULL;
static const sl_ssi_slave_number_t s_ssi_slave_number = SSI_SLAVE_0;

/* Circular Float32 sample buffer */
static acc_data_t s_sample_buffer[IMU_BUFFER_CAPACITY];
static volatile int s_head_index = 0;
static volatile uint32_t s_total_samples = 0;
static bool s_driver_initialized = false;

/*******************************************************************************
 * Local Function Prototypes
 ******************************************************************************/
static void ssi_master_event_callback(uint32_t event);
static sl_status_t enable_sensor_power(bool enable);

/*******************************************************************************
 * Interrupt & Sampling Handlers
 ******************************************************************************/
static void ssi_master_event_callback(uint32_t event)
{
  (void)event;
}

static sl_status_t enable_sensor_power(bool enable)
{
  sl_status_t status =
      sl_si91x_gpio_driver_enable_clock(
          (sl_si91x_gpio_select_clock_t)ULPCLK_GPIO);

  if (status != SL_STATUS_OK) {
    return status;
  }

  if (enable) {
    status = sl_si91x_gpio_driver_set_uulp_npss_pin_mux(
        SENSOR_ENABLE_GPIO_PIN,
        NPSS_GPIO_PIN_MUX_MODE1);

    if (status != SL_STATUS_OK) {
      return status;
    }

    status = sl_si91x_gpio_driver_set_uulp_npss_direction(
        SENSOR_ENABLE_GPIO_PIN,
        (sl_si91x_gpio_direction_t)GPIO_OUTPUT);

    if (status != SL_STATUS_OK) {
      return status;
    }

    status = sl_si91x_gpio_driver_set_uulp_npss_pin_value(
        SENSOR_ENABLE_GPIO_PIN,
        (sl_si91x_gpio_pin_value_t)1);

    if (status != SL_STATUS_OK) {
      return status;
    }
  } else {
    status = sl_si91x_gpio_driver_set_uulp_npss_pin_value(
        SENSOR_ENABLE_GPIO_PIN,
        (sl_si91x_gpio_pin_value_t)0);

    if (status != SL_STATUS_OK) {
      return status;
    }
  }

  return SL_STATUS_OK;
}

/*******************************************************************************
 * Public API
 ******************************************************************************/
sl_status_t accelerometer_setup(void)
{
  sl_status_t status;
  uint8_t dev_id = 0;
  sl_ssi_control_config_t ssi_master_config;

  printf("[IMU] Enabling sensor power rail...\r\n");
  fflush(stdout);

  status = enable_sensor_power(true);

  if (status != SL_STATUS_OK) {
    printf("[IMU] Power rail enable failed (0x%lx)\r\n",
           (unsigned long)status);
    fflush(stdout);
  }

  /* Allow sensor power rail and decoupling capacitors to stabilize */
  sl_sleeptimer_delay_millisecond(25);

  /*
   * 1. Initialize SSI ULP instance.
   */
  status = sl_si91x_ssi_init(
      SL_SSI_ULP_PRIMARY_ACTIVE,
      &s_ssi_handle);

  if (status != SL_STATUS_OK || s_ssi_handle == NULL) {
    printf("[IMU] SSI Init failed (0x%lx)\r\n",
           (unsigned long)status);
    fflush(stdout);
    return status;
  }

  /*
   * 2. Configure SSI master mode.
   */
  ssi_master_config.bit_width =
      SSI_MASTER_BIT_WIDTH;

  ssi_master_config.device_mode =
      SL_SSI_ULP_MASTER_ACTIVE;

  ssi_master_config.clock_mode =
      SL_SSI_PERIPHERAL_CPOL0_CPHA0;

  ssi_master_config.baud_rate =
      SSI_MASTER_BAUDRATE;

  ssi_master_config.receive_sample_delay =
      SSI_MASTER_RECEIVE_SAMPLE_DELAY;

  status = sl_si91x_ssi_set_configuration(
      s_ssi_handle,
      &ssi_master_config,
      s_ssi_slave_number);

  if (status != SL_STATUS_OK) {
    printf("[IMU] SSI set configuration failed (0x%lx)\r\n",
           (unsigned long)status);
    fflush(stdout);
    return status;
  }

  sl_si91x_ssi_register_event_callback(
      s_ssi_handle,
      ssi_master_event_callback);

  sl_si91x_ssi_set_slave_number(
      (uint8_t)s_ssi_slave_number);

  /* Allow SPI bus lines to stabilize */
  sl_sleeptimer_delay_millisecond(10);

  /*
   * 3. Reset ICM40627.
   */
  status = sl_si91x_icm40627_software_reset(
      s_ssi_handle);

  if (status != SL_STATUS_OK) {
    printf("[IMU] Reset failed (0x%lx)\r\n",
           (unsigned long)status);
    fflush(stdout);
    return status;
  }

  /* Allow sensor reset settling time */
  sl_sleeptimer_delay_millisecond(15);

  /*
   * 4. Verify WHO_AM_I device signature.
   */
  status = sl_si91x_icm40627_get_device_id(
      s_ssi_handle,
      &dev_id);

  if (status != SL_STATUS_OK ||
      dev_id != ICM40627_DEVICE_ID) {

    printf("[IMU] WHO_AM_I mismatch: 0x%02X "
           "(Expected 0x4E)\r\n",
           dev_id);
    fflush(stdout);

  } else {

    printf("[IMU] ICM40627 ID 0x%02X verified.\r\n",
           dev_id);
    fflush(stdout);
  }

  /*
   * 5. Directly initialize the ICM40627 registers
   *    using the native ICM40627 driver.
   */
  status = sl_si91x_icm40627_init(
      s_ssi_handle);

  if (status != SL_STATUS_OK) {
    printf("[IMU] Direct icm40627_init failed: 0x%lx\r\n",
           (unsigned long)status);
    fflush(stdout);
  }

  sl_sleeptimer_delay_millisecond(10);

  /*
   * Reset software sample buffer state.
   */
  s_head_index = 0;
  s_total_samples = 0;
  s_driver_initialized = true;

  printf("[IMU] Initialized at %d Hz sampling rate.\r\n",
         ACCELEROMETER_FREQ);
  fflush(stdout);

  return SL_STATUS_OK;
}

bool accelerometer_poll(void)
{
  if (!s_driver_initialized) {
    return false;
  }

  float accel_g[3] = {
    0.0f,
    0.0f,
    0.0f
  };

  /* Read accelerometer data directly from ICM40627 */
  sl_status_t status =
      sl_si91x_icm40627_get_accel_data(
          s_ssi_handle,
          accel_g);

  if (status != SL_STATUS_OK) {
    return false;
  }

  /* Push sample into circular buffer */
  int idx = s_head_index;

  s_sample_buffer[idx].x = accel_g[0];
  s_sample_buffer[idx].y = accel_g[1];
  s_sample_buffer[idx].z = accel_g[2];

  idx++;

  if (idx >= IMU_BUFFER_CAPACITY) {
    idx = 0;
  }

  s_head_index = idx;
  s_total_samples++;

  return true;
}

sl_status_t accelerometer_read(
    acc_data_t *dst,
    int n)
{
  if (!s_driver_initialized ||
      s_total_samples < (uint32_t)n) {
    return SL_STATUS_FAIL;
  }

  if (dst == NULL || n <= 0) {
    return SL_STATUS_OK;
  }

  int curr_head = s_head_index;

  for (int i = 0; i < n; i++) {

    int sample_idx =
        curr_head - n + i;

    while (sample_idx < 0) {
      sample_idx += IMU_BUFFER_CAPACITY;
    }

    sample_idx %= IMU_BUFFER_CAPACITY;

    dst[i] =
        s_sample_buffer[sample_idx];
  }

  return SL_STATUS_OK;
}

bool accelerometer_get_latest(
    float *x,
    float *y,
    float *z)
{
  if (!s_driver_initialized ||
      s_total_samples == 0) {
    return false;
  }

  int last_idx =
      s_head_index - 1;

  if (last_idx < 0) {
    last_idx += IMU_BUFFER_CAPACITY;
  }

  if (x != NULL) {
    *x = s_sample_buffer[last_idx].x;
  }

  if (y != NULL) {
    *y = s_sample_buffer[last_idx].y;
  }

  if (z != NULL) {
    *z = s_sample_buffer[last_idx].z;
  }

  return true;
}

bool accelerometer_is_ready(void)
{
  return (
      s_driver_initialized &&
      (s_total_samples >= SEQUENCE_LENGTH)
  );
}