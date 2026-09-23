/***************************************************************************//**
 * @file accelerometer.h
 * @brief Functionality for reading calibrated accelerometer data from IMU
 *******************************************************************************
 * # License
 * <b>Copyright 2024 Silicon Laboratories Inc. www.silabs.com</b>
 ******************************************************************************/

#ifndef ACCELEROMETER_H
#define ACCELEROMETER_H

#include <stdint.h>
#include <stdbool.h>
#include "sl_status.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 3-axis accelerometer float vector in g units
 *
 * 1.0 g is approximately 9.81 m/s^2.
 */
typedef struct acc_data {
  float x;
  float y;
  float z;
} acc_data_t;

/**
 * @brief Initialize SSI master interface, power rail,
 *        and ICM40627 IMU sensor.
 *
 * @return SL_STATUS_OK on success, error code on failure.
 */
sl_status_t accelerometer_setup(void);

/**
 * @brief Poll and process IMU hardware data into the
 *        circular sample buffer.
 *
 * @return true if a new sample was ingested,
 *         false otherwise.
 */
bool accelerometer_poll(void);

/**
 * @brief Read the last N accelerometer samples.
 *
 * @param[out] dst Destination array for model input.
 * @param[in] n Number of samples to read.
 *
 * @return SL_STATUS_OK if sufficient samples are available,
 *         SL_STATUS_FAIL otherwise.
 */
sl_status_t accelerometer_read(
    acc_data_t *dst,
    int n);

/**
 * @brief Get the most recent instantaneous acceleration sample.
 *
 * @param[out] x Acceleration X in g.
 * @param[out] y Acceleration Y in g.
 * @param[out] z Acceleration Z in g.
 *
 * @return true if valid data is available.
 */
bool accelerometer_get_latest(
    float *x,
    float *y,
    float *z);

/**
 * @brief Check whether enough samples have been collected
 *        for neural-network inference.
 *
 * @return true when at least SEQUENCE_LENGTH samples exist.
 */
bool accelerometer_is_ready(void);

#ifdef __cplusplus
}
#endif

#endif /* ACCELEROMETER_H */