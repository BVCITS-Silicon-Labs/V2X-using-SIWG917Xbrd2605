/***************************************************************************//**
 * @file constants.h
 * @brief Constants for use in the V2X AI/ML application
 ******************************************************************************/

#ifndef CONSTANTS_H
#define CONSTANTS_H

/* Accelerometer */
#define ACCELEROMETER_FREQ        25
#define ACCELEROMETER_CHANNELS    3

/* LED */
#define TOGGLE_DELAY_MS           2000

/* Neural-network inference */
#define INFERENCE_PERIOD_MS       200

/* Model input sequence */
#define SEQUENCE_LENGTH           90

/* Gesture classes */
#define GESTURE_COUNT             4

#define WING_GESTURE              0
#define RING_GESTURE              1
#define SLOPE_GESTURE             2
#define NO_GESTURE                3

/* Prediction filtering */
#define DETECTION_THRESHOLD       0.9f
#define PREDICTION_HISTORY_LEN    5
#define PREDICTION_SUPPRESSION    18

#endif /* CONSTANTS_H */
