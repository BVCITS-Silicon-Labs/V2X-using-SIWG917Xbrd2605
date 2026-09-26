/***************************************************************************//**
 * @file bsm_packet.h
 * @brief Versioned V2X Frame & Telemetry Packet Protocol
 ******************************************************************************/

#ifndef BSM_PACKET_H
#define BSM_PACKET_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif


/*******************************************************************************
 * V2X PROTOCOL
 ******************************************************************************/

#define V2X_FRAME_MAGIC          0x56325831u
#define V2X_FRAME_VERSION        1u

#define V2X_MSG_TYPE_PERIODIC    0x01
#define V2X_MSG_TYPE_EMERGENCY   0x02
#define V2X_MSG_TYPE_GESTURE     0x03

#define V2X_UDP_PORT             5000

#define V2X_BROADCAST_IP         "255.255.255.255"

#define V2X_DEFAULT_PERIOD_MS    100


/*******************************************************************************
 * V2X STATUS
 ******************************************************************************/

typedef enum {

  V2X_STATUS_SAFE      = 0,
  V2X_STATUS_WARNING   = 1,
  V2X_STATUS_CRITICAL  = 2,
  V2X_STATUS_EMERGENCY = 3

} v2x_status_t;


/*******************************************************************************
 * V2X FRAME
 ******************************************************************************/

#pragma pack(push, 1)

typedef struct {

  uint32_t magic;

  uint8_t version;

  uint8_t message_type;

  uint16_t payload_length;

  uint16_t sequence;

  uint8_t sender_id[6];

  uint32_t timestamp_ms;

  /*
   * Accelerometer data from the transmitting board.
   *
   * Units:
   *     g
   */
  float accel_x;

  float accel_y;

  float accel_z;

  /*
   * No AI/ML processing.
   *
   * Kept for protocol compatibility.
   */
  uint8_t gesture;

  /*
   * 0 = normal
   * 1 = emergency / sudden deceleration
   */
  uint8_t emergency;

  uint16_t reserved;

  /*
   * 0.0 for normal periodic packet.
   * 1.0 for confirmed emergency event.
   */
  float confidence;

  uint16_t checksum;

} v2x_frame_t;

#pragma pack(pop)


/*
 * Compatibility name.
 */
typedef v2x_frame_t bsm_packet_t;


/*******************************************************************************
 * CHECKSUM
 ******************************************************************************/

static inline uint16_t
v2x_compute_checksum(
    const v2x_frame_t *frame)
{
  const uint8_t *data =
      (const uint8_t *)frame;

  size_t len =
      offsetof(v2x_frame_t, checksum);

  uint16_t sum1 = 0;
  uint16_t sum2 = 0;


  for (size_t i = 0;
       i < len;
       i++) {

    sum1 =
        (uint16_t)(
            (sum1 + data[i]) % 255);

    sum2 =
        (uint16_t)(
            (sum2 + sum1) % 255);
  }


  return (uint16_t)(
      (sum2 << 8) | sum1);
}


/*******************************************************************************
 * SEAL FRAME
 ******************************************************************************/

static inline void
v2x_seal_frame(
    v2x_frame_t *frame)
{
  if (frame == NULL) {
    return;
  }


  frame->magic =
      V2X_FRAME_MAGIC;

  frame->version =
      V2X_FRAME_VERSION;

  frame->payload_length =
      (uint16_t)sizeof(v2x_frame_t);

  frame->reserved =
      0;

  /*
   * Calculate checksum with checksum field zero.
   */
  frame->checksum =
      0;

  frame->checksum =
      v2x_compute_checksum(frame);
}


/*******************************************************************************
 * VALIDATE FRAME
 ******************************************************************************/

static inline bool
v2x_validate_frame(
    const v2x_frame_t *frame,
    size_t bytes_received)
{
  if (frame == NULL) {
    return false;
  }


  if (bytes_received !=
      sizeof(v2x_frame_t)) {

    return false;
  }


  if (frame->magic !=
      V2X_FRAME_MAGIC) {

    return false;
  }


  if (frame->version !=
      V2X_FRAME_VERSION) {

    return false;
  }


  if (frame->payload_length !=
      (uint16_t)sizeof(v2x_frame_t)) {

    return false;
  }


  v2x_frame_t tmp =
      *frame;


  uint16_t received_chk =
      tmp.checksum;


  tmp.checksum =
      0;


  return (
      v2x_compute_checksum(&tmp) ==
      received_chk
  );
}


/*******************************************************************************
 * COMPATIBILITY FUNCTIONS
 ******************************************************************************/

static inline void
bsm_packet_set_checksum(
    bsm_packet_t *pkt)
{
  v2x_seal_frame(pkt);
}


static inline int
bsm_packet_verify_checksum(
    const bsm_packet_t *pkt)
{
  return
      v2x_validate_frame(
          pkt,
          sizeof(bsm_packet_t))
      ? 1
      : 0;
}


#ifdef __cplusplus
}
#endif

#endif /* BSM_PACKET_H */
