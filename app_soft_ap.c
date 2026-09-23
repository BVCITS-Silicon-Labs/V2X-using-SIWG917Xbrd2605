/***************************************************************************//**
 * @file app.c
 * @brief SiWG917 V2X UDP Soft AP TX/RX application with ICM40627
 ******************************************************************************/

#include <sys/types.h>
#include <string.h>
#include <stdio.h>
#include "errno.h"

#include "app.h"
#include "bsm_packet.h"
#include "constants.h"
#include "accelerometer.h"

#include "sl_board_configuration.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#include "sl_wifi.h"
#include "sl_net.h"
#include "socket.h"

#include "sl_si91x_socket_support.h"
#include "sl_si91x_socket_constants.h"
#include "sl_si91x_socket_utility.h"
#include "sl_utility.h"
#include "sl_si91x_driver.h"
#include "sl_net_wifi_types.h"
#include "sl_wifi_callback_framework.h"
#include "sl_sleeptimer.h"


/*******************************************************************************
 * V2X Configuration
 ******************************************************************************/

#define V2X_AP_SSID                 "V2X_AP_NODE"
#define V2X_AP_PASSWORD             "12345678"
#define V2X_AP_CHANNEL              6
#define V2X_AP_SECURITY_WPA2       1

#define V2X_TX_PERIOD_MS            100
#define V2X_TX_JITTER_MAX_MS        20


/*******************************************************************************
 * Thread Configuration
 ******************************************************************************/

static const osThreadAttr_t s_net_thread_attr = {
  .name       = "v2x_net_task",
  .attr_bits  = 0,
  .cb_mem     = NULL,
  .cb_size    = 0,
  .stack_mem  = NULL,
  .stack_size = 3584,
  .priority   = osPriorityNormal,
  .tz_module  = 0,
  .reserved   = 0
};


static const osThreadAttr_t s_rx_thread_attr = {
  .name       = "v2x_rx_task",
  .attr_bits  = 0,
  .cb_mem     = NULL,
  .cb_size    = 0,
  .stack_mem  = NULL,
  .stack_size = 3072,
  .priority   = osPriorityBelowNormal,
  .tz_module  = 0,
  .reserved   = 0
};


/*******************************************************************************
 * Global Network State
 ******************************************************************************/

static volatile bool s_peer_connected = false;

static uint8_t s_local_mac[6] = { 0 };

static int s_udp_socket = -1;

static struct sockaddr_in s_broadcast_addr;


/*******************************************************************************
 * Soft AP Device Configuration
 ******************************************************************************/

static const sl_wifi_device_configuration_t s_wifi_ap_device_config = {

  .boot_option = LOAD_NWP_FW,

  .mac_address = NULL,

  .band = SL_SI91X_WIFI_BAND_2_4GHZ,

  .region_code = US,

  .boot_config = {

    .oper_mode = SL_SI91X_ACCESS_POINT_MODE,

    .coex_mode = SL_SI91X_WLAN_ONLY_MODE,

    .feature_bit_map =
      (SL_SI91X_FEAT_SECURITY_PSK |
       SL_SI91X_FEAT_AGGREGATION),

    .tcp_ip_feature_bit_map =
      (SL_SI91X_TCP_IP_FEAT_DHCPV4_SERVER |
       SL_SI91X_TCP_IP_FEAT_ICMP |
       SL_SI91X_TCP_IP_FEAT_EXTENSION_VALID),

    .custom_feature_bit_map =
      SL_SI91X_CUSTOM_FEAT_EXTENTION_VALID,

    .ext_custom_feature_bit_map =
      MEMORY_CONFIG,

    .bt_feature_bit_map = 0,

    .ext_tcp_ip_feature_bit_map =
      SL_SI91X_EXT_TCP_IP_WAIT_FOR_SOCKET_CLOSE,

    .ble_feature_bit_map = 0,

    .ble_ext_feature_bit_map = 0,

    .config_feature_bit_map = 0
  }
};


/*******************************************************************************
 * FreeRTOS Stack Overflow Hook
 ******************************************************************************/

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   char *pcTaskName)
{
  (void)xTask;

  printf("\r\n");
  printf("========================================================\r\n");
  printf("FATAL: STACK OVERFLOW\r\n");
  printf("TASK: %s\r\n", pcTaskName);
  printf("========================================================\r\n");

  fflush(stdout);

  while (1) {
    __asm volatile("nop");
  }
}


/*******************************************************************************
 * AP Client Connected Callback
 ******************************************************************************/

static sl_status_t ap_client_connected_callback(
    sl_wifi_event_t event,
    sl_status_t status_code,
    void *data,
    uint32_t data_length,
    void *arg)
{
  (void)data_length;
  (void)arg;

  if (SL_WIFI_CHECK_IF_EVENT_FAILED(event)) {
    return status_code;
  }

  if (data == NULL) {
    return SL_STATUS_FAIL;
  }

  sl_mac_address_t *mac =
      (sl_mac_address_t *)data;

  s_peer_connected = true;

  printf(
      "\r\n[V2X NET] Peer Connected: "
      "%02X:%02X:%02X:%02X:%02X:%02X\r\n",

      mac->octet[0],
      mac->octet[1],
      mac->octet[2],
      mac->octet[3],
      mac->octet[4],
      mac->octet[5]);

  fflush(stdout);

  return SL_STATUS_OK;
}


/*******************************************************************************
 * AP Client Disconnected Callback
 ******************************************************************************/

static sl_status_t ap_client_disconnected_callback(
    sl_wifi_event_t event,
    sl_status_t status_code,
    void *data,
    uint32_t data_length,
    void *arg)
{
  (void)data_length;
  (void)arg;

  if (SL_WIFI_CHECK_IF_EVENT_FAILED(event)) {
    return status_code;
  }

  if (data == NULL) {
    return SL_STATUS_FAIL;
  }

  sl_mac_address_t *mac =
      (sl_mac_address_t *)data;

  s_peer_connected = false;

  printf(
      "\r\n[V2X NET] Peer Disconnected: "
      "%02X:%02X:%02X:%02X:%02X:%02X\r\n",

      mac->octet[0],
      mac->octet[1],
      mac->octet[2],
      mac->octet[3],
      mac->octet[4],
      mac->octet[5]);

  fflush(stdout);

  return SL_STATUS_OK;
}


/*******************************************************************************
 * Initialize Wi-Fi Soft AP
 ******************************************************************************/

static sl_status_t v2x_wifi_init_ap(void)
{
  sl_status_t status;

  sl_net_wifi_ap_profile_t ap_profile;

  memset(
      &ap_profile,
      0,
      sizeof(ap_profile));


  printf(
      "[V2X NET] Initializing Network Subsystem (Soft AP)...\r\n");

  fflush(stdout);


  /***************************************************************************
   * Network initialization
   **************************************************************************/

  printf(
      "[V2X NET] >>> Calling sl_net_init()...\r\n");

  fflush(stdout);

  osDelay(100);

  printf(
      "[V2X NET] >>> ENTERING sl_net_init NOW\r\n");

  fflush(stdout);

  status =
      sl_net_init(
          SL_NET_WIFI_AP_INTERFACE,
          &s_wifi_ap_device_config,
          NULL,
          NULL);

  printf(
      "[V2X NET] >>> sl_net_init() RETURNED\r\n");

  printf(
      "[V2X NET] Status = 0x%08lX\r\n",
      (unsigned long)status);

  fflush(stdout);


  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] ERROR: sl_net_init failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }


  printf(
      "[V2X NET] sl_net_init successful.\r\n");

  fflush(stdout);


  /***************************************************************************
   * Register Wi-Fi callbacks
   **************************************************************************/

  status =
      sl_wifi_set_callback_v2(
          SL_WIFI_CLIENT_CONNECTED_EVENTS,
          ap_client_connected_callback,
          NULL);

  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] WARNING: connected callback registration "
        "failed: 0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);
  }


  status =
      sl_wifi_set_callback_v2(
          SL_WIFI_CLIENT_DISCONNECTED_EVENTS,
          ap_client_disconnected_callback,
          NULL);

  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] WARNING: disconnected callback registration "
        "failed: 0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);
  }


  /***************************************************************************
   * AP radio configuration
   **************************************************************************/

  ap_profile.config.channel.channel =
      V2X_AP_CHANNEL;

  ap_profile.config.channel.band =
      SL_WIFI_BAND_2_4GHZ;

  ap_profile.config.rate_protocol =
      SL_WIFI_RATE_PROTOCOL_AUTO;

  ap_profile.config.options = 0;

  ap_profile.config.maximum_clients = 4;

  ap_profile.config.beacon_interval = 100;

  ap_profile.config.dtim_beacon_count = 3;

  ap_profile.config.client_idle_timeout = 0xFF;

  ap_profile.config.keepalive_type =
      SL_SI91X_AP_NULL_BASED_KEEP_ALIVE;

  ap_profile.config.beacon_stop = 0;

  ap_profile.config.tdi_flags =
      SL_WIFI_TDI_NONE;

  ap_profile.config.is_11n_enabled = 1;


  /***************************************************************************
   * SSID
   **************************************************************************/

  ap_profile.config.ssid.length =
      (uint8_t)strlen(V2X_AP_SSID);

  memcpy(
      ap_profile.config.ssid.value,
      V2X_AP_SSID,
      ap_profile.config.ssid.length);


  /***************************************************************************
   * Security
   **************************************************************************/

#if (V2X_AP_SECURITY_WPA2 == 1)

  status =
      sl_net_set_credential(
          SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID,
          SL_NET_WIFI_PSK,
          V2X_AP_PASSWORD,
          strlen(V2X_AP_PASSWORD));

  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] ERROR: Credential setup failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }

  ap_profile.config.security =
      SL_WIFI_WPA2;

  ap_profile.config.encryption =
      SL_WIFI_CCMP_ENCRYPTION;

  ap_profile.config.credential_id =
      SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID;

#else

  ap_profile.config.security =
      SL_WIFI_OPEN;

  ap_profile.config.encryption =
      SL_WIFI_NO_ENCRYPTION;

  ap_profile.config.credential_id =
      SL_NET_DEFAULT_WIFI_AP_CREDENTIAL_ID;

#endif


  /***************************************************************************
   * Static IPv4 configuration
   *
   * IP      : 192.168.10.1
   * Gateway : 192.168.10.1
   * Netmask : 255.255.255.0
   **************************************************************************/

  ap_profile.ip.type =
      SL_IPV4;

  ap_profile.ip.mode =
      SL_IP_MANAGEMENT_STATIC_IP;


  ap_profile.ip.ip.v4.ip_address.bytes[0] = 192;
  ap_profile.ip.ip.v4.ip_address.bytes[1] = 168;
  ap_profile.ip.ip.v4.ip_address.bytes[2] = 10;
  ap_profile.ip.ip.v4.ip_address.bytes[3] = 1;


  ap_profile.ip.ip.v4.gateway.bytes[0] = 192;
  ap_profile.ip.ip.v4.gateway.bytes[1] = 168;
  ap_profile.ip.ip.v4.gateway.bytes[2] = 10;
  ap_profile.ip.ip.v4.gateway.bytes[3] = 1;


  ap_profile.ip.ip.v4.netmask.bytes[0] = 255;
  ap_profile.ip.ip.v4.netmask.bytes[1] = 255;
  ap_profile.ip.ip.v4.netmask.bytes[2] = 255;
  ap_profile.ip.ip.v4.netmask.bytes[3] = 0;


  /***************************************************************************
   * Set AP profile
   **************************************************************************/

  printf(
      "[V2X NET] >>> Calling sl_net_set_profile()...\r\n");

  fflush(stdout);

  status =
      sl_net_set_profile(
          SL_NET_WIFI_AP_INTERFACE,
          SL_NET_DEFAULT_WIFI_AP_PROFILE_ID,
          &ap_profile);

  printf(
      "[V2X NET] <<< sl_net_set_profile() returned: "
      "0x%08lX\r\n",
      (unsigned long)status);

  fflush(stdout);


  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] ERROR: sl_net_set_profile failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }


  /***************************************************************************
   * Bring AP up
   **************************************************************************/

  printf(
      "[V2X NET] >>> Calling sl_net_up()...\r\n");

  fflush(stdout);

  status =
      sl_net_up(
          SL_NET_WIFI_AP_INTERFACE,
          SL_NET_DEFAULT_WIFI_AP_PROFILE_ID);

  printf(
      "[V2X NET] <<< sl_net_up() returned: "
      "0x%08lX\r\n",
      (unsigned long)status);

  fflush(stdout);


  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] ERROR: sl_net_up failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }


  /***************************************************************************
   * Read AP MAC address
   **************************************************************************/

  sl_mac_address_t mac_addr;

  memset(
      &mac_addr,
      0,
      sizeof(mac_addr));


  status =
      sl_wifi_get_mac_address(
          SL_WIFI_AP_INTERFACE,
          &mac_addr);

  if (status == SL_STATUS_OK) {

    memcpy(
        s_local_mac,
        mac_addr.octet,
        sizeof(s_local_mac));

  } else {

    printf(
        "[V2X NET] WARNING: MAC read failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);
  }


  /***************************************************************************
   * AP ready
   **************************************************************************/

  printf(
      "[V2X NET] AP Active: "
      "SSID='%s', "
      "IP=192.168.10.1, "
      "MAC=%02X:%02X:%02X:%02X:%02X:%02X\r\n",

      V2X_AP_SSID,

      s_local_mac[0],
      s_local_mac[1],
      s_local_mac[2],
      s_local_mac[3],
      s_local_mac[4],
      s_local_mac[5]);

  fflush(stdout);


  return SL_STATUS_OK;
}


/*******************************************************************************
 * Initialize UDP Socket
 ******************************************************************************/

static bool v2x_socket_init(void)
{
  /***************************************************************************
   * Close previous socket if present
   **************************************************************************/

  if (s_udp_socket >= 0) {

    close(s_udp_socket);

    s_udp_socket = -1;
  }


  printf(
      "[V2X NET] Creating UDP socket...\r\n");

  fflush(stdout);


  /***************************************************************************
   * Create UDP socket
   **************************************************************************/

  s_udp_socket =
      socket(
          AF_INET,
          SOCK_DGRAM,
          IPPROTO_UDP);


  if (s_udp_socket < 0) {

    printf(
        "[V2X NET] ERROR: socket() failed, errno=%d\r\n",
        errno);

    fflush(stdout);

    return false;
  }


  /***************************************************************************
   * Enable broadcast
   **************************************************************************/

  int enable_broadcast = 1;

  int broadcast_rc =
      setsockopt(
          s_udp_socket,
          SOL_SOCKET,
          SO_BROADCAST,
          &enable_broadcast,
          sizeof(enable_broadcast));


  if (broadcast_rc < 0) {

    printf(
        "[V2X NET] WARNING: SO_BROADCAST failed, "
        "errno=%d. Continuing.\r\n",
        errno);

    fflush(stdout);

  } else {

    printf(
        "[V2X NET] SO_BROADCAST enabled.\r\n");

    fflush(stdout);
  }


  /***************************************************************************
   * Bind UDP port 5000
   **************************************************************************/

  struct sockaddr_in bind_addr;

  memset(
      &bind_addr,
      0,
      sizeof(bind_addr));

  bind_addr.sin_family =
      AF_INET;

  bind_addr.sin_port =
      htons(V2X_UDP_PORT);

  bind_addr.sin_addr.s_addr =
      htonl(INADDR_ANY);


  if (bind(
          s_udp_socket,
          (struct sockaddr *)&bind_addr,
          sizeof(bind_addr)) < 0) {

    printf(
        "[V2X NET] ERROR: bind() failed, errno=%d\r\n",
        errno);

    fflush(stdout);

    close(s_udp_socket);

    s_udp_socket = -1;

    return false;
  }


  /***************************************************************************
   * Broadcast destination
   *
   * 255.255.255.255:5000
   **************************************************************************/

  memset(
      &s_broadcast_addr,
      0,
      sizeof(s_broadcast_addr));

  s_broadcast_addr.sin_family =
      AF_INET;

  s_broadcast_addr.sin_port =
      htons(V2X_UDP_PORT);

  s_broadcast_addr.sin_addr.s_addr =
      htonl(INADDR_BROADCAST);


  printf(
      "[V2X NET] UDP socket ready on port %d.\r\n",
      V2X_UDP_PORT);

  fflush(stdout);


  /*
   * IMPORTANT:
   *
   * There is intentionally NO:
   *
   *     #include <fcntl.h>
   *     fcntl()
   *     O_NONBLOCK
   *     SO_RCVTIMEO
   *
   * RX uses a dedicated task because recvfrom() may block indefinitely
   * on this SiWG917 socket implementation.
   */

  return true;
}


/*******************************************************************************
 * Build V2X Frame
 *
 * Accelerometer integration:
 *
 *     ICM40627
 *          |
 *     accelerometer_poll()
 *          |
 *     accelerometer_get_latest()
 *          |
 *     accel_x / accel_y / accel_z
 *          |
 *     V2X frame
 *
 * Values are transmitted in g units.
 ******************************************************************************/

static void v2x_build_frame(
    v2x_frame_t *frame,
    uint16_t seq)
{
  if (frame == NULL) {
    return;
  }


  memset(
      frame,
      0,
      sizeof(v2x_frame_t));


  frame->magic =
      V2X_FRAME_MAGIC;

  frame->version =
      V2X_FRAME_VERSION;

  frame->message_type =
      V2X_MSG_TYPE_PERIODIC;

  frame->payload_length =
      (uint16_t)sizeof(v2x_frame_t);

  frame->sequence =
      seq;


  memcpy(
      frame->sender_id,
      s_local_mac,
      sizeof(s_local_mac));


  frame->timestamp_ms =
      sl_sleeptimer_tick_to_ms(
          sl_sleeptimer_get_tick_count());


  /***************************************************************************
   * Read latest ICM40627 acceleration
   *
   * Units:
   *     X = g
   *     Y = g
   *     Z = g
   **************************************************************************/

  float accel_x = 0.0f;
  float accel_y = 0.0f;
  float accel_z = 0.0f;


  if (accelerometer_get_latest(
          &accel_x,
          &accel_y,
          &accel_z)) {

    frame->accel_x =
        accel_x;

    frame->accel_y =
        accel_y;

    frame->accel_z =
        accel_z;

  } else {

    /*
     * No valid sample available yet.
     */

    frame->accel_x =
        0.0f;

    frame->accel_y =
        0.0f;

    frame->accel_z =
        0.0f;
  }


  /***************************************************************************
   * AI / ML / gesture processing intentionally disabled.
   **************************************************************************/

  frame->gesture =
      NO_GESTURE;

  frame->emergency =
      0;

  frame->reserved =
      0;

  frame->confidence =
      0.0f;


  /***************************************************************************
   * Seal frame and calculate checksum
   **************************************************************************/

  v2x_seal_frame(frame);
}


/*******************************************************************************
 * TX Jitter Generator
 ******************************************************************************/

static uint32_t v2x_get_tx_jitter_ms(void)
{
  uint32_t tick =
      sl_sleeptimer_get_tick_count();


  uint32_t value =
      tick ^
      (tick >> 7) ^
      (tick << 9);


  return
      value %
      (V2X_TX_JITTER_MAX_MS + 1U);
}


/*******************************************************************************
 * Process One Incoming V2X Packet
 ******************************************************************************/

static void v2x_process_incoming_peer_frames(void)
{
  v2x_frame_t rx_frame;

  struct sockaddr_in peer_addr;

  socklen_t peer_addr_len =
      sizeof(peer_addr);


  memset(
      &rx_frame,
      0,
      sizeof(rx_frame));

  memset(
      &peer_addr,
      0,
      sizeof(peer_addr));


  printf(
      "[V2X RX] Waiting for UDP packet...\r\n");

  fflush(stdout);


  /***************************************************************************
   * BLOCKING recvfrom()
   *
   * This function runs only in v2x_rx_task_entry().
   * Therefore TX scheduling cannot be blocked by RX.
   **************************************************************************/

  int received =
      recvfrom(
          s_udp_socket,
          (char *)&rx_frame,
          sizeof(rx_frame),
          0,
          (struct sockaddr *)&peer_addr,
          &peer_addr_len);


  printf(
      "[V2X RX] recvfrom returned: %d bytes\r\n",
      received);

  fflush(stdout);


  /***************************************************************************
   * Receive error
   **************************************************************************/

  if (received < 0) {

    printf(
        "[V2X RX] ERROR: recvfrom failed | errno=%d\r\n",
        errno);

    fflush(stdout);

    osDelay(10);

    return;
  }


  /***************************************************************************
   * Check packet size
   **************************************************************************/

  if (received !=
      (int)sizeof(v2x_frame_t)) {

    printf(
        "[V2X RX] Invalid packet size | "
        "Received=%d | "
        "Expected=%u\r\n",

        received,

        (unsigned)sizeof(v2x_frame_t));

    fflush(stdout);

    return;
  }


  /***************************************************************************
   * Validate V2X packet
   **************************************************************************/

  if (!v2x_validate_frame(
          &rx_frame,
          (size_t)received)) {

    printf(
        "[V2X RX] INVALID V2X FRAME | "
        "Seq=%u | "
        "Magic=0x%08lX | "
        "Version=%u | "
        "Payload=%u | "
        "Checksum=0x%04X\r\n",

        rx_frame.sequence,

        (unsigned long)rx_frame.magic,

        rx_frame.version,

        rx_frame.payload_length,

        rx_frame.checksum);

    fflush(stdout);

    return;
  }


  /***************************************************************************
   * Ignore our own transmitted broadcast
   **************************************************************************/

  if (memcmp(
          rx_frame.sender_id,
          s_local_mac,
          sizeof(s_local_mac)) == 0) {

    printf(
        "[V2X RX] Own packet ignored | "
        "Seq=%u\r\n",
        rx_frame.sequence);

    fflush(stdout);

    return;
  }


  /***************************************************************************
   * Valid peer packet
   **************************************************************************/

  s_peer_connected = true;


  printf(
      "\r\n"
      "========================================================\r\n"
      "[V2X RX] VALID PEER PACKET\r\n"
      "========================================================\r\n"
      "[V2X RX] Bytes      : %d\r\n"
      "[V2X RX] Sequence   : %u\r\n"
      "[V2X RX] Sender MAC : "
      "%02X:%02X:%02X:%02X:%02X:%02X\r\n"
      "[V2X RX] Timestamp  : %lu ms\r\n"
      "[V2X RX] Message    : 0x%02X\r\n"
      "[V2X RX] Accel X    : %.3f g\r\n"
      "[V2X RX] Accel Y    : %.3f g\r\n"
      "[V2X RX] Accel Z    : %.3f g\r\n"
      "[V2X RX] Gesture    : %u\r\n"
      "[V2X RX] Emergency  : %u\r\n"
      "[V2X RX] Confidence : %.3f\r\n"
      "[V2X RX] Checksum   : 0x%04X\r\n"
      "========================================================\r\n",

      received,

      rx_frame.sequence,

      rx_frame.sender_id[0],
      rx_frame.sender_id[1],
      rx_frame.sender_id[2],
      rx_frame.sender_id[3],
      rx_frame.sender_id[4],
      rx_frame.sender_id[5],

      (unsigned long)rx_frame.timestamp_ms,

      rx_frame.message_type,

      rx_frame.accel_x,
      rx_frame.accel_y,
      rx_frame.accel_z,

      rx_frame.gesture,

      rx_frame.emergency,

      rx_frame.confidence,

      rx_frame.checksum);

  fflush(stdout);
}


/*******************************************************************************
 * Dedicated RX Task
 ******************************************************************************/

static void v2x_rx_task_entry(void *argument)
{
  (void)argument;


  printf(
      "\r\n"
      "[V2X RX] ================================================\r\n"
      "[V2X RX] RX TASK STARTED\r\n"
      "[V2X RX] UDP port : %d\r\n"
      "[V2X RX] Buffer   : %u bytes\r\n"
      "[V2X RX] Mode     : blocking recvfrom()\r\n"
      "[V2X RX] TX task  : independent\r\n"
      "[V2X RX] ================================================\r\n",

      V2X_UDP_PORT,

      (unsigned)sizeof(v2x_frame_t));

  fflush(stdout);


  while (1) {

    v2x_process_incoming_peer_frames();
  }
}


/*******************************************************************************
 * Start RX Task
 ******************************************************************************/

static bool v2x_start_rx_task(void)
{
  printf(
      "[V2X NET] Creating RX task...\r\n");

  fflush(stdout);


  osThreadId_t rx_tid =
      osThreadNew(
          (osThreadFunc_t)v2x_rx_task_entry,
          NULL,
          &s_rx_thread_attr);


  printf(
      "[V2X NET] RX task creation returned: %p\r\n",
      (void *)rx_tid);

  fflush(stdout);


  if (rx_tid == NULL) {

    printf(
        "[V2X NET] ERROR: RX task creation FAILED.\r\n");

    fflush(stdout);

    return false;
  }


  printf(
      "[V2X NET] RX task created successfully.\r\n");

  fflush(stdout);


  return true;
}


/*******************************************************************************
 * V2X Network Task
 ******************************************************************************/

static void v2x_net_task_entry(void *argument)
{
  (void)argument;


  uint16_t seq_num = 0;

  v2x_frame_t tx_frame;

  uint32_t next_tx_ms;


  printf(
      "[V2X NET] Network task starting...\r\n");

  fflush(stdout);


  /***************************************************************************
   * Initialize Soft AP
   **************************************************************************/

  if (v2x_wifi_init_ap() != SL_STATUS_OK) {

    printf(
        "[V2X NET] FATAL: Wi-Fi bringup failed.\r\n");

    fflush(stdout);

    osThreadExit();

    return;
  }


  /***************************************************************************
   * Initialize ICM40627 accelerometer
   **************************************************************************/

  printf(
      "\r\n"
      "[IMU] ================================================\r\n"
      "[IMU] Initializing ICM40627...\r\n"
      "[IMU] ================================================\r\n");

  fflush(stdout);


  sl_status_t imu_status =
      accelerometer_setup();


  if (imu_status != SL_STATUS_OK) {

    printf(
        "[IMU] ERROR: accelerometer_setup() failed: "
        "0x%08lX\r\n",
        (unsigned long)imu_status);

    fflush(stdout);


    /*
     * Stop because this version of the application is intended
     * to transmit actual accelerometer data.
     */

    printf(
        "[V2X NET] FATAL: IMU initialization failed.\r\n");

    fflush(stdout);

    osThreadExit();

    return;
  }


  printf(
      "[IMU] ICM40627 initialization complete.\r\n");

  printf(
      "[IMU] Sampling frequency configured: %d Hz\r\n",
      ACCELEROMETER_FREQ);

  fflush(stdout);


  /***************************************************************************
   * Give the sensor a little time before first read.
   **************************************************************************/

  osDelay(20);


  /***************************************************************************
   * Initialize UDP socket
   **************************************************************************/

  if (!v2x_socket_init()) {

    printf(
        "[V2X NET] FATAL: Socket initialization failed.\r\n");

    fflush(stdout);

    osThreadExit();

    return;
  }


  printf(
      "[V2X NET] Network subsystem ready.\r\n");

  printf(
      "[V2X NET] V2X packet period: %d ms\r\n",
      V2X_TX_PERIOD_MS);

  printf(
      "[V2X NET] TX application jitter: 0-%d ms\r\n",
      V2X_TX_JITTER_MAX_MS);

  printf(
      "[V2X NET] RX mode: dedicated blocking RX task\r\n");

  fflush(stdout);


  /***************************************************************************
   * Start RX task
   **************************************************************************/

  if (!v2x_start_rx_task()) {

    printf(
        "[V2X NET] WARNING: RX task unavailable. "
        "TX will continue.\r\n");

    fflush(stdout);
  }


  /***************************************************************************
   * First TX deadline
   **************************************************************************/

  next_tx_ms =
      sl_sleeptimer_tick_to_ms(
          sl_sleeptimer_get_tick_count());

  next_tx_ms += 100;


  printf(
      "[V2X NET] TX scheduler started. "
      "First TX in approximately 100 ms.\r\n");

  fflush(stdout);


  printf(
      "[V2X NET] TX scheduler loop entered.\r\n");

  fflush(stdout);


  /***************************************************************************
   * Main TX scheduler
   **************************************************************************/

  while (1) {

    uint32_t now_ms =
        sl_sleeptimer_tick_to_ms(
            sl_sleeptimer_get_tick_count());


    /*************************************************************************
     * TX deadline reached
     *************************************************************************/

    if ((int32_t)(now_ms - next_tx_ms) >= 0) {

      /***********************************************************************
       * Poll ICM40627
       *
       * This obtains a fresh accelerometer sample from the sensor.
       ***********************************************************************/

      bool imu_sample_ok =
          accelerometer_poll();


      if (!imu_sample_ok) {

        printf(
            "[IMU] WARNING: accelerometer_poll() failed.\r\n");

        fflush(stdout);

      } else {

#if 0
        /*
         * Optional raw sensor diagnostic.
         *
         * Keep disabled during normal operation to avoid excessive UART
         * output.
         */

        float debug_x = 0.0f;
        float debug_y = 0.0f;
        float debug_z = 0.0f;

        if (accelerometer_get_latest(
                &debug_x,
                &debug_y,
                &debug_z)) {

          printf(
              "[IMU] Raw acceleration: "
              "X=%.3f g | Y=%.3f g | Z=%.3f g\r\n",
              debug_x,
              debug_y,
              debug_z);

          fflush(stdout);
        }
#endif
      }


      /***********************************************************************
       * Build V2X packet
       *
       * v2x_build_frame() obtains the latest accelerometer values and
       * places them into accel_x/y/z.
       ***********************************************************************/

      v2x_build_frame(
          &tx_frame,
          seq_num);


      /***********************************************************************
       * Application-level jitter
       ***********************************************************************/

      uint32_t jitter_ms =
          v2x_get_tx_jitter_ms();


      if (jitter_ms > 0) {

        osDelay(jitter_ms);
      }


      /***********************************************************************
       * TX diagnostic
       *
       * Print the actual acceleration values that are going into the
       * transmitted V2X frame.
       ***********************************************************************/

      printf(
          "[V2X TX] Attempting send | "
          "Seq=%u | "
          "Size=%u | "
          "Accel=(%.3f, %.3f, %.3f) g | "
          "Jitter=%lu ms\r\n",

          tx_frame.sequence,

          (unsigned)sizeof(v2x_frame_t),

          tx_frame.accel_x,
          tx_frame.accel_y,
          tx_frame.accel_z,

          (unsigned long)jitter_ms);

      fflush(stdout);


      /***********************************************************************
       * Broadcast packet
       ***********************************************************************/

      int sent =
          sendto(
              s_udp_socket,

              (const char *)&tx_frame,

              sizeof(v2x_frame_t),

              0,

              (struct sockaddr *)&s_broadcast_addr,

              sizeof(s_broadcast_addr));


      /***********************************************************************
       * TX result
       ***********************************************************************/

      if (sent ==
          (int)sizeof(v2x_frame_t)) {

        printf(
            "[V2X TX] OK | "
            "Seq=%u | "
            "Bytes=%d | "
            "Accel=(%.3f, %.3f, %.3f) g | "
            "Peer=%s\r\n",

            tx_frame.sequence,

            sent,

            tx_frame.accel_x,
            tx_frame.accel_y,
            tx_frame.accel_z,

            s_peer_connected
                ? "YES"
                : "NO");

        fflush(stdout);

      } else if (sent < 0) {

        printf(
            "[V2X TX] FAILED | "
            "Seq=%u | "
            "sent=%d | "
            "errno=%d\r\n",

            tx_frame.sequence,

            sent,

            errno);

        fflush(stdout);

      } else {

        printf(
            "[V2X TX] SHORT SEND | "
            "Seq=%u | "
            "sent=%d | "
            "expected=%u\r\n",

            tx_frame.sequence,

            sent,

            (unsigned)sizeof(v2x_frame_t));

        fflush(stdout);
      }


      /***********************************************************************
       * Increment sequence
       ***********************************************************************/

      seq_num++;


      /***********************************************************************
       * Schedule next TX
       ***********************************************************************/

      next_tx_ms =
          sl_sleeptimer_tick_to_ms(
              sl_sleeptimer_get_tick_count());

      next_tx_ms +=
          V2X_TX_PERIOD_MS;


      printf(
          "[V2X NET] Next TX scheduled in %d ms.\r\n",
          V2X_TX_PERIOD_MS);

      fflush(stdout);
    }


    /*************************************************************************
     * Give CPU time to RX and other FreeRTOS tasks.
     *************************************************************************/

    osDelay(1);
  }
}


/*******************************************************************************
 * Application Initialization
 ******************************************************************************/

void app_init(void)
{
  /*
   * Disable stdout buffering so diagnostic messages appear immediately.
   */

  setvbuf(
      stdout,
      NULL,
      _IONBF,
      0);


  printf("\r\n");

  printf(
      "========================================================\r\n");

  printf(
      "             SiWG917 V2X PLATFORM\r\n");

  printf(
      "========================================================\r\n");


  printf(
      "[APP] Creating network task...\r\n");

  fflush(stdout);


  osThreadId_t net_tid =
      osThreadNew(
          (osThreadFunc_t)v2x_net_task_entry,
          NULL,
          &s_net_thread_attr);


  if (net_tid == NULL) {

    printf(
        "[APP] FATAL: Network task creation failed.\r\n");

    fflush(stdout);

    return;
  }


  printf(
      "[APP] Network task created successfully.\r\n");

  fflush(stdout);
}


/*******************************************************************************
 * Application Process Action
 ******************************************************************************/

void app_process_action(void)
{
  /*
   * All V2X networking and accelerometer sampling are handled
   * by the FreeRTOS network task and RX task.
   */
}