/***************************************************************************//**
 * @file app.c
 * @brief SiWG917 V2X Station TX/RX with ICM40627 accelerometer
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
 * V2X CONFIGURATION
 ******************************************************************************/

#define V2X_STA_SSID                 "V2X_AP_NODE"
#define V2X_STA_PASSWORD             "12345678"

#define V2X_UDP_PORT                 5000

#define V2X_TX_PERIOD_MS             100
#define V2X_TX_JITTER_MAX_MS         20

#define V2X_BROADCAST_ADDR \
  ((uint32_t)((192U << 24) | \
              (168U << 16) | \
              (10U  << 8)  | \
              255U))


/*******************************************************************************
 * THREAD ATTRIBUTES
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
 * GLOBAL STATE
 ******************************************************************************/

static volatile bool s_peer_connected = false;

static uint8_t s_local_mac[6] = { 0 };

static int s_udp_socket = -1;

static struct sockaddr_in s_broadcast_addr;


/*******************************************************************************
 * WIFI CLIENT DEVICE CONFIGURATION
 ******************************************************************************/

static const sl_wifi_device_configuration_t s_wifi_client_device_config = {

  .boot_option = LOAD_NWP_FW,

  .mac_address = NULL,

  .band = SL_SI91X_WIFI_BAND_2_4GHZ,

  .region_code = US,

  .boot_config = {

    .oper_mode = SL_SI91X_CLIENT_MODE,

    .coex_mode = SL_SI91X_WLAN_ONLY_MODE,

    .feature_bit_map =
      (SL_SI91X_FEAT_SECURITY_PSK |
       SL_SI91X_FEAT_AGGREGATION),

    .tcp_ip_feature_bit_map =
      (SL_SI91X_TCP_IP_FEAT_DHCPV4_CLIENT |
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
 * WIFI CLIENT PROFILE
 ******************************************************************************/

static const sl_net_wifi_client_profile_t s_wifi_client_profile = {

  .config = {

    .ssid = {
      .value = V2X_STA_SSID,
      .length = sizeof(V2X_STA_SSID) - 1
    },

    .channel = {
      .channel = SL_WIFI_AUTO_CHANNEL,
      .band = SL_WIFI_BAND_2_4GHZ,
      .bandwidth = SL_WIFI_AUTO_BANDWIDTH
    },

    .channel_bitmap = {
      .channel_bitmap_2_4 = 0,
      .channel_bitmap_5 = 0
    },

    .bssid = {{ 0 }},

    .bss_type =
      SL_WIFI_BSS_TYPE_INFRASTRUCTURE,

    .security =
      SL_WIFI_WPA2,

    .encryption =
      SL_WIFI_CCMP_ENCRYPTION,

    .client_options = 0,

    .credential_id =
      SL_NET_DEFAULT_WIFI_CLIENT_CREDENTIAL_ID
  },

  .ip = {

    .mode =
      SL_IP_MANAGEMENT_DHCP,

    .type =
      SL_IPV4,

    .host_name = NULL,

    .ip = {{{ 0 }}}
  },

  .priority = 0
};


/*******************************************************************************
 * STACK OVERFLOW HOOK
 ******************************************************************************/

void vApplicationStackOverflowHook(
    TaskHandle_t xTask,
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
 * WIFI CONNECTED CALLBACK
 ******************************************************************************/

static sl_status_t sta_connected_callback(
    sl_wifi_event_t event,
    sl_status_t status_code,
    void *data,
    uint32_t data_length,
    void *arg)
{
  (void)data;
  (void)data_length;
  (void)arg;

  if (SL_WIFI_CHECK_IF_EVENT_FAILED(event)) {
    return status_code;
  }

  s_peer_connected = true;

  printf(
      "[V2X NET] Wi-Fi CONNECTED to '%s'\r\n",
      V2X_STA_SSID);

  fflush(stdout);

  return SL_STATUS_OK;
}


/*******************************************************************************
 * WIFI DISCONNECTED CALLBACK
 ******************************************************************************/

static sl_status_t sta_disconnected_callback(
    sl_wifi_event_t event,
    sl_status_t status_code,
    void *data,
    uint32_t data_length,
    void *arg)
{
  (void)data;
  (void)data_length;
  (void)arg;

  s_peer_connected = false;

  printf(
      "[V2X NET] Wi-Fi DISCONNECTED from '%s'\r\n",
      V2X_STA_SSID);

  printf(
      "[V2X NET] Disconnect status: 0x%08lX\r\n",
      (unsigned long)status_code);

  fflush(stdout);

  return SL_STATUS_OK;
}


/*******************************************************************************
 * WIFI STATION INITIALIZATION
 ******************************************************************************/

static sl_status_t v2x_wifi_init_station(void)
{
  sl_status_t status;


  printf(
      "[V2X NET] Initializing Wi-Fi Station...\r\n");

  fflush(stdout);


  /***************************************************************************
   * Initialize client interface
   **************************************************************************/

  status =
      sl_net_init(
          SL_NET_WIFI_CLIENT_INTERFACE,
          &s_wifi_client_device_config,
          NULL,
          NULL);

  if (status != SL_STATUS_OK &&
      status != SL_STATUS_ALREADY_INITIALIZED) {

    printf(
        "[V2X NET] ERROR: sl_net_init(client) failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }


  printf(
      "[V2X NET] Wi-Fi client interface initialized.\r\n");

  fflush(stdout);


  /***************************************************************************
   * Connected callback
   **************************************************************************/

  status =
      sl_wifi_set_callback_v2(
          SL_WIFI_CLIENT_CONNECTED_EVENTS,
          sta_connected_callback,
          NULL);

  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] WARNING: connected callback failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);
  }


  /***************************************************************************
   * Disconnected callback
   **************************************************************************/

  status =
      sl_wifi_set_callback_v2(
          SL_WIFI_CLIENT_DISCONNECTED_EVENTS,
          sta_disconnected_callback,
          NULL);

  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] WARNING: disconnected callback failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);
  }


  /***************************************************************************
   * Configure PSK
   **************************************************************************/

  status =
      sl_net_set_credential(
          SL_NET_DEFAULT_WIFI_CLIENT_CREDENTIAL_ID,
          SL_NET_WIFI_PSK,
          V2X_STA_PASSWORD,
          strlen(V2X_STA_PASSWORD));

  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] ERROR: credential setup failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }


  printf(
      "[V2X NET] Wi-Fi credential configured.\r\n");

  fflush(stdout);


  /***************************************************************************
   * Profile
   **************************************************************************/

  printf(
      "[V2X NET] Setting client profile...\r\n");

  printf(
      "[V2X NET] SSID       : %s\r\n",
      V2X_STA_SSID);

  printf(
      "[V2X NET] Security   : WPA2\r\n");

  printf(
      "[V2X NET] Encryption : CCMP\r\n");

  printf(
      "[V2X NET] Band       : 2.4 GHz\r\n");

  printf(
      "[V2X NET] Channel    : AUTO\r\n");

  printf(
      "[V2X NET] IP mode    : DHCP\r\n");

  fflush(stdout);


  /***************************************************************************
   * Set profile
   **************************************************************************/

  status =
      sl_net_set_profile(
          SL_NET_WIFI_CLIENT_INTERFACE,
          SL_NET_DEFAULT_WIFI_CLIENT_PROFILE_ID,
          &s_wifi_client_profile);

  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] ERROR: client profile failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }


  printf(
      "[V2X NET] Client profile configured successfully.\r\n");

  fflush(stdout);


  /***************************************************************************
   * Bring Station UP
   **************************************************************************/

  printf(
      "[V2X NET] Scanning 2.4 GHz channels for AP '%s'...\r\n",
      V2X_STA_SSID);

  fflush(stdout);


  status =
      sl_net_up(
          SL_NET_WIFI_CLIENT_INTERFACE,
          SL_NET_DEFAULT_WIFI_CLIENT_PROFILE_ID);


  printf(
      "[V2X NET] sl_net_up() returned: 0x%08lX\r\n",
      (unsigned long)status);

  fflush(stdout);


  if (status != SL_STATUS_OK) {

    printf(
        "[V2X NET] ERROR: Station could not connect.\r\n");

    printf(
        "[V2X NET] SSID     : %s\r\n",
        V2X_STA_SSID);

    printf(
        "[V2X NET] Band     : 2.4 GHz\r\n");

    printf(
        "[V2X NET] Channel  : AUTO\r\n");

    printf(
        "[V2X NET] Security : WPA2-PSK\r\n");

    printf(
        "[V2X NET] Cipher   : CCMP\r\n");

    printf(
        "[V2X NET] Status   : 0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);

    return status;
  }


  printf(
      "[V2X NET] Station interface is UP.\r\n");

  fflush(stdout);


  /***************************************************************************
   * Read Station MAC
   **************************************************************************/

  sl_mac_address_t mac_addr;

  memset(
      &mac_addr,
      0,
      sizeof(mac_addr));


  status =
      sl_wifi_get_mac_address(
          SL_WIFI_CLIENT_INTERFACE,
          &mac_addr);

  if (status == SL_STATUS_OK) {

    memcpy(
        s_local_mac,
        mac_addr.octet,
        sizeof(s_local_mac));

    printf(
        "[V2X NET] Station MAC = "
        "%02X:%02X:%02X:%02X:%02X:%02X\r\n",

        s_local_mac[0],
        s_local_mac[1],
        s_local_mac[2],
        s_local_mac[3],
        s_local_mac[4],
        s_local_mac[5]);

    fflush(stdout);

  } else {

    printf(
        "[V2X NET] WARNING: MAC read failed: "
        "0x%08lX\r\n",
        (unsigned long)status);

    fflush(stdout);
  }


  return SL_STATUS_OK;
}


/*******************************************************************************
 * UDP SOCKET INITIALIZATION
 ******************************************************************************/

static bool v2x_socket_init(void)
{
  if (s_udp_socket >= 0) {

    close(s_udp_socket);

    s_udp_socket = -1;
  }


  printf(
      "[V2X NET] Creating UDP socket...\r\n");

  fflush(stdout);


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
      htonl(V2X_BROADCAST_ADDR);


  printf(
      "[V2X NET] UDP socket bound to port %d.\r\n",
      V2X_UDP_PORT);

  printf(
      "[V2X NET] Broadcast destination: "
      "192.168.10.255:%d\r\n",
      V2X_UDP_PORT);

  fflush(stdout);


  return true;
}


/*******************************************************************************
 * BUILD V2X FRAME
 *
 * Board B now inserts its own ICM40627 X/Y/Z values into the V2X frame.
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

    frame->accel_x = 0.0f;
    frame->accel_y = 0.0f;
    frame->accel_z = 0.0f;
  }


  /***************************************************************************
   * No AI / ML / gesture processing
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
   * Calculate checksum
   **************************************************************************/

  v2x_seal_frame(frame);
}


/*******************************************************************************
 * TX JITTER
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
 * RX PACKET PROCESSING
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


  if (received < 0) {

    printf(
        "[V2X RX] ERROR: recvfrom failed | errno=%d\r\n",
        errno);

    fflush(stdout);

    osDelay(10);

    return;
  }


  if (received !=
      (int)sizeof(v2x_frame_t)) {

    printf(
        "[V2X RX] Invalid packet size | "
        "Received=%d | Expected=%u\r\n",

        received,

        (unsigned)sizeof(v2x_frame_t));

    fflush(stdout);

    return;
  }


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
   * Ignore own packets
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


  s_peer_connected = true;


  /***************************************************************************
   * Valid peer packet
   **************************************************************************/

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
 * RX TASK
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
 * START RX TASK
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
        "[V2X NET] ERROR: RX task creation failed.\r\n");

    fflush(stdout);

    return false;
  }


  printf(
      "[V2X NET] RX task created successfully.\r\n");

  fflush(stdout);

  return true;
}


/*******************************************************************************
 * NETWORK TASK
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
   * Initialize Station
   **************************************************************************/

  sl_status_t wifi_status =
      v2x_wifi_init_station();

  if (wifi_status != SL_STATUS_OK) {

    printf(
        "[V2X NET] FATAL: Station initialization failed.\r\n");

    printf(
        "[V2X NET] Status = 0x%08lX\r\n",
        (unsigned long)wifi_status);

    fflush(stdout);

    osThreadExit();

    return;
  }


  /***************************************************************************
   * Initialize ICM40627
   **************************************************************************/

  printf(
      "\r\n"
      "[IMU] ================================================\r\n"
      "[IMU] Initializing ICM40627 on Board B...\r\n"
      "[IMU] ================================================\r\n");

  fflush(stdout);


  sl_status_t imu_status =
      accelerometer_setup();


  if (imu_status != SL_STATUS_OK) {

    printf(
        "[IMU] ERROR: accelerometer_setup() failed: "
        "0x%08lX\r\n",
        (unsigned long)imu_status);

    printf(
        "[V2X NET] FATAL: Board B IMU initialization failed.\r\n");

    fflush(stdout);

    osThreadExit();

    return;
  }


  printf(
      "[IMU] ICM40627 ready on Board B.\r\n");

  printf(
      "[IMU] Sampling frequency configured: %d Hz\r\n",
      ACCELEROMETER_FREQ);

  fflush(stdout);


  /*
   * Give sensor a little time before first sample.
   */
  osDelay(20);


  /***************************************************************************
   * Initialize UDP
   **************************************************************************/

  if (!v2x_socket_init()) {

    printf(
        "[V2X NET] FATAL: UDP socket initialization failed.\r\n");

    fflush(stdout);

    osThreadExit();

    return;
  }


  printf(
      "[V2X NET] Network subsystem ready.\r\n");

  fflush(stdout);


  /***************************************************************************
   * Start RX task
   **************************************************************************/

  if (!v2x_start_rx_task()) {

    printf(
        "[V2X NET] WARNING: RX task unavailable.\r\n");

    fflush(stdout);
  }


  /***************************************************************************
   * TX scheduler
   **************************************************************************/

  next_tx_ms =
      sl_sleeptimer_tick_to_ms(
          sl_sleeptimer_get_tick_count());

  next_tx_ms += 10;


  printf(
      "[V2X NET] TX scheduler started.\r\n");

  printf(
      "[V2X NET] TX period : %d ms\r\n",
      V2X_TX_PERIOD_MS);

  printf(
      "[V2X NET] TX jitter : 0-%d ms\r\n",
      V2X_TX_JITTER_MAX_MS);

  printf(
      "[V2X NET] Accelerometer: ENABLED\r\n");

  fflush(stdout);


  /***************************************************************************
   * TX loop
   **************************************************************************/

  while (1) {

    uint32_t now_ms =
        sl_sleeptimer_tick_to_ms(
            sl_sleeptimer_get_tick_count());


    if ((int32_t)(now_ms - next_tx_ms) >= 0) {

      /***********************************************************************
       * Poll ICM40627
       ***********************************************************************/

      bool imu_sample_ok =
          accelerometer_poll();


      if (!imu_sample_ok) {

        printf(
            "[IMU] WARNING: accelerometer_poll() failed\r\n");

        fflush(stdout);
      }


      /***********************************************************************
       * Build V2X frame using latest accelerometer data
       ***********************************************************************/

      v2x_build_frame(
          &tx_frame,
          seq_num);


      /***********************************************************************
       * TX jitter
       ***********************************************************************/

      uint32_t jitter_ms =
          v2x_get_tx_jitter_ms();


      if (jitter_ms > 0) {
        osDelay(jitter_ms);
      }


      /***********************************************************************
       * Send V2X packet
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
            "errno=%d\r\n",

            tx_frame.sequence,

            errno);

        fflush(stdout);

      } else {

        printf(
            "[V2X TX] SHORT SEND | "
            "Seq=%u | "
            "Bytes=%d | "
            "Expected=%u\r\n",

            tx_frame.sequence,

            sent,

            (unsigned)sizeof(v2x_frame_t));

        fflush(stdout);
      }


      /***********************************************************************
       * Next sequence
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
    }


    osDelay(1);
  }
}


/*******************************************************************************
 * APPLICATION INIT
 ******************************************************************************/

void app_init(void)
{
  setvbuf(
      stdout,
      NULL,
      _IONBF,
      0);


  printf("\r\n");

  printf(
      "========================================================\r\n");

  printf(
      "       SiWG917 V2X STATION NODE + ICM40627\r\n");

  printf(
      "========================================================\r\n");


  printf(
      "[APP] Creating V2X network task...\r\n");

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
 * APPLICATION PROCESS ACTION
 ******************************************************************************/

void app_process_action(void)
{
  /*
   * V2X networking and accelerometer sampling are handled
   * by the FreeRTOS network task.
   */
}
