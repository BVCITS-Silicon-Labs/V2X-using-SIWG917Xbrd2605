#include "sl_main_init.h"
#include "sl_main_kernel.h"
#include "app.h"

#include <stdio.h>

int main(void)
{
  sl_main_second_stage_init();

  setvbuf(stdout, NULL, _IONBF, 0);

  printf("\r\n");
  printf("========================================================\r\n");
  printf("       SiWG917 V2X BOOT DIAGNOSTICS\r\n");
  printf("========================================================\r\n");

  printf("[BOOT] sl_main_second_stage_init() completed.\r\n");
  fflush(stdout);

  printf("[BOOT] Calling app_init()...\r\n");
  fflush(stdout);

  app_init();

  printf("[BOOT] app_init() completed.\r\n");
  fflush(stdout);

  printf("[BOOT] Entering main task loop...\r\n");
  fflush(stdout);

  while (sl_main_start_task_should_continue()) {
    app_process_action();
  }

  return 0;
}