/*
* The MIT License (MIT)
*
* Copyright (c) 2020 Marco Russi
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in all
* copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/


#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include "FreeRTOS.h"
#include "task.h"
#include "app_error.h"
#include "nordic_common.h"
#include "nrf_drv_clock.h"
#include "sdk_errors.h"
#include "nrf_drv_gpiote.h"
#include "net.h"
#include "tim.h"
#include "launch.h"


#define TEST_LED_PIN_NUM              16


typedef enum
{
  LED_ID,
  NET_ID,
  ETH_ID,
  NUM_OF_MODULES
} modules_id_e;


/* Structure to store task info for each module */
typedef struct
{
  modules_id_e module_id;
  char *prefix;
  UBaseType_t priority;
  uint32_t period_ms;
  uint32_t stack_size;
  TaskHandle_t thread_id;
} task_info_st;


static task_info_st tasks_info[NUM_OF_MODULES] =
{
  {LED_ID,  "led",  3,  500,  256,  NULL},
  {NET_ID,  "net",  3,  100,  512,  NULL},
};


static bool createTask( uint8_t id, TaskFunction_t fn );
static void ledTask( void * arg );
static void ledTimCb( void );


void LAUNCH_go( void )
{
  APP_ERROR_CHECK(nrf_drv_clock_init());

  createTask(LED_ID, ledTask);
  createTask(NET_ID, NET_task);

  /* ATTENTION: this function should never return! */
  vTaskStartScheduler();
}


static bool createTask( uint8_t id, TaskFunction_t fn )
{
  bool ret = false;
  TaskHandle_t thread_id = NULL;
  BaseType_t xReturned;
  char buff[10];
  task_info_st *task_info = &tasks_info[id];

  sprintf(buff, "%s_task", task_info->prefix);

  xReturned = xTaskCreate((TaskFunction_t)fn,
                          (const portCHAR *)buff,
                          task_info->stack_size,
                          (void *)task_info->period_ms,
                          task_info->priority,
                          &thread_id);

  if (pdPASS == xReturned)
  {
    task_info->thread_id = thread_id;

    ret = true;
  }

  return ret;
}


static void ledTask( void * arg )
{
  uint32_t period_ms = (uint32_t)arg;
  nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

  if (true != nrf_drv_gpiote_is_init())
  {
    APP_ERROR_CHECK(nrf_drv_gpiote_init());
  }
  APP_ERROR_CHECK(nrf_drv_gpiote_out_init(TEST_LED_PIN_NUM, &out_config));
  nrfx_gpiote_out_set(TEST_LED_PIN_NUM);

  (void)TIM_init(TIM_1, 150, ledTimCb);

  while (true)
  {
    nrfx_gpiote_out_clear(TEST_LED_PIN_NUM);
    (void)TIM_start(TIM_1);

    vTaskDelay(pdMS_TO_TICKS(period_ms));
  }

  vTaskDelete(NULL);
}


static void ledTimCb( void )
{
  nrfx_gpiote_out_set(TEST_LED_PIN_NUM);
}

