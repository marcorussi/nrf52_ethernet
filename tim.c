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
#include "timers.h"
#include "tim.h"


typedef struct
{
  TimerHandle_t handle;
  char name[5];
  uint16_t ms;
  TIM_callback_fn cb;
} timer_info_st;


static timer_info_st timers[TIM_NUM_OF_TIMERS] =
{
  {
    NULL,
    "TIM1",
    0,
    NULL
  },
  {
    NULL,
    "TIM2",
    0,
    NULL
  },
  {
    NULL,
    "TIM3",
    0,
    NULL
  }
};


static void timer_callback (TimerHandle_t tim);


bool TIM_init( uint8_t id, uint16_t ms, TIM_callback_fn cb )
{
  bool ret = false;
  TimerHandle_t handle;

  if (id < TIM_NUM_OF_TIMERS)
  {
    handle = xTimerCreate((const char *)timers[id].name,
                          (TickType_t)ms,
                          pdFALSE,
                          (void *)id,
                          timer_callback);

    if (NULL != handle)
    {
      timers[id].handle = handle;
      timers[id].ms = ms;
      timers[id].cb = cb;

      ret = true;
    }
  }

  return ret;
}


bool TIM_start( uint8_t id )
{
  bool ret = false;

  if (id < TIM_NUM_OF_TIMERS)
  {
    if (pdPASS == xTimerStart(timers[id].handle, 0))
    {
      ret = true;
    }
  }

  return ret;
}


static void timer_callback (TimerHandle_t tim)
{
  uint8_t id = (uint8_t)pvTimerGetTimerID( tim );

  if ((id < TIM_NUM_OF_TIMERS)
  &&  (NULL != timers[id].cb))
  {
    timers[id].cb();
  }
}



