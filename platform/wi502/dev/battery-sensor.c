/*
 * Copyright (c) 2013, ADVANSEE - http://www.advansee.com/
 * Benoît Thébaudeau <benoit.thebaudeau@advansee.com>
 * Copyright (c) 2016, Luca Zulberti <luca.zulberti@cosino.io>
 * Copyright (c) 2016, HCE Engineering <info@hce-engineering.com>
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/**
 * \addtogroup wi502-hce-sensors
 * @{
 *
 * \defgroup wi502-battery-sensor WI502 battery sensor
 *
 * \file
 *  Driver for the WI502 battery sensor
 */
#include "contiki.h"
#include "sys/clock.h"
#include "dev/ioc.h"
#include "dev/gpio.h"
#include "dev/adc.h"
#include "dev/battery-sensor.h"

#include <stdint.h>
#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define ADC_BAT_PORT_BASE    GPIO_PORT_TO_BASE(ADC_BAT_PORT)
#define ADC_BAT_PIN_MASK     GPIO_PIN_MASK(ADC_BAT_PIN)
/*---------------------------------------------------------------------------*/
static int
value(int type)
{
  uint8_t channel = SOC_ADC_ADCCON_CH_AIN0 + ADC_BAT_PIN;
  int16_t res;

  res = adc_get(channel, SOC_ADC_ADCCON_REF_INT, SOC_ADC_ADCCON_DIV_512);

  res = res >> 4;

  printf("---- ADC VALUE = %d ----\n", res);

  if (res > ADC_BAT_THR_CHARGED)
    res = 100;
  else if (res > ADC_BAT_THR_HIGH)
    res = 75 + 25 * (res - ADC_BAT_THR_HIGH)
	    / (ADC_BAT_THR_CHARGED - ADC_BAT_THR_HIGH);
  else if (res > ADC_BAT_THR_MIDDLE)
    res = 50 + 25 * (res - ADC_BAT_THR_MIDDLE)
	    / (ADC_BAT_THR_HIGH - ADC_BAT_THR_MIDDLE);
  else if (res > ADC_BAT_THR_LOW)
    res = 25 + 25 * (res - ADC_BAT_THR_LOW)
	    / (ADC_BAT_THR_MIDDLE - ADC_BAT_THR_LOW);
  else if (res > ADC_BAT_THR_CRIT)
    res = 5 + 20 * (res - ADC_BAT_THR_CRIT)
	    / (ADC_BAT_THR_LOW - ADC_BAT_THR_CRIT);
  else
    res = 0 + 5 * res / ADC_BAT_THR_CRIT;

  return res;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  switch(type) {
  case SENSORS_HW_INIT:
    GPIO_SOFTWARE_CONTROL(ADC_BAT_PORT_BASE, ADC_BAT_PIN_MASK);
    GPIO_SET_INPUT(ADC_BAT_PORT_BASE, ADC_BAT_PIN_MASK);
    ioc_set_over(ADC_BAT_PORT, ADC_BAT_PIN, IOC_OVERRIDE_ANA);

    break;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  return 1;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(battery_sensor, BATTERY_SENSOR, value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
