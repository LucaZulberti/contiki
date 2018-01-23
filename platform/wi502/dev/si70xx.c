/*
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
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup wi502-si70xx-sensor
 * @{
 *
 * \file
 *  Driver for the external SI70XX Humidity and Temperature sensor
 *
 * \author
 *         Luca Zulberti <luca.zulberti@cosino.io>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/gpio.h"
#include "dev/i2c.h"
#include "lib/sensors.h"

#include "dev/si70xx.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(fmt, args...) printf(fmt, ## args)
#define INFO(fmt, args...) PRINTF("[INFO - SI70XX] %s: %s: " fmt "\n", \
                                        __FILE__, __func__ , ## args)
#define ERROR(fmt, args...) PRINTF("[ERROR - SI70XX] %s: %s: " fmt "\n", \
                                        __FILE__, __func__ , ## args)
#else
#define PRINTF(fmt, args...)
#define INFO(fmt, args...)
#define ERROR(fmt, args...)
#endif
/*---------------------------------------------------------------------------*/
static uint8_t enabled, powered_on;
/*---------------------------------------------------------------------------*/
static int
si70xx_write_reg(uint8_t reg, uint8_t data)
{
  uint8_t buf[2] = {reg, data};

  if(i2c_burst_send(SI70XX_ADDR, buf, 2) == I2C_MASTER_ERR_NONE) {
    return SI70XX_SUCCESS;
  }
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_read_reg(uint8_t reg, uint8_t *data, uint8_t num)
{
  if(i2c_single_send(SI70XX_ADDR, reg) == I2C_MASTER_ERR_NONE) {
    if(i2c_burst_receive(SI70XX_ADDR, data, num) == I2C_MASTER_ERR_NONE) {
      return SI70XX_SUCCESS;
    }
  }
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_write_byte(uint8_t *buf)
{
  int ret;
  if(buf == NULL) {
    ERROR("invalid write value");
    return SI70XX_ERROR;
  }

  ret = i2c_single_send(SI70XX_ADDR, *buf);
  if(ret == I2C_MASTER_ERR_NONE) {
    return SI70XX_SUCCESS;
  }
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_write_bytes(uint8_t *buf, uint8_t num)
{
  int ret;

  if(buf == NULL) {
    ERROR("invalid write values");
    return SI70XX_ERROR;
  }

  ret = i2c_burst_send(SI70XX_ADDR, buf, num);
  if(ret == I2C_MASTER_ERR_NONE) {
    return SI70XX_SUCCESS;
  }
  ERROR("burst_send return value (0x%x)", ret);
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_read_byte(uint8_t *buf)
{
  int ret;
  if(buf == NULL) {
    ERROR("invalid read value");
    return SI70XX_ERROR;
  }

  ret = i2c_single_receive(SI70XX_ADDR, buf);
  if(ret == I2C_MASTER_ERR_NONE)
    return SI70XX_SUCCESS;

  ERROR("single_receive return value (0x%x)", ret);
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_read_bytes(uint8_t *buf, uint8_t num)
{
  int ret;
  if((buf == NULL) || (num <= 0)) {
    ERROR("invalid read values");
    return SI70XX_ERROR;
  }

  ret = i2c_burst_receive(SI70XX_ADDR, buf, num);
  if(ret == I2C_MASTER_ERR_NONE) {
    return SI70XX_SUCCESS;
  }
  ERROR("burst_receive return value (0x%x)", ret);
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_get_id(uint8_t *id)
{
  uint8_t out[2];
  uint8_t in_first[8] = { 0 };
  uint8_t in_second[8] = { 0 };
  uint32_t id_first, id_second;
  uint64_t serial_id;

  /* read the lower bytes */
  out[0] = SI70XX_READ_ID_FIRST_A;
  out[1] = SI70XX_READ_ID_FIRST_B;
  if(si70xx_write_bytes(out, 2) != SI70XX_SUCCESS)
    return SI70XX_ERROR;
  if(si70xx_read_bytes(in_first, 8) != SI70XX_SUCCESS)
    return SI70XX_ERROR;

  /* read the higher bytes */
  out[0] = SI70XX_READ_ID_SECOND_A;
  out[1] = SI70XX_READ_ID_SECOND_B;
  if(si70xx_write_bytes(out, 2) != SI70XX_SUCCESS)
    return SI70XX_ERROR;
  if(si70xx_read_bytes(in_second, 8) != SI70XX_SUCCESS)
    return SI70XX_ERROR;

  /* calculate the ID */
  id_first = ((uint32_t)in_first[0] << 24) + ((uint32_t)in_first[2] << 16) +
                (in_first[4] << 8) + (in_first[6] << 0);
  id_second = ((uint32_t)in_second[0] << 24) + ((uint32_t)in_second[2] << 16) +
                (in_second[4] << 8) + (in_second[6] << 0);
  serial_id = (((uint64_t) id_first) << 32) + id_second;

  *id = (uint8_t)((serial_id >> 24) & 0xff);

  return SI70XX_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_get_revision(uint8_t *buf)
{
  uint8_t out[2];

  /* read the revision number */
  out[0] = SI70XX_READ_REVISION_A;
  out[1] = SI70XX_READ_REVISION_B;
  if(si70xx_write_bytes(out, 2) != SI70XX_SUCCESS)
    return SI70XX_ERROR;
  if(si70xx_read_byte(buf) != SI70XX_SUCCESS)
    return SI70XX_ERROR;

  return SI70XX_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_test(void)
{
  uint8_t id = 0, rev = 0;

  if(si70xx_get_revision(&rev) != SI70XX_SUCCESS) {
    ERROR("cannot get rev number");
    return SI70XX_ERROR;
  }

  if(rev != SI70XX_REVISION_1 && rev != SI70XX_REVISION_2) {
    ERROR("bad rev number (0x%x)", rev);
    return SI70XX_ERROR;
  }

  if(si70xx_get_id(&id) != SI70XX_SUCCESS)
    return SI70XX_ERROR;

  if(id != SI70XX_ID_SI7006 && id != SI70XX_ID_SI7013 &&
    id != SI70XX_ID_SI7020 && id != SI70XX_ID_SI7021) {
    ERROR("bad id (0x%x)", id);
    return SI70XX_ERROR;
  }

  INFO("test passed");

  return SI70XX_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_on(void)
{
  struct timer reset_timer;
  uint8_t cmd = SI70XX_RESET;

  if (powered_on)
    return SI70XX_SUCCESS;

  if(si70xx_write_byte(&cmd) == SI70XX_SUCCESS) {
    /* sensor is ready after at most 25 ms */
    timer_set(&reset_timer, CLOCK_SECOND / 40);
    while (!timer_expired(&reset_timer)) {}
    powered_on = 1;
    INFO("powered on");
    return SI70XX_SUCCESS;
  }

  ERROR("failed to power on (I2C write error)");
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
si70xx_off(void)
{
  /* TODO */
  powered_on = 0;
  INFO("powered off");
  return SI70XX_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  if(type == SI70XX_ACTIVE) {
    if(value) {
      if (enabled)
        return SI70XX_SUCCESS;
      /* Power on the sensor and check for the part number */
      if(si70xx_on() != SI70XX_SUCCESS)
        return SI70XX_ERROR;
      if(si70xx_test() != SI70XX_SUCCESS)
        return SI70XX_ERROR;
      enabled = 1;
      /* Configure max resolution */
      if(si70xx_write_reg(SI70XX_WRITE_USER_REG, 0x3a) != SI70XX_SUCCESS)
        return SI70XX_ERROR;
      INFO("resolution setted");
      return SI70XX_SUCCESS;
    } else {
      if (!enabled)
        return SI70XX_SUCCESS;
      if(si70xx_off() != SI70XX_SUCCESS)
        return SI70XX_ERROR;

      INFO("stopped");
      enabled = 0;
      return SI70XX_SUCCESS;
    }
  }

  if(type == SI70XX_HW_INIT) {
    return SI70XX_SUCCESS;
  }

  if(!enabled) {
    ERROR("sensor not started");
    return SI70XX_ERROR;
  }

  /* if(type) {TODO}*/

  return SI70XX_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
status(int type)
{
  switch(type) {
  case SENSORS_ACTIVE:
  case SENSORS_READY:
    return enabled;
  }
  return 0;
}
/*---------------------------------------------------------------------------*/
static int
temp_value(int type)
{
  uint8_t buf[2];
  int raw, temp;

  if(!enabled) {
    ERROR("sensor not started");
    return SI70XX_ERROR;
  }

  if(type == SI70XX_VAL_READ) {
    if(si70xx_read_reg(SI70XX_MEASURE_TEMP_HOLD, buf, 2) != SI70XX_ERROR) {
      raw = ((int)buf[0] << 8) + (buf[1] & 0xfc);
      temp = ((17572 * raw) / 65536) - 4685;
      return temp;
    }
    ERROR("failed to read temperature");
  }
  return SI70XX_ERROR;
}

/*---------------------------------------------------------------------------*/
static int
humi_value(int type)
{
  uint8_t buf[2];
  int raw, humidity;

  if(!enabled) {
    ERROR("sensor not started");
    return SI70XX_ERROR;
  }

  if(type == SI70XX_VAL_READ) {
    if(si70xx_read_reg(SI70XX_MEASURE_RH_HOLD, buf, 2) != SI70XX_ERROR) {
      raw = ((int)buf[0] << 8) + (buf[1] & 0xf0);
      humidity = ((12500 * raw) / 65536) - 600;
      if (humidity < 0) {
        return 0;
      } else if (humidity > 10000) {
        return 10000;
      } else {
        return humidity;
      }
    }
    ERROR("failed to read humidity");
  }
  return SI70XX_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(temperature_sensor, SI70XX_TEMP_SENSOR,
        temp_value, configure, status);
SENSORS_SENSOR(humidity_sensor, SI70XX_HUMI_SENSOR,
        humi_value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
