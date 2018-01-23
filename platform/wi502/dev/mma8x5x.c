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
 * \addtogroup wi502-mma8x5x-sensor
 * @{
 *
 * \file
 *  Driver for the external MMA8X5X Accelerometer sensor
 *
 * \author
 *         Luca Zulberti <luca.zulberti@cosino.io>
 */
/*---------------------------------------------------------------------------*/
#include "contiki.h"
#include "dev/gpio.h"
#include "dev/i2c.h"
#include "lib/sensors.h"

#include "dev/mma8x5x.h"

#include <stdio.h>
/*---------------------------------------------------------------------------*/
#define DEBUG 1
#if DEBUG
#define PRINTF(fmt, args...) printf(fmt, ## args)
#define INFO(fmt, args...) PRINTF("[INFO - MMA8X5X] %s: %s: " fmt "\n", \
                                        __FILE__, __func__ , ## args)
#define ERROR(fmt, args...) PRINTF("[ERROR - MMA8X5X] %s: %s: " fmt "\n", \
                                        __FILE__, __func__ , ## args)
#else
#define PRINTF(fmt, args...)
#define INFO(fmt, args...)
#define ERROR(fmt, args...)
#endif
/*---------------------------------------------------------------------------*/
static int scale, enabled, *acc, *off;
/*---------------------------------------------------------------------------*/
static int
mma8x5x_write_reg(uint8_t reg, uint8_t *data, uint8_t num)
{
  uint8_t buf[10];
  int i;

  buf[0] = reg;
  for(i = 0; i < num; i++)
    buf[i + 1] = data[i];

  if(i2c_burst_send(MMA8X5X_ADDR, buf, num + 1) == I2C_MASTER_ERR_NONE) {
    return MMA8X5X_SUCCESS;
  }
  return MMA8X5X_ERROR;
}
/*---------------------------------------------------------------------------*/
static void
dump_data(uint8_t *data, uint8_t num) {
  char dbg[50];
  int i, ret = 0;

  for (i = 0; i < num; i++) {
    ret += sprintf(&dbg[ret], "%x ", data[i]);
  }
  dbg[ret-1] = 0x0;
  INFO("Bytes read: %s", dbg);
}
/*---------------------------------------------------------------------------*/
static int
mma8x5x_read_reg(uint8_t reg, uint8_t *data, uint8_t num)
{
  uint8_t temp;

  i2c_master_set_slave_address(MMA8X5X_ADDR, I2C_SEND);
  i2c_master_data_put(reg);
  i2c_master_command(I2C_MASTER_CMD_BURST_SEND_START);

  while(i2c_master_busy());

  i2c_master_set_slave_address(MMA8X5X_ADDR, I2C_RECEIVE);
  if(num == 1) {
    i2c_master_command(I2C_MASTER_CMD_SINGLE_RECEIVE);
    while(i2c_master_busy());
    temp = i2c_master_error();
    if(temp == I2C_MASTER_ERR_NONE) {
      *data = i2c_master_data_get();
      dump_data(data, 1);
      return MMA8X5X_SUCCESS;
    }
    return MMA8X5X_ERROR;
  } else {
    i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_START);
  }

  while(i2c_master_busy());

  if(i2c_master_error() == I2C_MASTER_ERR_NONE) {
    data[0] = i2c_master_data_get();
    /* If we got 2 or more bytes pending to be received, keep going*/
    for(temp = 1; temp <= (num - 2); temp++) {
      i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_CONT);
      while(i2c_master_busy());
      data[temp] = i2c_master_data_get();
    }
    /* This should be the last byte, stop receiving */
    i2c_master_command(I2C_MASTER_CMD_BURST_RECEIVE_FINISH);
    while(i2c_master_busy());
    data[num - 1] = i2c_master_data_get();
    dump_data(data, num);
    return MMA8X5X_SUCCESS;
  }
  return MMA8X5X_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
mma8x5x_test(void)
{
  uint8_t reg;
  int i;

  if(mma8x5x_read_reg(MMA8X5X_WHO_AM_I, &reg, 1) != MMA8X5X_SUCCESS) {
    ERROR("I2C write error");
	return MMA8X5X_ERROR;
  }

  for(i = 0; i < sizeof(mma8x5x_device_id); i++) {
    if(reg == mma8x5x_device_id[i]) {
      INFO("test passed");
      return MMA8X5X_SUCCESS;
    }
  }

  ERROR("test failed");
  return MMA8X5X_ERROR;
}
/*---------------------------------------------------------------------------*/
static int
mma8x5x_on(void)
{
  uint8_t reg;

  if(mma8x5x_read_reg(MMA8X5X_CTRL_REG1, &reg, 1) != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  reg |= MMA8X5X_CTRL_REG1_ACTIVE;

  if(mma8x5x_write_reg(MMA8X5X_CTRL_REG1, &reg, 1) != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  INFO("activated");
  return MMA8X5X_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
mma8x5x_set_standby(void)
{
  uint8_t reg;

  if(mma8x5x_read_reg(MMA8X5X_CTRL_REG1, &reg, 1) != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  reg &= ~MMA8X5X_CTRL_REG1_ACTIVE;

  if(mma8x5x_write_reg(MMA8X5X_CTRL_REG1, &reg, 1) != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  INFO("entered standby");
  return MMA8X5X_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
mma8x5x_set_user_offset(int8_t x, int8_t y, int8_t z)
{
  uint8_t buf[3];

  buf[0] = (uint8_t)x;
  buf[1] = (uint8_t)y;
  buf[2] = (uint8_t)z;

  if(mma8x5x_write_reg(MMA8X5X_OFF_X, buf, 3) != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  INFO("user offset set");
  return MMA8X5X_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
mma8x5x_init(uint8_t dr, uint8_t range)
{
  uint8_t reg;

  if (dr > MMA8X5X_DATARATE_1HZ56 || range > MMA8X5X_FS_RANGE_8G) {
    return MMA8X5X_ERROR;
  }

  if (mma8x5x_set_standby() != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  reg = MMA8X5X_XYZ_DATA_CFG_FS(range);

  if(mma8x5x_write_reg(MMA8X5X_XYZ_DATA_CFG, &reg, 1) != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  switch(range) {
    case 0:
      scale = 1;
      break;
    case 1:
      scale = 2;
      break;
    case 2:
      scale = 3;
      break;
  };

  reg = MMA8X5X_CTRL_REG1_DR(dr);

  if(mma8x5x_write_reg(MMA8X5X_CTRL_REG1, &reg, 1) != MMA8X5X_SUCCESS)
    return MMA8X5X_ERROR;

  INFO("init completed");
  return MMA8X5X_SUCCESS;
}
/*---------------------------------------------------------------------------*/
static int
configure(int type, int value)
{
  if(type == MMA8X5X_ACTIVE) {
    if(value) {
      if (enabled)
        return MMA8X5X_SUCCESS;
      /* Check for the part number and power on the sensor */
      if(mma8x5x_test() != MMA8X5X_SUCCESS)
        return MMA8X5X_ERROR;
      if(mma8x5x_init(MMA8X5X_DATARATE, MMA8X5X_FS_RANGE) != MMA8X5X_SUCCESS)
        return MMA8X5X_ERROR;
      if(mma8x5x_on() != MMA8X5X_SUCCESS)
        return MMA8X5X_ERROR;
      enabled = 1;
      INFO("sensor started\n");
      return MMA8X5X_SUCCESS;
    } else {
      if (!enabled)
        return MMA8X5X_SUCCESS;
      if(mma8x5x_set_standby() != MMA8X5X_SUCCESS)
        return MMA8X5X_ERROR;
      INFO("sensor stopped\n");
      enabled = 0;
      return MMA8X5X_SUCCESS;
    }
  }

  if(type == MMA8X5X_HW_INIT) {
    return MMA8X5X_SUCCESS;
  }

  if(!enabled) {
    INFO("sensor not started\n");
    return MMA8X5X_ERROR;
  }

  if(type == MMA8X5X_OFF_WRITE && off != NULL) {
    if(mma8x5x_set_user_offset(off[0], off[1], off[2]) != MMA8X5X_SUCCESS)
      return MMA8X5X_ERROR;
  }
  if(type == MMA8X5X_OFF_POINTER) {
    off = (int *)value;
  }

  if(type == MMA8X5X_VAL_POINTER) {
    acc = (int *)value;
  }

  return MMA8X5X_SUCCESS;
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
value(int type)
{
  uint8_t buf[6];
  uint32_t tmp;

  if(!enabled) {
    INFO("sensor not started");
    //return MMA8X5X_ERROR;
  }

  if(type == MMA8X5X_VAL_READ && acc != NULL) {
    if(mma8x5x_read_reg(MMA8X5X_OUT_X_MSB, buf, 6) != MMA8X5X_ERROR) {
      tmp = ((int32_t)(((uint16_t)buf[0] << 8) | buf[1]) >> 4);
      tmp = (tmp & 0x800) ? (tmp | 0xFFFFF000) : (tmp & 0xFFF);
      acc[0] = tmp * scale;
      tmp = ((int32_t)(((uint16_t)buf[2] << 8) | buf[3]) >> 4);
      tmp = (tmp & 0x800) ? (tmp | 0xFFFFF000) : (tmp & 0xFFF);
      acc[1] = tmp * scale;
      tmp = ((int32_t)(((uint16_t)buf[4] << 8) | buf[5]) >> 4);
      tmp = (tmp & 0x800) ? (tmp | 0xFFFFF000) : (tmp & 0xFFF);
      acc[2] = tmp * scale;
      return MMA8X5X_SUCCESS;
    }
    ERROR("fail to read acceleration");
  }

  return MMA8X5X_ERROR;
}
/*---------------------------------------------------------------------------*/
SENSORS_SENSOR(accelerometer_sensor, MMA8X5X_SENSOR, value, configure, status);
/*---------------------------------------------------------------------------*/
/** @} */
