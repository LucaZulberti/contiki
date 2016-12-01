/*
 * Copyright (c) 2016, Bas Stottelaar <basstottelaar@gmail.com>
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
 * This library is adapted by the original from the RIOT OS project in
 * "riot/drivers/tsl256x".
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup wi502-hce-sensors
 * @{
 *
 * \defgroup wi502-si70xx-sensor SI70XX Humidity and Temperature sensor
 *
 * Driver for the SI70XX Humidity and Temperature sensor
 *
 * The SI70XX driver returns the humidity value in percent and the temperature
 * value in Celsius degree
 * @{
 *
 * \file
 * Header file for the external SI70XX Humidity and Temperature sensor driver
 *
 * \author
 *         Luca Zulberti <luca.zulberti@cosino.io>
 */
/*---------------------------------------------------------------------------*/
#ifndef SI70XX_H
#define SI70XX_H
/*---------------------------------------------------------------------------*/
#include "dev/wi502-sensors.h"
/*---------------------------------------------------------------------------*/
/**
 * \name SI70XX specific model information
 * @{
 */
#ifndef SI70XX_CONF_ADDR
#define SI70XX_ADDR                   0x80
#else
#define SI70XX_ADDR                   SI70XX_CONF_ADDR
#endif
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * @name SI70XX device commands
 * @{
 */
#define SI70XX_MEASURE_RH_HOLD        0xE5
#define SI70XX_MEASURE_RH             0xF5
#define SI70XX_MEASURE_TEMP_HOLD      0xE3
#define SI70XX_MEASURE_TEMP           0xF3
#define SI70XX_MEASURE_TEMP_PREV      0xE0
#define SI70XX_RESET                  0xFE
#define SI70XX_WRITE_USER_REG         0xE6
#define SI70XX_READ_USER_REG          0xE7
#define SI70XX_WRITE_HEATER_REG       0x51
#define SI70XX_READ_HEATER_REG        0x11
#define SI70XX_READ_ID_FIRST_A        0xFA
#define SI70XX_READ_ID_FIRST_B        0x0F
#define SI70XX_READ_ID_SECOND_A       0xFC
#define SI70XX_READ_ID_SECOND_B       0xC9
#define SI70XX_READ_REVISION_A        0x84
#define SI70XX_READ_REVISION_B        0xB8
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * /name SI70XX register values
 * @{
 */
#define SI70XX_ID_SI7006              0x06
#define SI70XX_ID_SI7013              0x0D
#define SI70XX_ID_SI7020              0x14
#define SI70XX_ID_SI7021              0x15

#define SI70XX_REVISION_1             0xFF
#define SI70XX_REVISION_2             0x20
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SI70XX return and command values
 * @{
 */
#define SI70XX_SUCCESS         0x00
#define SI70XX_TEMP            0x01
#define SI70XX_HUMI            0x02
#define SI70XX_ERROR             -1

#define SI70XX_ACTIVE          SENSORS_ACTIVE
#define SI70XX_HW_INIT         SENSORS_HW_INIT

#define SI70XX_VAL_READ        0
/** @} */
/*---------------------------------------------------------------------------*/
#define SI70XX_TEMP_SENSOR "SI70XX Temperature Sensor"
#define SI70XX_HUMI_SENSOR "SI70XX Humidity Sensor"
/*---------------------------------------------------------------------------*/
#endif /* SI70XX_H */
/*---------------------------------------------------------------------------*/
/**
 * @}
 * @}
 */
