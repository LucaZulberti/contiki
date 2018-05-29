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
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup wi502-hce-sensors
 * @{
 *
 * WI502 on-board sensors exports
 * @{
 *
 * \file
 * Exporting structures of WI502 on-board sensors
 */
/*---------------------------------------------------------------------------*/
#ifndef BOARD_SENSORS_H
#define BOARD_SENSORS_H
/*---------------------------------------------------------------------------*/
/**
 * \name On-Board sensors structures
 *
 * Sensors supported by Contiki OS in /core/dev folder.
 * @{
 */
#include "dev/button-sensor.h"
#include "dev/battery-sensor.h"
#include "dev/cc2538-sensors.h"
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name On-Board sensors structures
 *
 * Sensors on WI502 board defined in /platform/wi502/dev folder.
 * @{
 */
/** TSL256X light_sensor sensor structure */
extern const struct sensors_sensor light_sensor;
/** SI70XX temperature and humidity sensor structures */
extern const struct sensors_sensor temperature_sensor;
extern const struct sensors_sensor humidity_sensor;
/** MMA8X5X accelerometer sensor structure */
extern const struct sensors_sensor accelerometer_sensor;
/** @} */
/*---------------------------------------------------------------------------*/
#endif /* BOARD_SENSORS_H */
/* -------------------------------------------------------------------------- */
/**
 * @}
 * @}
 */
