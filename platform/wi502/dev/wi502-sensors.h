/*
 * Copyright (c) 2015, Zolertia - http://www.zolertia.com
 * Copyright (c) 2015, University of Bristol - http://www.bristol.ac.uk
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
 * \addtogroup wi502-hce
 * @{
 *
 * \defgroup wi502-hce-sensors WI502 Sensors
 *
 * WI502 on-board sensors configuration and costants
 * @{
 *
 * \file
 * Configuration of WI502 on-board sensors
 */
/*---------------------------------------------------------------------------*/
#ifndef WI502_SENSORS_H
#define WI502_SENSORS_H
/*---------------------------------------------------------------------------*/
#include "dev/board-sensors.h"
/*---------------------------------------------------------------------------*/
/**
 * \name WI502 sensor constants
 *
 * These constants are used by various sensors on the WI502. They can be used
 * to configure ADC decimation rate (where applicable), enable interrupts, etc.
 * @{
 */
#define HW_INT_OVER_THRS                              0x01
#define HW_INT_BELOW_THRS                             0x02
#define HW_INT_DISABLE                                0x03
#define WI502_SENSORS_CONFIGURE_TYPE_DECIMATION_RATE  0x0100
#define WI502_SENSORS_ERROR                           CC2538_SENSORS_ERROR
/** @} */
/*---------------------------------------------------------------------------*/
#endif /* WI502_SENSORS_H */
/* -------------------------------------------------------------------------- */
/**
 * @}
 * @}
 */
