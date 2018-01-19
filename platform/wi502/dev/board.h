/*
 * Copyright (c) 2012, Texas Instruments Incorporated - http://www.ti.com/
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
 * \addtogroup wi502
 * @{
 *
 * \defgroup wi502-hce WI502 Peripherals
 *
 * Defines related to the WI502 board by HCE Engineering
 *
 * This file provides connectivity information on LEDs, Buttons, UART and
 * other HCE peripherals
 *
 * This file can be used as the basis to configure other platforms using the
 * cc2538 SoC.
 * @{
 *
 * \file
 * Header file with definitions related to the I/O connections on the HCE
 * WI502 board
 *
 * \note   Do not include this file directly. It gets included by contiki-conf
 *         after all relevant directives have been set.
 */
#ifndef BOARD_H_
#define BOARD_H_

#include "dev/gpio.h"
#include "dev/nvic.h"
/*---------------------------------------------------------------------------*/
/** \name LED configuration
 *
 * There is no LEDs on WI502 board
 * @{
 */

/* Notify various examples that we don't have LEDs */
#define PLATFORM_HAS_LEDS       0
/** @} */
/*---------------------------------------------------------------------------*/
/** \name USB configuration
 *
 * The USB pullup is driven by PC0
 * {@
 */
#define USB_PULLUP_PORT         GPIO_C_NUM
#define USB_PULLUP_PIN          0
/** @} */
/*---------------------------------------------------------------------------*/
/** \name UART configuration
 *
 * On the WI502, the UART (XDS back channel) is connected to the
 * following ports/pins
 * - RX:  PA0/PA2
 * - TX:  PA1/PA3
 * - RTS: PD0 (Can only be used with UART1)
 * - CTS: PD1 (Can only be used with UART1)
 *
 * We configure the port to use UART0. To use UART1, replace UART0_* with
 * UART1_* below.
 * @{
 */
#define UART0_RX_PORT           GPIO_A_NUM
#define UART0_RX_PIN            0

#define UART0_TX_PORT           GPIO_A_NUM
#define UART0_TX_PIN            1

#define UART1_RX_PORT           GPIO_A_NUM
#define UART1_RX_PIN            2

#define UART1_TX_PORT           GPIO_A_NUM
#define UART1_TX_PIN            3

#define UART1_RTS_PORT          GPIO_D_NUM
#define UART1_RTS_PIN           0

#define UART1_CTS_PORT          GPIO_D_NUM
#define UART1_CTS_PIN           1
/** @} */
/*---------------------------------------------------------------------------*/
/** \name BUTTON configuration
 *
 * There is no Buttons on WI502 board
 * {@
 */
/* Notify various examples that we don't have Buttons */
#define PLATFORM_HAS_BUTTON     1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name ADC configuration
 *
 * These values configure which CC2538 pins and ADC channels to use for the ADC
 * inputs.
 *
 * ADC inputs can only be on port A.
 * @{
 */
#define ADC_BAT_PORT            GPIO_A_NUM /**< ALS power GPIO control port */
#define ADC_BAT_PIN             6 /**< battery voltage ANA pin */

#define ADC_BAT_IN_SCALE_FACTOR_CENT 5450

#ifndef USER_BAT_THR_CONF
#define BAT_THR_CHARGED         4.1
#define BAT_THR_HIGH            3.9
#define BAT_THR_MIDDLE          3.7
#define BAT_THR_LOW             3.5
#define BAT_THR_CRIT            3.3
#endif

#define ADC_BAT_THR_CHARGED     (BAT_THR_CHARGED * 2048 * 10000) / (119 * ADC_BAT_IN_SCALE_FACTOR_CENT)
#define ADC_BAT_THR_HIGH        (BAT_THR_HIGH * 2048 * 10000) / (119 * ADC_BAT_IN_SCALE_FACTOR_CENT)
#define ADC_BAT_THR_MIDDLE      (BAT_THR_MIDDLE * 2048 * 10000) / (119 * ADC_BAT_IN_SCALE_FACTOR_CENT)
#define ADC_BAT_THR_LOW         (BAT_THR_LOW * 2048 * 10000) / (119 * ADC_BAT_IN_SCALE_FACTOR_CENT)
#define ADC_BAT_THR_CRIT        (BAT_THR_CRIT * 2048 * 10000) / (119 * ADC_BAT_IN_SCALE_FACTOR_CENT)

/* Notify various examples that we have Battery */
#define PLATFORM_HAS_BATTERY    1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Light Sensor configuration
 *
 * These values configure which CC2538 pins to use for the TSL256X Light Sensor
 * device, i2c address and part number
 */
/** TSL256X interrupt vector */
#define TSL256X_INT_VECTOR                GPIO_B_IRQn
/** TSL256X interrupt port */
#define TSL256X_INT_PORT                  GPIO_B_NUM
/** TSL256X interrupt pin */
#define TSL256X_INT_PIN                   0
/** TSL256X i2c address */
#define TSL256X_CONF_ADDR                 0x39
/** TSL256X expected part number */
#define TSL256X_CONF_EXPECTED_PARTNO      0x50

/* Notify various examples that we have Light sensor */
#define PLATFORM_HAS_LIGHT    1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Temperature and Humidity Sensor configuration
 *
 * These values configure which CC2538 i2c address to use for the TSL256X Light
 * Sensor device
 */
/** SI70XX i2c address */
#define SI70XX_CONF_ADDR                 0x40

/* Notify various examples that we have Temperature and Humidity sensors */
#define PLATFORM_HAS_TEMPERATURE    1
#define PLATFORM_HAS_HUMIDITY       1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Accelerometer Sensor configuration
 *
 * These values configure which CC2538 pins to use for the MMA8X5X Accelerometer
 * Sensor device and i2c address
 */
/** MMA8X5X interrupt vector */
#define MMA8X5X_INT_VECTOR                GPIO_B_IRQn
/** MMA8X5X interrupt port */
#define MMA8X5X_INT_PORT                  GPIO_B_NUM
/** MMA8X5X interrupt pin */
#define MMA8X5X_INT_PIN                   3
/** MMA8X5X i2c address */
#define MMA8X5X_CONF_ADDR                 0x1C
/** MMA8X5X data rate */
#define MMA8X5X_CONF_DATARATE             6     /** 6.25 Hz */
/** MMA8X5X full scale range */
#define MMA8X5X_CONF_FS_RANGE             0     /** 2G */

/* Notify various examples that we have Accelerometer sensor */
#define PLATFORM_HAS_ACCELEROMETER    1
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name SPI configuration
 *
 * These values configure which CC2538 pins to use for the SPI lines. Both
 * SPI instances can be used independently by providing the corresponding
 * port / pin macros.
 * @{
 */
#define SPI0_IN_USE             1
#define SPI1_IN_USE             0
#if SPI0_IN_USE
/** Clock port SPI0 */
#define SPI0_CLK_PORT           GPIO_D_NUM
/** Clock pin SPI0 */
#define SPI0_CLK_PIN            4
/** TX port SPI0 (master mode: MOSI) */
#define SPI0_TX_PORT            GPIO_D_NUM
/** TX pin SPI0 */
#define SPI0_TX_PIN             2
/** RX port SPI0 (master mode: MISO */
#define SPI0_RX_PORT            GPIO_D_NUM
/** RX pin SPI0 */
#define SPI0_RX_PIN             3
#endif /* #if SPI0_IN_USE */
#if SPI1_IN_USE
/** Clock port SPI1 */
#define SPI1_CLK_PORT           GPIO_A_NUM
/** Clock pin SPI1 */
#define SPI1_CLK_PIN            2
/** TX port SPI1 (master mode: MOSI) */
#define SPI1_TX_PORT            GPIO_A_NUM
/** TX pin SPI1 */
#define SPI1_TX_PIN             4
/** RX port SPI1 (master mode: MISO) */
#define SPI1_RX_PORT            GPIO_A_NUM
/** RX pin SPI1 */
#define SPI1_RX_PIN             5
#endif /* #if SPI1_IN_USE */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name I2C configuration
 *
 * These values configure which CC2538 pins to use for the I2C lines.
 * @{
 */
#define I2C_IN_USE             1
#if I2C_IN_USE
/** Data port I2C */
#define I2C_SDA_PORT           GPIO_B_NUM
/** Data pin I2C */
#define I2C_SDA_PIN            1
/** Clock port I2C */
#define I2C_SCL_PORT           GPIO_B_NUM
/** Clock pin I2C */
#define I2C_SCL_PIN            2
#endif /* #if I2C_IN_USE */
/** @} */
/*---------------------------------------------------------------------------*/
/**
 * \name Device string used on startup
 * @{
 */
#define BOARD_STRING "HCE WI502 IoT Module, cc2538-powered board"
/** @} */

#endif /* BOARD_H_ */

/**
 * @}
 * @}
 */
