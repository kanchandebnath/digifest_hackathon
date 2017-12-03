/*
 * Copyright (c) 2016, Zolertia - http://www.zolertia.com
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
 * This file is part of the Contiki operating system.
 *
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup zoul-sensors
 * @{
 *
 * \defgroup zoul-ac-dimmer AC light dimmer with zero-crossing driver
 *
 * Driver for an AC light dimmer with zero-crossing driver
 * @{
 *
 * \file
 * Header file for an AC light dimmer with zero-crossing driver
 */
/*---------------------------------------------------------------------------*/
#ifndef AC_DIMMER_H_
#define AC_DIMMER_H_
/* -------------------------------------------------------------------------- */
/**
 * \name AC dimmer default pins, ports and interrupt vector
 * @{
 */
#ifdef DIMMER_SYNC_CONF_PIN
#define DIMMER_SYNC_PIN        DIMMER_SYNC_CONF_PIN
#else
#define DIMMER_SYNC_PIN        3
#endif
#ifdef DIMMER_SYNC_CONF_PORT
#define DIMMER_SYNC_PORT       DIMMER_SYNC_CONF_PORT
#else
#define DIMMER_SYNC_PORT       GPIO_B_NUM
#endif
#ifdef DIMMER_GATE_CONF_PIN
#define DIMMER_GATE_PIN        DIMMER_GATE_CONF_PIN
#else
#define DIMMER_GATE_PIN        5  //LED5 connected to this 
#endif
#ifdef DIMMER_GATE_CONF_PORT
#define DIMMER_GATE_PORT       DIMMER_GATE_CONF_PORT
#else
#define DIMMER_GATE_PORT       GPIO_B_NUM
#endif
#ifdef DIMMER_CONF_INT_VECTOR
#define DIMMER_INT_VECTOR      DIMMER_CONF_INT_VECTOR
#else
#define DIMMER_INT_VECTOR      GPIO_B_IRQn
#endif
/** @} */
/* -------------------------------------------------------------------------- */
/**
 * \name AC dimmer values
 * @{
 */
#define DIMMER_DEFAULT_START_VALUE     10
#define DIMMER_DEFAULT_GATE_PULSE_US   50
#define DIMMER_DEFAULT_MIN_DIMM_VALUE  10
#define DIMMER_DEFAULT_MAX_DIMM_VALUE  95


/** @} */
/* -------------------------------------------------------------------------- */
/**
 * \name AC dimmer return types
 * @{
 */
#define DIMMER_ERROR             (-1)
#define DIMMER_SUCCESS           0x00
/** @} */
/* -------------------------------------------------------------------------- */
#define AC_DIMMER_ACTUATOR "AC light dimmer zero-cross"
/* -------------------------------------------------------------------------- */
extern const struct sensors_sensor ac_dimmer;
/* -------------------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */
#endif /* RELAY_H_ */
/* -------------------------------------------------------------------------- */
/**
 * @}
 * @}
 */