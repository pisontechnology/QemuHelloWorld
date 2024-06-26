/*
 * Copyright (c) 2018 Arm Limited
 *
 * Licensed under the Apache License Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing software
 * distributed under the License is distributed on an "AS IS" BASIS
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef __ARM_LTD_DEVICE_CFG_H__
#define __ARM_LTD_DEVICE_CFG_H__

/**
 * \file device_cfg.h
 * \brief Configuration file native driver re-targeting
 *
 * \details This file can be used to add native driver specific macro
 *          definitions to select which peripherals are available in the build.
 *
 * This is a default device configuration file with all peripherals enabled.
 */

/* ARM SCC */
#define MUSCA_A1_SCC_S
#define MUSCA_A1_SCC_NS

/* ARM GPIO */
#define GPIO0_CMSDK_S
#define GPIO0_CMSDK_NS

/* ARM Memory Protection Controller (MPC) SIE 200 */
#define MPC_ISRAM0_S
#define MPC_ISRAM1_S
#define MPC_ISRAM2_S
#define MPC_ISRAM3_S
#define MPC_CODE_SRAM_S
#define MPC_CODE_SRAM_NS
#define MPC_QSPI_S
#define MPC_QSPI_NS

/* ARM Peripheral Protection Controllers (PPC) */
#define AHB_PPC0_S
#define AHB_PPCEXP0_S
#define AHB_PPCEXP1_S
#define AHB_PPCEXP2_S
#define AHB_PPCEXP3_S
#define APB_PPC0_S
#define APB_PPC1_S
#define APB_PPCEXP0_S
#define APB_PPCEXP1_S
#define APB_PPCEXP2_S
#define APB_PPCEXP3_S

/* ARM Message Handler Units (MHU) */
#define ARM_MHU0_S
#define ARM_MHU0_NS
#define ARM_MHU1_S
#define ARM_MHU1_NS

/* ARM UART Controller PL011 */
#define UART0_PL011_S
#define UART0_PL011_NS
#define UART1_PL011_S
#define UART1_PL011_NS

/* APB Dualtimers */
#define CMSDK_DUALTIMER_S
#define CMSDK_DUALTIMER_NS

/* CMSDK Timers */
#define CMSDK_TIMER0_S
#define CMSDK_TIMER0_NS
#define CMSDK_TIMER1_S
#define CMSDK_TIMER1_NS

/* General-Purpose Timers */
#define GP_TIMER_S
#define GP_TIMER_NS

/* Cadence QSPI Flash Controller */
#define QSPI_IP6514E_S
#define QSPI_IP6514E_NS

/* MT25QL Flash memory library */
#define MT25QL_S
#define MT25QL_NS

/* Cadence SPI IP6524 */
#define SPI0_IP6524_S
#define SPI0_IP6524_NS

#endif  /* __ARM_LTD_DEVICE_CFG_H__ */
