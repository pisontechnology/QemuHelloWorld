/*
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/**
 * \file spi_ip6524_drv.h
 * \brief Generic driver for SPI IP6524.
 */

#ifndef __SPI_IP6524_DRV_H__
#define __SPI_IP6524_DRV_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

#define BITMASK(width) ((1u<<(width))-1)

/* --- Interrupt Defines --- */

/** RX FIFO Overflow (current FIFO status) */
#define SPI_IP6524_RX_OVERFLOW_POS           0ul
#define SPI_IP6524_RX_OVERFLOW_BIT_WIDTH     1ul
#define SPI_IP6524_RX_OVERFLOW_MSK     \
    (BITMASK(SPI_IP6524_RX_OVERFLOW_BIT_WIDTH)<<SPI_IP6524_RX_OVERFLOW_POS)

/** Mode Fail */
#define SPI_IP6524_MODE_FAIL_POS             1ul
#define SPI_IP6524_MODE_FAIL_BIT_WIDTH       1ul
#define SPI_IP6524_MODE_FAIL_MSK       \
    (BITMASK(SPI_IP6524_MODE_FAIL_BIT_WIDTH)<<SPI_IP6524_MODE_FAIL_POS)

/** TX FIFO Not Full (current FIFO status) */
#define SPI_IP6524_TX_NOT_FULL_POS           2ul
#define SPI_IP6524_TX_NOT_FULL_BIT_WIDTH     1ul
#define SPI_IP6524_TX_NOT_FULL_MSK     \
    (BITMASK(SPI_IP6524_TX_NOT_FULL_BIT_WIDTH)<<SPI_IP6524_TX_NOT_FULL_POS)

/** TX FIFO Full (current FIFO status) */
#define SPI_IP6524_TX_FULL_POS               3ul
#define SPI_IP6524_TX_FULL_BIT_WIDTH         1ul
#define SPI_IP6524_TX_FULL_MSK         \
    (BITMASK(SPI_IP6524_TX_FULL_BIT_WIDTH)<<SPI_IP6524_TX_FULL_POS)

/** RX FIFO Not Empty (current FIFO status) */
#define SPI_IP6524_RX_NOT_EMPTY_POS          4ul
#define SPI_IP6524_RX_NOT_EMPTY_BIT_WIDTH    1ul
#define SPI_IP6524_RX_NOT_EMPTY_MSK    \
    (BITMASK(SPI_IP6524_RX_NOT_EMPTY_BIT_WIDTH)<<SPI_IP6524_RX_NOT_EMPTY_POS)

/** RX FIFO Full (current FIFO status) */
#define SPI_IP6524_RX_FULL_POS               5ul
#define SPI_IP6524_RX_FULL_BIT_WIDTH         1ul
#define SPI_IP6524_RX_FULL_MSK         \
    (BITMASK(SPI_IP6524_RX_FULL_BIT_WIDTH)<<SPI_IP6524_RX_FULL_POS)

/** TX FIFO Underflow (current FIFO status) */
#define SPI_IP6524_TX_UNDERFLOW_POS          6ul
#define SPI_IP6524_TX_UNDERFLOW_BIT_WIDTH    1ul
#define SPI_IP6524_TX_UNDERFLOW_MSK    \
    (BITMASK(SPI_IP6524_TX_UNDERFLOW_BIT_WIDTH)<<SPI_IP6524_TX_UNDERFLOW_POS)

/** Enum type for Mode selection */
enum spi_ip6524_mode_select_t {
    SPI_IP6524_SLAVE_SELECT  = 0,
    SPI_IP6524_MASTER_SELECT = 1
};

/**
 * Enum type for Clock Polarity selection
 *      - bit 0 - The SPI clock is quiescent low
 *      - bit 1 - The SPI clock is quiescent high
 */
enum spi_ip6524_clock_polarity_t {
    SPI_IP6524_SPI_CLK_LOW  = 0,
    SPI_IP6524_SPI_CLK_HIGH = 1
};

/** Enum type for Clock Phase selection
 *       - bit 0 - The SPI clk is active outside the word
 *       - bit 1 - The SPI clk is inactive outside the word
 */
enum spi_ip6524_clock_phase_t {
    SPI_IP6524_SPI_ACTIVE_OUTSIDE   = 0,
    SPI_IP6524_SPI_INACTIVE_OUTSIDE = 1
};

/** Enum type for Master Baud Rate Divisor selection */
enum spi_ip6524_master_baud_rate_divisor_t {
    SPI_IP6524_BAUD_RATE_DIVISOR_2   = 0,
    SPI_IP6524_BAUD_RATE_DIVISOR_4   = 1,
    SPI_IP6524_BAUD_RATE_DIVISOR_8   = 2,
    SPI_IP6524_BAUD_RATE_DIVISOR_16  = 3,
    SPI_IP6524_BAUD_RATE_DIVISOR_32  = 4,
    SPI_IP6524_BAUD_RATE_DIVISOR_64  = 5,
    SPI_IP6524_BAUD_RATE_DIVISOR_128 = 6,
    SPI_IP6524_BAUD_RATE_DIVISOR_256 = 7
};

/** Enum type for reference Clock selection */
enum spi_ip6524_reference_clock_select_t {
    SPI_IP6524_SPI_REF_CLK_SELECT = 0,
    SPI_IP6524_EXT_CLK_SELECT     = 1
};

/**
 * Enum type for Peripheral decode selection
 *      - bit 0 - Only 1 bit in n_ss_out[3:0] is active
 *      - bit 1 - 4-to-16 decode (n_ss_out [3:0 = PCSL [3:0])
 */
enum spi_ip6524_peripheral_select_decode_t {
    SPI_IP6524_1_TO_4_DECODE  = 0,
    SPI_IP6524_4_TO_16_DECODE = 1
};

/**
 *   Enum type for Peripheral chip select lines
 *   When Peripheral Select Decode is set then PCSL[3:0] directly drives
 *   n_ss_out[3:0], else (PSD is written with ‘0’) PCSL[3:0] drives
 *   n_ss_out[3:0] as follow:
 *                            PCSL [3:0]   n_ss_out [3:0]
 *                              xxx0         1110
 *                              xx01         1101
 *                              x011         1011
 *                              0111         0111
 *                              1111         1111 (no peripheral selected)
 */
enum spi_ip6524_peripheral_chip_select_lines_t {
    SPI_IP6524_PCSL_0         = 14,
    SPI_IP6524_PCSL_1         = 13,
    SPI_IP6524_PCSL_2         = 11,
    SPI_IP6524_PCSL_3         = 7,
    SPI_IP6524_PCSL_NO_PERIPH = 15
};

/** Enum type for SPI error code returns */
enum spi_ip6524_error_t {
    SPI_IP6524_ERR_NONE         = 0,  /*!< No error */
    SPI_IP6524_ERR_INVALID_ARGS = 1,  /*!< Invalid input arguments */
    SPI_IP6524_ERR_NOT_INIT     = 2,  /*!< SPI driver is not initialized */
    SPI_IP6524_ERR_NO_TX        = 3,  /*!< SPI TX FIFO full */
    SPI_IP6524_ERR_NO_RX        = 4,  /*!< SPI RX FIFO empty */
    SPI_IP6524_ERR_BAD_CONFIG   = 5,  /*!< Bad SPI configuration */
};

/** SPI IP6524 device configuration structure */
struct spi_ip6524_dev_cfg_t {
    const uint32_t base;    /*!< SPI IP6524 base address */
};

/** SPI IP6524 device data structure */
struct spi_ip6524_dev_data_t {
    uint32_t state;    /*!< Indicates if the SPI driver
                               is initialized and enabled */
};

/** SPI IP6524 device structure */
struct spi_ip6524_dev_t {
    /*!< SPI driver configuration */
    const struct spi_ip6524_dev_cfg_t* const cfg;
    /*!< SPI driver data */
    struct spi_ip6524_dev_data_t* const data;
};

/**
 * \brief Enables SPI IP6524, when this bit is set the SPI controller is
 * enabled, otherwise SPI is disabled. When SPI controller is disabled all
 * output enables are inactive and all peripheral pins are set to input mode.
 * Writing ‘0’ disables the SPI controller once current transfer of the data
 * word (FF_W) is complete.
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_spi_enable(struct spi_ip6524_dev_t* dev);

/**
 * \brief Disables SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_spi_disable(struct spi_ip6524_dev_t* dev);

/**
 * \brief Returns whether the SPI IP6524 device is enabled or not
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return Status of the SPI Enable flag
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_is_spi_enabled(struct spi_ip6524_dev_t* dev);

/**
 * \brief Returns SPI Master Baud Rate Divisor for the given frequency
 *
 * \param[in]  sys_clk     System Clock in Hz
 * \param[in]  hz          SPI baud rate in Hz
 *
 * \return Master Baud Rate Divisor value from
 * \ref spi_ip6524_master_baud_rate_divisor_t
 *
 */
enum spi_ip6524_master_baud_rate_divisor_t
            spi_ip6524_get_divisor_for_frequency(uint32_t sys_clk, uint32_t hz);

/**
 * \brief Initializes the SPI IP6524 device.
 *        - Master mode selected
 *        - Master Baud Rate Divisor set based on the specified frequency
 *        - Interrupt sources enabled:
 *                 -Mode Fail,
 *                 -TX FIFO Not Full
 *                 -RX FIFO Not Empty enabled
 *        Init should be called prior to any other process and
 *        it's the caller's responsibility to follow proper call order.
 *
 * \param[in]  dev         SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  sys_clk     System Clock in Hz
 * \param[in]  hz          SPI baud rate in Hz
 *
 * \return Error code from \ref spi_ip6524_error_t
 *
 * \note This function doesn't check if dev is NULL.
 */
enum spi_ip6524_error_t spi_ip6524_init(struct spi_ip6524_dev_t* dev,
                                        uint32_t sys_clk,
                                        uint32_t hz);

/**
 * \brief Returns SPI Mode of operation (Master or Slave)
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI operation mode
 * \ref spi_ip6524_mode_select_t
 *
 */
enum spi_ip6524_mode_select_t spi_ip6524_get_mode(struct spi_ip6524_dev_t* dev);

/**
 * \brief Selects SPI IP6524 device as Master or Slave
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  mode   Mode selection \ref spi_ip6524_mode_select_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_select_mode(struct spi_ip6524_dev_t* dev,
                            enum spi_ip6524_mode_select_t mode);

/**
 * \brief Returns SPI Clock Polarity
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI clock polarity
 * \ref spi_ip6524_clock_polarity_t
 *
 */
enum spi_ip6524_clock_polarity_t
                    spi_ip6524_get_clock_polarity(struct spi_ip6524_dev_t* dev);

/**
 * \brief Selects the clock polarity outside the SPI word
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  pol    Ext clock edge  \ref spi_ip6524_clock_polarity_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_select_clock_polarity(struct spi_ip6524_dev_t* dev,
                                      enum spi_ip6524_clock_polarity_t pol);

/**
 * \brief Returns SPI Clock Phase
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI clock phase
 * \ref spi_ip6524_clock_phase_t
 *
 */
enum spi_ip6524_clock_phase_t
                       spi_ip6524_get_clock_phase(struct spi_ip6524_dev_t* dev);

/**
 * \brief Selects whether the SPI clock is in active
 *        or inactive phase outside the SPI word
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  phase  Clock Phase, see \ref spi_ip6524_clock_phase_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_select_clock_phase(struct spi_ip6524_dev_t* dev,
                                   enum spi_ip6524_clock_phase_t phase);

/**
 * \brief Returns SPI Master Baud Rate Divisor
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI Master Baud Rate Divisor
 * \ref spi_ip6524_get_master_baud_rate_divisor
 *
 */
enum spi_ip6524_master_baud_rate_divisor_t
          spi_ip6524_get_master_baud_rate_divisor(struct spi_ip6524_dev_t* dev);

/**
 * \brief Sets Master Baud Rate Divisor value for SPI IP6524 device
 *
 * \param[in]  dev        SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  divisor    Master Baud Rate Divisor selection, see
 *                             \ref spi_ip6524_master_baud_rate_divisor_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_set_master_baud_rate_divisor(struct spi_ip6524_dev_t* dev,
                      enum spi_ip6524_master_baud_rate_divisor_t divisor);

/**
 * \brief Returns SPI Reference Clock source
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI Reference Clock source
 * \ref spi_ip6524_reference_clock_select_t
 *
 */
enum spi_ip6524_reference_clock_select_t
                      spi_ip6524_get_clock_source(struct spi_ip6524_dev_t* dev);

/**
 * \brief Selects Reference Clock for SPI IP6524 device
 *
 * \param[in]  dev           SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  clk_source    Reference clock, see
 *                           \ref spi_ip6524_reference_clock_select_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_select_clock_source(struct spi_ip6524_dev_t* dev,
            enum spi_ip6524_reference_clock_select_t clk_source);

/**
 * \brief Returns SPI Peripheral Select Decode mode
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI Peripheral Select Decode mode
 * \ref spi_ip6524_peripheral_select_decode_t
 *
 */
enum spi_ip6524_peripheral_select_decode_t
                            spi_ip6524_get_decode(struct spi_ip6524_dev_t* dev);

/**
 * \brief Selects Reference Clock for SPI IP6524 device
 *
 * \param[in]  dev            SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  decode_mode    Peripheral Select Decode, see
 *                            \ref spi_ip6524_peripheral_select_decode_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_select_decode(struct spi_ip6524_dev_t* dev,
                        enum spi_ip6524_peripheral_select_decode_t decode_mode);

/**
 * \brief Returns SPI Peripheral Chip Select Lines
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI Peripheral Chip Select Lines
 * \ref spi_ip6524_peripheral_chip_select_lines_t
 *
 */
enum spi_ip6524_peripheral_chip_select_lines_t
                          spi_ip6524_get_cs_lines(struct spi_ip6524_dev_t* dev);

/**
 * \brief Selects Peripheral Chip Select Lines for SPI IP6524 device
 *
 * \param[in]  dev         SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  cs_lines    Peripheral Chip Select Lines, see
 *                         \ref spi_ip6524_peripheral_chip_select_lines_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_select_cs_lines(struct spi_ip6524_dev_t* dev,
                       enum spi_ip6524_peripheral_chip_select_lines_t cs_lines);

/**
 * \brief Returns true if SPI Manual Chip Select is enabled
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return State of SPI Manual Chip Select configuration bit
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_is_manual_chip_select_enabled(struct spi_ip6524_dev_t* dev);

/**
 * \brief Enables Chip Select for SPI IP6524 device,
 *        so n_ss_out lines will be driven permanently
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_enable_manual_chip_select(struct spi_ip6524_dev_t* dev);

/**
 * \brief Disables Chip Select for SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_disable_manual_chip_select(struct spi_ip6524_dev_t* dev);

/**
 * \brief Returns true if SPI Manual Start is enabled
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return State of SPI Manual Start Enable configuration bit
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_is_manual_start_enabled(struct spi_ip6524_dev_t* dev);

/**
 * \brief Enables Manual Start for SPI IP6524 device, so transmission is not
 *        allowed to start until Manual Start Command bit is set
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_enable_manual_start(struct spi_ip6524_dev_t* dev);

/**
 * \brief Disables Manual Start for SPI IP6524 device, so transmission starts
 *        as soon as there is a word in the FIFO
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_disable_manual_start(struct spi_ip6524_dev_t* dev);

/**
 * \brief Sends Manual Start Command to SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_manual_start_command(struct spi_ip6524_dev_t* dev);

/**
 * \brief Returns true if SPI Mode Fail Generation is enabled
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return State of SPI Mode Fail Generation Enable configuration bit
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_is_mode_fail_generation_enabled(struct spi_ip6524_dev_t* dev);

/**
 * \brief Enables Mode Fail Generation for SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_enable_mode_fail_generation(struct spi_ip6524_dev_t* dev);

/**
 * \brief Disables Mode Fail Generation for SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_disable_mode_fail_generation(struct spi_ip6524_dev_t* dev);

/**
 * \brief Returns true if SPI Sample Point Shift is enabled
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return State of SPI Sample Point Shift Enable configuration bit
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_is_sample_point_shift_enabled(struct spi_ip6524_dev_t* dev);

/**
 * \brief Enables Sample Point Shift for SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_enable_sample_point_shift(struct spi_ip6524_dev_t* dev);

/**
 * \brief Disables Sample Point Shift for SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_disable_sample_point_shift(struct spi_ip6524_dev_t* dev);

/**
 * \brief Clears RX FIFO in SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_clear_rx_fifo(struct spi_ip6524_dev_t* dev);

/**
 * \brief Clears TX FIFO in SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_clear_tx_fifo(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets the Interrupt Status of the SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return  Returns interrupt status value
 *
 * \note This function doesn't check if dev is NULL.
 */
uint32_t spi_ip6524_get_irq_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets SPI RX FIFO Overflow Interrupt Status
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI RX Overflow Interrupt Status
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_get_irq_rx_overflow_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets SPI Mode Fail Interrupt Status
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI Mode Fail Interrupt Status
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_get_irq_mode_fail_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets SPI TX FIFO Not Full Interrupt Status
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI TX FIFO Not Full Interrupt Status
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_get_irq_tx_not_full_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets SPI TX FIFO Full Interrupt Status
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI TX FIFO Full Interrupt Status
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_get_irq_tx_full_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets SPI RX FIFO Not Empty Interrupt Status
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI RX FIFO Not Empty Interrupt Status
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_get_irq_rx_not_empty_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets SPI RX FIFO Full Interrupt Status
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI RX FIFO Full Interrupt Status
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_get_irq_rx_full_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets SPI TX FIFO Underflow Interrupt Status
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return SPI TX Underflow Interrupt Status
 *
 * \note This function doesn't check if dev is NULL.
 */
bool spi_ip6524_get_irq_tx_underflow_status(struct spi_ip6524_dev_t* dev);

/**
 * \brief Gets the Interrupt Mask of the SPI IP6524 device
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return  Returns interrupt mask value
 *
 * \note This function doesn't check if dev is NULL.
 */
uint32_t spi_ip6524_get_irq_mask(struct spi_ip6524_dev_t* dev);

/**
 * \brief Enables the given interrupts in the SPI IP6524 device
 *
 * \param[in]  dev         SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  irq_mask    Mask for interrupts to enable
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_enable_interrupt(struct spi_ip6524_dev_t* dev,
                                 uint32_t irq_mask);

/**
 * \brief Enables the default interrupts in the SPI IP6524 device, namely:
 *        MODE FAIL Generation,
 *        TX FIFO NOT FULL,
 *        RX FIFO NOT EMPTY
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_enable_default_interrupts(struct spi_ip6524_dev_t* dev);

/**
 * \brief Disables the given interrupts in the SPI IP6524 device
 *
 * \param[in]  dev         SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  irq_mask    Mask for interrupts to disable
 *
 * \note This function doesn't check if dev is NULL.
 */
void spi_ip6524_disable_interrupt(struct spi_ip6524_dev_t* dev,
                                  uint32_t irq_mask);

/**
 * \brief Reads single data from SPI. Non blocking.
 *
 * \param[in]  dev       SPI device structure, see \ref spi_ip6524_dev_t
 * \param[out] rx_ptr    Buffer pointer to be filled must be enough for
 *                       configured word size
 *
 * \return  Error code from \ref spi_ip6524_error_t
 *
 * \note This function doesn't check if dev is NULL and if the driver is
 *       initialized to reduce the number of checks and make the function
 *       execution faster.
 */
enum spi_ip6524_error_t spi_ip6524_read(struct spi_ip6524_dev_t* dev,
                                        uint8_t* rx_ptr);

/**
 * \brief Reads single data from slave SPI. Non blocking.
 *
 * \param[in]  dev    SPI device structure, see \ref spi_ip6524_dev_t
 *
 * \return  Returns data value from the device
 *
 * \note This function doesn't check if dev is NULL and does not validate
 *       whether there is any data in the RX buffer.
 */
uint32_t spi_ip6524_slave_read(struct spi_ip6524_dev_t* dev);

/**
 * \brief  Writes single data to SPI. Non blocking.
 *
 * \param[in]  dev       SPI device structure, see \ref spi_ip6524_dev_t
 * \param[out] tx_ptr    Pointer to the data to be sent
 *
 * \return  Error code from \ref spi_ip6524_error_t
 *
 * \note This function doesn't check if dev is NULL and if the driver is
 *       initialized to reduce the number of checks and make the function
 *       execution faster.
 */
enum spi_ip6524_error_t spi_ip6524_write(struct spi_ip6524_dev_t* dev,
                                         const uint8_t* tx_ptr);

/**
 * \brief  Transmit and Receive data on SPI in a blocking call
 *
 * \param[in]  dev              SPI device structure, see \ref spi_ip6524_dev_t
 * \param[in]  tx_ptr           Buffer pointer to be filled
 * \param[in,out] tx_len_ptr    Num values to transfer (updated on error)
 *                              need to be multiples of transfer word length
 * \param[out] rx_ptr           Buffer pointer to be filled
 * \param[in,out] rx_len_ptr    Num values to receive (updated on error)
 *                              need to be multiples of transfer word length
 *
 * \return  Error code from \ref spi_ip6524_error_t
 *
 * \note This function doesn't check if dev is NULL and if the driver is
 *       initialized to reduce the number of checks and make the function
 *       execution faster.
 */
enum spi_ip6524_error_t spi_ip6524_txrx_blocking(struct spi_ip6524_dev_t* dev,
                                                 const void *tx_ptr,
                                                 uint32_t *tx_len_ptr,
                                                 void *rx_ptr,
                                                 uint32_t *rx_len_ptr);

#ifdef __cplusplus
}
#endif
#endif /* __SPI_IP6524_DRV_H__ */
