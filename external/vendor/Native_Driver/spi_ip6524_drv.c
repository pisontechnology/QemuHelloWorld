/**
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
 * \file spi_ip6524_drv.c
 * \brief Generic driver for SPI IP6524.
 */

#include "spi_ip6524_drv.h"

/*******************************************************************************
 * Cadence IP6524 SPI device specific definitions based on:
 *       Serial Peripheral Interface IP User Guide
 *       Part Number:           IP6524
 *       IP Version Number:     r111_f04
 *       User Guide Revision:   1.21
 *
 ******************************************************************************/

/** Setter bit manipulation macro */
#define SET_BIT(WORD, BIT_INDEX) ((WORD) |= (1U << (BIT_INDEX)))
/** Clearing bit manipulation macro */
#define CLR_BIT(WORD, BIT_INDEX) ((WORD) &= ~(1U << (BIT_INDEX)))
/** Getter bit manipulation macro */
#define GET_BIT(WORD, BIT_INDEX) (bool)(((WORD) & (1U << (BIT_INDEX))))
/** Clear-and-Set bit manipulation macro */
#define ASSIGN_BIT(WORD, BIT_INDEX, VALUE) \
          (WORD = ((WORD & ~(1U << (BIT_INDEX))) | ((VALUE & 1U) << (BIT_INDEX))))

/** Getter bit-field manipulation macro */
#define GET_BIT_FIELD(WORD, BIT_MASK, BIT_OFFSET) \
            ((WORD & BIT_MASK) >> BIT_OFFSET)

/** Clear-and-Set bit-field manipulation macro */
#define ASSIGN_BIT_FIELD(WORD, BIT_MASK, BIT_OFFSET, VALUE) \
            (WORD = ((WORD & ~(BIT_MASK)) | ((VALUE << BIT_OFFSET) & BIT_MASK)))

/* ---  Configuration Register --- */

/** Mode Select */
#define SPI_IP6524_MODE_SEL_POS                     0ul
#define SPI_IP6524_MODE_SEL_BIT_WIDTH               1ul
#define SPI_IP6524_MODE_SEL_MSK                     \
    (BITMASK(SPI_IP6524_MODE_SEL_BIT_WIDTH)<<SPI_IP6524_MODE_SEL_POS)

/** External Clock Edge */
#define SPI_IP6524_CLOCK_POL_POS                    1ul
#define SPI_IP6524_CLOCK_POL_BIT_WIDTH              1ul
#define SPI_IP6524_CLOCK_POL_MSK                    \
    (BITMASK(SPI_IP6524_CLOCK_POL_BIT_WIDTH)<<SPI_IP6524_CLOCK_POL_POS)

/** Clock Phase */
#define SPI_IP6524_CLOCK_PHASE_POS                  2ul
#define SPI_IP6524_CLOCK_PHASE_BIT_WIDTH            1ul
#define SPI_IP6524_CLOCK_PHASE_MSK                  \
    (BITMASK(SPI_IP6524_CLOCK_PHASE_BIT_WIDTH)<<SPI_IP6524_CLOCK_PHASE_POS)

/** Master Baud Rate Divisor */
#define SPI_IP6524_MBRD_POS                         3ul
#define SPI_IP6524_MBRD_BIT_WIDTH                   3ul
#define SPI_IP6524_MBRD_MSK                         \
    (BITMASK(SPI_IP6524_MBRD_BIT_WIDTH)<<SPI_IP6524_MBRD_POS)

/** Reference Clock Select */
#define SPI_IP6524_REF_CLOCK_SEL_POS                8ul
#define SPI_IP6524_REF_CLOCK_SEL_BIT_WIDTH          1ul
#define SPI_IP6524_REF_CLOCK_SEL_MSK                \
    (BITMASK(SPI_IP6524_REF_CLOCK_SEL_BIT_WIDTH)<<SPI_IP6524_REF_CLOCK_SEL_POS)

/** Peripheral Select Decode */
#define SPI_IP6524_SEL_DECODE_POS                   9ul
#define SPI_IP6524_SEL_DECODE_BIT_WIDTH             1ul
#define SPI_IP6524_SEL_DECODE_MSK                   \
    (BITMASK(SPI_IP6524_SEL_DECODE_BIT_WIDTH)<<SPI_IP6524_SEL_DECODE_POS)

/** Peripheral Chip Select Lines (Master mode only) */
#define SPI_IP6524_CHIP_SEL_LINES_POS               10ul
#define SPI_IP6524_CHIP_SEL_LINES_BIT_WIDTH         4ul
#define SPI_IP6524_CHIP_SEL_LINES_MSK               \
    (BITMASK(SPI_IP6524_CHIP_SEL_LINES_BIT_WIDTH)<<SPI_IP6524_CHIP_SEL_LINES_POS)

/** Manual Chip Select Enable */
#define SPI_IP6524_MANUAL_CS_EN_POS                 14ul
#define SPI_IP6524_MANUAL_CS_EN_BIT_WIDTH           1ul
#define SPI_IP6524_MANUAL_CS_EN_MSK                 \
    (BITMASK(SPI_IP6524_MANUAL_CS_EN_BIT_WIDTH)<<SPI_IP6524_MANUAL_CS_EN_POS)

/** Manual Start Enable */
#define SPI_IP6524_MANUAL_START_EN_POS              15ul
#define SPI_IP6524_MANUAL_START_EN_BIT_WIDTH        1ul
#define SPI_IP6524_MANUAL_START_EN_MSK              \
    (BITMASK(SPI_IP6524_MANUAL_START_EN_BIT_WIDTH)<<SPI_IP6524_MANUAL_START_EN_POS)

/** Manual Start Command */
#define SPI_IP6524_MANUAL_START_COMMAND_POS         16ul
#define SPI_IP6524_MANUAL_START_COMMAND_BIT_WIDTH   1ul
#define SPI_IP6524_MANUAL_START_COMMAND_MSK         \
    (BITMASK(SPI_IP6524_MANUAL_START_COMMAND_BIT_WIDTH)<< \
    SPI_IP6524_MANUAL_START_COMMAND_POS)

/** Mode Fail Generation Enable */
#define SPI_IP6524_MODE_FAIL_GEN_POS                17ul
#define SPI_IP6524_MODE_FAIL_GEN_BIT_WIDTH          1ul
#define SPI_IP6524_MODE_FAIL_GEN_MSK                \
    (BITMASK(SPI_IP6524_MODE_FAIL_GEN_BIT_WIDTH)<<SPI_IP6524_MODE_FAIL_GEN_POS)

/** Sample Point Shift Enable */
#define SPI_IP6524_SAMPLE_POINT_SHIFT_EN_POS        18ul
#define SPI_IP6524_SAMPLE_POINT_SHIFT_EN_BIT_WIDTH  1ul
#define SPI_IP6524_SAMPLE_POINT_SHIFT_EN_MSK        \
    (BITMASK(SPI_IP6524_SAMPLE_POINT_SHIFT_EN_BIT_WIDTH)<< \
    SPI_IP6524_SAMPLE_POINT_SHIFT_EN_POS)

/** RX FIFO Clear */
#define SPI_IP6524_RX_FIFO_CLEAR_POS                19ul
#define SPI_IP6524_RX_FIFO_CLEAR_BIT_WIDTH          1ul
#define SPI_IP6524_RX_FIFO_CLEAR_MSK                \
    (BITMASK(SPI_IP6524_RX_FIFO_CLEAR_BIT_WIDTH)<<SPI_IP6524_RX_FIFO_CLEAR_POS)

/** TX FIFO Clear */
#define SPI_IP6524_TX_FIFO_CLEAR_POS                20ul
#define SPI_IP6524_TX_FIFO_CLEAR_BIT_WIDTH          1ul
#define SPI_IP6524_TX_FIFO_CLEAR_MSK                \
    (BITMASK(SPI_IP6524_TX_FIFO_CLEAR_BIT_WIDTH)<<SPI_IP6524_TX_FIFO_CLEAR_POS)

/* ---  SPI Enable Register --- */

/** SPI Enable */
#define SPI_IP6524_SPI_ENABLE_POS                   0ul
#define SPI_IP6524_SPI_ENABLE_BIT_WIDTH             1ul
#define SPI_IP6524_SPI_ENABLE_MSK                   \
    (BITMASK(SPI_IP6524_SPI_ENABLE_BIT_WIDTH)<<SPI_IP6524_SPI_ENABLE_POS)

#define SPI_IP6524_INITIALIZED                      (1U << 0)

/** SPI IP6524 Hardware implementation dependent definitions */
#define FIFO_DEPTH                                  3ul
#define FIFO_WORD_BIT_WIDTH                         8ul
#define TRANSFER_WORD_BYTE_SIZE                     1ul   /** 1 Byte data size*/
#define WORD_BYTE_MASK                              BITMASK(FIFO_WORD_BIT_WIDTH)
#define DEFAULT_DATA                                WORD_BYTE_MASK

/** Structure for the device registers */
struct spi_ip6524_dev_reg_map_t {
    volatile uint32_t config_reg;         /** Configuration register */
    volatile uint32_t irq_status_reg;     /** Interrupt Status register */
    volatile uint32_t irq_enable_reg;     /** Interrupt Enable register */
    volatile uint32_t irq_disable_reg;    /** Interrupt Disable register */
    volatile uint32_t irq_mask_reg;       /** Interrupt Mask register */
    volatile uint32_t device_enable_reg;  /** SPI Enable register */
    volatile uint32_t reserved_0;         /** Reserved between Base+0x18-0x1C */
    volatile uint32_t tx_data_reg;        /** Transmit Data register */
    volatile uint32_t rx_data_reg;        /** Receive Data register */
    volatile uint32_t reserved_1[55];     /** Reserved between Base+0x24-0xFC */
};

void spi_ip6524_spi_enable(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;
    SET_BIT(p_spi->device_enable_reg, SPI_IP6524_SPI_ENABLE_POS);
}

void spi_ip6524_spi_disable(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;
    CLR_BIT(p_spi->device_enable_reg, SPI_IP6524_SPI_ENABLE_POS);
}

bool spi_ip6524_is_spi_enabled(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;
    return GET_BIT(p_spi->device_enable_reg, SPI_IP6524_SPI_ENABLE_POS);
}

enum spi_ip6524_master_baud_rate_divisor_t
             spi_ip6524_get_divisor_for_frequency(uint32_t sys_clk, uint32_t hz)
{
    const uint32_t valid_frequencies_hz[] = {sys_clk>>1,
                                             sys_clk>>2,
                                             sys_clk>>3,
                                             sys_clk>>4,
                                             sys_clk>>5,
                                             sys_clk>>6,
                                             sys_clk>>7,
                                             sys_clk>>8};

    enum spi_ip6524_master_baud_rate_divisor_t i;
    for(i = SPI_IP6524_BAUD_RATE_DIVISOR_2;
    i <= SPI_IP6524_BAUD_RATE_DIVISOR_256; i++) {
        if(hz >= valid_frequencies_hz[i]) {
            return i;
        }
    }
    /* Always returns the closest frequency in the current operating interval */
    return SPI_IP6524_BAUD_RATE_DIVISOR_256;
}

enum spi_ip6524_error_t spi_ip6524_init(struct spi_ip6524_dev_t* dev,
                                        uint32_t sys_clk,
                                        uint32_t hz)
{
    spi_ip6524_select_mode(dev, SPI_IP6524_MASTER_SELECT);

    /* Calculate Master Baud Rate Divisor */
    uint32_t clk_divisor = spi_ip6524_get_divisor_for_frequency(sys_clk, hz);
    spi_ip6524_set_master_baud_rate_divisor(dev, clk_divisor);

    dev->data->state = SPI_IP6524_INITIALIZED;

    return SPI_IP6524_ERR_NONE;
}

enum spi_ip6524_mode_select_t spi_ip6524_get_mode(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_MODE_SEL_POS);

}

void spi_ip6524_select_mode(struct spi_ip6524_dev_t* dev,
                            enum spi_ip6524_mode_select_t mode)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    bool spi_is_enabled = spi_ip6524_is_spi_enabled(dev);

    if(spi_is_enabled) {
        spi_ip6524_spi_disable(dev);
        ASSIGN_BIT(p_spi->config_reg, SPI_IP6524_MODE_SEL_POS, mode);
        spi_ip6524_spi_enable(dev);
    } else {
        ASSIGN_BIT(p_spi->config_reg, SPI_IP6524_MODE_SEL_POS, mode);
    }
}

enum spi_ip6524_clock_polarity_t spi_ip6524_get_clock_polarity(
                                                   struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_CLOCK_POL_POS);
}

void spi_ip6524_select_clock_polarity(struct spi_ip6524_dev_t* dev,
                                      enum spi_ip6524_clock_polarity_t pol)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    ASSIGN_BIT(p_spi->config_reg, SPI_IP6524_CLOCK_POL_POS, pol);
}

enum spi_ip6524_clock_phase_t spi_ip6524_get_clock_phase(
                                                   struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_CLOCK_PHASE_POS);
}

void spi_ip6524_select_clock_phase(struct spi_ip6524_dev_t* dev,
                                            enum spi_ip6524_clock_phase_t phase)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    ASSIGN_BIT(p_spi->config_reg, SPI_IP6524_CLOCK_PHASE_POS, phase);
}

enum spi_ip6524_master_baud_rate_divisor_t
           spi_ip6524_get_master_baud_rate_divisor(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT_FIELD(p_spi->config_reg,
                         SPI_IP6524_MBRD_MSK,
                         SPI_IP6524_MBRD_POS);
}

void spi_ip6524_set_master_baud_rate_divisor(struct spi_ip6524_dev_t* dev,
                             enum spi_ip6524_master_baud_rate_divisor_t divisor)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    ASSIGN_BIT_FIELD(p_spi->config_reg,
                     SPI_IP6524_MBRD_MSK,
                     SPI_IP6524_MBRD_POS,
                     divisor);
}

enum spi_ip6524_reference_clock_select_t
                       spi_ip6524_get_clock_source(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_REF_CLOCK_SEL_POS);
}

void spi_ip6524_select_clock_source(struct spi_ip6524_dev_t* dev,
                            enum spi_ip6524_reference_clock_select_t clk_source)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    ASSIGN_BIT(p_spi->config_reg, SPI_IP6524_REF_CLOCK_SEL_POS, clk_source);
}

enum spi_ip6524_peripheral_select_decode_t
                             spi_ip6524_get_decode(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_SEL_DECODE_POS);
}

void spi_ip6524_select_decode(struct spi_ip6524_dev_t* dev,
                         enum spi_ip6524_peripheral_select_decode_t decode_mode)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    /* SW must wait until the current transfer is complete */
    while(GET_BIT(p_spi->irq_status_reg, SPI_IP6524_TX_FULL_POS)) {
    }

    ASSIGN_BIT(p_spi->config_reg, SPI_IP6524_SEL_DECODE_POS, decode_mode);
}

enum spi_ip6524_peripheral_chip_select_lines_t
                           spi_ip6524_get_cs_lines(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT_FIELD(p_spi->config_reg,
                         SPI_IP6524_CHIP_SEL_LINES_MSK,
                         SPI_IP6524_CHIP_SEL_LINES_POS);
}

void spi_ip6524_select_cs_lines(struct spi_ip6524_dev_t* dev,
                        enum spi_ip6524_peripheral_chip_select_lines_t cs_lines)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    /* SW must wait until the current transfer is complete */
    while(GET_BIT(p_spi->irq_status_reg, SPI_IP6524_TX_FULL_POS)) {
    }

    ASSIGN_BIT_FIELD(p_spi->config_reg,
                     SPI_IP6524_CHIP_SEL_LINES_MSK,
                     SPI_IP6524_CHIP_SEL_LINES_POS,
                     cs_lines);
}

bool spi_ip6524_is_manual_chip_select_enabled(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_MANUAL_CS_EN_POS);
}

void spi_ip6524_enable_manual_chip_select(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    SET_BIT(p_spi->config_reg, SPI_IP6524_MANUAL_CS_EN_POS);
}

void spi_ip6524_disable_manual_chip_select(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    CLR_BIT(p_spi->config_reg, SPI_IP6524_MANUAL_CS_EN_POS);
}

bool spi_ip6524_is_manual_start_enabled(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_MANUAL_START_COMMAND_POS);
}

void spi_ip6524_enable_manual_start(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    SET_BIT(p_spi->config_reg, SPI_IP6524_MANUAL_START_COMMAND_POS);
}

void spi_ip6524_disable_manual_start(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    CLR_BIT(p_spi->config_reg, SPI_IP6524_MANUAL_START_EN_POS);
}

void spi_ip6524_manual_start_command(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    SET_BIT(p_spi->config_reg, SPI_IP6524_MANUAL_START_COMMAND_POS);
}

bool spi_ip6524_is_mode_fail_generation_enabled(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_MODE_FAIL_GEN_POS);
}

void spi_ip6524_enable_mode_fail_generation(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    CLR_BIT(p_spi->config_reg, SPI_IP6524_MODE_FAIL_GEN_POS);
}

void spi_ip6524_disable_mode_fail_generation(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    CLR_BIT(p_spi->config_reg, SPI_IP6524_MODE_FAIL_GEN_POS);
}

bool spi_ip6524_is_sample_point_shift_enabled(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->config_reg, SPI_IP6524_SAMPLE_POINT_SHIFT_EN_POS);
}

void spi_ip6524_enable_sample_point_shift(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    SET_BIT(p_spi->config_reg, SPI_IP6524_SAMPLE_POINT_SHIFT_EN_POS);
}

void spi_ip6524_disable_sample_point_shift(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    CLR_BIT(p_spi->config_reg, SPI_IP6524_SAMPLE_POINT_SHIFT_EN_POS);
}

void spi_ip6524_clear_rx_fifo(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    bool spi_current_status = spi_ip6524_is_spi_enabled(dev);
    spi_ip6524_spi_disable(dev);

    SET_BIT(p_spi->config_reg, SPI_IP6524_RX_FIFO_CLEAR_POS);

    if (spi_current_status) {
        spi_ip6524_spi_enable(dev);
    }
}

void spi_ip6524_clear_tx_fifo(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    bool spi_current_status = spi_ip6524_is_spi_enabled(dev);
    spi_ip6524_spi_disable(dev);

    SET_BIT(p_spi->config_reg, SPI_IP6524_TX_FIFO_CLEAR_POS);

    if (spi_current_status) {
        spi_ip6524_spi_enable(dev);
    }
}

uint32_t spi_ip6524_get_irq_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return p_spi->irq_status_reg;
}

bool spi_ip6524_get_irq_rx_overflow_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->irq_status_reg, SPI_IP6524_RX_OVERFLOW_POS);
}

bool spi_ip6524_get_irq_mode_fail_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->irq_status_reg, SPI_IP6524_MODE_FAIL_POS);
}

bool spi_ip6524_get_irq_tx_not_full_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->irq_status_reg, SPI_IP6524_TX_NOT_FULL_POS);
}

bool spi_ip6524_get_irq_tx_full_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->irq_status_reg, SPI_IP6524_TX_FULL_POS);
}

bool spi_ip6524_get_irq_rx_not_empty_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->irq_status_reg, SPI_IP6524_RX_NOT_EMPTY_POS);
}

bool spi_ip6524_get_irq_rx_full_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->irq_status_reg, SPI_IP6524_RX_FULL_POS);
}

bool spi_ip6524_get_irq_tx_underflow_status(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return GET_BIT(p_spi->irq_status_reg, SPI_IP6524_TX_UNDERFLOW_POS);
}

uint32_t spi_ip6524_get_irq_mask(struct spi_ip6524_dev_t* dev)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    return p_spi->irq_mask_reg;
}

void spi_ip6524_enable_interrupt(struct spi_ip6524_dev_t* dev,
                                 uint32_t irq_mask)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    p_spi->irq_enable_reg = irq_mask;
}

void spi_ip6524_disable_interrupt(struct spi_ip6524_dev_t* dev,
                                  uint32_t irq_mask)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    p_spi->irq_disable_reg = irq_mask;
}

void spi_ip6524_enable_default_interrupts(struct spi_ip6524_dev_t* dev)
{
    spi_ip6524_enable_interrupt(dev, (SPI_IP6524_MODE_FAIL_MSK   |
                                      SPI_IP6524_TX_NOT_FULL_MSK |
                                      SPI_IP6524_RX_NOT_EMPTY_MSK));
}

enum spi_ip6524_error_t spi_ip6524_read(struct spi_ip6524_dev_t* dev,
                                        uint8_t* rx_ptr)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    if(GET_BIT(p_spi->irq_status_reg, SPI_IP6524_RX_NOT_EMPTY_POS)) {
        *rx_ptr = p_spi->rx_data_reg & WORD_BYTE_MASK;
        return SPI_IP6524_ERR_NONE;
    }

    return SPI_IP6524_ERR_NO_RX;
}

uint32_t spi_ip6524_slave_read(struct spi_ip6524_dev_t* dev)
{
    uint32_t data;
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;

    while(!(GET_BIT(p_spi->irq_status_reg, SPI_IP6524_RX_NOT_EMPTY_POS))){
    }
    data = p_spi->rx_data_reg & WORD_BYTE_MASK;

    return data;
}

enum spi_ip6524_error_t spi_ip6524_write(struct spi_ip6524_dev_t* dev,
                                         const uint8_t* tx_ptr)
{
    struct spi_ip6524_dev_reg_map_t* p_spi =
                              (struct spi_ip6524_dev_reg_map_t*) dev->cfg->base;


    if(GET_BIT(p_spi->irq_status_reg, SPI_IP6524_TX_NOT_FULL_POS)) {
        p_spi->tx_data_reg = (*tx_ptr) & WORD_BYTE_MASK;

        while(!(GET_BIT(p_spi->irq_status_reg, SPI_IP6524_TX_NOT_FULL_POS))){
        }
        return SPI_IP6524_ERR_NONE;
    }

    return SPI_IP6524_ERR_NO_TX;
}

enum spi_ip6524_error_t spi_ip6524_txrx_blocking(struct spi_ip6524_dev_t* dev,
                                               const void *tx_ptr,
                                               uint32_t *tx_len_ptr,
                                               void *rx_ptr,
                                               uint32_t *rx_len_ptr)
{
    uint32_t i = 0;
    uint8_t rx_data = 0;
    uint8_t tx_data = 0;
    uint32_t total_len = 0;
    enum spi_ip6524_error_t retval = SPI_IP6524_ERR_NONE;

    total_len = (*tx_len_ptr > *rx_len_ptr) ? (*tx_len_ptr) : (*rx_len_ptr);

    for(i = 0; i < total_len; i += TRANSFER_WORD_BYTE_SIZE){
        if (i < (*tx_len_ptr)) {
            tx_data = *(const uint8_t*)tx_ptr;
        } else {
            /* Send default data if there is no more valid data to send */
            tx_data = DEFAULT_DATA;
        }

        retval = spi_ip6524_write(dev, &tx_data);
        if(retval != SPI_IP6524_ERR_NONE) {
            *tx_len_ptr = i;
            *rx_len_ptr = i;
            break;
        }

        if(i < (*tx_len_ptr)) {
            tx_ptr = (const uint8_t*)tx_ptr + TRANSFER_WORD_BYTE_SIZE;
        }

        retval = spi_ip6524_read(dev, &rx_data);
        if(retval != SPI_IP6524_ERR_NONE) {
            /* Send went through, align tx_len to the updated tx_ptr */
            *tx_len_ptr = i + TRANSFER_WORD_BYTE_SIZE;
            /* Don't update rx_len if there is an overflow */
            if (i < (*rx_len_ptr)) {
                *rx_len_ptr = i;
            }
            break;
        }

        /* Do not overflow RX buffer */
        if(i < (*rx_len_ptr)) {
            *(uint8_t*)rx_ptr = rx_data;
            rx_ptr = (uint8_t*)rx_ptr + TRANSFER_WORD_BYTE_SIZE;
        }
    }

    return retval;
}
