/*
 * Copyright (c) 2018 Arm Limited
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

/*
 * This library provides function to control the MT25QL256ABA-1EW7-OSIT flash
 * memory from Micron and should work for similar devices from the same vendor.
 */

#ifndef __MT25QL_H__
#define __MT25QL_H__

#include "qspi_ip6514e_drv.h"

#ifdef __cplusplus
extern "C" {
#endif

enum mt25ql_error_t {
    MT25QL_ERR_NONE = QSPI_IP6514E_ERR_NONE, /*
                                              * Force the error enumeration
                                              * values not to overlap. If the
                                              * error value is less than this
                                              * one, this is a controller
                                              * error which value maps to the
                                              * qspi_ip6514e_error_t
                                              * enumeration.
                                              */
    MT25QL_ERR_WRONG_ARGUMENT = QSPI_IP6514E_ERR_MAX, /*
                                                       * The first error
                                                       * enumeration should be
                                                       * equal to ERR_MAX.
                                                       */
    MT25QL_ERR_ADDR_NOT_ALIGNED,
    MT25QL_ERR_ADDR_TOO_BIG,
};

enum mt25ql_erase_t {
    MT25QL_ERASE_ALL_FLASH     = 0U,          /*!< Erase all flash */
    MT25QL_ERASE_SUBSECTOR_4K  = 0x00001000U, /*!< Erase a 4 KiB subsector */
    MT25QL_ERASE_SUBSECTOR_32K = 0x00008000U, /*!< Erase a 32 KiB subsector */
    MT25QL_ERASE_SECTOR_64K    = 0x00010000U, /*!< Erase a sector (64 KiB) */
};

struct mt25ql_dev_t {
    struct qspi_ip6514e_dev_t *controller; /*!< QSPI Flash controller */
    uint32_t direct_access_start_addr;   /*!< AHB address to directly access
                                                the contents of the Flash memory
                                                through the QSPI Controller */
    uint32_t baud_rate_div; /*!< Clock divisor that will be used to configure
                                 the QSPI Flash Controller to access the Flash
                                 memory. The clock which frequency is divived is
                                 the one linked to the QSPI Flash controller.
                                 It can only be an even number between 2 and 32
                                 (both included). It needs to be high enough
                                 to support the Quad Output Fast Read command
                                 with 8 dummy cycles and the Quad Input Fast
                                 Program with 0 dummy cycles. */
    uint32_t size; /*!< Total size of the MT25QL Flash memory */

};

/**
 * \brief Configure the QSPI Flash controller and MT25QL for optimal use.
 *
 * \param[in] dev     Pointer to MT25QL device structure \ref mt25ql_dev_t
 *
 * \return Return error code as specified in \ref mt25ql_error_t
 *
 * \note This function assumes that the Flash memory device currently operates
 *       with single line SPI protocol with Double Data Rate protocol disabled.
 *       This function will not work and cause harm if the Flash device is in
 *       another configuration.
 * \note The configuration used is the following:
 *         * QSPI mode (4 lines for instruction, address and data)
 *         * Read command is Quad Output Fast Read with 8 dummy cycles
 *         * Write command is Quad Input Fast Program with 0 dummy cycles
 *         * Bytes per page set to 256
 *         * Number of address bytes set to 3
 */
enum mt25ql_error_t mt25ql_cfg_optimal(struct mt25ql_dev_t* dev);

/**
 * \brief Read bytes from the flash memory (direct access)
 *
 * \param[in] dev   Pointer to MT25QL device structure \ref mt25ql_dev_t
 * \param[in] addr  Flash memory address for the read operation
 * \param[in] data  Pointer where len bytes read from the flash memory will be
 *                  written to
 * \param[in] len   Number of bytes to read
 *
 * \return Return error code as specified in \ref mt25ql_error_t
 *
 * \note This function will use direct access to read from the Flash memory. It
 *       can be used to access above the direct accessible memory zone if
 *       not all the AHB address wires are connected.
 * \note The address given should be the address of the data inside the flash
 *       memory. To read the first byte inside the memory, use 0x00000000.
 */
enum mt25ql_error_t mt25ql_direct_read(struct mt25ql_dev_t* dev,
                                       uint32_t addr,
                                       void *data,
                                       uint32_t len);

/**
 * \brief Write bytes in the flash memory, at a location where data has already
 *        been erased (direct access)
 *
 * \param[in] dev   Pointer to MT25QL device structure \ref mt25ql_dev_t
 * \param[in] addr  Flash memory address for the write operation
 * \param[in] data  Pointer to the len bytes that will be written to the flash
 *                  memory
 * \param[in] len   Number of bytes to write
 *
 * \return Return error code as specified in \ref mt25ql_error_t
 *
 * \note This function will use direct access to write to the Flash memory. It
 *       can be used to access outside of the direct accessible memory zone if
 *       not all the AHB address wires are connected.
 * \note The address given should be the address of the data inside the flash
 *       memory. To write the first byte inside the memory, use 0x00000000.
 * \note Writing bytes in the flash memory clear them from 1 to 0, for that
 *       matter the location where data is written needs to be erased
 *       beforehand.
 */
enum mt25ql_error_t mt25ql_direct_write(struct mt25ql_dev_t* dev,
                                        uint32_t addr,
                                        void *data,
                                        uint32_t len);

/**
 * \brief Read bytes from the flash memory (using Flash commands)
 *
 * \param[in] dev   Pointer to MT25QL device structure \ref mt25ql_dev_t
 * \param[in] addr  Flash memory address for the read operation
 * \param[in] data  Pointer where len bytes read from the flash memory will be
 *                  written to
 * \param[in] len   Number of bytes to read
 *
 * \return Return error code as specified in \ref mt25ql_error_t
 *
 * \note This function will use the Software Triggered Instruction Generator to
 *       read from the Flash memory using Flash commands.
 * \note The address given should be the address of the data inside the flash
 *       memory. To read the first byte inside the memory, use 0x00000000.
 */
enum mt25ql_error_t mt25ql_command_read(struct mt25ql_dev_t* dev,
                                        uint32_t addr,
                                        void *data,
                                        uint32_t len);

/**
 * \brief Write bytes in the flash memory, at a location where data has already
 *        been erased (using Flash commands)
 *
 * \param[in] dev   Pointer to MT25QL device structure \ref mt25ql_dev_t
 * \param[in] addr  Flash memory address for the write operation
 * \param[in] data  Pointer to the len bytes that will be written to the flash
 *                  memory
 * \param[in] len   Number of bytes to write
 *
 * \return Return error code as specified in \ref mt25ql_error_t
 *
 * \note This function will use the Software Triggered Instruction Generator to
 *       write to the Flash memory using Flash commands.
 * \note The address given should be the address of the data inside the flash
 *       memory. To write the first byte inside the memory, use 0x00000000.
 * \note Writing bytes in the flash memory clear them from 1 to 0, for that
 *       matter the location where data is written needs to be erased
 *       beforehand.
 */
enum mt25ql_error_t mt25ql_command_write(struct mt25ql_dev_t* dev,
                                         uint32_t addr,
                                         void *data,
                                         uint32_t len);

/**
 * \brief Erase all flash memory, a sector (64 KiB) or a subsector
 *        (32 KiB or 4 KiB)
 *
 * \param[in] dev        Pointer to MT25QL device structure \ref mt25ql_dev_t
 * \param[in] addr       Address where to erase in the flash memory
 * \param[in] erase_type Type of what to erase at the specified address:
 *                         * whole flash memory
 *                         * a subsector (4 KiB or 32 KiB)
 *                         * a sector (64 KiB)
 * \return Return error code as specified in \ref mt25ql_error_t
 *
 * \note The address need to be aligned with the size of what is erased or 0 if
 *       all flash memory is to be erased.
 */
enum mt25ql_error_t mt25ql_erase(struct mt25ql_dev_t* dev,
                                 uint32_t addr,
                                 enum mt25ql_erase_t erase_type);

#ifdef __cplusplus
}
#endif

#endif /* __MT25QL_H__ */
