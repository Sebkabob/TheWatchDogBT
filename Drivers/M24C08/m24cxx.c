/**
 ******************************************************************************
 * @file           : m24cxx.c
 * @brief          : M24Cxx Library Source
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include "main.h"

#include <string.h>

#include "m24cxx.h"

/* Internal Library Functions */

/**
 * @brief  Checks if target device is ready for communication.
 * @note   This function is used with Memory devices
 * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C.
 * @param  i2c_address Target device address
 * @retval M24CXX status
 */
M24CXX_StatusTypeDef i2c_wait(I2C_HandleTypeDef *i2c, uint16_t i2c_address) {

    uint32_t start_time = HAL_GetTick();
    while (HAL_I2C_IsDeviceReady(i2c, i2c_address << 1, 1, HAL_MAX_DELAY) != HAL_OK) {
        if (HAL_GetTick() - start_time >= M24CXX_WRITE_TIMEOUT)
            return M24CXX_Err;
    }

    return M24CXX_Ok;
}

/* Public functions */

/**
 * @brief  Initialize m24cxx library
 * @param  m24cxx Pointer to a M24CXX_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C.
 * @param  a pointer to I2C_HandleTypeDef
 * @param  i2c_address Target device address
 * @retval M24CXX status
 */
M24CXX_StatusTypeDef m24cxx_init(M24CXX_HandleTypeDef *m24cxx, I2C_HandleTypeDef *i2c, uint8_t i2c_address) {

    M24CXXDBG("M24CXX Init type = %s size = %d ", M24CXX_TYPE, M24CXX_SIZE);

    m24cxx->i2c = i2c;
    m24cxx->i2c_address = i2c_address;

#ifdef EEPROM_WP_Pin
    HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_SET);
#endif

    if (m24cxx_isconnected(m24cxx) != M24CXX_Ok)
        return M24CXX_Err;

    return M24CXX_Ok;
}

/**
 * @brief  Check if m24cxx is connected
 * @param  m24cxx Pointer to a M24CXX_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C.
 * @retval M24CXX status
 */
M24CXX_StatusTypeDef m24cxx_isconnected(M24CXX_HandleTypeDef *m24cxx) {

    if (HAL_I2C_IsDeviceReady(m24cxx->i2c, m24cxx->i2c_address << 1, 2, HAL_MAX_DELAY) != HAL_OK) {
        return M24CXX_Err;
    }

    return M24CXX_Ok;
}

/**
 * @brief  Read data from EEPROM
 * @param  m24cxx Pointer to a M24CXX_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C.
 * @param  address from which to read data
 * @param  data buffer
 * @param  length of data buffer
 * @retval M24CXX status
 */
M24CXX_StatusTypeDef m24cxx_read(M24CXX_HandleTypeDef *m24cxx, uint32_t address, uint8_t *data, uint32_t len) {

    M24CXXDBG("M24CXX read - address = 0x%04lx len = 0x%04lx", address, len);

    uint32_t page_start = address / M24CXX_READ_PAGE_SIZE;
    uint32_t page_end = ((address + len - 1) / M24CXX_READ_PAGE_SIZE);
    uint32_t data_offset = 0;

    M24CXXDBG("Reading %lu pages from %lu to %lu", 1 + page_end - page_start, page_start, page_end);

    for (uint32_t page = page_start; page <= page_end; ++page) {

        uint32_t i2c_address, start_address, read_len;

        start_address = page == page_start ? address : page * M24CXX_READ_PAGE_SIZE;
        read_len = page == page_end ? len - data_offset : (page == page_start ? ((page + 1) * M24CXX_READ_PAGE_SIZE) - start_address : M24CXX_READ_PAGE_SIZE);
        i2c_address = m24cxx->i2c_address + (start_address >> M24CXX_ADDRESS_BITS);

        M24CXXDBG("Reading page %lu, i2c address = 0x%02lx start = 0x%04lx len = 0x%04lx offset = 0x%04lx", page, i2c_address, start_address & M24CXX_ADDRESS_MASK, read_len, data_offset);

        HAL_StatusTypeDef result = HAL_I2C_Mem_Read(m24cxx->i2c, i2c_address << 1, start_address & M24CXX_ADDRESS_MASK, M24CXX_ADDRESS_SIZE, data + data_offset, read_len,
        HAL_MAX_DELAY);

        if (result != HAL_OK) {
            M24CXXDBG("Failed to read memory");
            return M24CXX_Err;
        }

        data_offset += read_len;

    }

    return M24CXX_Ok;
}

/**
 * @brief  Write data to EEPROM
 * @param  m24cxx Pointer to a M24CXX_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C.
 * @param  address to which to write data
 * @param  data buffer
 * @param  length of data buffer
 * @retval M24CXX status
 */
M24CXX_StatusTypeDef m24cxx_write(M24CXX_HandleTypeDef *m24cxx, uint32_t address, uint8_t *data, uint32_t len) {

    M24CXXDBG("M24CXX write - address = 0x%04lx len = 0x%04lx", address, len);

    uint32_t page_start = address / M24CXX_WRITE_PAGE_SIZE;
    uint32_t page_end = ((address + len - 1) / M24CXX_WRITE_PAGE_SIZE);
    uint32_t data_offset = 0;

    M24CXXDBG("Writing %lu pages from %lu to %lu", 1 + page_end - page_start, page_start, page_end);

#ifdef EEPROM_WP_Pin
    HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_RESET);
#endif

    for (uint32_t page = page_start; page <= page_end; ++page) {

        uint32_t i2c_address, start_address, start_address_masked, write_len;

        start_address = page == page_start ? address : page * M24CXX_WRITE_PAGE_SIZE;
        start_address_masked = start_address & M24CXX_ADDRESS_MASK;
        write_len = page == page_end ? len - data_offset : (page == page_start ? ((page + 1) * M24CXX_WRITE_PAGE_SIZE) - start_address : M24CXX_WRITE_PAGE_SIZE);
        i2c_address = m24cxx->i2c_address + (start_address >> M24CXX_ADDRESS_BITS);

        M24CXXDBG("Writing page %lu, i2c address = 0x%02lx start = 0x%06lx masked = 0x%06lx len = 0x%04lx offset = 0x%04lx", page, i2c_address, start_address, start_address_masked, write_len, data_offset);

        HAL_StatusTypeDef result = HAL_I2C_Mem_Write(m24cxx->i2c, i2c_address << 1, start_address_masked, M24CXX_ADDRESS_SIZE, data + data_offset, write_len, HAL_MAX_DELAY);

        if (result != HAL_OK) {
            M24CXXDBG("Failed to write memory");
            return M24CXX_Err;
        }

        data_offset += write_len;

        if (i2c_wait(m24cxx->i2c, i2c_address) != M24CXX_Ok) {
            M24CXXDBG("M24Cxx Device never got ready");
            return M24CXX_Err;
        }

    }

#ifdef EEPROM_WP_Pin
    HAL_GPIO_WritePin(EEPROM_WP_GPIO_Port, EEPROM_WP_Pin, GPIO_PIN_SET);
#endif

    return M24CXX_Ok;
}

M24CXX_StatusTypeDef m24cxx_erase(M24CXX_HandleTypeDef *m24cxx, uint32_t address, uint32_t len) {

    M24CXXDBG("M24CXX erase - address = 0x%04lx len = 0x%04lx", address, len);

    uint8_t buf[len];

    memset(buf, 0xff, len);

    M24CXX_StatusTypeDef result = m24cxx_write(m24cxx, address, (uint8_t*) &buf, len);
    if (result != M24CXX_Ok)
        return result;

    return M24CXX_Ok;
}


/**
 * @brief  Erase entire EEPROM (set all data to 0xff
 * @param  m24cxx Pointer to a M24CXX_HandleTypeDef structure that contains
 *         the configuration information for the specified I2C.
 * @retval M24CXX status
 */

M24CXX_StatusTypeDef m24cxx_erase_all(M24CXX_HandleTypeDef *m24cxx) {

    M24CXXDBG("M24CXX erase all");

    uint8_t buf[M24CXX_WRITE_PAGE_SIZE];

    memset(buf, 0xff, sizeof(buf));

    for (uint32_t i = 0; i < M24CXX_SIZE / M24CXX_WRITE_PAGE_SIZE; ++i) {
        M24CXX_StatusTypeDef result = m24cxx_write(m24cxx, i * M24CXX_WRITE_PAGE_SIZE, (uint8_t*) &buf, sizeof(buf));
        if (result != M24CXX_Ok)
            return result;
    }

    return M24CXX_Ok;
}
