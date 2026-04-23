/**
 ******************************************************************************
 * @file           : m24cxx.h
 * @brief          : M24Cxx Library Header
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 Lars Boegild Thomsen <lbthomsen@gmail.com>
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#ifndef M24CXX_H_
#define M24CXX_H_

#include <stdio.h>

#define M24C08 0
#define M24M01 1
#define M24M01X4 2

#if M24CXX_MODEL == M24C08
#define M24CXX_TYPE              "24C08"
#define M24CXX_SIZE                1024
#define M24CXX_ADDRESS_BITS           8
#define M24CXX_ADDRESS_SIZE           I2C_MEMADD_SIZE_8BIT
#define M24CXX_ADDRESS_MASK  0x000000ff
#define M24CXX_READ_PAGE_SIZE       256
#define M24CXX_WRITE_PAGE_SIZE       16
#define M24CXX_WRITE_TIMEOUT        100
#elif M24CXX_MODEL == M24M01
#define M24CXX_TYPE              "24M01"
#define M24CXX_SIZE              131072
#define M24CXX_PAGE_ADDRESS_BITS      1
#define M24CXX_ADDRESS_BITS          16
#define M24CXX_ADDRESS_SIZE           I2C_MEMADD_SIZE_16BIT
#define M24CXX_ADDRESS_MASK  0x0000ffff
#define M24CXX_READ_PAGE_SIZE       256
#define M24CXX_WRITE_PAGE_SIZE      256
#define M24CXX_WRITE_TIMEOUT        100
#elif M24CXX_MODEL == M24M01X4
#define M24CXX_TYPE              "4 x 24M01"
#define M24CXX_SIZE              524288
#define M24CXX_PAGE_ADDRESS_BITS      1
#define M24CXX_ADDRESS_BITS          16
#define M24CXX_ADDRESS_SIZE           I2C_MEMADD_SIZE_16BIT
#define M24CXX_ADDRESS_MASK  0x0000ffff
#define M24CXX_READ_PAGE_SIZE       256
#define M24CXX_WRITE_PAGE_SIZE      256
#define M24CXX_WRITE_TIMEOUT        100
#else
#error "M24CXX_MODEL must be defined in project properties"
#endif

#ifdef xxxDEBUG
#define M24CXXDBG(...) printf(__VA_ARGS__);\
                       printf("\r\n")
#else
#define M24CXXDBG(...)
#endif

typedef struct {
    I2C_HandleTypeDef *i2c;
    uint8_t i2c_address;
} M24CXX_HandleTypeDef;

typedef enum {
    M24CXX_Ok, M24CXX_Err
} M24CXX_StatusTypeDef;

M24CXX_StatusTypeDef m24cxx_init(M24CXX_HandleTypeDef *m24cxx, I2C_HandleTypeDef *i2c, uint8_t i2c_address);
M24CXX_StatusTypeDef m24cxx_isconnected(M24CXX_HandleTypeDef *m24cxx);
M24CXX_StatusTypeDef m24cxx_read(M24CXX_HandleTypeDef *m24cxx, uint32_t address, uint8_t *data, uint32_t len);
M24CXX_StatusTypeDef m24cxx_write(M24CXX_HandleTypeDef *m24cxx, uint32_t address, uint8_t *data, uint32_t len);
M24CXX_StatusTypeDef m24cxx_erase(M24CXX_HandleTypeDef *m24cxx, uint32_t address, uint32_t len);
M24CXX_StatusTypeDef m24cxx_erase_all(M24CXX_HandleTypeDef *m24cxx);

#endif /* M24CXX_H_ */
