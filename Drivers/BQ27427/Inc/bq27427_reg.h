/******************************************************************************
 * bq27427.h
 * BQ27427 Battery Fuel Gauge STM32 HAL Driver
 *
 * Adapted from SparkFun BQ27441 Arduino Library
 * Original Author: Jim Lindblom @ SparkFun Electronics
 * Original Date: May 9, 2016
 * Original Repo: https://github.com/sparkfun/SparkFun_BQ27441_Arduino_Library
 *
 * Arduino Adaptation by: Edrean Ernst
 * Arduino Adaptation Date: June 2025
 * Arduino Repository: https://github.com/edreanernst/BQ27427_Arduino_Library
 *
 * STM32 HAL Adaptation by: Claude
 * STM32 HAL Adaptation Date: 2025
 *
 * This library implements all features of the BQ27427 Battery Fuel Gauge
 * for STM32 microcontrollers using the HAL library.
 *
 * Original License: MIT License
 ******************************************************************************/

#ifndef BQ27427_H
#define BQ27427_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32wb0x_hal.h"
#include <stdbool.h>
#include <stdint.h>

/******************************************************************************
 * Configuration Defines
 ******************************************************************************/

/* I2C Configuration */
#define BQ27427_I2C_TIMEOUT         100     /* I2C timeout in milliseconds */

/* Default Battery Configuration */
#define BQ27427_DEFAULT_CAPACITY    300     /* Default design capacity in mAh */
#define BQ27427_DEFAULT_TERM_V      3400    /* Default terminate voltage in mV */

/* GPOUT Pin Configuration - Modify these for your hardware */
#define BQ27427_GPOUT_PORT          GPIOA
#define BQ27427_GPOUT_PIN           GPIO_PIN_0

/******************************************************************************
 * BQ27427 Constants
 ******************************************************************************/

#define BQ27427_I2C_ADDRESS         0x55    /* 7-bit I2C address */
#define BQ27427_I2C_ADDRESS_WRITE   (BQ27427_I2C_ADDRESS << 1)
#define BQ27427_I2C_ADDRESS_READ    ((BQ27427_I2C_ADDRESS << 1) | 0x01)

#define BQ27427_UNSEAL_KEY          0x8000  /* Secret code to unseal the BQ27427 */
#define BQ27427_DEVICE_ID           0x0427  /* Default device ID */

/******************************************************************************
 * Standard Commands
 ******************************************************************************/

#define BQ27427_COMMAND_CONTROL         0x00
#define BQ27427_COMMAND_TEMP            0x02
#define BQ27427_COMMAND_VOLTAGE         0x04
#define BQ27427_COMMAND_FLAGS           0x06
#define BQ27427_COMMAND_NOM_CAPACITY    0x08
#define BQ27427_COMMAND_AVAIL_CAPACITY  0x0A
#define BQ27427_COMMAND_REM_CAPACITY    0x0C
#define BQ27427_COMMAND_FULL_CAPACITY   0x0E
#define BQ27427_COMMAND_AVG_CURRENT     0x10
#define BQ27427_COMMAND_STDBY_CURRENT   0x12
#define BQ27427_COMMAND_MAX_CURRENT     0x14
#define BQ27427_COMMAND_AVG_POWER       0x18
#define BQ27427_COMMAND_SOC             0x1C
#define BQ27427_COMMAND_INT_TEMP        0x1E
#define BQ27427_COMMAND_SOH             0x20
#define BQ27427_COMMAND_REM_CAP_UNFL    0x28
#define BQ27427_COMMAND_REM_CAP_FIL     0x2A
#define BQ27427_COMMAND_FULL_CAP_UNFL   0x2C
#define BQ27427_COMMAND_FULL_CAP_FIL    0x2E
#define BQ27427_COMMAND_SOC_UNFL        0x30

/******************************************************************************
 * Control Sub-commands
 ******************************************************************************/

#define BQ27427_CONTROL_STATUS          0x00
#define BQ27427_CONTROL_DEVICE_TYPE     0x01
#define BQ27427_CONTROL_FW_VERSION      0x02
#define BQ27427_CONTROL_DM_CODE         0x04
#define BQ27427_CONTROL_PREV_MACWRITE   0x07
#define BQ27427_CONTROL_CHEM_ID         0x08
#define BQ27427_CONTROL_BAT_INSERT      0x0C
#define BQ27427_CONTROL_BAT_REMOVE      0x0D
#define BQ27427_CONTROL_SET_CFGUPDATE   0x13
#define BQ27427_CONTROL_SHUTDOWN_ENABLE 0x1B
#define BQ27427_CONTROL_SHUTDOWN        0x1C
#define BQ27427_CONTROL_SEALED          0x20
#define BQ27427_CONTROL_PULSE_SOC_INT   0x23
#define BQ27427_CONTROL_CHEM_A          0x30
#define BQ27427_CONTROL_CHEM_B          0x31
#define BQ27427_CONTROL_CHEM_C          0x32
#define BQ27427_CONTROL_RESET           0x41
#define BQ27427_CONTROL_SOFT_RESET      0x42

/******************************************************************************
 * Control Status Word - Bit Definitions
 ******************************************************************************/

#define BQ27427_STATUS_SHUTDOWNEN       (1 << 15)
#define BQ27427_STATUS_WDRESET          (1 << 14)
#define BQ27427_STATUS_SS               (1 << 13)
#define BQ27427_STATUS_CALMODE          (1 << 12)
#define BQ27427_STATUS_CCA              (1 << 11)
#define BQ27427_STATUS_BCA              (1 << 10)
#define BQ27427_STATUS_QMAX_UP          (1 << 9)
#define BQ27427_STATUS_RES_UP           (1 << 8)
#define BQ27427_STATUS_INITCOMP         (1 << 7)
#define BQ27427_STATUS_SLEEP            (1 << 4)
#define BQ27427_STATUS_LDMD             (1 << 3)
#define BQ27427_STATUS_RUP_DIS          (1 << 2)
#define BQ27427_STATUS_VOK              (1 << 1)

/******************************************************************************
 * Flag Command - Bit Definitions
 ******************************************************************************/

#define BQ27427_FLAG_OT                 (1 << 15)
#define BQ27427_FLAG_UT                 (1 << 14)
#define BQ27427_FLAG_FC                 (1 << 9)
#define BQ27427_FLAG_CHG                (1 << 8)
#define BQ27427_FLAG_OCVTAKEN           (1 << 7)
#define BQ27427_FLAG_DOD_CRRCT          (1 << 6)
#define BQ27427_FLAG_ITPOR              (1 << 5)
#define BQ27427_FLAG_CFGUPMODE          (1 << 4)
#define BQ27427_FLAG_BAT_DET            (1 << 3)
#define BQ27427_FLAG_SOC1               (1 << 2)
#define BQ27427_FLAG_SOCF               (1 << 1)
#define BQ27427_FLAG_DSG                (1 << 0)

/******************************************************************************
 * Extended Data Commands
 ******************************************************************************/

#define BQ27427_EXTENDED_DATACLASS      0x3E
#define BQ27427_EXTENDED_DATABLOCK      0x3F
#define BQ27427_EXTENDED_BLOCKDATA      0x40
#define BQ27427_EXTENDED_CHECKSUM       0x60
#define BQ27427_EXTENDED_CONTROL        0x61

/******************************************************************************
 * Configuration Class, Subclass ID's
 ******************************************************************************/

#define BQ27427_ID_SAFETY               2
#define BQ27427_ID_CHG_TERMINATION      36
#define BQ27427_ID_DISCHARGE            49
#define BQ27427_ID_REGISTERS            64
#define BQ27427_ID_IT_CFG               80
#define BQ27427_ID_CURRENT_THRESH       81
#define BQ27427_ID_STATE                82
#define BQ27427_ID_R_A_RAM              89
#define BQ27427_ID_CHEM_DATA            109
#define BQ27427_ID_CALIB_DATA           104
#define BQ27427_ID_CC_CAL               105
#define BQ27427_ID_CURRENT              107
#define BQ27427_ID_CODES                112

/******************************************************************************
 * OpConfig Register - Bit Definitions
 ******************************************************************************/

#define BQ27427_OPCONFIG_BIE            (1 << 13)
#define BQ27427_OPCONFIG_GPIOPOL        (1 << 11)
#define BQ27427_OPCONFIG_RS_FCT_STP     (1 << 6)
#define BQ27427_OPCONFIG_SLEEP          (1 << 5)
#define BQ27427_OPCONFIG_RMFCC          (1 << 4)
#define BQ27427_OPCONFIG_FASTCNV_EN     (1 << 3)
#define BQ27427_OPCONFIG_BATLOWEN       (1 << 2)
#define BQ27427_OPCONFIG_TEMPS          (1 << 0)

/******************************************************************************
 * Enumerations
 ******************************************************************************/

/* Chemistry profiles */
typedef enum {
    BQ27427_CHEM_A = BQ27427_CONTROL_CHEM_A,    /* 4.35V */
    BQ27427_CHEM_B = BQ27427_CONTROL_CHEM_B,    /* 4.2V */
    BQ27427_CHEM_C = BQ27427_CONTROL_CHEM_C     /* 4.4V */
} bq27427_chemistry_t;

/* Current measurement types */
typedef enum {
    BQ27427_CURRENT_AVG,    /* Average Current */
    BQ27427_CURRENT_STBY,   /* Standby Current */
    BQ27427_CURRENT_MAX     /* Max Current */
} bq27427_current_measure_t;

/* Capacity measurement types */
typedef enum {
    BQ27427_CAPACITY_REMAIN,        /* Remaining Capacity */
    BQ27427_CAPACITY_FULL,          /* Full Capacity */
    BQ27427_CAPACITY_AVAIL,         /* Available Capacity */
    BQ27427_CAPACITY_AVAIL_FULL,    /* Full Available Capacity */
    BQ27427_CAPACITY_REMAIN_F,      /* Remaining Capacity Filtered */
    BQ27427_CAPACITY_REMAIN_UF,     /* Remaining Capacity Unfiltered */
    BQ27427_CAPACITY_FULL_F,        /* Full Capacity Filtered */
    BQ27427_CAPACITY_FULL_UF,       /* Full Capacity Unfiltered */
    BQ27427_CAPACITY_DESIGN         /* Design Capacity */
} bq27427_capacity_measure_t;

/* State of charge measurement types */
typedef enum {
    BQ27427_SOC_FILTERED,       /* State of Charge Filtered */
    BQ27427_SOC_UNFILTERED      /* State of Charge Unfiltered */
} bq27427_soc_measure_t;

/* State of health measurement types */
typedef enum {
    BQ27427_SOH_PERCENT,        /* State of Health Percentage */
    BQ27427_SOH_STATUS          /* State of Health Status Bits */
} bq27427_soh_measure_t;

/* Temperature measurement types */
typedef enum {
    BQ27427_TEMP_INTERNAL       /* Internal IC Temperature */
} bq27427_temp_measure_t;

/* GPOUT function types */
typedef enum {
    BQ27427_GPOUT_SOC_INT,      /* SOC_INT functionality */
    BQ27427_GPOUT_BAT_LOW       /* BAT_LOW functionality */
} bq27427_gpout_function_t;

/******************************************************************************
 * Function Prototypes - Initialization
 ******************************************************************************/

/**
 * @brief Initialize the BQ27427 driver
 * @return true if communication was successful, false otherwise
 */
bool bq27427_init(void);

/**
 * @brief Set the design capacity of the connected battery
 * @param capacity Battery capacity in mAh
 * @return true on success
 */
bool bq27427_set_capacity(uint16_t capacity);

/**
 * @brief Read the design energy of the connected battery
 * @return Design energy in mWh
 */
uint16_t bq27427_design_energy(void);

/**
 * @brief Set the design energy of the connected battery
 * @param energy Battery energy in mWh
 * @return true on success
 */
bool bq27427_set_design_energy(uint16_t energy);

/**
 * @brief Read the terminate voltage of the connected battery
 * @return Terminate voltage in mV
 */
uint16_t bq27427_terminate_voltage(void);

/**
 * @brief Set the terminate voltage
 * @param voltage Terminate voltage in mV (2500-3700)
 * @return true on success
 */
bool bq27427_set_terminate_voltage(uint16_t voltage);

/**
 * @brief Read the discharge current threshold
 * @return Discharge current threshold in 0.1h units
 */
uint16_t bq27427_discharge_current_threshold(void);

/**
 * @brief Set the discharge current threshold
 * @param value Threshold in 0.1h units (0-2000)
 * @return true on success
 */
bool bq27427_set_discharge_current_threshold(uint16_t value);

/**
 * @brief Read the taper voltage
 * @return Taper voltage in mV
 */
uint16_t bq27427_taper_voltage(void);

/**
 * @brief Set the taper voltage
 * @param voltage Taper voltage in mV (0-5000)
 * @return true on success
 */
bool bq27427_set_taper_voltage(uint16_t voltage);

/**
 * @brief Read the taper rate
 * @return Taper rate in 0.1h units
 */
uint16_t bq27427_taper_rate(void);

/**
 * @brief Set the taper rate
 * @param rate Taper rate in 0.1h units (max 2000)
 * @return true on success
 */
bool bq27427_set_taper_rate(uint16_t rate);

/**
 * @brief Read the current polarity setting
 * @return true if positive polarity, false if negative
 */
bool bq27427_current_polarity(void);

/**
 * @brief Toggle the current polarity
 * @return true on success
 */
bool bq27427_change_current_polarity(void);

/******************************************************************************
 * Function Prototypes - Battery Characteristics
 ******************************************************************************/

/**
 * @brief Read the battery voltage
 * @return Battery voltage in mV
 */
uint16_t bq27427_voltage(void);

/**
 * @brief Read the specified current measurement
 * @param type Current measurement type
 * @return Current in mA (positive = charging)
 */
int16_t bq27427_current(bq27427_current_measure_t type);

/**
 * @brief Read the specified capacity measurement
 * @param type Capacity measurement type
 * @return Capacity in mAh
 */
uint16_t bq27427_capacity(bq27427_capacity_measure_t type);

/**
 * @brief Read the average power
 * @return Average power in mW (positive = charging)
 */
int16_t bq27427_power(void);

/**
 * @brief Read the state of charge
 * @param type SOC measurement type
 * @return State of charge in %
 */
uint16_t bq27427_soc(bq27427_soc_measure_t type);

/**
 * @brief Read the state of health
 * @param type SOH measurement type
 * @return State of health in % or status bits
 */
uint8_t bq27427_soh(bq27427_soh_measure_t type);

/**
 * @brief Read the internal temperature
 * @return Temperature in 0.1K units
 */
uint16_t bq27427_temperature(void);

/******************************************************************************
 * Function Prototypes - GPOUT Control
 ******************************************************************************/

/**
 * @brief Get GPOUT polarity setting
 * @return true if active-high, false if active-low
 */
bool bq27427_gpout_polarity(void);

/**
 * @brief Set GPOUT polarity
 * @param active_high true for active-high, false for active-low
 * @return true on success
 */
bool bq27427_set_gpout_polarity(bool active_high);

/**
 * @brief Get GPOUT function
 * @return true if BAT_LOW, false if SOC_INT
 */
bool bq27427_gpout_function(void);

/**
 * @brief Set GPOUT function
 * @param function GPOUT function type
 * @return true on success
 */
bool bq27427_set_gpout_function(bq27427_gpout_function_t function);

/**
 * @brief Get SOC1 set threshold
 * @return Threshold percentage (0-100)
 */
uint8_t bq27427_soc1_set_threshold(void);

/**
 * @brief Get SOC1 clear threshold
 * @return Threshold percentage (0-100)
 */
uint8_t bq27427_soc1_clear_threshold(void);

/**
 * @brief Set SOC1 thresholds
 * @param set Set threshold percentage
 * @param clear Clear threshold percentage
 * @return true on success
 */
bool bq27427_set_soc1_thresholds(uint8_t set, uint8_t clear);

/**
 * @brief Get SOCF set threshold
 * @return Threshold percentage (0-100)
 */
uint8_t bq27427_socf_set_threshold(void);

/**
 * @brief Get SOCF clear threshold
 * @return Threshold percentage (0-100)
 */
uint8_t bq27427_socf_clear_threshold(void);

/**
 * @brief Set SOCF thresholds
 * @param set Set threshold percentage
 * @param clear Clear threshold percentage
 * @return true on success
 */
bool bq27427_set_socf_thresholds(uint8_t set, uint8_t clear);

/**
 * @brief Check if SOC1 flag is set
 * @return true if flag is set
 */
bool bq27427_soc_flag(void);

/**
 * @brief Check if SOCF flag is set
 * @return true if flag is set
 */
bool bq27427_socf_flag(void);

/**
 * @brief Check if ITPOR flag is set
 * @return true if flag is set
 */
bool bq27427_itpor_flag(void);

/**
 * @brief Check if FC (full charge) flag is set
 * @return true if flag is set
 */
bool bq27427_fc_flag(void);

/**
 * @brief Check if CHG (charging) flag is set
 * @return true if flag is set
 */
bool bq27427_chg_flag(void);

/**
 * @brief Check if DSG (discharging) flag is set
 * @return true if flag is set
 */
bool bq27427_dsg_flag(void);

/**
 * @brief Get SOC_INT interval delta
 * @return Interval percentage (1-100)
 */
uint8_t bq27427_soci_delta(void);

/**
 * @brief Set SOC_INT interval delta
 * @param delta Interval percentage (1-100)
 * @return true on success
 */
bool bq27427_set_soci_delta(uint8_t delta);

/**
 * @brief Pulse the GPOUT pin (must be in SOC_INT mode)
 * @return true on success
 */
bool bq27427_pulse_gpout(void);

/**
 * @brief Initialize GPOUT GPIO pin for interrupt
 */
void bq27427_gpout_init(void);

/**
 * @brief Read the GPOUT pin state
 * @return GPIO pin state
 */
GPIO_PinState bq27427_gpout_read(void);

/******************************************************************************
 * Function Prototypes - Control Sub-commands
 ******************************************************************************/

/**
 * @brief Read the device type
 * @return Device type (should be 0x0427)
 */
uint16_t bq27427_device_type(void);

/**
 * @brief Set the chemistry profile
 * @param chem_id Chemistry profile
 * @return true on success
 */
bool bq27427_set_chem_id(bq27427_chemistry_t chem_id);

/**
 * @brief Read the chemistry profile
 * @return Chemistry profile
 */
bq27427_chemistry_t bq27427_chem_id(void);

/**
 * @brief Enter configuration mode
 * @param user_control true if user wants control over entering/exiting config
 * @return true on success
 */
bool bq27427_enter_config(bool user_control);

/**
 * @brief Exit configuration mode
 * @param user_control true if user had control
 * @return true on success
 */
bool bq27427_exit_config(bool user_control);

/**
 * @brief Read the flags register
 * @return 16-bit flags value
 */
uint16_t bq27427_flags(void);

/**
 * @brief Read the control status
 * @return 16-bit control status value
 */
uint16_t bq27427_status(void);

/**
 * @brief Issue a factory reset
 * @return true on success
 */
bool bq27427_reset(void);

#ifdef __cplusplus
}
#endif

#endif /* BQ27427_H */
