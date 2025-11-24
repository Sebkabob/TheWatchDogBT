/**
 * @file bq25186.h
 * @brief BQ25186 1-Cell, 1A I2C Linear Battery Charger Driver
 * @author Driver implementation based on datasheet SLUSF69A
 */

#ifndef BQ25186_H
#define BQ25186_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                             Device I2C Address                              */
/* ========================================================================== */
#define BQ25186_I2C_ADDR                    0x6A  // 7-bit address

/* ========================================================================== */
/*                             Register Addresses                              */
/* ========================================================================== */
#define BQ25186_REG_STAT0                   0x00
#define BQ25186_REG_STAT1                   0x01
#define BQ25186_REG_FLAG0                   0x02
#define BQ25186_REG_VBAT_CTRL               0x03
#define BQ25186_REG_ICHG_CTRL               0x04
#define BQ25186_REG_CHARGECTRL0             0x05
#define BQ25186_REG_CHARGECTRL1             0x06
#define BQ25186_REG_IC_CTRL                 0x07
#define BQ25186_REG_TMR_ILIM                0x08
#define BQ25186_REG_SHIP_RST                0x09
#define BQ25186_REG_SYS_REG                 0x0A
#define BQ25186_REG_TS_CONTROL              0x0B
#define BQ25186_REG_MASK_ID                 0x0C

/* ========================================================================== */
/*                             Register Bit Masks                              */
/* ========================================================================== */

/* STAT0 Register (0x00) */
#define BQ25186_STAT0_TS_OPEN_STAT          (1 << 7)
#define BQ25186_STAT0_CHG_STAT_MASK         (3 << 5)
#define BQ25186_STAT0_CHG_STAT_SHIFT        5
#define BQ25186_STAT0_ILIM_ACTIVE_STAT      (1 << 4)
#define BQ25186_STAT0_VDPPM_ACTIVE_STAT     (1 << 3)
#define BQ25186_STAT0_VINDPM_ACTIVE_STAT    (1 << 2)
#define BQ25186_STAT0_THERMREG_ACTIVE_STAT  (1 << 1)
#define BQ25186_STAT0_VIN_PGOOD_STAT        (1 << 0)

/* STAT1 Register (0x01) */
#define BQ25186_STAT1_VIN_OVP_STAT          (1 << 7)
#define BQ25186_STAT1_BUVLO_STAT            (1 << 6)
#define BQ25186_STAT1_TS_STAT_MASK          (3 << 3)
#define BQ25186_STAT1_TS_STAT_SHIFT         3
#define BQ25186_STAT1_SAFETY_TMR_FAULT      (1 << 2)
#define BQ25186_STAT1_WAKE1_FLAG            (1 << 1)
#define BQ25186_STAT1_WAKE2_FLAG            (1 << 0)

/* FLAG0 Register (0x02) */
#define BQ25186_FLAG0_TS_FAULT              (1 << 7)
#define BQ25186_FLAG0_ILIM_ACTIVE_FLAG      (1 << 6)
#define BQ25186_FLAG0_VDPPM_ACTIVE_FLAG     (1 << 5)
#define BQ25186_FLAG0_VINDPM_ACTIVE_FLAG    (1 << 4)
#define BQ25186_FLAG0_THERMREG_ACTIVE_FLAG  (1 << 3)
#define BQ25186_FLAG0_VIN_OVP_FAULT_FLAG    (1 << 2)
#define BQ25186_FLAG0_BUVLO_FAULT_FLAG      (1 << 1)
#define BQ25186_FLAG0_BAT_OCP_FAULT         (1 << 0)

/* VBAT_CTRL Register (0x03) */
#define BQ25186_VBAT_CTRL_PG_MODE           (1 << 7)
#define BQ25186_VBAT_CTRL_VBATREG_MASK      0x7F
#define BQ25186_VBAT_CTRL_VBATREG_SHIFT     0

/* ICHG_CTRL Register (0x04) */
#define BQ25186_ICHG_CTRL_CHG_DIS           (1 << 7)
#define BQ25186_ICHG_CTRL_ICHG_MASK         0x7F
#define BQ25186_ICHG_CTRL_ICHG_SHIFT        0

/* CHARGECTRL0 Register (0x05) */
#define BQ25186_CHARGECTRL0_EN_FC_MODE      (1 << 7)
#define BQ25186_CHARGECTRL0_IPRECHG         (1 << 6)
#define BQ25186_CHARGECTRL0_ITERM_MASK      (3 << 4)
#define BQ25186_CHARGECTRL0_ITERM_SHIFT     4
#define BQ25186_CHARGECTRL0_VINDPM_MASK     (3 << 2)
#define BQ25186_CHARGECTRL0_VINDPM_SHIFT    2
#define BQ25186_CHARGECTRL0_THERM_REG_MASK  (3 << 0)
#define BQ25186_CHARGECTRL0_THERM_REG_SHIFT 0

/* CHARGECTRL1 Register (0x06) */
#define BQ25186_CHARGECTRL1_IBAT_OCP_MASK       (3 << 6)
#define BQ25186_CHARGECTRL1_IBAT_OCP_SHIFT      6
#define BQ25186_CHARGECTRL1_BUVLO_MASK          (7 << 3)
#define BQ25186_CHARGECTRL1_BUVLO_SHIFT         3
#define BQ25186_CHARGECTRL1_CHG_STATUS_INT_MASK (1 << 2)
#define BQ25186_CHARGECTRL1_ILIM_INT_MASK       (1 << 1)
#define BQ25186_CHARGECTRL1_VINDPM_INT_MASK     (1 << 0)

/* IC_CTRL Register (0x07) */
#define BQ25186_IC_CTRL_TS_EN               (1 << 7)
#define BQ25186_IC_CTRL_VLOWV_SEL           (1 << 6)
#define BQ25186_IC_CTRL_VRCH_0              (1 << 5)
#define BQ25186_IC_CTRL_2XTMR_EN            (1 << 4)
#define BQ25186_IC_CTRL_SAFETY_TIMER_MASK   (3 << 2)
#define BQ25186_IC_CTRL_SAFETY_TIMER_SHIFT  2
#define BQ25186_IC_CTRL_WATCHDOG_SEL_MASK   (3 << 0)
#define BQ25186_IC_CTRL_WATCHDOG_SEL_SHIFT  0

/* TMR_ILIM Register (0x08) */
#define BQ25186_TMR_ILIM_MR_LPRESS_MASK     (3 << 6)
#define BQ25186_TMR_ILIM_MR_LPRESS_SHIFT    6
#define BQ25186_TMR_ILIM_MR_RESET_VIN       (1 << 5)
#define BQ25186_TMR_ILIM_AUTOWAKE_MASK      (3 << 3)
#define BQ25186_TMR_ILIM_AUTOWAKE_SHIFT     3
#define BQ25186_TMR_ILIM_ILIM_MASK          (7 << 0)
#define BQ25186_TMR_ILIM_ILIM_SHIFT         0

/* SHIP_RST Register (0x09) */
#define BQ25186_SHIP_RST_REG_RST            (1 << 7)
#define BQ25186_SHIP_RST_EN_RST_SHIP_MASK   (3 << 5)
#define BQ25186_SHIP_RST_EN_RST_SHIP_SHIFT  5
#define BQ25186_SHIP_RST_PB_LPRESS_ACTION_MASK  (3 << 3)
#define BQ25186_SHIP_RST_PB_LPRESS_ACTION_SHIFT 3
#define BQ25186_SHIP_RST_WAKE1_TMR          (1 << 2)
#define BQ25186_SHIP_RST_WAKE2_TMR          (1 << 1)
#define BQ25186_SHIP_RST_EN_PUSH            (1 << 0)

/* SYS_REG Register (0x0A) */
#define BQ25186_SYS_REG_SYS_REG_CTRL_MASK       (7 << 5)
#define BQ25186_SYS_REG_SYS_REG_CTRL_SHIFT      5
#define BQ25186_SYS_REG_PG_GPO                  (1 << 4)
#define BQ25186_SYS_REG_SYS_MODE_MASK           (3 << 2)
#define BQ25186_SYS_REG_SYS_MODE_SHIFT          2
#define BQ25186_SYS_REG_WATCHDOG_15S_ENABLE     (1 << 1)
#define BQ25186_SYS_REG_VDPPM_DIS               (1 << 0)

/* TS_CONTROL Register (0x0B) */
#define BQ25186_TS_CONTROL_TS_HOT_MASK      (3 << 6)
#define BQ25186_TS_CONTROL_TS_HOT_SHIFT     6
#define BQ25186_TS_CONTROL_TS_COLD_MASK     (3 << 4)
#define BQ25186_TS_CONTROL_TS_COLD_SHIFT    4
#define BQ25186_TS_CONTROL_TS_WARM          (1 << 3)
#define BQ25186_TS_CONTROL_TS_COOL          (1 << 2)
#define BQ25186_TS_CONTROL_TS_ICHG          (1 << 1)
#define BQ25186_TS_CONTROL_TS_VRCG          (1 << 0)

/* MASK_ID Register (0x0C) */
#define BQ25186_MASK_ID_TS_INT_MASK         (1 << 7)
#define BQ25186_MASK_ID_TREG_INT_MASK       (1 << 6)
#define BQ25186_MASK_ID_BAT_INT_MASK        (1 << 5)
#define BQ25186_MASK_ID_PG_INT_MASK         (1 << 4)
#define BQ25186_MASK_ID_DEVICE_ID_MASK      0x0F
#define BQ25186_MASK_ID_DEVICE_ID_SHIFT     0

/* ========================================================================== */
/*                             Enumeration Types                               */
/* ========================================================================== */

/**
 * @brief Charging Status
 */
typedef enum {
    BQ25186_CHG_STAT_NOT_CHARGING = 0,  // Not charging while charging is enabled
    BQ25186_CHG_STAT_CC_MODE = 1,       // Constant Current (trickle/pre/fast charge)
    BQ25186_CHG_STAT_CV_MODE = 2,       // Constant Voltage
    BQ25186_CHG_STAT_DONE = 3           // Charge done or charging disabled
} BQ25186_ChgStat_t;

/**
 * @brief TS (Temperature Sensor) Status
 */
typedef enum {
    BQ25186_TS_STAT_NORMAL = 0,         // Normal temperature range
    BQ25186_TS_STAT_HOT_COLD = 1,       // VTS < VHOT or VTS > VCOLD (suspended)
    BQ25186_TS_STAT_COOL = 2,           // VCOOL < VTS < VCOLD (current reduced)
    BQ25186_TS_STAT_WARM = 3            // VWARM > VTS > VHOT (voltage reduced)
} BQ25186_TsStat_t;

/**
 * @brief Battery Regulation Voltage Range: 3.5V to 4.65V in 10mV steps
 * VBATREG = 3.5V + (code * 10mV), max 4.65V
 */
typedef enum {
    BQ25186_VBATREG_3500MV = 0,         // 3.5V
    BQ25186_VBATREG_3600MV = 10,        // 3.6V
    BQ25186_VBATREG_3700MV = 20,        // 3.7V
    BQ25186_VBATREG_3800MV = 30,        // 3.8V
    BQ25186_VBATREG_3900MV = 40,        // 3.9V
    BQ25186_VBATREG_4000MV = 50,        // 4.0V
    BQ25186_VBATREG_4100MV = 60,        // 4.1V
    BQ25186_VBATREG_4200MV = 70,        // 4.2V (default)
    BQ25186_VBATREG_4300MV = 80,        // 4.3V
    BQ25186_VBATREG_4350MV = 85,        // 4.35V
    BQ25186_VBATREG_4400MV = 90,        // 4.4V
    BQ25186_VBATREG_4500MV = 100,       // 4.5V
    BQ25186_VBATREG_4650MV = 115        // 4.65V (max)
} BQ25186_VbatReg_t;

/**
 * @brief Termination Current (as percentage of ICHG)
 */
typedef enum {
    BQ25186_ITERM_DISABLED = 0,         // Termination disabled
    BQ25186_ITERM_5_PERCENT = 1,        // 5% of ICHG
    BQ25186_ITERM_10_PERCENT = 2,       // 10% of ICHG (default)
    BQ25186_ITERM_20_PERCENT = 3        // 20% of ICHG
} BQ25186_Iterm_t;

/**
 * @brief VINDPM Level Selection
 */
typedef enum {
    BQ25186_VINDPM_VBAT_PLUS_300MV = 0, // VBAT + 300mV (Battery Tracking)
    BQ25186_VINDPM_4500MV = 1,          // 4.5V (default)
    BQ25186_VINDPM_4700MV = 2,          // 4.7V
    BQ25186_VINDPM_DISABLED = 3         // Disabled
} BQ25186_Vindpm_t;

/**
 * @brief Thermal Regulation Threshold
 */
typedef enum {
    BQ25186_TREG_100C = 0,              // 100°C (default)
    BQ25186_TREG_80C = 1,               // 80°C
    BQ25186_TREG_60C = 2,               // 60°C
    BQ25186_TREG_DISABLED = 3           // Disabled
} BQ25186_ThermalReg_t;

/**
 * @brief Battery Undervoltage Lockout (BUVLO) Threshold
 */
typedef enum {
    BQ25186_BUVLO_3000MV = 0,           // 3.0V
    BQ25186_BUVLO_2800MV = 3,           // 2.8V (default)
    BQ25186_BUVLO_2600MV = 4,           // 2.6V
    BQ25186_BUVLO_2400MV = 5,           // 2.4V
    BQ25186_BUVLO_2200MV = 6,           // 2.2V
    BQ25186_BUVLO_2000MV = 7            // 2.0V
} BQ25186_Buvlo_t;

/**
 * @brief Battery Discharge Current Limit (BATOCP)
 */
typedef enum {
    BQ25186_BATOCP_500MA = 0,           // 500mA
    BQ25186_BATOCP_1000MA = 1,          // 1000mA (1A) (default)
    BQ25186_BATOCP_1500MA = 2,          // 1500mA (1.5A)
    BQ25186_BATOCP_3000MA = 3           // 3000mA (3A)
} BQ25186_BatOcp_t;

/**
 * @brief Precharge Voltage Threshold (VLOWV)
 */
typedef enum {
    BQ25186_VLOWV_3000MV = 0,           // 3.0V (default)
    BQ25186_VLOWV_2800MV = 1            // 2.8V
} BQ25186_Vlowv_t;

/**
 * @brief Recharge Voltage Threshold
 */
typedef enum {
    BQ25186_VRCH_100MV = 0,             // 100mV below VBATREG (default)
    BQ25186_VRCH_200MV = 1              // 200mV below VBATREG
} BQ25186_Vrch_t;

/**
 * @brief Fast Charge Safety Timer
 */
typedef enum {
    BQ25186_SAFETY_TIMER_3HRS = 0,      // 3 hours
    BQ25186_SAFETY_TIMER_6HRS = 1,      // 6 hours (default)
    BQ25186_SAFETY_TIMER_12HRS = 2,     // 12 hours
    BQ25186_SAFETY_TIMER_DISABLED = 3   // Disabled
} BQ25186_SafetyTimer_t;

/**
 * @brief Watchdog Timer Selection
 */
typedef enum {
    BQ25186_WATCHDOG_160S_REG_RST = 0,  // 160s software reset (default)
    BQ25186_WATCHDOG_160S_HW_RST = 1,   // 160s hardware reset
    BQ25186_WATCHDOG_40S_HW_RST = 2,    // 40s hardware reset
    BQ25186_WATCHDOG_DISABLED = 3       // Disabled
} BQ25186_Watchdog_t;

/**
 * @brief Input Current Limit (ILIM)
 */
typedef enum {
    BQ25186_ILIM_50MA = 0,              // 50mA
    BQ25186_ILIM_100MA = 1,             // 100mA
    BQ25186_ILIM_200MA = 2,             // 200mA
    BQ25186_ILIM_300MA = 3,             // 300mA
    BQ25186_ILIM_400MA = 4,             // 400mA
    BQ25186_ILIM_500MA = 5,             // 500mA (default)
    BQ25186_ILIM_665MA = 6,             // 665mA
    BQ25186_ILIM_1050MA = 7             // 1050mA
} BQ25186_Ilim_t;

/**
 * @brief Push Button Long Press Timer
 */
typedef enum {
    BQ25186_MR_LPRESS_5S = 0,           // 5 seconds
    BQ25186_MR_LPRESS_10S = 1,          // 10 seconds (default)
    BQ25186_MR_LPRESS_15S = 2,          // 15 seconds
    BQ25186_MR_LPRESS_20S = 3           // 20 seconds
} BQ25186_MrLongPress_t;

/**
 * @brief Auto Wake-Up Timer (after hardware reset)
 */
typedef enum {
    BQ25186_AUTOWAKE_500MS = 0,         // 0.5 seconds
    BQ25186_AUTOWAKE_1S = 1,            // 1 second (default)
    BQ25186_AUTOWAKE_2S = 2,            // 2 seconds
    BQ25186_AUTOWAKE_4S = 3             // 4 seconds
} BQ25186_AutoWake_t;

/**
 * @brief Shipmode/Reset Enable
 */
typedef enum {
    BQ25186_EN_RST_SHIP_DO_NOTHING = 0, // Do nothing (default)
    BQ25186_EN_RST_SHIP_SHUTDOWN = 1,   // Enable shutdown mode
    BQ25186_EN_RST_SHIP_SHIPMODE = 2,   // Enable ship mode
    BQ25186_EN_RST_SHIP_HW_RESET = 3    // Hardware reset
} BQ25186_EnRstShip_t;

/**
 * @brief Push Button Long Press Action
 */
typedef enum {
    BQ25186_PB_LPRESS_DO_NOTHING = 0,   // Do nothing (default)
    BQ25186_PB_LPRESS_HW_RESET = 1,     // Hardware reset
    BQ25186_PB_LPRESS_SHIPMODE = 2,     // Enable ship mode (default)
    BQ25186_PB_LPRESS_SHUTDOWN = 3      // Enable shutdown mode
} BQ25186_PbLpressAction_t;

/**
 * @brief SYS Regulation Control
 */
typedef enum {
    BQ25186_SYS_REG_BATTERY_TRACKING = 0, // VBAT + 225mV (3.8V min)
    BQ25186_SYS_REG_4400MV = 1,           // 4.4V
    BQ25186_SYS_REG_4500MV = 2,           // 4.5V (default)
    BQ25186_SYS_REG_4600MV = 3,           // 4.6V
    BQ25186_SYS_REG_4700MV = 4,           // 4.7V
    BQ25186_SYS_REG_4800MV = 5,           // 4.8V
    BQ25186_SYS_REG_4900MV = 6,           // 4.9V
    BQ25186_SYS_REG_PASSTHROUGH = 7       // Passthrough (up to 5.5V)
} BQ25186_SysReg_t;

/**
 * @brief SYS Power Mode
 */
typedef enum {
    BQ25186_SYS_MODE_NORMAL = 0,        // VIN if present, else VBAT (default)
    BQ25186_SYS_MODE_BAT_ONLY = 1,      // VBAT only (USB Suspend mode)
    BQ25186_SYS_MODE_OFF_FLOATING = 2,  // SYS disconnected, floating
    BQ25186_SYS_MODE_OFF_PULLDOWN = 3   // SYS disconnected, pulled down
} BQ25186_SysMode_t;

/**
 * @brief TS Hot Threshold
 */
typedef enum {
    BQ25186_TS_HOT_60C = 0,             // 60°C (default)
    BQ25186_TS_HOT_65C = 1,             // 65°C
    BQ25186_TS_HOT_50C = 2,             // 50°C
    BQ25186_TS_HOT_45C = 3              // 45°C
} BQ25186_TsHot_t;

/**
 * @brief TS Cold Threshold
 */
typedef enum {
    BQ25186_TS_COLD_0C = 0,             // 0°C (default)
    BQ25186_TS_COLD_3C = 1,             // 3°C
    BQ25186_TS_COLD_5C = 2,             // 5°C
    BQ25186_TS_COLD_NEG3C = 3           // -3°C
} BQ25186_TsCold_t;

/**
 * @brief Error codes
 */
typedef enum {
    BQ25186_OK = 0,                     // Success
    BQ25186_ERROR_I2C = -1,             // I2C communication error
    BQ25186_ERROR_INVALID_PARAM = -2,   // Invalid parameter
    BQ25186_ERROR_TIMEOUT = -3,         // Operation timeout
    BQ25186_ERROR_DEVICE_ID = -4        // Device ID mismatch
} BQ25186_Status_t;

/* ========================================================================== */
/*                             Structure Types                                 */
/* ========================================================================== */

/**
 * @brief Device status structure
 */
typedef struct {
    bool ts_open;                       // TS pin open status
    BQ25186_ChgStat_t chg_stat;        // Charging status
    bool ilim_active;                   // Input current limit active
    bool vdppm_active;                  // VDPPM active
    bool vindpm_active;                 // VINDPM active
    bool thermreg_active;               // Thermal regulation active
    bool vin_pgood;                     // VIN power good status
    bool vin_ovp;                       // VIN overvoltage status
    bool buvlo;                         // Battery UVLO status
    BQ25186_TsStat_t ts_stat;          // TS status
    bool safety_timer_fault;            // Safety timer fault
    bool wake1_flag;                    // Wake1 flag
    bool wake2_flag;                    // Wake2 flag
} BQ25186_Status_Regs_t;

/**
 * @brief Fault flags structure
 */
typedef struct {
    bool ts_fault;                      // TS fault
    bool ilim_active;                   // ILIM active flag
    bool vdppm_active;                  // VDPPM active flag
    bool vindpm_active;                 // VINDPM active flag
    bool thermreg_active;               // Thermal regulation active flag
    bool vin_ovp_fault;                 // VIN OVP fault
    bool buvlo_fault;                   // Battery UVLO fault
    bool bat_ocp_fault;                 // Battery OCP fault
} BQ25186_Faults_t;

/**
 * @brief Charge configuration structure
 */
typedef struct {
    uint16_t vbat_reg_mv;               // Battery regulation voltage (mV)
    uint16_t ichg_ma;                   // Fast charge current (mA)
    BQ25186_Iterm_t iterm;             // Termination current
    bool iprechg_1x;                    // Pre-charge = 1x term (true) or 2x term (false)
    BQ25186_Vindpm_t vindpm;           // VINDPM setting
    BQ25186_ThermalReg_t thermal_reg;  // Thermal regulation threshold
    BQ25186_Buvlo_t buvlo;             // Battery UVLO threshold
    BQ25186_Vlowv_t vlowv;             // Pre-charge voltage threshold
    BQ25186_Vrch_t vrch;               // Recharge threshold
    BQ25186_SafetyTimer_t safety_timer;// Safety timer duration
    bool timer_2x_en;                   // 2x timer enable (slow timer in DPM)
    BQ25186_Watchdog_t watchdog;       // Watchdog timer setting
    BQ25186_Ilim_t ilim;               // Input current limit
    bool ts_enable;                     // TS function enable
    bool charge_enable;                 // Charge enable
} BQ25186_ChargeConfig_t;

/**
 * @brief I2C function pointer types (user must implement)
 */
typedef int (*BQ25186_I2C_Write_t)(uint8_t dev_addr, uint8_t reg_addr,
                                    uint8_t *data, uint16_t len);
typedef int (*BQ25186_I2C_Read_t)(uint8_t dev_addr, uint8_t reg_addr,
                                   uint8_t *data, uint16_t len);
typedef void (*BQ25186_Delay_Ms_t)(uint32_t ms);

/**
 * @brief Device handle structure
 */
typedef struct {
    BQ25186_I2C_Write_t i2c_write;     // I2C write function
    BQ25186_I2C_Read_t i2c_read;       // I2C read function
    BQ25186_Delay_Ms_t delay_ms;       // Delay function
    uint8_t device_id;                  // Device ID (read from chip)
} BQ25186_Handle_t;

/* ========================================================================== */
/*                          Function Declarations                              */
/* ========================================================================== */

/**
 * @brief Initialize the BQ25186 device
 *
 * @param handle Pointer to device handle
 * @param i2c_write I2C write function pointer
 * @param i2c_read I2C read function pointer
 * @param delay_ms Delay function pointer
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_Init(BQ25186_Handle_t *handle,
                               BQ25186_I2C_Write_t i2c_write,
                               BQ25186_I2C_Read_t i2c_read,
                               BQ25186_Delay_Ms_t delay_ms);

/**
 * @brief Read device ID
 *
 * @param handle Pointer to device handle
 * @param device_id Pointer to store device ID
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_GetDeviceID(BQ25186_Handle_t *handle, uint8_t *device_id);

/**
 * @brief Perform software reset
 *
 * @param handle Pointer to device handle
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SoftwareReset(BQ25186_Handle_t *handle);

/**
 * @brief Read all status registers
 *
 * @param handle Pointer to device handle
 * @param status Pointer to status structure
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_GetStatus(BQ25186_Handle_t *handle,
                                    BQ25186_Status_Regs_t *status);

/**
 * @brief Read all fault flags
 *
 * @param handle Pointer to device handle
 * @param faults Pointer to faults structure
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_GetFaults(BQ25186_Handle_t *handle,
                                    BQ25186_Faults_t *faults);

/**
 * @brief Clear fault flags (read to clear)
 *
 * @param handle Pointer to device handle
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_ClearFaults(BQ25186_Handle_t *handle);

/**
 * @brief Set battery regulation voltage
 *
 * @param handle Pointer to device handle
 * @param voltage_mv Battery regulation voltage in mV (3500-4650mV, 10mV steps)
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetBatteryVoltage(BQ25186_Handle_t *handle,
                                            uint16_t voltage_mv);

/**
 * @brief Set fast charge current
 *
 * @param handle Pointer to device handle
 * @param current_ma Charge current in mA (5-1000mA)
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetChargeCurrent(BQ25186_Handle_t *handle,
                                           uint16_t current_ma);

/**
 * @brief Get fast charge current
 *
 * @param handle Pointer to device handle
 * @param current_ma Pointer to store current in mA
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_GetChargeCurrent(BQ25186_Handle_t *handle,
                                           uint16_t *current_ma);

/**
 * @brief Enable or disable charging
 *
 * @param handle Pointer to device handle
 * @param enable true to enable, false to disable
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetChargeEnable(BQ25186_Handle_t *handle, bool enable);

/**
 * @brief Get charge enable status
 *
 * @param handle Pointer to device handle
 * @param enabled Pointer to store enable status
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_GetChargeEnable(BQ25186_Handle_t *handle, bool *enabled);

/**
 * @brief Set termination current
 *
 * @param handle Pointer to device handle
 * @param iterm Termination current setting
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTerminationCurrent(BQ25186_Handle_t *handle,
                                                BQ25186_Iterm_t iterm);

/**
 * @brief Set VINDPM threshold
 *
 * @param handle Pointer to device handle
 * @param vindpm VINDPM setting
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetVINDPM(BQ25186_Handle_t *handle,
                                    BQ25186_Vindpm_t vindpm);

/**
 * @brief Set thermal regulation threshold
 *
 * @param handle Pointer to device handle
 * @param thermal_reg Thermal regulation threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetThermalRegulation(BQ25186_Handle_t *handle,
                                               BQ25186_ThermalReg_t thermal_reg);

/**
 * @brief Set battery undervoltage lockout threshold
 *
 * @param handle Pointer to device handle
 * @param buvlo BUVLO threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetBUVLO(BQ25186_Handle_t *handle,
                                   BQ25186_Buvlo_t buvlo);

/**
 * @brief Set battery discharge overcurrent protection limit
 *
 * @param handle Pointer to device handle
 * @param bat_ocp Battery OCP limit
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetBatteryOCP(BQ25186_Handle_t *handle,
                                        BQ25186_BatOcp_t bat_ocp);

/**
 * @brief Set precharge voltage threshold
 *
 * @param handle Pointer to device handle
 * @param vlowv Pre-charge voltage threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetPrechargeVoltage(BQ25186_Handle_t *handle,
                                              BQ25186_Vlowv_t vlowv);

/**
 * @brief Set recharge voltage threshold
 *
 * @param handle Pointer to device handle
 * @param vrch Recharge threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetRechargeThreshold(BQ25186_Handle_t *handle,
                                               BQ25186_Vrch_t vrch);

/**
 * @brief Set safety timer duration
 *
 * @param handle Pointer to device handle
 * @param timer Safety timer duration
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetSafetyTimer(BQ25186_Handle_t *handle,
                                        BQ25186_SafetyTimer_t timer);

/**
 * @brief Enable or disable 2x timer (slows timer in DPM/VINDPM)
 *
 * @param handle Pointer to device handle
 * @param enable true to enable 2x timer
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_Set2xTimerEnable(BQ25186_Handle_t *handle, bool enable);

/**
 * @brief Set watchdog timer
 *
 * @param handle Pointer to device handle
 * @param Watchdog timer setting
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetWatchdog(BQ25186_Handle_t *handle,
                                      BQ25186_Watchdog_t watchdog);

/**
 * @brief Reset watchdog timer (pet the watchdog)
 *
 * @param handle Pointer to device handle
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_ResetWatchdog(BQ25186_Handle_t *handle);

/**
 * @brief Set input current limit
 *
 * @param handle Pointer to device handle
 * @param ilim Input current limit
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetInputCurrentLimit(BQ25186_Handle_t *handle,
                                               BQ25186_Ilim_t ilim);

/**
 * @brief Set push button long press timer
 *
 * @param handle Pointer to device handle
 * @param timer Long press timer duration
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetLongPressTimer(BQ25186_Handle_t *handle,
                                            BQ25186_MrLongPress_t timer);

/**
 * @brief Set auto wake-up timer (after hardware reset)
 *
 * @param handle Pointer to device handle
 * @param timer Auto wake timer duration
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetAutoWakeTimer(BQ25186_Handle_t *handle,
                                           BQ25186_AutoWake_t timer);

/**
 * @brief Enter ship mode or shutdown mode
 *
 * @param handle Pointer to device handle
 * @param mode Ship/shutdown/reset mode
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetPowerMode(BQ25186_Handle_t *handle,
                                       BQ25186_EnRstShip_t mode);

/**
 * @brief Set push button long press action
 *
 * @param handle Pointer to device handle
 * @param action Action to perform on long press
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetLongPressAction(BQ25186_Handle_t *handle,
                                             BQ25186_PbLpressAction_t action);

/**
 * @brief Enable or disable push button functionality
 *
 * @param handle Pointer to device handle
 * @param enable true to enable push button
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetPushButtonEnable(BQ25186_Handle_t *handle,
                                              bool enable);

/**
 * @brief Set SYS regulation voltage
 *
 * @param handle Pointer to device handle
 * @param sys_reg SYS regulation setting
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetSysRegulation(BQ25186_Handle_t *handle,
                                           BQ25186_SysReg_t sys_reg);

/**
 * @brief Set SYS power mode
 *
 * @param handle Pointer to device handle
 * @param mode SYS power mode
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetSysMode(BQ25186_Handle_t *handle,
                                     BQ25186_SysMode_t mode);

/**
 * @brief Enable or disable 15-second I2C watchdog (HW reset if no I2C)
 *
 * @param handle Pointer to device handle
 * @param enable true to enable 15s watchdog
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_Set15sWatchdogEnable(BQ25186_Handle_t *handle,
                                               bool enable);

/**
 * @brief Enable or disable VDPPM
 *
 * @param handle Pointer to device handle
 * @param enable true to enable VDPPM
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetVDPPMEnable(BQ25186_Handle_t *handle, bool enable);

/**
 * @brief Enable or disable TS (temperature sensor) function
 *
 * @param handle Pointer to device handle
 * @param enable true to enable TS
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTSEnable(BQ25186_Handle_t *handle, bool enable);

/**
 * @brief Set TS hot threshold
 *
 * @param handle Pointer to device handle
 * @param hot_threshold TS hot threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTSHotThreshold(BQ25186_Handle_t *handle,
                                            BQ25186_TsHot_t hot_threshold);

/**
 * @brief Set TS cold threshold
 *
 * @param handle Pointer to device handle
 * @param cold_threshold TS cold threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTSColdThreshold(BQ25186_Handle_t *handle,
                                             BQ25186_TsCold_t cold_threshold);

/**
 * @brief Enable or disable TS warm threshold
 *
 * @param handle Pointer to device handle
 * @param enable true to enable warm threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTSWarmEnable(BQ25186_Handle_t *handle, bool enable);

/**
 * @brief Enable or disable TS cool threshold
 *
 * @param handle Pointer to device handle
 * @param enable true to enable cool threshold
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTSCoolEnable(BQ25186_Handle_t *handle, bool enable);

/**
 * @brief Set TS charge current reduction (during cool)
 *
 * @param handle Pointer to device handle
 * @param reduce_to_20_percent true for 0.2x ICHG, false for 0.5x ICHG
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTSCurrentReduction(BQ25186_Handle_t *handle,
                                                bool reduce_to_20_percent);

/**
 * @brief Set TS voltage reduction (during warm)
 *
 * @param handle Pointer to device handle
 * @param reduce_200mv true for -200mV, false for -100mV
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetTSVoltageReduction(BQ25186_Handle_t *handle,
                                                bool reduce_200mv);

/**
 * @brief Set PG/GPO pin mode
 *
 * @param handle Pointer to device handle
 * @param gpo_mode true for GPO mode, false for PG mode
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetPGMode(BQ25186_Handle_t *handle, bool gpo_mode);

/**
 * @brief Set GPO pin state (only when in GPO mode)
 *
 * @param handle Pointer to device handle
 * @param state true for low, false for high-Z
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetGPOState(BQ25186_Handle_t *handle, bool state);

/**
 * @brief Set interrupt masks
 *
 * @param handle Pointer to device handle
 * @param mask_ts Mask TS interrupt
 * @param mask_treg Mask thermal regulation interrupt
 * @param mask_bat Mask battery interrupt (OCP/UVLO)
 * @param mask_pg Mask power good interrupt
 * @param mask_ilim Mask ILIM interrupt
 * @param mask_vindpm Mask VINDPM/VDPPM interrupt
 * @param mask_chg_status Mask charging status change interrupt
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_SetInterruptMasks(BQ25186_Handle_t *handle,
                                            bool mask_ts, bool mask_treg,
                                            bool mask_bat, bool mask_pg,
                                            bool mask_ilim, bool mask_vindpm,
                                            bool mask_chg_status);

/**
 * @brief Apply a complete charge configuration
 *
 * @param handle Pointer to device handle
 * @param config Pointer to charge configuration structure
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_ApplyChargeConfig(BQ25186_Handle_t *handle,
                                            const BQ25186_ChargeConfig_t *config);

/**
 * @brief Read a single register
 *
 * @param handle Pointer to device handle
 * @param reg_addr Register address
 * @param data Pointer to store register value
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_ReadRegister(BQ25186_Handle_t *handle,
                                       uint8_t reg_addr, uint8_t *data);

/**
 * @brief Write a single register
 *
 * @param handle Pointer to device handle
 * @param reg_addr Register address
 * @param data Register value to write
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_WriteRegister(BQ25186_Handle_t *handle,
                                        uint8_t reg_addr, uint8_t data);

/**
 * @brief Modify bits in a register (read-modify-write)
 *
 * @param handle Pointer to device handle
 * @param reg_addr Register address
 * @param mask Bit mask
 * @param value Value to write (after masking)
 * @return BQ25186_Status_t Status code
 */
BQ25186_Status_t BQ25186_ModifyRegister(BQ25186_Handle_t *handle,
                                         uint8_t reg_addr,
                                         uint8_t mask, uint8_t value);

#ifdef __cplusplus
}
#endif

#endif /* BQ25186_H */
