#ifndef __BQ27427_H
#define __BQ27427_H
#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define HAL_I2C_INSTANCE                    hi2c1
#define HAL_BQ27427_TIMEOUT                 5
#define BQ27427_DELAY                       1

#define BQ27427_I2C_ADDRESS                 0xAA

#define BQ27427_CONTROL_LOW                 0x00
#define BQ27427_CONTROL_HIGH                0x01
#define BQ27427_TEMP_LOW                    0x02
#define BQ27427_TEMP_HIGH                   0x03
#define BQ27427_VOLTAGE_LOW                 0x04
#define BQ27427_VOLTAGE_HIGH                0x05
#define BQ27427_FLAGS_LOW                   0x06
#define BQ27427_FLAGS_HIGH                  0x07
#define BQ27427_NOM_AVAILABLE_CAP_LOW       0x08
#define BQ27427_NOM_AVAILABLE_CAP_HIGH      0x09
#define BQ27427_FULL_AVAILABLE_CAP_LOW      0x0A
#define BQ27427_FULL_AVAILABLE_CAP_HIGH     0x0B
#define BQ27427_REMAINING_CAP_LOW           0x0C
#define BQ27427_REMAINING_CAP_HIGH          0x0D
#define BQ27427_FULL_CHARGE_CAP_LOW         0x0E
#define BQ27427_FULL_CHARGE_CAP_HIGH        0x0F
#define BQ27427_AVG_CURRENT_LOW             0x10
#define BQ27427_AVG_CURRENT_HIGH            0x11
#define BQ27427_AVG_POWER_LOW               0x18
#define BQ27427_AVG_POWER_HIGH              0x19
#define BQ27427_STATE_OF_CHARGE_LOW         0x1C
#define BQ27427_STATE_OF_CHARGE_HIGH        0x1D
#define BQ27427_INT_TEMP_LOW                0x1E
#define BQ27427_INT_TEMP_HIGH               0x1F
#define BQ27427_REMAINING_CAP_UNFILT_LOW    0x28
#define BQ27427_REMAINING_CAP_UNFILT_HIGH   0x29
#define BQ27427_REMAINING_CAP_FILT_LOW      0x2A
#define BQ27427_REMAINING_CAP_FILT_HIGH     0x2B
#define BQ27427_FULL_CHARGE_UNFILT_CAP_LOW  0x2C
#define BQ27427_FULL_CHARGE_UNFILT_CAP_HIGH 0x2D
#define BQ27427_FULL_CHARGE_FILT_CAP_LOW    0x2E
#define BQ27427_FULL_CHARGE_FILT_CAP_HIGH   0x2F
#define BQ27427_STATE_OF_CHARGE_UNFILT_LOW  0x30
#define BQ27427_STATE_OF_CHARGE_UNFILT_HIGH 0x31
#define BQ27427_OPCONFIG_LOW                0x3A
#define BQ27427_OPCONFIG_HIGH               0x3B
#define BQ27427_DESIGN_CAP_LOW              0x3C
#define BQ27427_DESIGN_CAP_HIGH             0x3D
#define BQ27427_DATA_CLASS                  0x3E
#define BQ27427_DATA_BLOCK                  0x3F
#define BQ27427_BLOCK_DATA_START            0x40
#define BQ27427_BLOCK_DATA_END              0x5F
#define BQ27427_BLOCK_DATA_CHECKSUM         0x60
#define BQ27427_BLOCK_DATA_CONTROL          0x61

#define BQ27427_CONTROL_STATUS              0x0000
#define BQ27427_CONTROL_DEVICE_TYPE         0x0001
#define BQ27427_CONTROL_FW_VERSION          0x0002
#define BQ27427_CONTROL_DM_CODE             0x0004
#define BQ27427_CONTROL_PREV_MACWRITE       0x0007
#define BQ27427_CONTROL_CHEM_ID             0x0008
#define BQ27427_CONTROL_BAT_INSERT          0x000C
#define BQ27427_CONTROL_BAT_REMOVE          0x000D
#define BQ27427_CONTROL_SET_HIBERNATE       0x0011
#define BQ27427_CONTROL_CLEAR_HIBERNATE     0x0012
#define BQ27427_CONTROL_SET_CFGUPDATE       0x0013
#define BQ27427_CONTROL_SHUTDOWN_ENABLE     0x001B
#define BQ27427_CONTROL_SHUTDOWN            0x001C
#define BQ27427_CONTROL_SEALED              0x0020
#define BQ27427_CONTROL_TOGGLE_GPOUT        0x0023
#define BQ27427_CONTROL_RESET               0x0041
#define BQ27427_CONTROL_SOFT_RESET          0x0042
#define BQ27427_CONTROL_EXIT_CFGUPDATE      0x0043
#define BQ27427_CONTROL_EXIT_RESIM          0x0044
#define BQ27427_CONTROL_UNSEAL              0x8000

typedef struct
{
    uint16_t    voltage_mV;
    int16_t     current_mA;
    double      temp_degC;
    uint16_t    soc_percent;
    uint16_t    designCapacity_mAh;
    uint16_t    remainingCapacity_mAh;
    uint16_t    fullChargeCapacity_mAh;

    bool        isCritical;
    bool        isLow;
    bool        isFull;
    bool        isCharging;
    bool        isDischarging;
} bq27427_info;

bool bq27427_init( uint16_t designCapacity_mAh, uint16_t terminateVoltage_mV, uint16_t taperCurrent_mA );
bool bq27427_update( bq27427_info *battery );
bool bq27427_readDeviceType( uint16_t *deviceType );
bool bq27427_readDeviceFWver( uint16_t *deviceFWver );
bool bq27427_readDesignCapacity_mAh( uint16_t *capacity_mAh );

bool bq27427_readVoltage_mV( uint16_t *voltage_mV );
bool bq27427_readTemp_degK( uint16_t *temp_degKbyTen );
bool bq27427_readAvgCurrent_mA( int16_t *avgCurrent_mA );
bool bq27427_readStateofCharge_percent( uint16_t *soc_percent );

bool bq27427_readControlReg( uint16_t *control );
bool bq27427_readFlagsReg( uint16_t *flags );
bool bq27427_readopConfig( uint16_t *opConfig );
bool bq27427_readRemainingCapacity_mAh( uint16_t *capacity_mAh );
bool bq27427_readFullChargeCapacity_mAh( uint16_t *capacity_mAh );

bool bq27427_i2c_command_write( uint8_t command, uint16_t data );
bool bq27427_i2c_command_read( uint8_t command, uint16_t *data );

bool bq27427_i2c_control_write( uint16_t subcommand );
bool bq27427_i2c_control_read( uint16_t subcommand, uint16_t *data );
bool bq27427_i2c_write_data_block( uint8_t offset, uint8_t *data, uint8_t bytes );
bool bq27427_i2c_read_data_block( uint8_t offset, uint8_t *data, uint8_t bytes );

#ifdef __cplusplus
}
#endif
#endif /* __BQ27427_H */
