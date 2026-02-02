#include "bq27427_reg.h"
#include "stm32wb0x_hal.h"

extern I2C_HandleTypeDef hi2c1;

bool bq27427_i2c_command_write( uint8_t command, uint16_t data )
{
    HAL_StatusTypeDef ret;
    uint8_t i2c_data[3];

    i2c_data[0] = command;
    i2c_data[1] = ( uint8_t )( data & 0x00FF );
    i2c_data[2] = ( uint8_t )( ( data >> 8 ) & 0x00FF );

    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 3, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    return true;
}

bool bq27427_i2c_command_read( uint8_t command, uint16_t *data )
{
    HAL_StatusTypeDef ret;
    uint8_t i2c_data[2];

    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, &command, 1, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    ret = HAL_I2C_Master_Receive( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 2, HAL_BQ27427_TIMEOUT );
    if(ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    *data = ( i2c_data[1] << 8 ) | i2c_data[0];

    return true;
}

bool bq27427_i2c_control_write( uint16_t subcommand )
{
    HAL_StatusTypeDef ret;
    uint8_t i2c_data[2];

    i2c_data[0] = BQ27427_CONTROL_LOW;
    i2c_data[1] = (uint8_t)( ( subcommand ) & 0x00FF );


    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 2, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    i2c_data[0] = BQ27427_CONTROL_HIGH;
    i2c_data[1] = (uint8_t)( ( subcommand >> 8 ) & 0x00FF );

    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 2, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    return true;
}

bool bq27427_i2c_control_read( uint16_t subcommand, uint16_t *data )
{
    HAL_StatusTypeDef ret;
    uint8_t i2c_data[2];

    i2c_data[0] = BQ27427_CONTROL_LOW;
    i2c_data[1] = (uint8_t)( ( subcommand ) & 0x00FF );

    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 2, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    i2c_data[0] = BQ27427_CONTROL_HIGH;
    i2c_data[1] = (uint8_t)( ( subcommand >> 8 ) & 0x00FF );

    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 2, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    ret = HAL_I2C_Master_Receive( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 2, HAL_BQ27427_TIMEOUT );
    if(ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    *data = ( i2c_data[1] << 8 ) | i2c_data[0];

    return true;
}

bool bq27427_i2c_write_data_block( uint8_t offset, uint8_t *data, uint8_t bytes )
{
    HAL_StatusTypeDef ret;
    uint8_t i2c_data[2], i;

    for( i = 0; i < bytes; i++ )
    {
        i2c_data[0] = BQ27427_BLOCK_DATA_START + offset + i;
        i2c_data[1] = data[i];

        ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, i2c_data, 2, HAL_BQ27427_TIMEOUT );
        if( ret != HAL_OK )
        {
            return false;
        }
        HAL_Delay( BQ27427_DELAY );
    }

    return true;
}

bool bq27427_i2c_read_data_block( uint8_t offset, uint8_t *data, uint8_t bytes )
{
    HAL_StatusTypeDef ret;

    uint8_t i2c_data;

    i2c_data = BQ27427_BLOCK_DATA_START + offset;

    ret = HAL_I2C_Master_Transmit( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, &i2c_data, 1, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( 5 );

    ret = HAL_I2C_Master_Receive( &HAL_I2C_INSTANCE, (uint16_t)BQ27427_I2C_ADDRESS, data, bytes, HAL_BQ27427_TIMEOUT );
    if( ret != HAL_OK )
    {
        return false;
    }

    HAL_Delay( BQ27427_DELAY );

    return true;
}

bool bq27427_init( uint16_t designCapacity_mAh, uint16_t terminateVoltage_mV, uint16_t taperCurrent_mA )
{
    uint16_t designEnergy_mWh, taperRate, flags, checksumOld, checksumRead;
    uint8_t checksumNew;


    designEnergy_mWh = 3.7 * designCapacity_mAh;
    taperRate = designCapacity_mAh / ( 0.1 * taperCurrent_mA );

    // Unseal gauge
    bq27427_i2c_control_write( BQ27427_CONTROL_UNSEAL );
    bq27427_i2c_control_write( BQ27427_CONTROL_UNSEAL );

    // Send CFG_UPDATE
    bq27427_i2c_control_write( BQ27427_CONTROL_SET_CFGUPDATE );

    // Poll flags until CFGUPMODE bit is set
    uint8_t cfg_timeout = 100;  // 10 second timeout
    do
    {
        bq27427_i2c_command_read( BQ27427_FLAGS_LOW, &flags );
        if( !(flags & 0x0010) )
        {
            HAL_Delay( 100 );
        }
        cfg_timeout--;
        if( cfg_timeout == 0 )
        {
            return false;  // Timeout entering CONFIG UPDATE mode
        }
    }
    while( !(flags & 0x0010) );

    // Wait minimum 1100ms before parameter modifications (per datasheet section 8.5)
    HAL_Delay( 1100 );

    // Enable Block Data Memory Control
    bq27427_i2c_command_write( BQ27427_BLOCK_DATA_CONTROL, 0x0000 );

    HAL_Delay( BQ27427_DELAY );

    // Access State subclass
    bq27427_i2c_command_write( BQ27427_DATA_CLASS, 0x0052 );

    // Write the block offset
    bq27427_i2c_command_write( BQ27427_DATA_BLOCK, 0x0000 );

    // Read block checksum
    bq27427_i2c_command_read( BQ27427_BLOCK_DATA_CHECKSUM, &checksumOld );

    // Read 32-byte block of data
    uint8_t block[32];
    for(uint8_t i = 0; i < 32; i++ )
    {
        block[i] = 0x00;
    }

    bq27427_i2c_read_data_block( 0x00, block, 32 );

    // Calculate checksum
    uint8_t checksumCalc = 0x00;

    for(uint8_t i = 0; i < 32; i++ )
    {
        checksumCalc += block[i];
    }
    checksumCalc = 0xFF - checksumCalc;

    // Update design capacity
    block[10] = (uint8_t)( designCapacity_mAh >> 8 );
    block[11] = (uint8_t)( designCapacity_mAh & 0x00FF );
    // Update design energy
    block[12] = (uint8_t)( designEnergy_mWh >> 8 );
    block[13] = (uint8_t)( designEnergy_mWh & 0x00FF );
    // Update terminate voltage
    block[16] = (uint8_t)( terminateVoltage_mV >> 8 );
    block[17] = (uint8_t)( terminateVoltage_mV & 0x00FF );
    // Update taper rate
    block[27] = (uint8_t)( taperRate >> 8 );
    block[28] = (uint8_t)( taperRate & 0x00FF );

    // Calculate new checksum
    checksumNew = 0x00;
    for(int i = 0; i < 32; i++ )
    {
        checksumNew += block[i];
    }
    checksumNew = 0xFF - checksumNew;

    // Enable Block Data Memory Control
    bq27427_i2c_command_write( BQ27427_BLOCK_DATA_CONTROL, 0x0000 );

    HAL_Delay( BQ27427_DELAY );

    // Access State subclass
    bq27427_i2c_command_write( BQ27427_DATA_CLASS, 0x0052 );

    // Write the block offset
    bq27427_i2c_command_write( BQ27427_DATA_BLOCK, 0x0000 );

    // Write 32-byte block of updated data
    bq27427_i2c_write_data_block( 0x00, block, 32 );

    // Write new checksum
    bq27427_i2c_command_write( BQ27427_BLOCK_DATA_CHECKSUM, checksumNew );

    // Access State subclass
    bq27427_i2c_command_write( BQ27427_DATA_CLASS, 0x0052 );

    // Write the block offset
    bq27427_i2c_command_write( BQ27427_DATA_BLOCK, 0x0000 );

    // Read block checksum
    bq27427_i2c_command_read( BQ27427_BLOCK_DATA_CHECKSUM, &checksumRead );

    // Verify
    if( checksumRead != (uint8_t)checksumNew )
    {
        return false;
    }

    // Enable Block Data Memory Control
    bq27427_i2c_command_write( BQ27427_BLOCK_DATA_CONTROL, 0x0000 );

    HAL_Delay( BQ27427_DELAY );

    // Access Registers subclass
    bq27427_i2c_command_write( BQ27427_DATA_CLASS, 0x0040 );

    // Write the block offset
    bq27427_i2c_command_write( BQ27427_DATA_BLOCK, 0x0000 );

    // Read block checksum
    bq27427_i2c_command_read( BQ27427_BLOCK_DATA_CHECKSUM, &checksumOld );

    // Read 32-byte block of data
    for(uint8_t i = 0; i < 32; i++ )
    {
        block[i] = 0x00;
    }

    bq27427_i2c_read_data_block( 0x00, block, 32 );

    // Calculate checksum
    checksumCalc = 0x00;

    for(uint8_t i = 0; i < 32; i++ )
    {
        checksumCalc += block[i];
    }
    checksumCalc = 0xFF - checksumCalc;

    // Update OpConfig (enable battery insertion detection)
    block[0] |= 0x01;   // BAT_INS_EN bit

    // Calculate new checksum
    checksumNew = 0x00;
    for(int i = 0; i < 32; i++ )
    {
        checksumNew += block[i];
    }
    checksumNew = 0xFF - checksumNew;

    // Enable Block Data Memory Control
    bq27427_i2c_command_write( BQ27427_BLOCK_DATA_CONTROL, 0x0000 );

    HAL_Delay( BQ27427_DELAY );

    // Access Registers subclass
    bq27427_i2c_command_write( BQ27427_DATA_CLASS, 0x0040 );

    // Write the block offset
    bq27427_i2c_command_write( BQ27427_DATA_BLOCK, 0x0000 );

    // Write 32-byte block of updated data
    bq27427_i2c_write_data_block( 0x00, block, 32 );

    // Write new checksum
    bq27427_i2c_command_write( BQ27427_BLOCK_DATA_CHECKSUM, checksumNew );

    // Access Registers subclass
    bq27427_i2c_command_write( BQ27427_DATA_CLASS, 0x0040 );

    // Write the block offset
    bq27427_i2c_command_write( BQ27427_DATA_BLOCK, 0x0000 );

    // Read block checksum
    bq27427_i2c_command_read( BQ27427_BLOCK_DATA_CHECKSUM, &checksumRead );

    // Verify
    if( checksumRead != (uint8_t)checksumNew )
    {
        return false;
    }

    // ============================================================================
    // CRITICAL FIX: Proper exit from CONFIG UPDATE mode
    // ============================================================================

    // Step 1: Send SOFT_RESET subcommand to exit CONFIG UPDATE mode
    // This is required per datasheet section 8.5
    bq27427_i2c_control_write( BQ27427_CONTROL_SOFT_RESET );

    HAL_Delay(1000);  // Wait 1 second for reset to complete

    // Step 2: Poll Flags register until CFGUPMODE (bit 4) is cleared
    // ITPOR (bit 2) may remain set until battery learning is complete - this is normal
    uint8_t exit_timeout = 100;  // 10 second timeout
    do
    {
        bq27427_i2c_command_read( BQ27427_FLAGS_LOW, &flags );
        HAL_Delay( 100 );
        exit_timeout--;
        if( exit_timeout == 0 )
        {
            return false;  // Timeout - device stuck in CONFIG UPDATE mode
        }
    }
    while( flags & 0x0010 );  // Wait ONLY for CFGUPMODE to clear (bit 4)

    // Step 3: Configure BAT_DET to enable battery detection
    bq27427_i2c_control_write( BQ27427_CONTROL_BAT_INSERT );

    HAL_Delay(500);

    bq27427_readFlagsReg(&flags);

    if (!(flags & 0x0008)) {   // BAT_DET bit
        return false;         // Battery not detected → SOC invalid
    }


    // Step 4: Check CONTROL_STATUS for initialization status
    // Note: INITCOMP (bit 7) may not be set immediately on fresh battery
    uint16_t control_status;
    bq27427_i2c_control_read( BQ27427_CONTROL_STATUS, &control_status );

    // We won't fail if INITCOMP is not set - it will set after first charge cycle
    // Just log the status for debugging
    // if( !(control_status & 0x0080) ) {
    //     return false;  // Initialization not complete
    // }

    // Step 5: Seal gauge to protect configuration
    bq27427_i2c_control_write( BQ27427_CONTROL_SEALED );

    return true;
}

bool bq27427_update( bq27427_info *battery )
{
    uint16_t temp;

    if( !bq27427_readVoltage_mV( &(battery->voltage_mV) ) )
    {
        return false;
    }
    if( !bq27427_readAvgCurrent_mA( &(battery->current_mA) ) )
    {
        return false;
    }
    if( !bq27427_readTemp_degK( &temp ) )
    {
        return false;
    }
    battery->temp_degC = ( (double)temp / 10 ) - 273.15;

    if( !bq27427_readStateofCharge_percent( &(battery->soc_percent) ) )
    {
        return false;
    }
    if( !bq27427_readDesignCapacity_mAh( &(battery->designCapacity_mAh) ) )
    {
        return false;
    }
    if( !bq27427_readRemainingCapacity_mAh( &(battery->remainingCapacity_mAh) ) )
    {
        return false;
    }
    if( !bq27427_readFullChargeCapacity_mAh( &(battery->fullChargeCapacity_mAh) ) )
    {
        return false;
    }
    if( !bq27427_readFlagsReg( &temp ) )
    {
        return false;
    }
    battery->isCritical = temp & 0x0002;
    battery->isLow = temp & 0x0004;
    battery->isFull = temp & 0x0200;
    if( battery->current_mA <= 0 )
    {
        battery->isDischarging = 1;
        battery->isCharging = 0;
    }
    else
    {
        battery->isDischarging = 0;
        battery->isCharging = 1;
    }

    return true;
}

bool bq27427_readDeviceType( uint16_t *deviceType )
{
    if( !bq27427_i2c_control_write( BQ27427_CONTROL_DEVICE_TYPE ) )
    {
        return false;
    }
    if( !bq27427_i2c_command_read( BQ27427_CONTROL_LOW, deviceType ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readDeviceFWver( uint16_t *deviceFWver )
{
    if( !bq27427_i2c_control_write( BQ27427_CONTROL_FW_VERSION ) )
    {
        return false;
    }
    if( !bq27427_i2c_command_read( BQ27427_CONTROL_LOW, deviceFWver ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readDesignCapacity_mAh( uint16_t *capacity_mAh )
{
    if( !bq27427_i2c_command_read( BQ27427_DESIGN_CAP_LOW, capacity_mAh ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readVoltage_mV( uint16_t *voltage_mV )
{
    if( !bq27427_i2c_command_read( BQ27427_VOLTAGE_LOW, voltage_mV ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readTemp_degK( uint16_t *temp_degKbyTen )
{
    if( !bq27427_i2c_command_read( BQ27427_TEMP_LOW, temp_degKbyTen ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readAvgCurrent_mA( int16_t *avgCurrent_mA )
{
    if( !bq27427_i2c_command_read( BQ27427_AVG_CURRENT_LOW, (uint16_t *)avgCurrent_mA ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readStateofCharge_percent( uint16_t *soc_percent )
{
    if( !bq27427_i2c_command_read( BQ27427_STATE_OF_CHARGE_LOW, soc_percent ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readControlReg( uint16_t *control )
{
    if( !bq27427_i2c_control_write( BQ27427_CONTROL_STATUS ) )
    {
        return false;
    }
    if( !bq27427_i2c_command_read( BQ27427_CONTROL_LOW, control ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readFlagsReg( uint16_t *flags )
{
    if( !bq27427_i2c_command_read( BQ27427_FLAGS_LOW, flags ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readopConfig( uint16_t *opConfig )
{
    if( !bq27427_i2c_command_read( BQ27427_OPCONFIG_LOW, opConfig ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readRemainingCapacity_mAh( uint16_t *capacity_mAh )
{
    if( !bq27427_i2c_command_read( BQ27427_REMAINING_CAP_LOW, capacity_mAh ) )
    {
        return false;
    }

    return true;
}

bool bq27427_readFullChargeCapacity_mAh( uint16_t *capacity_mAh )
{
    if( !bq27427_i2c_command_read( BQ27427_FULL_CHARGE_CAP_LOW, capacity_mAh ) )
    {
        return false;
    }

    return true;
}
