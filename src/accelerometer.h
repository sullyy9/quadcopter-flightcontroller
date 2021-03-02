/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : accelerometer.h
    Description     : defines for the LSM303AGR accelerometer and magnetometer
*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * Data Rates
 */
enum
{
    DATA_RATE_POWER_DOWN = 0b0000,
    DATA_RATE_1HZ        = 0b0001,
    DATA_RATE_10HZ       = 0b0010,
    DATA_RATE_25HZ       = 0b0011,
    DATA_RATE_50HZ       = 0b0100,
    DATA_RATE_100HZ      = 0b0101,
    DATA_RATE_200HZ      = 0b0110,
    DATA_RATE_400HZ      = 0b0111,
    DATA_RATE_1344HZ     = 0b1001,
    DATA_RATE_1620HZ     = 0b1000,  //requires low power mode
    DATA_RATE_5376HZ     = 0b1001,  //requires low power mode
};

/*
 * Device addresses
 */
#define ACCEL_I2C_ADDRESS   0b00110010
#define MAG_I2C_ADDRESS     0b00111100

/*
 * Register addresses
 */
#define TEMP_OUT_REG_L_ADDRESS      0x0C
#define TEMP_OUT_REG_H_ADDRESS      0x0D

#define ACCEL_CTRL_REG_1_ADDRESS    0x20
#define ACCEL_CTRL_REG_2_ADDRESS    0x21
#define ACCEL_CTRL_REG_3_ADDRESS    0x22
#define ACCEL_CTRL_REG_4_ADDRESS    0x23
#define ACCEL_CTRL_REG_5_ADDRESS    0x24
#define ACCEL_CTRL_REG_6_ADDRESS    0x25

#define ACCEL_OUT_REG_X_L_ADDRESS   0x28
#define ACCEL_OUT_REG_X_H_ADDRESS   0x29
#define ACCEL_OUT_REG_Y_L_ADDRESS   0x2A
#define ACCEL_OUT_REG_Y_H_ADDRESS   0x2B
#define ACCEL_OUT_REG_Z_L_ADDRESS   0x2C
#define ACCEL_OUT_REG_Z_H_ADDRESS   0x2D

#define ACCEL_INT1_CFG_REG_ADDRESS  0x30
#define ACCEL_INT1_SRC_REG_ADDRESS  0x31
#define ACCEL_INT1_THS_REG_ADDRESS  0x32
#define ACCEL_INT1_DUR_REG_ADDRESS  0x33

#define MAG_OFFSET_REG_X_L_ADDRESS  0x45
#define MAG_OFFSET_REG_X_H_ADDRESS  0x46
#define MAG_OFFSET_REG_Y_L_ADDRESS  0x47
#define MAG_OFFSET_REG_Y_H_ADDRESS  0x48
#define MAG_OFFSET_REG_Z_L_ADDRESS  0x49
#define MAG_OFFSET_REG_Z_H_ADDRESS  0x4A

#define MAG_CONFIG_REG_A_ADDRESS    0x60
#define MAG_CONFIG_REG_B_ADDRESS    0x61
#define MAG_CONFIG_REG_C_ADDRESS    0x62

#define MAG_OUT_REG_X_L_ADDRESS     0x68
#define MAG_OUT_REG_X_H_ADDRESS     0x69
#define MAG_OUT_REG_Y_L_ADDRESS     0x6A
#define MAG_OUT_REG_Y_H_ADDRESS     0x6B
#define MAG_OUT_REG_Z_L_ADDRESS     0x6C
#define MAG_OUT_REG_Z_H_ADDRESS     0x6D

/*
 * Register values
 *
 * control register 1 MSB to LSB
 */
#define DATA_RATE               DATA_RATE_25HZ  //1Hz
#define LOW_POWER_ENABLE        0               //disable
#define Z_AXIS_ENABLE           1               //enable
#define Y_AXIS_ENABLE           1               //enable
#define X_AXIS_ENABLE           1               //enable
#define ACCEL_CTRL_REG_1_VALUE ( ( DATA_RATE     << 4 ) + ( LOW_POWER_ENABLE << 3 ) + ( Z_AXIS_ENABLE << 2 ) + \
                                 ( Y_AXIS_ENABLE << 1 ) + ( X_AXIS_ENABLE    << 0 ) )

/*
 * control register 2 MSB to LSB
 */
#define HP_FILTER_MODE          0b00    //normal
#define HP_CUTOFF_FREQ          0b00    //2Hz
#define FILTER_DATA_ENABLE      0       //disable
#define FILTER_CLICK_ENABLE     0       //disable
#define FILTER_INT1_ENABLE      0       //disable
#define FILTER_INT2_ENABLE      0       //disable
#define ACCEL_CTRL_REG_2_VALUE  ( ( HP_FILTER_MODE      << 6 ) + ( HP_CUTOFF_FREQ     << 4 ) + ( FILTER_DATA_ENABLE << 3 ) + \
                                  ( FILTER_CLICK_ENABLE << 2 ) + ( FILTER_INT1_ENABLE << 1 ) + ( FILTER_INT2_ENABLE << 0 ) )

/*
 * control register 3 MSB to LSB
 */
#define INT1_CLICK_ENABLE       0       //disable
#define INT1_AOI1_ENABLE        0       //disable
#define INT1_AOI2_ENABLE        0       //disable
#define INT1_DRDY1_ENABLE       1       //enable
#define INT1_DRDY2_ENABLE       0       //disable
#define INT1_WTM_ENABLE         0       //disable
#define INT1_OVERUN_ENABLE      0       //disable
#define ACR3_UNUSED             0       //unused
#define ACCEL_CTRL_REG_3_VALUE  ( ( INT1_CLICK_ENABLE  << 7 ) + ( INT1_AOI1_ENABLE  << 6 ) + ( INT1_AOI2_ENABLE << 5 ) + \
                                  ( INT1_DRDY1_ENABLE  << 4 ) + ( INT1_DRDY2_ENABLE << 3 ) + ( INT1_WTM_ENABLE  << 2 ) + \
                                  ( INT1_OVERUN_ENABLE << 1 ) + ( ACR3_UNUSED       << 0 ) )

/*
 * control register 4 MSB to LSB
 */
#define BLOCK_DATA_UPDATE       1       //enable
#define LITTLE_BIG_ENDIAN       0       //little endian
#define MEASUREMENT_RANGE       0b01    //+-4g
#define HIGH_RES_ENABLE         0       //normal
#define SELF_TEST_MODE          0b00    //normal
#define SPI_3_WIRE_ENABLE       0       //disable
#define ACCEL_CTRL_REG_4_VALUE  ( ( BLOCK_DATA_UPDATE << 7 ) + ( LITTLE_BIG_ENDIAN << 6 ) + ( MEASUREMENT_RANGE << 4 ) + \
                                  ( HIGH_RES_ENABLE   << 3 ) + ( SELF_TEST_MODE    << 1 ) + ( SPI_3_WIRE_ENABLE << 0 ) )

/*
 * control register 5 MSB to LSB
 */
#define REBOOT_MEMORY_CONTENT   0       //disable
#define FIFO_ENABLE             0       //disable
#define ACR5_UNUSED             0b00    //unused
#define INT1_LATCH_ENABLE       0       //disable
#define INT1_4D_DETECT_ENABLE   0       //disable
#define INT2_LATCH_ENABLE       0       //disable
#define INT2_4D_DETECT_ENABLE   0       //disable
#define ACCEL_CTRL_REG_5_VALUE  ( ( REBOOT_MEMORY_CONTENT << 7 ) + ( FIFO_ENABLE           << 6 ) + ( ACR5_UNUSED       << 4 ) + \
                                  ( INT1_LATCH_ENABLE     << 3 ) + ( INT1_4D_DETECT_ENABLE << 2 ) + ( INT2_LATCH_ENABLE << 1 ) + \
                                  ( INT2_4D_DETECT_ENABLE << 0 ) )

/*
 * control register 6 MSB to LSB
 */
#define INT2_CLICK_ENABLE       0       //disable
#define INT2_INT1_FUNC_ENABLE   0       //disable
#define INT2_INT2_FUNC_ENABLE   0       //disable
#define INT2_BOOT_ENABLE        0       //disable
#define INT2_ACTIVITY_ENABLE    0       //disable
#define ACR5_UNUSED_1           0       //unused
#define INT_ACTIVE_HIGH_LOW     0       //active high
#define ACR5_UNUSED_2           0       //unused
#define ACCEL_CTRL_REG_6_VALUE  ( ( INT2_CLICK_ENABLE   << 7 ) + ( INT2_INT1_FUNC_ENABLE << 6 ) + ( INT2_INT2_FUNC_ENABLE << 5 ) + \
                                  ( INT2_BOOT_ENABLE    << 4 ) + ( INT2_ACTIVITY_ENABLE  << 3 ) + ( ACR5_UNUSED_1         << 2 ) + \
                                  ( INT_ACTIVE_HIGH_LOW << 1 ) + ( ACR5_UNUSED_2         << 0 ) )

/*
 * address increment macro
 */
#define INCREMENT( address ) ( 0b10000000 + address )

/*
 * Accelerometer data resolution
 */
#if   ( ( LOW_POWER_ENABLE == 1 ) && ( HIGH_RES_ENABLE == 0 ) )
#define ACCEL_RESOLUTION 8
#elif ( ( LOW_POWER_ENABLE == 0 ) && ( HIGH_RES_ENABLE == 0 ) )
#define ACCEL_RESOLUTION 10
#elif ( ( LOW_POWER_ENABLE == 0 ) && ( HIGH_RES_ENABLE == 1 ) )
#define ACCEL_RESOLUTION 12
#else
#error  Invalid values
#endif

/*----------------------------------------------------------------------------*/
/*-exported-variables---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-exported-functions---------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-end-of-module--------------------------------------------------------------*/
/*----------------------------------------------------------------------------*/
