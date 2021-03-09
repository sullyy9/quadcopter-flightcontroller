/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : lsm303agr.h
    Description     : defines for the LSM303AGR accelerometer and magnetometer
*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * Device addresses
 */
#define ACCEL_I2C_ADDR  0b00110010
#define MAG_I2C_ADDR    0b00111100

#define ACCEL_INC( address ) ( 0b10000000 + address )

/*
 * Register addresses
 */
#define TEMP_OUT_REG_L_ADDR         0x0C
#define TEMP_OUT_REG_H_ADDR         0x0D

#define ACCEL_CTRL_REG_1_ADDR       0x20
#define ACCEL_CTRL_REG_2_ADDR       0x21
#define ACCEL_CTRL_REG_3_ADDR       0x22
#define ACCEL_CTRL_REG_4_ADDR       0x23
#define ACCEL_CTRL_REG_5_ADDR       0x24
#define ACCEL_CTRL_REG_6_ADDR       0x25

#define ACCEL_OUT_REG_X_L_ADDR      0x28
#define ACCEL_OUT_REG_X_H_ADDR      0x29
#define ACCEL_OUT_REG_Y_L_ADDR      0x2A
#define ACCEL_OUT_REG_Y_H_ADDR      0x2B
#define ACCEL_OUT_REG_Z_L_ADDR      0x2C
#define ACCEL_OUT_REG_Z_H_ADDR      0x2D

#define ACCEL_INT1_CFG_REG_ADDR     0x30
#define ACCEL_INT1_SRC_REG_ADDR     0x31
#define ACCEL_INT1_THS_REG_ADDR     0x32
#define ACCEL_INT1_DUR_REG_ADDR     0x33

#define MAG_OFFSET_REG_X_L_ADDR     0x45
#define MAG_OFFSET_REG_X_H_ADDR     0x46
#define MAG_OFFSET_REG_Y_L_ADDR     0x47
#define MAG_OFFSET_REG_Y_H_ADDR     0x48
#define MAG_OFFSET_REG_Z_L_ADDR     0x49
#define MAG_OFFSET_REG_Z_H_ADDR     0x4A

#define MAG_CONFIG_REG_A_ADDR       0x60
#define MAG_CONFIG_REG_B_ADDR       0x61
#define MAG_CONFIG_REG_C_ADDR       0x62

#define MAG_OUT_REG_X_L_ADDR        0x68
#define MAG_OUT_REG_X_H_ADDR        0x69
#define MAG_OUT_REG_Y_L_ADDR        0x6A
#define MAG_OUT_REG_Y_H_ADDR        0x6B
#define MAG_OUT_REG_Z_L_ADDR        0x6C
#define MAG_OUT_REG_Z_H_ADDR        0x6D

/*
 * Parameter defines
 */
enum
{
    ACCEL_DATA_RATE_POWER_DOWN = 0b0000,
    ACCEL_DATA_RATE_1HZ        = 0b0001,
    ACCEL_DATA_RATE_10HZ       = 0b0010,
    ACCEL_DATA_RATE_25HZ       = 0b0011,
    ACCEL_DATA_RATE_50HZ       = 0b0100,
    ACCEL_DATA_RATE_100HZ      = 0b0101,
    ACCEL_DATA_RATE_200HZ      = 0b0110,
    ACCEL_DATA_RATE_400HZ      = 0b0111,
    ACCEL_DATA_RATE_1344HZ     = 0b1001,
    ACCEL_DATA_RATE_1620HZ     = 0b1000,  //requires low power mode
    ACCEL_DATA_RATE_5376HZ     = 0b1001,  //requires low power mode
};

enum
{
    MAG_DATA_RATE_10HZ  = 0b00,
    MAG_DATA_RATE_20HZ  = 0b01,
    MAG_DATA_RATE_50HZ  = 0b10,
    MAG_DATA_RATE_100HZ = 0b11
};

enum
{
    MAG_CONTINUOUS_MODE = 0b00,
    MAG_SINGLE_MODE     = 0b01,
    MAG_IDLE_MODE       = 0b10
};

/*
 * Accelerometer register values
 *
 * accelerometer control register 1 MSB to LSB
 */
#define ACCEL_DATA_RATE         ACCEL_DATA_RATE_50HZ
#define ACCEL_LOW_POWER_ON      0               //disable
#define ACCEL_Z_AXIS_ON         1               //enable
#define ACCEL_Y_AXIS_ON         1               //enable
#define ACCEL_X_AXIS_ON         1               //enable
#define ACCEL_CTRL_REG_1_VAL    ( ( ACCEL_DATA_RATE << 4 ) + ( ACCEL_LOW_POWER_ON << 3 ) + \
                                  ( ACCEL_Z_AXIS_ON << 2 ) + ( ACCEL_Y_AXIS_ON    << 1 ) + \
                                  ( ACCEL_X_AXIS_ON << 0 ) )

/*
 * accelerometer control register 2 MSB to LSB
 */
#define ACCEL_HP_FILTER_MODE    0b00    //normal
#define ACCEL_HP_CUTOFF_FREQ    0b00    //2Hz
#define ACCEL_FILTER_DATA_ON    0       //disable
#define ACCEL_FILTER_CLICK_ON   0       //disable
#define ACCEL_FILTER_INT1_ON    0       //disable
#define ACCEL_FILTER_INT2_ON    0       //disable
#define ACCEL_CTRL_REG_2_VAL    ( ( ACCEL_HP_FILTER_MODE << 6 ) + ( ACCEL_HP_CUTOFF_FREQ  << 4 ) + \
                                  ( ACCEL_FILTER_DATA_ON << 3 ) + ( ACCEL_FILTER_CLICK_ON << 2 ) + \
                                  ( ACCEL_FILTER_INT1_ON << 1 ) + ( ACCEL_FILTER_INT2_ON  << 0 ) )

/*
 * accelerometer control register 3 MSB to LSB
 */
#define ACCEL_INT1_CLICK_ON     0       //disable
#define ACCEL_INT1_AOI1_ON      0       //disable
#define ACCEL_INT1_AOI2_ON      0       //disable
#define ACCEL_INT1_DRDY1_ON     1       //enable
#define ACCEL_INT1_DRDY2_ON     0       //disable
#define ACCEL_INT1_WTM_ON       0       //disable
#define ACCEL_INT1_OVERUN_ON    0       //disable
#define ACR3_UNUSED             0       //unused
#define ACCEL_CTRL_REG_3_VAL    ( ( ACCEL_INT1_CLICK_ON  << 7 ) + ( ACCEL_INT1_AOI1_ON  << 6 ) + \
                                  ( ACCEL_INT1_AOI2_ON   << 5 ) + ( ACCEL_INT1_DRDY1_ON << 4 ) + \
                                  ( ACCEL_INT1_DRDY2_ON  << 3 ) + ( ACCEL_INT1_WTM_ON   << 2 ) + \
                                  ( ACCEL_INT1_OVERUN_ON << 1 ) + ( ACR3_UNUSED         << 0 ) )

/*
 * accelerometer control register 4 MSB to LSB
 */
#define ACCEL_BLOCK_UPDATE      1       //enable
#define ACCEL_LITTLE_BIG_ENDIAN 0       //little endian
#define ACCEL_MEAS_RANGE        0b01    //+-4g
#define ACCEL_HIGH_RES_ON       0       //normal
#define ACCEL_SELF_TEST         0b00    //normal
#define ACCEL_SPI_3_WIRE_ON     0       //disable
#define ACCEL_CTRL_REG_4_VAL    ( ( ACCEL_BLOCK_UPDATE << 7 ) + ( ACCEL_LITTLE_BIG_ENDIAN << 6 ) + \
                                  ( ACCEL_MEAS_RANGE   << 4 ) + ( ACCEL_HIGH_RES_ON       << 3 ) + \
                                  ( ACCEL_SELF_TEST    << 1 ) + ( ACCEL_SPI_3_WIRE_ON     << 0 ) )

/*
 * accelerometer control register 5 MSB to LSB
 */
#define ACCEL_REBOOT_MEM        0       //disable
#define ACCEL_FIFO_ON           0       //disable
#define ACR5_UNUSED             0b00    //unused
#define ACCEL_INT1_LATCH_ON     0       //disable
#define ACCEL_INT1_4D_DETECT_ON 0       //disable
#define ACCEL_INT2_LATCH_ON     0       //disable
#define ACCEL_INT2_4D_DETECT_ON 0       //disable
#define ACCEL_CTRL_REG_5_VAL    ( ( ACCEL_REBOOT_MEM        << 7 ) + ( ACCEL_FIFO_ON       << 6 ) + \
                                  ( ACR5_UNUSED             << 4 ) + ( ACCEL_INT1_LATCH_ON << 3 ) + \
                                  ( ACCEL_INT1_4D_DETECT_ON << 2 ) + ( ACCEL_INT2_LATCH_ON << 1 ) + \
                                  ( ACCEL_INT2_4D_DETECT_ON << 0 ) )

/*
 * accelerometer control register 6 MSB to LSB
 */
#define ACCEL_INT2_CLICK_ON     0       //disable
#define ACCEL_INT2_INT1_FUNC_ON 0       //disable
#define ACCEL_INT2_INT2_FUNC_ON 0       //disable
#define ACCEL_INT2_BOOT_ON      0       //disable
#define ACCEL_INT2_ACTIVITY_ON  0       //disable
#define ACR5_UNUSED_1           0       //unused
#define ACCEL_INT_ACTIVE_H_L    0       //active high
#define ACR5_UNUSED_2           0       //unused
#define ACCEL_CTRL_REG_6_VAL    ( ( ACCEL_INT2_CLICK_ON     << 7 ) + ( ACCEL_INT2_INT1_FUNC_ON << 6 ) + \
                                  ( ACCEL_INT2_INT2_FUNC_ON << 5 ) + ( ACCEL_INT2_BOOT_ON      << 4 ) + \
                                  ( ACCEL_INT2_ACTIVITY_ON  << 3 ) + ( ACR5_UNUSED_1           << 2 ) + \
                                  ( ACCEL_INT_ACTIVE_H_L    << 1 ) + ( ACR5_UNUSED_2           << 0 ) )

/*
 * Magnetometer register values
 *
 * magnetometer configuration register A MSB to LSB
 */
#define MAG_TEMP_COMP_ON        0       //disable
#define MAG_REBOOT_MEM          0       //disable
#define MAG_SOFT_RESET          0       //disable
#define MAG_LOW_POWER_ON        0       //high res mode
#define MAG_DATA_RATE           MAG_DATA_RATE_50HZ
#define MAG_MODE_SELECT         MAG_CONTINUOUS_MODE
#define MAG_CONFIG_REG_A_VAL    ( ( MAG_TEMP_COMP_ON << 7 ) + ( MAG_REBOOT_MEM   << 6 ) + \
                                  ( MAG_SOFT_RESET   << 5 ) + ( MAG_LOW_POWER_ON << 4 ) + \
                                  ( MAG_DATA_RATE    << 2 ) + ( MAG_MODE_SELECT  << 0 ) )

/*
 * magnetometer configuration register B MSB to LSB
 */
#define MCRB_UNUSED_1           0       //unused
#define MCRB_UNUSED_2           0       //unused
#define MCRB_UNUSED_3           0       //unused
#define MAG_OFFSET_SINGLE_MODE  0       //disable
#define MAG_INT_CORRECTION      0       //disable
#define MAG_SET_PULSE_FREQ      0       //1/( 63 ODR )
#define MAG_OFFSET_ON           1       //enabled
#define MAG_LOW_PASS_FILTER_ON  0       //disable
#define MAG_CONFIG_REG_B_VAL    ( ( MCRB_UNUSED_1      << 7 ) + ( MCRB_UNUSED_2          << 6 ) + \
                                  ( MCRB_UNUSED_3      << 5 ) + ( MAG_OFFSET_SINGLE_MODE << 4 ) + \
                                  ( MAG_INT_CORRECTION << 3 ) + ( MAG_SET_PULSE_FREQ     << 2 ) + \
                                  ( MAG_OFFSET_ON      << 1 ) + ( MAG_LOW_PASS_FILTER_ON << 0 ) )

/*
 * magnetometer configuration register C MSB to LSB
 */
#define MCRC_UNUSED_1           0       //unused
#define MAG_INT_MAG_PIN_ON      0       //disable
#define MAG_I2C_OFF             0       //enable
#define MAG_BLOCK_DATA_UPDATE   1       //enable
#define MAG_BIG_LITTLE_ENDIAN   0       //little endian
#define MCRC_UNUSED_2           0       //unused
#define MAG_SELF_TEST           0       //disable
#define MAG_DRDY_ON             1       //enable
#define MAG_CONFIG_REG_C_VAL    ( ( MCRC_UNUSED_1         << 7 ) + ( MAG_INT_MAG_PIN_ON    << 6 ) + \
                                  ( MAG_I2C_OFF           << 5 ) + ( MAG_BLOCK_DATA_UPDATE << 4 ) + \
                                  ( MAG_BIG_LITTLE_ENDIAN << 3 ) + ( MCRC_UNUSED_2         << 2 ) + \
                                  ( MAG_SELF_TEST         << 1 ) + ( MAG_DRDY_ON           << 0 ) )

/*
 * magnetometer interrupt control register MSB to LSB
 */
#define MAG_INT_X_ON            0       //disable
#define MAG_INT_Y_ON            0       //disable
#define MAG_INT_Z_ON            0       //disable
#define MICR_UNUSED_1           0       //unused
#define MICR_UNUSED_2           0       //unused
#define MAG_INT_ACTIVE_LOW_HIGH 1       //active high
#define MAG_INT_LATCH_ON        0       //disable
#define MAG_INT_ON              1       //enable
#define MAG_INT_REG_VAL         ( ( MAG_INT_X_ON     << 7 ) + ( MAG_INT_Y_ON         << 6 ) + \
                                  ( MAG_INT_Z_ON     << 5 ) + ( MICR_UNUSED_1        << 4 ) + \
                                  ( MICR_UNUSED_2 << 3 ) + ( MAG_INT_ACTIVE_LOW_HIGH << 2 ) + \
                                  ( MAG_INT_LATCH_ON << 1 ) + ( MAG_INT_ON           << 0 ) )

/*
 * Accelerometer data resolution
 */
#if   ( ( ACCEL_LOW_POWER_ON == 1 ) && ( ACCEL_HIGH_RES_ON == 0 ) )
#define ACCEL_RESOLUTION 8
#elif ( ( ACCEL_LOW_POWER_ON == 0 ) && ( ACCEL_HIGH_RES_ON == 0 ) )
#define ACCEL_RESOLUTION 10
#elif ( ( ACCEL_LOW_POWER_ON == 0 ) && ( ACCEL_HIGH_RES_ON == 1 ) )
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
