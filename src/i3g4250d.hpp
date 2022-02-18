/*----------------------------------------------------------------------------*/
/*
    Ryan Sullivan

    Module Name     : i3g4250d
    Description     : defines for the I3G4250D gyroscope
*/
/*----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------*/
/*-constant-definitions-------------------------------------------------------*/
/*----------------------------------------------------------------------------*/

/*
 * Read, write and auto increment macros
 */
#define GYRO_SET_READ_BIT(address)  (0b10000000 + address)
#define GYRO_SET_WRITE_BIT(address) (address)

#define GYRO_SET_INCREMENT_BIT(address) (0b01000000 + address)

/*
 * Register addresses
 */
#define GYRO_CTRL_REG_1_ADDR 0x20
#define GYRO_CTRL_REG_2_ADDR 0x21
#define GYRO_CTRL_REG_3_ADDR 0x22
#define GYRO_CTRL_REG_4_ADDR 0x23
#define GYRO_CTRL_REG_5_ADDR 0x24

#define GYRO_OUT_REG_X_L_ADDR 0x28
#define GYRO_OUT_REG_X_H_ADDR 0x29
#define GYRO_OUT_REG_Y_L_ADDR 0x2A
#define GYRO_OUT_REG_Y_H_ADDR 0x2B
#define GYRO_OUT_REG_Z_L_ADDR 0x2C
#define GYRO_OUT_REG_Z_H_ADDR 0x2D

/*
 * Parameter defines
 */
#define GYRO_DATA_RATE_100HZ 0b00
#define GYRO_DATA_RATE_200HZ 0b01
#define GYRO_DATA_RATE_400HZ 0b10
#define GYRO_DATA_RATE_800HZ 0b11

#define GYRO_DATA_RANGE_245DPS  0b00
#define GYRO_DATA_RANGE_500DPS  0b01
#define GYRO_DATA_RANGE_2000DPS 0b10

/*
 * Register values
 *
 * gyroscope control register 1 MSB to LSB
 */
#define GYRO_DATA_RATE GYRO_DATA_RATE_100HZ
#define GYRO_BANDWIDTH 0b11 //??
#define GYRO_POWER_ON  1    // enable
#define GYRO_Z_AXIS_ON 1    // enable
#define GYRO_Y_AXIS_ON 1    // enable
#define GYRO_X_AXIS_ON 1    // enable
#define GYRO_CTRL_REG_1_VAL                                                                       \
    ((GYRO_DATA_RATE << 6) + (GYRO_BANDWIDTH << 4) + (GYRO_POWER_ON << 3) + (GYRO_Z_AXIS_ON << 2) \
     + (GYRO_Y_AXIS_ON << 1) + (GYRO_X_AXIS_ON << 0))

/*
 * gyroscope control register 2 MSB to LSB
 */
#define GCR1_UNUSED_1      0      // unused
#define GCR1_UNUSED_2      0      // unused
#define GYRO_FILTER_MODE   0b00   // normal mode
#define GYRO_FILTER_CUTOFF 0b0100 // middle'ish
#define GYRO_CTRL_REG_2_VAL                                                \
    ((GCR1_UNUSED_1 << 7) + (GCR1_UNUSED_2 << 6) + (GYRO_FILTER_MODE << 4) \
     + (GYRO_FILTER_CUTOFF << 0))

/*
 * gyroscope control register 3 MSB to LSB
 */
#define GYRO_INT1_ON         0 // disable
#define GYRO_INT1_BOOT       0 // disable
#define GYRO_INT1_ACTIVE_L_H 0 // active high
#define GYRO_PP_OD           0 // push pull
#define GYRO_INT2_DRDY       1 // enable
#define GYRO_INT2_WATERMARK  0 // disable
#define GYRO_INT2_OVERRUN    0 // disable
#define GYRO_INT2_EMPTY      0 // disable
#define GYRO_CTRL_REG_3_VAL                                                                        \
    ((GYRO_INT1_ON << 7) + (GYRO_INT1_BOOT << 6) + (GYRO_INT1_ACTIVE_L_H << 5) + (GYRO_PP_OD << 4) \
     + (GYRO_INT2_DRDY << 3) + (GYRO_INT2_WATERMARK << 2) + (GYRO_INT2_OVERRUN << 1)               \
     + (GYRO_INT2_EMPTY << 0))

/*
 * gyroscope control register 4 MSB to LSB
 */
#define GCR4_UNUSED_1   0 // unused
#define GYRO_B_L_ENDIAN 0 // little endian
#define GYRO_SCALE      GYRO_DATA_RANGE_500DPS
#define GCR4_UNUSED_2   0    // unused
#define GYRO_SELF_TEST  0b00 // normal mode
#define GYRO_SPI_MODE   0    // 4 wire
#define GYRO_CTRL_REG_4_VAL                                                                   \
    ((GCR4_UNUSED_1 << 7) + (GYRO_B_L_ENDIAN << 6) + (GYRO_SCALE << 4) + (GCR4_UNUSED_2 << 3) \
     + (GYRO_SELF_TEST << 1) + (GYRO_SPI_MODE << 0))

/*
 * gyroscope control register 5 MSB to LSB
 */
#define GYRO_REBOOT_MEM   0    // disable
#define GYRO_FIFO_ON      0    // disable
#define GCR5_UNUSED_1     0    // unused
#define GYRO_HP_FILTER_ON 0    // disable
#define GYRO_INT1_MODE    0b00 // no filtering
#define GYRO_OUT_MODE     0b00 // no filtering
#define GYRO_CTRL_REG_5_VAL                                              \
    ((GYRO_REBOOT_MEM << 7) + (GYRO_FIFO_ON << 6) + (GCR5_UNUSED_1 << 5) \
     + (GYRO_HP_FILTER_ON << 4) + (GYRO_INT1_MODE << 2) + (GYRO_OUT_MODE << 0))

/*
 * Gyroscope data range
 */
#if(GYRO_SCALE == GYRO_DATA_RANGE_245DPS)
#define GYRO_RANGE 245
#elif(GYRO_SCALE == GYRO_DATA_RANGE_500DPS)
#define GYRO_RANGE 500
#elif(GYRO_SCALE == GYRO_DATA_RANGE_2000DPS)
#define GYRO_RANGE 2000
#else
#error Invalid values
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
