/**
 * \file
 *         Device drivers for ADXL34X acceleration sensor in OpenMote-CC2538.
 * \author
 *         Pere Tuset, OpenMote <peretuset@openmote.com>
 */

#include <headers/hw_ints.h>
#include <headers/hw_memmap.h>

#include "i2c.h"
#include "gpio.h"
#include "adxl346.h"

//=========================== define ==========================================

/* ADDRESS AND IDENTIFIER */
#define ADXL34X_ADDRESS                     ( 0x53 )
#define ADXL34X_DEVID_VALUE                 ( 0xE6 )

/* REGISTER ADDRESSES */
#define ADXL34X_DEVID_ADDR                  ( 0x00 )
#define ADXL34X_THRES_TAP_ADDR              ( 0x1D )
#define ADXL34X_OFSX_ADDR                   ( 0x1E )
#define ADXL34X_OFSY_ADDR                   ( 0x1F )
#define ADXL34X_OFSZ_ADDR                   ( 0x20 )
#define ADXL34X_DUR_ADDR                    ( 0x21 )
#define ADXL34X_LATENT_ADDR                 ( 0x22 )
#define ADXL34X_WINDOW_ADDR                 ( 0x23 )
#define ADXL34X_THRESH_ACT_ADDR             ( 0x24 )
#define ADXL34X_THRESH_INACT_ADDR           ( 0x25 )
#define ADXL34X_TIME_INACT_ADDR             ( 0x26 )
#define ADXL34X_ACT_INACT_CTL_ADDR          ( 0x27 )
#define ADXL34X_THRESH_FF_ADDR              ( 0x28 )
#define ADXL34X_TIME_FF_ADDR                ( 0x29 )
#define ADXL34X_TAP_AXES_ADDR               ( 0x2A )
#define ADXL34X_ACT_TAP_STATUS_ADDR         ( 0x2B )
#define ADXL34X_BW_RATE_ADDR                ( 0x2C )
#define ADXL34X_POWER_CTL_ADDR              ( 0x2D )
#define ADXL34X_INT_ENABLE_ADDR             ( 0x2E )
#define ADXL34X_INT_MAP_ADDR                ( 0x2F )
#define ADXL34X_INT_SOURCE_ADDR             ( 0x30 )
#define ADXL34X_DATA_FORMAT_ADDR            ( 0x31 )
#define ADXL34X_DATAX0_ADDR                 ( 0x32 )
#define ADXL34X_DATAX1_ADDR                 ( 0x33 )
#define ADXL34X_DATAY0_ADDR                 ( 0x34 )
#define ADXL34X_DATAY1_ADDR                 ( 0x35 )
#define ADXL34X_DATAZ0_ADDR                 ( 0x36 )
#define ADXL34X_DATAZ1_ADDR                 ( 0x37 )
#define ADXL34X_FIFO_CTL_ADDR               ( 0x38 )
#define ADXL34X_FIFO_STATUS_ADDR            ( 0x39 )
#define ADXL34X_TAP_SIGN_ADDR               ( 0x3A )
#define ADXL34X_ORIENT_CONF_ADDR            ( 0x3B )
#define ADXL34X_ORIENT_ADDR                 ( 0x3C )

/* INT_ENABLE/INT_MAP/INT_SOURCE */
#define ADXL34X_INT_ENABLE_DATA_READY      ( 1 << 7 )
#define ADXL34X_INT_ENABLE_SINGLE_TAP      ( 1 << 6 )
#define ADXL34X_INT_ENABLE_DOUBLE_TAP      ( 1 << 5 )
#define ADXL34X_INT_ENABLE_ACTIVITY        ( 1 << 4 )
#define ADXL34X_INT_ENABLE_INACTIVITY      ( 1 << 3 )
#define ADXL34X_INT_ENABLE_FREE_FALL       ( 1 << 2 )
#define ADXL34X_INT_ENABLE_WATERMARK       ( 1 << 1 )
#define ADXL34X_INT_ENABLE_OVERRUN         ( 1 << 0 )

/* ACT_INACT_CONTROL */
#define ADXL34X_ACT_INACT_CTL_ACT_ACDC     ( 1 << 7 )
#define ADXL34X_ACT_INACT_CTL_ACT_X_EN     ( 1 << 6 )
#define ADXL34X_ACT_INACT_CTL_ACT_Y_EN     ( 1 << 5 )
#define ADXL34X_ACT_INACT_CTL_ACT_Z_EN     ( 1 << 4 )
#define ADXL34X_ACT_INACT_CTL_INACT_ACDC   ( 1 << 3 )
#define ADXL34X_ACT_INACT_CTL_INACT_X_EN   ( 1 << 2 )
#define ADXL34X_ACT_INACT_CTL_INACT_Y_EN   ( 1 << 1 )
#define ADXL34X_ACT_INACT_CTL_INACT_Z_EN   ( 1 << 0 )

/* TAP_AXES */
#define ADXL34X_TAP_AXES_SUPPRESS           ( 1 << 3 )
#define ADXL34X_TAP_AXES_TAP_X_EN           ( 1 << 2 )
#define ADXL34X_TAP_AXES_TAP_Y_EN           ( 1 << 1 )
#define ADXL34X_TAP_AXES_TAP_Z_EN           ( 1 << 0 )

/* ACT_TAP_STATUS */
#define ADXL34X_ACT_TAP_STATUS_ACT_X_SRC    ( 1 << 6 )
#define ADXL34X_ACT_TAP_STATUS_ACT_Y_SRC    ( 1 << 5 )
#define ADXL34X_ACT_TAP_STATUS_ACT_Z_SRC    ( 1 << 4 )
#define ADXL34X_ACT_TAP_STATUS_ASLEEP       ( 1 << 3 )
#define ADXL34X_ACT_TAP_STATUS_TAP_X_SRC    ( 1 << 2 )
#define ADXL34X_ACT_TAP_STATUS_TAP_Y_SRC    ( 1 << 1 )
#define ADXL34X_ACT_TAP_STATUS_TAP_Z_SRC    ( 1 << 0 )

/* BW_RATE */
#define ADXL34X_BW_RATE_LOW_POWER           ( 1 << 4 )
#define ADXL34X_BW_RATE_RATE(x)             ( (x) & 0x0F)

/* POWER CONTROL */
#define ADXL34X_POWER_CTL_LINK              ( 1 << 5 )
#define ADXL34X_POWER_CTL_AUTO_SLEEP        ( 1 << 4 )
#define ADXL34X_POWER_CTL_MEASURE           ( 1 << 3 )
#define ADXL34X_POWER_CTL_SLEEP             ( 1 << 2 )
#define ADXL34X_POWER_CTL_WAKEUP(x)         ( (x) & 0x03 )

/* DATA_FORMAT */
#define ADXL34X_DATA_FORMAT_SELF_TEST       ( 1 << 7 )
#define ADXL34X_DATA_FORMAT_SPI             ( 1 << 6 )
#define ADXL34X_DATA_FORMAT_INT_INVERT      ( 1 << 5 )
#define ADXL34X_DATA_FORMAT_FULL_RES        ( 1 << 3 )
#define ADXL34X_DATA_FORMAT_JUSTIFY         ( 1 << 2 )
#define ADXL34X_DATA_FORMAT_RANGE_PM_2g     ( 0 )
#define ADXL34X_DATA_FORMAT_RANGE_PM_4g     ( 1 )
#define ADXL34X_DATA_FORMAT_RANGE_PM_8g     ( 2 )
#define ADXL34X_DATA_FORMAT_RANGE_PM_16g    ( 3 )

//=========================== variables =======================================


//=========================== prototypes ======================================


//=========================== public ==========================================

void adxl346_init(void) {
    uint8_t config[2];
    uint8_t buffer[32];
    uint8_t samples;

    // Clear the power register
    config[0] = ADXL34X_POWER_CTL_ADDR;
    config[1] = 0x00;
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    // Read the FIFO status register
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_FIFO_STATUS_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &samples);

    // Read the FIFO to clean it
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAX0_ADDR);
    i2c_read_bytes(ADXL34X_ADDRESS, buffer, 6);


    // Write the bandwidth register
    config[0] = ADXL34X_BW_RATE_ADDR;
    config[1] = (ADXL34X_BW_RATE_RATE(13));
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    // Write the FIFO control register
    config[0] = ADXL34X_FIFO_CTL_ADDR;
    config[1] = (0x86);
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    // Write the format register
    config[0] = ADXL34X_DATA_FORMAT_ADDR;
    config[1] = (ADXL34X_DATA_FORMAT_RANGE_PM_2g);
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    // Write the bandwidth register
    config[0] = ADXL34X_INT_ENABLE_ADDR;
    config[1] = (ADXL34X_INT_ENABLE_WATERMARK);
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    // Write the power register
    config[0] = ADXL34X_POWER_CTL_ADDR;
    config[1] = ADXL34X_POWER_CTL_MEASURE;
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

}

void adxl346_reset(void) {
}

uint8_t adxl346_is_present(void) {
    uint8_t is_present;

    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DEVID_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &is_present);

    return (is_present == ADXL34X_DEVID_VALUE);
}

int16_t adxl346_read_x(void) {
    uint8_t acceleration[2];
    int16_t x;

    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAX0_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &acceleration[0]);
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAX1_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &acceleration[1]);

    x = (acceleration[1] << 8) | acceleration[0];

    return x;
}

int16_t adxl346_read_y(void) {
    uint8_t acceleration[2];
    int16_t y;
    
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAY0_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &acceleration[0]);
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAY1_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &acceleration[1]);

    y = (acceleration[1] << 8) | acceleration[0];
    
    return y;
}

int16_t adxl346_read_z(void) {
    uint8_t acceleration[2];
    int16_t z;
    
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAZ0_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &acceleration[0]);
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAZ1_ADDR);
    i2c_read_byte(ADXL34X_ADDRESS, &acceleration[1]);

    z = (acceleration[1] << 8) | acceleration[0];
    
    return z;
}

float adxl346_convert_acceleration(int16_t acceleration) {
    float result = 4.0;
    result *= (acceleration & 0x9FFF);
    return result;
}

bool adxl346_selfTest(bool test){
    uint8_t config[2];

    // Read the data format register
     i2c_read_byte(ADXL34X_DATA_FORMAT_ADDR, &config[1]);

    // Write the data format register
    config[0]  = ADXL34X_DATA_FORMAT_ADDR;
    if (test) config[1] |= ADXL34X_DATA_FORMAT_SELF_TEST;
    else      config[1] &= ~ADXL34X_DATA_FORMAT_SELF_TEST;

    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    return TRUE;
}

void adxl346_calibrate(void){
    int32_t accum_x = 0;
    int32_t accum_y = 0;
    int32_t accum_z = 0;
    uint8_t config[2];
    int8_t offset;

    config[0] = ADXL34X_OFSX_ADDR;
    config[1] = 0;
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    config[0] = ADXL34X_OFSY_ADDR;
    config[1] = 0;
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    config[0] = ADXL34X_OFSZ_ADDR;
    config[1] = 0;
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    for (uint16_t i = 0; i < 100; i++) {
        uint16_t x, y, z;

        adxl346_readSample(&x, &y, &z);
        accum_x += x;
        accum_y += y;
        accum_z += z;
    }

    offset    = (accum_x) / 100;
    config[0] = ADXL34X_OFSX_ADDR;
    config[1] = -(64 * offset / 256);
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    offset    = (accum_y) / 100;
    config[0] = ADXL34X_OFSY_ADDR;
    config[1] = -(64 * offset / 256);
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));

    offset    = (accum_z) / 100;
    config[0] = ADXL34X_OFSZ_ADDR;
    config[1] = -(64 * offset / 256);
    i2c_write_bytes(ADXL34X_ADDRESS, config, sizeof(config));
}

void adxl346_setCallback(void (*cb_func)(void)){

    /* Set the pin as input */
    GPIOPinTypeGPIOInput(GPIO_B_BASE, GPIO_PIN_2);

    /* Set the edge of the interrupt */
    GPIOIntTypeSet(GPIO_B_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);
    GPIOPowIntTypeSet(GPIO_B_BASE, GPIO_PIN_2, GPIO_RISING_EDGE);

    /* Enable the interrupt wakeup capability of the port */
    GPIOIntWakeupEnable(GPIO_IWE_PORT_B);

    /* Register the interrupt */
    GPIOPortIntRegister(GPIO_B_BASE, cb_func);

    /* Clear and enable the interrupt */
    GPIOPinIntClear(GPIO_B_BASE, GPIO_PIN_2);
    GPIOPinIntEnable(GPIO_B_BASE, GPIO_PIN_2);

}

bool adxl346_readSample(uint16_t* x, uint16_t* y, uint16_t* z){
    uint16_t acceleration[3];
    uint8_t  address[6] = {ADXL34X_DATAX0_ADDR, ADXL34X_DATAX1_ADDR,
                           ADXL34X_DATAY0_ADDR, ADXL34X_DATAY1_ADDR,
                           ADXL34X_DATAZ0_ADDR, ADXL34X_DATAZ1_ADDR};
    uint8_t scratch[2];

    // Iterate for all addresses, each direction has two addresses
    for (uint8_t i = 0; i < sizeof(address); i += 2)
    {
        // I2C write register address
         i2c_write_byte(ADXL34X_ADDRESS, address[i + 0]);

        // I2C read acceleration value
         i2c_read_byte(ADXL34X_ADDRESS, &scratch[0]);

        // I2C write register address
         i2c_write_byte(ADXL34X_ADDRESS, address[i + 1]);

        // I2C read acceleration value
         i2c_read_byte(ADXL34X_ADDRESS, &scratch[1]);

        // Convert ADXL34X data
        acceleration[i>>1] = (scratch[1] << 8) | scratch[0];
    }

    // Update acceleration variables
    *x = acceleration[0];
    *y = acceleration[1];
    *z = acceleration[2];

    return TRUE;
}

bool adxl346_readSamples(uint8_t* buffer, uint8_t samples){

    // Read the data format register
     i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_DATAX0_ADDR);

    // Read the number of samples
     i2c_read_bytes(ADXL34X_ADDRESS, buffer, samples);

    return true;
}

uint8_t adxl346_samplesAvailable(void){
    uint8_t samples;

    // Read the data format register
    i2c_write_byte(ADXL34X_ADDRESS, ADXL34X_FIFO_STATUS_ADDR);

    // Read the data format register
    i2c_read_byte(ADXL34X_ADDRESS, &samples);

    return (samples & 0x3F);
}

//=========================== private =========================================

