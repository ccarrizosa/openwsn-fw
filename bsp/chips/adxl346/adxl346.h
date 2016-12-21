/**
 * \file
 *         Device drivers for ADXL346 acceleration sensor in OpenMote-CC2538.
 * \author
 *         Pere Tuset, OpenMote <peretuset@openmote.com>
 */

#ifndef __ADXL346_H__
#define __ADXL346_H__

#include "opendefs.h"

void adxl346_init(void);
void adxl346_reset(void);
uint8_t adxl346_is_present(void);
int16_t adxl346_read_x(void);
int16_t adxl346_read_y(void);
int16_t adxl346_read_z(void);
float adxl346_convert_acceleration(int16_t acceleration);
bool adxl346_selfTest(bool test);
void adxl346_calibrate(void);
void adxl346_setCallback(void (*cb_func)(void));
bool adxl346_readSample(uint16_t* x, uint16_t* y, uint16_t* z);
bool adxl346_readSamples(uint8_t* buffer, uint8_t samples);
uint8_t adxl346_samplesAvailable(void);
#endif /* ifndef __ADXL346_H__ */
