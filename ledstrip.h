/*
 * ledstrip.h
 *
 *  Created on: Nov 6, 2021
 *      Author: seanpanpan
 */

#ifndef __LEDSTRIP_H
#define __LEDSTRIP_H
#include "stm32f1xx_hal.h"


void targetShot(uint8_t isShot);

void bulletIndicator(uint8_t bulletCount, uint8_t isReloading);

void disableMode();

#endif
