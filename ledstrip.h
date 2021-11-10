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

void disableMode(uint8_t *lives, uint8_t target1, uint8_t target2, uint8_t target3);

void allGreen();

#endif
