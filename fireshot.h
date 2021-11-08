/*
 * fireshot.h
 *
 *  Created on: Nov 6, 2021
 *      Author: seanpanpan
 */

#ifndef __FIRESHOT_H
#define __FIRESHOT_H
#include "stm32f1xx_hal.h"

void fire(uint8_t triggerPressed, uint8_t *bulletCount);

void isShot(uint8_t target1, uint8_t target2, uint8_t target3);

uint8_t reload(uint8_t curBulletCount);

#endif
