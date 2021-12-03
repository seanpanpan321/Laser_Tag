/*
 * fireshot.h
 *
 *  Created on: Nov 6, 2021
 *      Author: seanpanpan
 */

#ifndef __FIRESHOT_H
#define __FIRESHOT_H
#include "stm32f1xx_hal.h"

void fire(uint8_t triggerPressed, uint8_t *bulletCount, uint8_t team, uint8_t charge);


uint8_t reload(uint8_t curBulletCount);

#endif
