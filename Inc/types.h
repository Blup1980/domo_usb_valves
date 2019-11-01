/*
 * types.h
 *
 *  Created on: 1 nov. 2019
 *      Author: xraemy
 */

#ifndef TYPES_H_
#define TYPES_H_

#include "stm32l0xx.h"

#define USBCMDLENGTH 2

typedef enum onoffstate {ON,OFF}  OnoffstateTypeDef;
typedef enum indicatorstate {IND_OFF, IND_ON, IND_BLINK}  IndicatorstateTypeDef;
typedef enum ssrstate {SSR_OFF, SSR_ON, SSR_PENDING_ON}  SsrstateTypeDef;
typedef struct {
    uint8_t byte[USBCMDLENGTH];
} usbCommand_t;

#endif /* TYPES_H_ */
