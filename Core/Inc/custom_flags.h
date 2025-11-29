/*
 * custom_flags.h
 *
 *  Created on: Nov 16, 2025
 *      Author: space-monkey
 */

#ifndef INC_CUSTOM_FLAGS_H_
#define INC_CUSTOM_FLAGS_H_

#include <stdint.h>

void setAdcDataReady(void);
void resetAdcDataReady(void);
uint8_t isAdcDataReady(void);

void setUsartBusy(void);
void resetUsartBusy(void);
uint8_t isUsartBusy(void);

#endif /* INC_CUSTOM_FLAGS_H_ */
