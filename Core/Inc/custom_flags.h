/*
 * custom_flags.h
 *
 *  Created on: Nov 16, 2025
 *      Author: space-monkey
 */

#ifndef INC_CUSTOM_FLAGS_H_
#define INC_CUSTOM_FLAGS_H_

#include <stdbool.h>

void setAdcDataReady(void);
void resetAdcDataReady(void);
bool isAdcDataReady(void);

void setUsartBusy(void);
void resetUsartBusy(void);
bool isUsartBusy(void);

void setUsartNewData(void);
void resetUsartNewData(void);
bool isUsartNewData(void);

#endif /* INC_CUSTOM_FLAGS_H_ */
