/*
 * custom_flags.c
 *
 *  Created on: Nov 16, 2025
 *      Author: space-monkey
 */

#include "custom_flags.h"
#include <stdbool.h>

volatile bool adcDataReady = 0;
volatile bool usartBusy = 0;


/*
 *  @brief Set the adcDataReady flag
 *  @retval void
 */
void setAdcDataReady(void) { adcDataReady = 1; }

/*
 *  @brief Reset the adcDataReady flag
 *  @retval void
 */
inline void resetAdcDataReady(void) { adcDataReady = 0; }

/*
 *  @brief Get the adcDataReady flag state
 *  @retval adcDataReady flag state
 */
uint8_t isAdcDataReady(void) { return adcDataReady; }



/*
 *  @brief Set the usartBusy flag
 *  @retval void
 */
void setUsartBusy(void) { usartBusy = 1; }

/*
 *  @brief Reset the usartBusy flag
 *  @retval void
 */
void resetUsartBusy(void) { usartBusy = 0; }

/*
 *  @brief Get the usartBusy flag state
 *  @retval usartBusy flag state
 */
uint8_t isUsartBusy(void) { return usartBusy; }
