#include "custom_flags.h"
#include <stdbool.h>

volatile bool adcDataReady = 0;
volatile bool usartBusy = 0;
volatile bool usartNewData = 0;



/************************************************************
 *                                                          *
 *                        ADC FLAGS                         *
 *                                                          *
 ************************************************************/


/*
 *  @brief Set the adcDataReady flag
 *  @retval void
 */
void setAdcDataReady(void) { adcDataReady = 1; }

/*
 *  @brief Reset the adcDataReady flag
 *  @retval void
 */
void resetAdcDataReady(void) { adcDataReady = 0; }

/*
 *  @brief Get the adcDataReady flag state
 *  @retval adcDataReady flag state
 */
bool isAdcDataReady(void) { return adcDataReady; }



/************************************************************
 *                                                          *
 *                        USART FLAGS                       *
 *                                                          *
 ************************************************************/


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
bool isUsartBusy(void) { return usartBusy; }


/*
 *  @brief Set the usartNewData flag
 *  @retval void
 */
void setUsartNewData(void) { usartNewData = 1; }

/*
 *  @brief Reset the usartNewData flag
 *  @retval void
 */
void resetUsartNewData(void) { usartNewData = 0; }

/*
 *  @brief Get the usartNewData flag state
 *  @retval usartNewData flag state
 */
bool isUsartNewData(void) { return usartNewData; }

