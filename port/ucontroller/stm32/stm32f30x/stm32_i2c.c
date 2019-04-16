/*
 *   openMMC  --
 *
 *   Copyright (C) 2019 CERN
 *
 *   This program is free software: you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, either version 3 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*!
 * @file stm_i2c.c
 * @author Adam Wujek <adam.wujek@cern.ch>, CERN
 * @date April 2019
 *
 * @brief I2C driver for STM32f30x
 */

#include "port.h"
#include "string.h"
#include "i2c_mapping.h"
#include "stm32f30x_i2c_cpal.h"
#include "stm32f30x.h"
#include "task.h"
#include "semphr.h"

/* If the speed of a uC is changed this field has to be adjusted!
 * Specifies the I2C_TIMINGR_register value.
 * This parameter calculated by referring to I2C initialization.
 * section in Reference manual. 8MHz, RM page 849, table 147 */
#define I2C_TIMING              0x10420f13

#define SLAVE_MASK 0xFF

__IO uint32_t RX_cint = 0;
__IO uint32_t TX_cint = 0;

CPAL_InitTypeDef* I2C_DevStructure[I2C_NUM_INTERFACE] = {
    &I2C1_DevStructure,
    &I2C2_DevStructure,
    &I2C3_DevStructure
};

static SemaphoreHandle_t i2c_acc_mutex[I2C_NUM_INTERFACE];
/* For notification from ISR use xTaskToNotify instead of semaphore for
 * performance reasons. However, performance is not critical here. */
static TaskHandle_t i2c_xTaskToNotify[I2C_NUM_INTERFACE];

#define MAX_BUFF_SIZE           200
uint8_t tTxBuffer[MAX_BUFF_SIZE];
uint8_t tRxBuffer[MAX_BUFF_SIZE];

CPAL_TransferTypeDef  sRxStructure[I2C_NUM_INTERFACE], sTxStructure[I2C_NUM_INTERFACE];

void CPAL_I2C_TC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
    BaseType_t xHigherPriorityTaskWoken = pdTRUE;
    uint8_t id;

    switch(pDevInitStruct->CPAL_Dev){
	case CPAL_I2C1:
	    id = I2C1_ID;
	    break;
	case CPAL_I2C2:
	    id = I2C2_ID;
	    break;
	case CPAL_I2C3:
	    id = I2C3_ID;
	    break;
	default:
	    return;
    }

    /* At this point xTaskToNotify should not be NULL as a transmission was
    in progress. */
    configASSERT( i2c_xTaskToNotify[id] != NULL );

    /* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR( i2c_xTaskToNotify[id], &xHigherPriorityTaskWoken );

    /* There are no transmissions in progress, so no tasks to notify. */
    i2c_xTaskToNotify[id] = NULL;

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

/**
  * @brief  Manages the End of Tx transfer event
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
    TX_cint++;
    CPAL_I2C_TC_UserCallback(pDevInitStruct);
}

/**
  * @brief  Manages the End of Rx transfer event
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
    RX_cint++;
    CPAL_I2C_TC_UserCallback(pDevInitStruct);
}

void cpal_read(I2C_ID_T id)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 2000 );

    /* At this point i2dc_xTaskToNotify should be NULL as no transmission
    is in progress. */
    configASSERT( i2c_xTaskToNotify[id] == NULL );

    i2c_xTaskToNotify[id] = xTaskGetCurrentTaskHandle();


    I2C_DevStructure[id]->CPAL_State = CPAL_STATE_READY;
    /* Start the transmission by calling the function shown above. */
    if (CPAL_I2C_Read(I2C_DevStructure[id]) == CPAL_PASS) {

    }

    /* Wait to be notified that the transmission is complete.  Note the first
    parameter is pdTRUE, which has the effect of clearing the task's notification
    value back to 0, making the notification value act like a binary (rather than
    a counting) semaphore.  */
    while(!ulTaskNotifyTake(pdTRUE, xMaxBlockTime)) {
	printf("%s timeout\r\n", __func__);
    }
}

void cpal_write(I2C_ID_T id)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 2000 );

    /* At this point i2dc_xTaskToNotify should be NULL as no transmission
    is in progress. */
    configASSERT( i2c_xTaskToNotify[id] == NULL );

    i2c_xTaskToNotify[id] = xTaskGetCurrentTaskHandle();


    I2C_DevStructure[id]->CPAL_State = CPAL_STATE_READY;
    /* Start the transmission by calling the function shown above. */
    if (CPAL_I2C_Write(I2C_DevStructure[id]) == CPAL_PASS) {

    }

    /* Wait to be notified that the transmission is complete.  Note the first
    parameter is pdTRUE, which has the effect of clearing the task's notification
    value back to 0, making the notification value act like a binary (rather than
    a counting) semaphore.  */
    while(!ulTaskNotifyTake(pdTRUE, xMaxBlockTime)) {
	printf("%s timeout\r\n", __func__);
    }
}

int xI2CMasterWrite(I2C_ID_T id, uint8_t addr, uint8_t *tx_buff, uint8_t tx_len)
{
    int ret;
    while( xSemaphoreTake(i2c_acc_mutex[id], ( TickType_t ) 100 ) != pdTRUE )
	printf("%s: Unable to take sempaphore for i2c %d. Try once more time.\r\n", __func__, id);

    sTxStructure[id].wAddr1 = addr;
    sTxStructure[id].wAddr2 = 0;
    sTxStructure[id].wNumData = tx_len;
    sTxStructure[id].pbBuffer = tx_buff;
    cpal_write(id);

    ret = tx_len - sTxStructure[id].wNumData;
    xSemaphoreGive(i2c_acc_mutex[id]);

    return ret;
}

int xI2CMasterRead(I2C_ID_T id, uint8_t addr, uint8_t *rx_buff, uint8_t rx_len)
{
    int ret;
    while( xSemaphoreTake(i2c_acc_mutex[id], ( TickType_t ) 100 ) != pdTRUE )
	printf("%s: Unable to take sempaphore for i2c %d. Try once more time.\r\n", __func__, id);

    sRxStructure[id].wAddr1 = addr;
    sRxStructure[id].wAddr2 = 0;
    sRxStructure[id].wNumData = rx_len;
    sRxStructure[id].pbBuffer = rx_buff;
    cpal_read(id);

    ret = rx_len - sRxStructure[id].wNumData;
    xSemaphoreGive(i2c_acc_mutex[id]);

    return ret;
}

int xI2CMasterWriteRead(I2C_ID_T id, uint8_t addr, uint8_t cmd, uint8_t *rx_buff, uint8_t rx_len)
{
    int ret;

    while( xSemaphoreTake(i2c_acc_mutex[id], ( TickType_t ) 100 ) != pdTRUE )
	printf("%s: Unable to take sempaphore for i2c %d. Try once more time.\r\n", __func__, id);

    sRxStructure[id].wAddr1 = addr;
    sRxStructure[id].wAddr2 = cmd;
    sRxStructure[id].wNumData = rx_len;
    sRxStructure[id].pbBuffer = rx_buff;
    cpal_read(id);
    ret = rx_len - sRxStructure[id].wNumData;

    xSemaphoreGive(i2c_acc_mutex[id]);

    return ret;
}

void vI2CConfig( I2C_ID_T id, uint32_t speed )
{

    /* for now the speed parameter is ignored */

    memset(&sRxStructure[id], 0, sizeof(sRxStructure[id]));
    memset(&sTxStructure[id], 0, sizeof(sTxStructure[id]));

    printf("I2C init %d\r\n", id);
    /* Set all fields to default values */
    CPAL_I2C_StructInit(I2C_DevStructure[id]);

    I2C_DevStructure[id]->pCPAL_TransferRx = &sRxStructure[id];
    I2C_DevStructure[id]->pCPAL_TransferTx = &sTxStructure[id];

    switch (id) {
    case I2C1_ID:
	/* Set HSI as I2C clock source */
	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	/* Configure the device structure */
	I2C1_DevStructure.CPAL_Mode = CPAL_MODE_MASTER;
	I2C1_DevStructure.wCPAL_Options =  CPAL_OPT_I2C_AUTOMATIC_END;
	#ifdef CPAL_I2C_DMA_PROGMODEL
	I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_DMA;
	#elif defined (CPAL_I2C_IT_PROGMODEL)
	I2C1_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
	#else
	#error "Please select one of the programming model (in stm32f30x_i2c_cpal_conf.h)"
	#endif
	I2C1_DevStructure.pCPAL_I2C_Struct->I2C_Timing = I2C_TIMING;
	I2C1_DevStructure.pCPAL_I2C_Struct->I2C_OwnAddress1 = 0;
	I2C1_DevStructure.pCPAL_TransferRx = &sRxStructure[id];
	I2C1_DevStructure.pCPAL_TransferTx = &sTxStructure[id];

        break;

    case I2C2_ID:
	/* this one should be configuraed as a slave */
        break;

    case I2C3_ID:
	/* Set HSI as I2C clock source */
	RCC_I2CCLKConfig(RCC_I2C1CLK_HSI);
	/* Configure the device structure */
	I2C3_DevStructure.CPAL_Mode = CPAL_MODE_MASTER;
	I2C3_DevStructure.wCPAL_Options =  CPAL_OPT_NO_MEM_ADDR | CPAL_OPT_I2C_AUTOMATIC_END;
	I2C3_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;

	I2C3_DevStructure.pCPAL_I2C_Struct->I2C_Timing = I2C_TIMING;
	I2C3_DevStructure.pCPAL_I2C_Struct->I2C_OwnAddress1 = 0;
	I2C3_DevStructure.pCPAL_TransferRx = &sRxStructure[id];
	I2C3_DevStructure.pCPAL_TransferTx = &sTxStructure[id];
        break;

    default:
        return;
    }
    /* create mutex */
    i2c_acc_mutex[id] = xSemaphoreCreateMutex();
    if (!i2c_acc_mutex[id])
	printf("Unable to create mutex for i2c %d\r\n", id);

    i2c_xTaskToNotify[id] = NULL;

    /* Initialize CPAL device with the selected parameters,
     * set pins directionetc */
    CPAL_I2C_Init(I2C_DevStructure[id]);


}
