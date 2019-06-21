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
#include "board_ipmb.h"
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

extern I2C_TypeDef* CPAL_I2C_DEVICE[3];

static SemaphoreHandle_t i2c_acc_mutex[I2C_NUM_INTERFACE];
/* For notification from ISR use xTaskToNotify instead of semaphore for
 * performance reasons. However, performance is not critical here. */
static TaskHandle_t i2c_xTaskToNotify_read[I2C_NUM_INTERFACE];
static TaskHandle_t i2c_xTaskToNotify_write[I2C_NUM_INTERFACE];
static uint8_t recv_msg[i2cMAX_MSG_LENGTH];
static uint8_t recv_bytes;


static __IO uint8_t i2c_isr_poll_done = 0;

CPAL_TransferTypeDef  sRxStructure[I2C_NUM_INTERFACE], sTxStructure[I2C_NUM_INTERFACE];

static void CPAL_I2C_TC_UserCallback_poll(CPAL_InitTypeDef* pDevInitStruct);
static void cpal_write_spinlock(I2C_ID_T id);
static void cpal_read_spinlock(I2C_ID_T id);

/* Pointer to a function handling IRQ. Firstly initialized with a poll version.
 * Before Scheduler is running, there is not task context. So it is not
 * possible to unblock code waiting on semaphore. */
static void (* CPAL_I2C_TC_UserCallbackRead_func)(CPAL_InitTypeDef* pDevInitStruct) = CPAL_I2C_TC_UserCallback_poll;
static void (* CPAL_I2C_TC_UserCallbackWrite_func)(CPAL_InitTypeDef* pDevInitStruct) = CPAL_I2C_TC_UserCallback_poll;

static void (* cpal_write_func)(I2C_ID_T id) = cpal_write_spinlock;
static void (* cpal_read_func)(I2C_ID_T id) = cpal_read_spinlock;

void CPAL_I2C_ERR_UserCallback(CPAL_DevTypeDef pDevInstance, uint32_t DeviceError)
{
//     printf("%s, dev %d, error 0x%x\n", __func__, pDevInstance + 1, (int)DeviceError);
    if (I2C_DevStructure[pDevInstance]->CPAL_State == CPAL_STATE_BUSY_RX) {
// 	printf("%s, read callback\n", __func__);
	/* If in read, call the callback. Upper layers shall handle read of 0
	 * bytes */
	CPAL_I2C_RXTC_UserCallback(I2C_DevStructure[pDevInstance]);
	return;
    }

    if (I2C_DevStructure[pDevInstance]->CPAL_State == CPAL_STATE_BUSY_TX) {
	// 	printf("%s, write callback\n", __func__);
	/* If in write, call the callback. Upper layers shall handle write of 0
	 * bytes */
	CPAL_I2C_TXTC_UserCallback(I2C_DevStructure[pDevInstance]);
	return;
    }
    printf("%s, unknown state... 0x%x\n", __func__, (int)I2C_DevStructure[pDevInstance]->CPAL_State);
}

static void CPAL_I2C_TC_UserCallback_read(CPAL_InitTypeDef* pDevInitStruct)
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
    configASSERT( i2c_xTaskToNotify_read[id] != NULL );

    /* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR( i2c_xTaskToNotify_read[id], &xHigherPriorityTaskWoken );

    /* There are no transmissions in progress, so no tasks to notify. */
    i2c_xTaskToNotify_read[id] = NULL;

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void CPAL_I2C_TC_UserCallback_write(CPAL_InitTypeDef* pDevInitStruct)
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
    configASSERT( i2c_xTaskToNotify_write[id] != NULL );

    /* Notify the task that the transmission is complete. */
    vTaskNotifyGiveFromISR( i2c_xTaskToNotify_write[id], &xHigherPriorityTaskWoken );

    /* There are no transmissions in progress, so no tasks to notify. */
    i2c_xTaskToNotify_write[id] = NULL;

    /* If xHigherPriorityTaskWoken is now set to pdTRUE then a context switch
    should be performed to ensure the interrupt returns directly to the highest
    priority task.  The macro used for this purpose is dependent on the port in
    use and may be called portEND_SWITCHING_ISR(). */
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
}

static void CPAL_I2C_TC_UserCallback_poll(CPAL_InitTypeDef* pDevInitStruct)
{
    i2c_isr_poll_done = 1;
}

/**
  * @brief  Manages the End of Tx transfer event
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_TXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
    TX_cint++;
    (*CPAL_I2C_TC_UserCallbackWrite_func)(pDevInitStruct);
}

/**
  * @brief  Manages the End of Rx transfer event
  * @param  pDevInitStruct
  * @retval None
  */
void CPAL_I2C_RXTC_UserCallback(CPAL_InitTypeDef* pDevInitStruct)
{
    RX_cint++;
    (*CPAL_I2C_TC_UserCallbackRead_func)(pDevInitStruct);
}


static void cpal_read(I2C_ID_T id)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 2000 );

    /* At this point i2dc_xTaskToNotify should be NULL as no transmission
    is in progress. */
    configASSERT( i2c_xTaskToNotify_read[id] == NULL );

    i2c_xTaskToNotify_read[id] = xTaskGetCurrentTaskHandle();

    if (CPAL_I2C_Read(I2C_DevStructure[id]) != CPAL_PASS) {
	printf("%s read failed state 0x%x error 0x%x\n", __func__,
	       (int) I2C_DevStructure[id]->CPAL_State,
	       (int) I2C_DevStructure[id]->wCPAL_DevError);
	vTaskDelay(10);
	i2c_xTaskToNotify_read[id] = NULL;
	return;
    }

    /* Wait to be notified that the transmission is complete.  Note the first
    parameter is pdTRUE, which has the effect of clearing the task's notification
    value back to 0, making the notification value act like a binary (rather than
    a counting) semaphore.  */
    while(!ulTaskNotifyTake(pdTRUE, xMaxBlockTime)) {
// 	printf("%s timeout\r\n", __func__);
    }
}

static void cpal_write(I2C_ID_T id)
{
    const TickType_t xMaxBlockTime = pdMS_TO_TICKS( 2000 );
    CPAL_StateTypeDef saved_state;
    CPAL_ModeTypeDef saved_mode;
    uint32_t saved_error;

    /* At this point i2dc_xTaskToNotify should be NULL as no transmission
    is in progress. */
    configASSERT( i2c_xTaskToNotify_write[id] == NULL );

    i2c_xTaskToNotify_write[id] = xTaskGetCurrentTaskHandle();


    /* For IPMB_I2C interface use writes in master mode */
    if (id == IPMB_I2C) {
	/* save I2C state */
	saved_state = I2C_DevStructure[id]->CPAL_State;
	saved_mode = I2C_DevStructure[id]->CPAL_Mode;
	saved_error = I2C_DevStructure[id]->wCPAL_DevError;
	__CPAL_I2C_HAL_DISABLE_ADDRIE_IT(id);
	/* Waiting for read in slave mode may be pending, force a write in
	 * master mode */
	I2C_DevStructure[id]->CPAL_State = CPAL_STATE_READY;
	I2C_DevStructure[id]->CPAL_Mode = CPAL_MODE_MASTER;
	I2C_DevStructure[id]->wCPAL_DevError = 0;
    }

    /* Start the transmission by calling the function shown above. */
    while (CPAL_I2C_Write(I2C_DevStructure[id]) != CPAL_PASS) {
	printf("%s write failed\n", __func__);
	vTaskDelay(1);
    }


    /* Wait to be notified that the transmission is complete.  Note the first
    parameter is pdTRUE, which has the effect of clearing the task's notification
    value back to 0, making the notification value act like a binary (rather than
    a counting) semaphore.  */
    while(!ulTaskNotifyTake(pdTRUE, xMaxBlockTime)) {
	printf("%s timeout\r\n", __func__);
    }

//     printf("%s restore state %d, current %d\n", __func__, saved_state, I2C_DevStructure[id]->CPAL_State);
    /* If write for IPMB device */
    if (id == IPMB_I2C) {
	/* Restore the saved mode and state */
	I2C_DevStructure[id]->CPAL_Mode = saved_mode;
	__CPAL_I2C_HAL_ENABLE_STOPIE_IT(id);
	__CPAL_I2C_HAL_ENABLE_ADDRIE_IT(id);
	I2C_DevStructure[id]->wCPAL_DevError = saved_error;
	/* If I2C was waiting for a read before, then resume it */
	if (saved_state == CPAL_STATE_BUSY_RX) {
	    if (CPAL_I2C_Read(I2C_DevStructure[id]) == CPAL_FAIL) {
	    }
	} else {
	    I2C_DevStructure[id]->CPAL_State = saved_state;
	}
    }

}

/* Before scheduler (function vTaskStartScheduler()) is running the main init
 * code is running outside a task. By this it is not possible to wake it
 * with vTaskNotify*() nor xSemaphoreGive() from the ISR. Instead use
 * i2c_isr_poll_done variable for notification.  */
static void cpal_write_spinlock(I2C_ID_T id)
{
    printf("%s\n", __func__);
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {

	i2c_isr_poll_done = 0;
	if (CPAL_I2C_Write(I2C_DevStructure[id]) == CPAL_PASS) {
	}

	while(!i2c_isr_poll_done) {
	    /* Spin until transmission is done */
	}

	return;
    } else {
	/* Scheduler is already running, so use normal functions. */
	cpal_write_func = cpal_write;
	CPAL_I2C_TC_UserCallbackWrite_func = CPAL_I2C_TC_UserCallback_write;
	(*cpal_write_func)(id);
    }
}

/* Before scheduler (function vTaskStartScheduler()) is running the main init
 * code is running outside a task. By this it is not possible to wake it
 * with vTaskNotify*() nor xSemaphoreGive() from the ISR. Instead use
 * i2c_isr_poll_done variable for notification.  */
static void cpal_read_spinlock(I2C_ID_T id)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_RUNNING) {

	i2c_isr_poll_done = 0;
	if (CPAL_I2C_Read(I2C_DevStructure[id]) == CPAL_PASS) {
	}

	while(!i2c_isr_poll_done) {
	    /* Spin until transmission is done */
	}

	return;
    } else {
	/* Scheduler is already running, so use normal functions. */
	cpal_read_func = cpal_read;
	CPAL_I2C_TC_UserCallbackRead_func = CPAL_I2C_TC_UserCallback_read;
	(*cpal_read_func)(id);
    }
}

/* expects i2c addr < 0x80 */
int xI2CMasterWrite(I2C_ID_T id, uint8_t addr, uint8_t *tx_buff, uint8_t tx_len)
{
    int ret;

    addr <<= 1;
//    printf("%s: id %d addr 0x%x tx_len %d\n", __func__, id, addr, tx_len);
    while( xSemaphoreTake(i2c_acc_mutex[id], ( TickType_t ) 2000 ) != pdTRUE )
	printf("%s: Unable to take sempaphore for i2c %d. Try once more time.\r\n", __func__, id);

//     printf("%s addr 0x%x size %d\r\n", __func__, addr, tx_len);
    sTxStructure[id].wAddr1 = addr;
    sTxStructure[id].wAddr2 = 0;
    sTxStructure[id].wNumData = tx_len;
    sTxStructure[id].pbBuffer = tx_buff;
//     printf("%s to write %d\n", __func__, tx_len);

    (*cpal_write_func)(id);

    ret = tx_len - sTxStructure[id].wNumData;
//    printf("%s wrote %d %d %d\n", __func__, ret, tx_len, (int) sTxStructure[id].wNumData);
    xSemaphoreGive(i2c_acc_mutex[id]);

    return ret;
}

/* expects i2c addr < 0x80 */
int xI2CMasterRead(I2C_ID_T id, uint8_t addr, uint8_t *rx_buff, uint8_t rx_len)
{
    int ret;

    addr <<= 1;
    while( xSemaphoreTake(i2c_acc_mutex[id], ( TickType_t ) 100 ) != pdTRUE )
	printf("%s: Unable to take sempaphore for i2c %d. Try once more time.\r\n", __func__, id);

    printf("%s addr 0x%x size %d\r\n", __func__, addr, rx_len);
    sRxStructure[id].wAddr1 = addr;
    sRxStructure[id].wAddr2 = 0;
    sRxStructure[id].wNumData = rx_len;
    sRxStructure[id].pbBuffer = rx_buff;
    (*cpal_read_func)(id);

    ret = rx_len - sRxStructure[id].wNumData;
    xSemaphoreGive(i2c_acc_mutex[id]);

    return ret;
}

/* expects i2c addr < 0x80 */
int xI2CMasterWriteRead(I2C_ID_T id, uint8_t addr, uint8_t cmd, uint8_t *rx_buff, uint8_t rx_len)
{
    int ret;

    addr <<= 1;
    while( xSemaphoreTake(i2c_acc_mutex[id], ( TickType_t ) 100 ) != pdTRUE )
	printf("%s: Unable to take sempaphore for i2c %d. Try once more time.\r\n", __func__, id);

//     printf("%s addr 0x%x size %d cmd 0x%x\r\n", __func__, addr, rx_len, cmd);
    sRxStructure[id].wAddr1 = addr;
    sRxStructure[id].wAddr2 = cmd;
    sRxStructure[id].wNumData = rx_len;
    sRxStructure[id].pbBuffer = rx_buff;
    (*cpal_read_func)(id);
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
	/* This one should be configuraed as a RX slave, then used as a master
	 * for TX */
	/* TODO: Mode should be a part of configuration */
	configASSERT(id == IPMB_I2C);
	/* Set HSI as I2C clock source */
	RCC_I2CCLKConfig(RCC_I2C2CLK_HSI);
	I2C2_DevStructure.CPAL_Mode = CPAL_MODE_SLAVE;
	I2C2_DevStructure.wCPAL_Options =  CPAL_OPT_NO_MEM_ADDR | CPAL_OPT_I2C_AUTOMATIC_END;
	I2C2_DevStructure.CPAL_ProgModel = CPAL_PROGMODEL_INTERRUPT;
	I2C2_DevStructure.pCPAL_I2C_Struct->I2C_Timing = I2C_TIMING;
	I2C2_DevStructure.pCPAL_I2C_Struct->I2C_OwnAddress1 = 0;
	I2C2_DevStructure.pCPAL_TransferRx = &sRxStructure[id];
	I2C2_DevStructure.pCPAL_TransferTx = &sTxStructure[id];

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

    i2c_xTaskToNotify_read[id] = NULL;
    i2c_xTaskToNotify_write[id] = NULL;

    /* Initialize CPAL device with the selected parameters,
     * set pins directionetc */
    CPAL_I2C_Init(I2C_DevStructure[id]);
}

uint8_t xI2CSlaveReceive( I2C_ID_T id, uint8_t * rx_buff, uint8_t buff_len, uint32_t timeout )
{
    uint8_t bytes_to_copy = 0;
    printf("%s id %d\r\n", __func__, id);

    I2C_DevStructure[id]->pCPAL_TransferRx->pbBuffer = recv_msg;
    I2C_DevStructure[id]->pCPAL_TransferRx->wNumData = (sizeof(recv_msg)/sizeof(recv_msg[0]));
//     printf("%s own address 0x%02x\n", __func__, (int) I2C_DevStructure[id]->pCPAL_I2C_Struct->I2C_OwnAddress1);

    /* call the read function */
    (*cpal_read_func)(id);
    recv_bytes = buff_len - I2C_DevStructure[id]->pCPAL_TransferRx->wNumData;

    if (recv_bytes > buff_len) {
	bytes_to_copy = buff_len;
    } else {
	bytes_to_copy = recv_bytes;
    }
    /* Copy the rx buffer to the pointer given */
    memcpy( rx_buff, &recv_msg[0], bytes_to_copy );
    return bytes_to_copy;
}

/* expects i2c addr < 0x80 */
void vI2CSlaveSetup ( I2C_ID_T id, uint8_t slave_addr )
{
    slave_addr <<= 1;
    /* I2C already configured by vI2CConfig */
    I2C_DevStructure[id]->pCPAL_TransferRx->pbBuffer = recv_msg;
    I2C_DevStructure[id]->pCPAL_TransferRx->wNumData = (sizeof(recv_msg)/sizeof(recv_msg[0]));
    I2C_DevStructure[id]->pCPAL_I2C_Struct->I2C_OwnAddress1 = slave_addr;

    CPAL_I2C_Init(I2C_DevStructure[id]);
}
