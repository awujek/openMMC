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

/* If the speed of a uC is changed this field has to be adjusted!
 * Specifies the I2C_TIMINGR_register value.
 * This parameter calculated by referring to I2C initialization.
 * section in Reference manual. 8MHz, RM page 849, table 147 */
#define I2C_TIMING              0x10420f13

#define SLAVE_MASK 0xFF

__IO uint32_t RX_done = 0;
__IO uint32_t TX_done = 0;
__IO uint32_t RX_cint = 0;
__IO uint32_t TX_cint = 0;
__IO uint32_t RX_cdma = 0;
__IO uint32_t TX_cdma = 0;

/**
 * @brief       I2C0 Interrupt handler
 * @return      None
 */
// void I2C0_IRQHandler(void)
// {
// //     i2c_state_handling(I2C0);
// }
// 
// void I2C1_IRQHandler(void)
// {
// //     i2c_state_handling(I2C1);
// }
// 
// void I2C2_IRQHandler(void)
// {
// //     i2c_state_handling(I2C2);
// }

CPAL_InitTypeDef* I2C_DevStructure[I2C_NUM_INTERFACE] = {
    &I2C1_DevStructure,
    &I2C2_DevStructure,
    &I2C3_DevStructure
};

#define MAX_BUFF_SIZE           200
uint8_t tTxBuffer[MAX_BUFF_SIZE];
uint8_t tRxBuffer[MAX_BUFF_SIZE];

CPAL_TransferTypeDef  sRxStructure[I2C_NUM_INTERFACE], sTxStructure[I2C_NUM_INTERFACE];

void cpal_read(CPAL_InitTypeDef *cpal_i2c)
{
    printf("wait state %d %d\r\n", cpal_i2c->CPAL_State, (int)cpal_i2c->wCPAL_DevError);
    cpal_i2c->CPAL_State = CPAL_STATE_READY;
    RX_done = 0;
    if (CPAL_I2C_Read(cpal_i2c) == CPAL_PASS) {

    }
     while (!RX_done)
     {
	printf(".");
     }

    if (RX_done == 1)
	printf(".");
    else
	printf(",");
}

void cpal_write(CPAL_InitTypeDef *cpal_i2c)
{
    cpal_i2c->CPAL_State = CPAL_STATE_READY;
    TX_done = 0;
    if (CPAL_I2C_Write(cpal_i2c) == CPAL_PASS) {
    }
     while (!TX_done)
     {
	 printf(".");
     }
    if (TX_done == 1)
	printf(".");
    else
	printf(",");
}



int xI2CMasterWrite(I2C_ID_T id, uint8_t addr, uint8_t *tx_buff, uint8_t tx_len)
{
    sTxStructure[id].wAddr1 = addr;
    sTxStructure[id].wAddr2 = 0;
    sTxStructure[id].wNumData = tx_len;
    sTxStructure[id].pbBuffer = tx_buff;
    cpal_write(I2C_DevStructure[id]);

    return tx_len - sTxStructure[id].wNumData;
}

int xI2CMasterRead(I2C_ID_T id, uint8_t addr, uint8_t *rx_buff, uint8_t rx_len)
{
    sRxStructure[id].wAddr1 = addr;
    sRxStructure[id].wAddr2 = 0;
    sRxStructure[id].wNumData = rx_len;
    sRxStructure[id].pbBuffer = rx_buff;
    cpal_read(I2C_DevStructure[id]);

    return rx_len - sRxStructure[id].wNumData;
}

int xI2CMasterWriteRead(I2C_ID_T id, uint8_t addr, uint8_t cmd, uint8_t *rx_buff, uint8_t rx_len)
{
    sRxStructure[id].wAddr1 = addr;
    sRxStructure[id].wAddr2 = cmd;
    sRxStructure[id].wNumData = rx_len;
    sRxStructure[id].pbBuffer = rx_buff;
    cpal_read(I2C_DevStructure[id]);

    return rx_len - sRxStructure[id].wNumData;
}

void I2C_Taskx( void *Parameters );
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

    /* Initialize CPAL device with the selected parameters,
     * set pins directionetc */
    CPAL_I2C_Init(I2C_DevStructure[id]);


//     Chip_I2C_Init(id);
//     Chip_I2C_SetClockRate(id, speed);
//     NVIC_SetPriority( irq, configMAX_SYSCALL_INTERRUPT_PRIORITY -1 );
//     NVIC_EnableIRQ( irq );
//     Chip_I2C_Enable( id );
// 
//     Chip_I2C_SetMasterEventHandler(id, Chip_I2C_EventHandler);

}

// static TaskHandle_t slave_task_id;
// I2C_XFER_T slave_cfg;
// uint8_t recv_msg[i2cMAX_MSG_LENGTH];
// uint8_t recv_bytes;
// 
// uint8_t xI2CSlaveReceive( I2C_ID_T id, uint8_t * rx_buff, uint8_t buff_len, uint32_t timeout )
// {
//     uint8_t bytes_to_copy = 0;
//     slave_task_id = xTaskGetCurrentTaskHandle();
// 
//     if ( ulTaskNotifyTake( pdTRUE, timeout ) == pdTRUE )
//     {
//         if (recv_bytes > buff_len) {
//             bytes_to_copy = buff_len;
//         } else {
//             bytes_to_copy = recv_bytes;
//         }
//         /* Copy the rx buffer to the pointer given */
//         memcpy( rx_buff, &recv_msg[0], bytes_to_copy );
//         return bytes_to_copy;
//     } else {
//         return 0;
//     }
// }
// 
// static void I2C_Slave_Event(I2C_ID_T id, I2C_EVENT_T event)
// {
//     static BaseType_t xHigherPriorityTaskWoken;
//     switch (event) {
//     case I2C_EVENT_DONE:
//         recv_bytes = i2cMAX_MSG_LENGTH - slave_cfg.rxSz;
//         slave_cfg.rxSz = i2cMAX_MSG_LENGTH;
//         slave_cfg.rxBuff = &recv_msg[0];
// 
//         vTaskNotifyGiveFromISR( slave_task_id, &xHigherPriorityTaskWoken );
//         portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
// 
//     case I2C_EVENT_SLAVE_RX:
//         break;
//     default:
//         break;
//     }
// }
// 
// void vI2CSlaveSetup ( I2C_ID_T id, uint8_t slave_addr )
// {
//     slave_cfg.slaveAddr = slave_addr;
//     slave_cfg.txBuff = NULL; /* Not using Slave transmitter right now */
//     slave_cfg.txSz = 0;
//     slave_cfg.rxBuff = &recv_msg[0];
//     slave_cfg.rxSz = (sizeof(recv_msg)/sizeof(recv_msg[0]));
//     Chip_I2C_SlaveSetup( id, I2C_SLAVE_0, &slave_cfg, I2C_Slave_Event, SLAVE_MASK);
// }
