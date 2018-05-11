#include <string.h>
#include <stdbool.h>
//#include <stm32l4xx_hal_def.h>
//#include "stm32l4xx_hal_dma.h"
#include "uart_if.h"
//#include "_dma_cfg.h"

//#include "dbgPrintf.h"
//#include "..\modules\sacpClient\sacpClientSingle.h"
//#include "systemCtrl.h"
//#include "..\modules\ble\ble_lierda.h"

////////////////////////////////////////////////////////////////////////////////// 	 

#if 1
UartProperty  stUartProperty[UART_MAX_NUM];

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;
//extern volatile bool bEnableCableButtonclick;
//extern DMA_HandleTypeDef hdma_usart6_rx;
//extern DMA_HandleTypeDef hdma_usart6_tx;

//extern HAL_StatusTypeDef HAL_UART_DMAStopRx(UART_HandleTypeDef *huart);
extern volatile bool bgUpProbeResetHumidityAdjOngoing;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
void USART1_IRQHandler(void);
void USART2_IRQHandler(void);
void USART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
//void USART6_IRQHandler(void);
static UART_IF_ErrorCode_t tszUART_Write_Reg(
	  UART_HandleTypeDef	*huart,
	  const uint8_t		   *pu8Data,
	  uint32_t				u32DataLength
	 
	  );
static UART_IF_ErrorCode_t tszUART_Write_Dma(
	  UART_IF_DeviceId_t	enuDevice,
	  UART_HandleTypeDef	*huart,
	  const uint8_t		   *pu8Data,
	  uint32_t				u32DataLength
	  );


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t Id = 0;int i = 0;
	uint32_t u32Len = 0;
	uint32_t tempLen = 0;
	//receive data finish,uart1
	if(huart->Instance==USART1)
	{
		Id = 0;
	}
	else if(huart->Instance==USART2)
	{
		Id = 1;
	}
	else if(huart->Instance==USART3)
	{
		Id = 2;
	}
	else if(huart->Instance==UART4)
	{
		Id = 3;
	}
	else if(huart->Instance==UART5)
	{
		Id = 4;
	}
	else
	{
		return ;

	}
	stUartProperty[Id].u32UARTRxDataLength = UART_RX_BUFFERSIZE;

		for(i = 0; i<UART_RX_BUFFERSIZE;i++)
		{
			if(IsQueueFull(&(stUartProperty[Id].qUartRxDataQ)))
				{
					ClearQueue(&(stUartProperty[Id].qUartRxDataQ));
				}
				EnQueue(&(stUartProperty[Id].qUartRxDataQ),(char)stUartProperty[Id].u8UARTDmaRX_Buffer[i]);
		}

/*---------------------------------------------------------------------------------------------------------------------*/
	memset(stUartProperty[Id].u8UARTDmaRX_Buffer,0,UART_RX_BUFFERSIZE);

	stUartProperty[Id].u32UARTRxDataLength = 0;
	HAL_UART_Receive_DMA(&(stUartProperty[Id].hUardHandle), stUartProperty[Id].u8UARTDmaRX_Buffer, 1);
}
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{

	if(huart->Instance == USART1)
	{

		FREERTOS_CUSTOMIZE_SET_EVENT(stUartProperty[0].eUARTDMATx_Done_Event);

		/*
		FREERTOS_CUSTOMIZE_SET_EVENT(eUART1_DMA_TX_CPLT_EVENT);
		BaseType_t xYieldRequired = pdFALSE;

		if( xSemaphoreGiveFromISR( xUart1_Tx_DoneEvent,&xYieldRequired) == pdTRUE )
	     {
	         // We should switch context so the ISR returns to a different task.
	         // NOTE:  How this is done depends on the port you are using.  Check
	         // the documentation and examples for your port.
	         if(xYieldRequired == pdTRUE)
	         	{
	         		portYIELD_FROM_ISR(xYieldRequired);
	         	}
	     }
	     */
	}
	else if(huart->Instance == USART2)
	{
		FREERTOS_CUSTOMIZE_SET_EVENT(stUartProperty[1].eUARTDMATx_Done_Event);
	}
	else if(huart->Instance == USART3)
	{
		FREERTOS_CUSTOMIZE_SET_EVENT(stUartProperty[2].eUARTDMATx_Done_Event);
	}
	else if(huart->Instance==UART4)
	{
		FREERTOS_CUSTOMIZE_SET_EVENT(stUartProperty[3].eUARTDMATx_Done_Event);
	}
	else if(huart->Instance==UART5)
	{
		FREERTOS_CUSTOMIZE_SET_EVENT(stUartProperty[4].eUARTDMATx_Done_Event);
	}
}

void USART1_IRQHandler(void)
{
  volatile uint32_t temp, tmp_flag;
  uint32_t isrflags   = READ_REG(stUartProperty[UART_IF_DEV_1].hUardHandle.Instance->ISR);
  int i = 0;
  tmp_flag = isrflags&USART_ISR_IDLE;
//  tmp_flag = READ_BIT(stUartProperty[UART_IF_DEV_1].hUardHandle.Instance->ISR, USART_ISR_IDLE);
  if((tmp_flag != RESET))
  {
	  __HAL_UART_CLEAR_IDLEFLAG(&(stUartProperty[UART_IF_DEV_1].hUardHandle));
		temp = stUartProperty[UART_IF_DEV_1].hUardHandle.Instance->ISR;
		temp = stUartProperty[UART_IF_DEV_1].hUardHandle.Instance->RDR;
//	HAL_UART_DMAStop(&(stUartProperty[UART_IF_DEV_1].hUardHandle));
	  HAL_UART_DMAStop(&(stUartProperty[UART_IF_DEV_1].hUardHandle));
			//__HAL_DMA_DISABLE(UART1_Handler.hdmarx);

			stUartProperty[UART_IF_DEV_1].u32UARTRxDataLength = UART_RX_BUFFERSIZE - hdma_usart1_rx.Instance->CNDTR;
			if(stUartProperty[UART_IF_DEV_1].u32UARTRxDataLength == UART_RX_BUFFERSIZE)
			{
				//let HAL_UART_RxCpltCallback function do the job.
				return ;
			}
			for(i = 0; i<stUartProperty[UART_IF_DEV_1].u32UARTRxDataLength;i++)
			{
				if(IsQueueFull(&(stUartProperty[UART_IF_DEV_1].qUartRxDataQ)))
					{
						ClearQueue(&(stUartProperty[UART_IF_DEV_1].qUartRxDataQ));
					}
				EnQueue(&(stUartProperty[UART_IF_DEV_1].qUartRxDataQ),(char)stUartProperty[UART_IF_DEV_1].u8UARTDmaRX_Buffer[i]);
			}
			memset(stUartProperty[UART_IF_DEV_1].u8UARTDmaRX_Buffer,0,UART_RX_BUFFERSIZE);
			HAL_UART_Receive_DMA(&stUartProperty[UART_IF_DEV_1].hUardHandle, stUartProperty[UART_IF_DEV_1].u8UARTDmaRX_Buffer, UART_RX_BUFFERSIZE);
  }
  else
  {
  	HAL_UART_IRQHandler(&(stUartProperty[UART_IF_DEV_1].hUardHandle));
  }

}
void USART2_IRQHandler(void)
{
  volatile uint32_t temp = 0,tmp_flag;

  int i = 0;

  tmp_flag = READ_BIT(stUartProperty[UART_IF_DEV_2].hUardHandle.Instance->ISR, USART_ISR_IDLE);
  if((tmp_flag != RESET))
  { 

	  	  __HAL_UART_CLEAR_IDLEFLAG(&(stUartProperty[UART_IF_DEV_2].hUardHandle));
			temp = stUartProperty[UART_IF_DEV_2].hUardHandle.Instance->ISR;
			temp = stUartProperty[UART_IF_DEV_2].hUardHandle.Instance->RDR;
			HAL_UART_DMAStop(&(stUartProperty[UART_IF_DEV_2].hUardHandle));
			//__HAL_DMA_DISABLE(UART1_Handler.hdmarx);

			stUartProperty[UART_IF_DEV_2].u32UARTRxDataLength = UART_RX_BUFFERSIZE - hdma_usart2_rx.Instance->CNDTR;
			if(stUartProperty[UART_IF_DEV_2].u32UARTRxDataLength == UART_RX_BUFFERSIZE)
			{
				//let HAL_UART_RxCpltCallback function do the job.
				return ;
			}
			for(i = 0; i<stUartProperty[UART_IF_DEV_2].u32UARTRxDataLength;i++)
			{
				if(IsQueueFull(&(stUartProperty[UART_IF_DEV_2].qUartRxDataQ)))
					{
						ClearQueue(&(stUartProperty[UART_IF_DEV_2].qUartRxDataQ));
					}
				EnQueue(&(stUartProperty[UART_IF_DEV_2].qUartRxDataQ),(char)stUartProperty[UART_IF_DEV_2].u8UARTDmaRX_Buffer[i]);
			}
			memset(stUartProperty[UART_IF_DEV_2].u8UARTDmaRX_Buffer,0,UART_RX_BUFFERSIZE);
			HAL_UART_Receive_DMA(&stUartProperty[UART_IF_DEV_2].hUardHandle, stUartProperty[UART_IF_DEV_2].u8UARTDmaRX_Buffer, UART_RX_BUFFERSIZE);
		
  }
  else
  {
  	HAL_UART_IRQHandler(&(stUartProperty[UART_IF_DEV_2].hUardHandle));
  }

  UNUSED(temp);
}
void USART3_IRQHandler(void)
{
  volatile uint32_t temp = 0,tmp_flag;

  int i = 0;

  tmp_flag = READ_BIT(stUartProperty[UART_IF_DEV_3].hUardHandle.Instance->ISR, USART_ISR_IDLE);
  if((tmp_flag != RESET))
  { 

	  	  __HAL_UART_CLEAR_IDLEFLAG(&(stUartProperty[UART_IF_DEV_3].hUardHandle));
			temp = stUartProperty[UART_IF_DEV_3].hUardHandle.Instance->ISR;
			temp = stUartProperty[UART_IF_DEV_3].hUardHandle.Instance->RDR;
			HAL_UART_DMAStop(&(stUartProperty[UART_IF_DEV_3].hUardHandle));
			//__HAL_DMA_DISABLE(UART1_Handler.hdmarx);

			stUartProperty[UART_IF_DEV_3].u32UARTRxDataLength = UART_RX_BUFFERSIZE - hdma_usart3_rx.Instance->CNDTR;
			if(stUartProperty[UART_IF_DEV_3].u32UARTRxDataLength == UART_RX_BUFFERSIZE)
			{
				//let HAL_UART_RxCpltCallback function do the job.
				return ;
			}

			for(i = 0; i<stUartProperty[UART_IF_DEV_3].u32UARTRxDataLength;i++)
			{
				if(IsQueueFull(&(stUartProperty[UART_IF_DEV_3].qUartRxDataQ)))
					{
						ClearQueue(&(stUartProperty[UART_IF_DEV_3].qUartRxDataQ));
					}
				EnQueue(&(stUartProperty[UART_IF_DEV_3].qUartRxDataQ),(char)stUartProperty[UART_IF_DEV_3].u8UARTDmaRX_Buffer[i]);
			}
			memset(stUartProperty[UART_IF_DEV_3].u8UARTDmaRX_Buffer,0,UART_RX_BUFFERSIZE);
			HAL_UART_Receive_DMA(&stUartProperty[UART_IF_DEV_3].hUardHandle, stUartProperty[UART_IF_DEV_3].u8UARTDmaRX_Buffer, UART_RX_BUFFERSIZE);
		
  }
  else
   {
    	HAL_UART_IRQHandler(&(stUartProperty[UART_IF_DEV_3].hUardHandle));
    }
  UNUSED(temp);
}
void UART4_IRQHandler(void)
{
  volatile uint32_t temp = 0,tmp_flag;

  int i = 0;

  tmp_flag = READ_BIT(stUartProperty[UART_IF_DEV_4].hUardHandle.Instance->ISR, USART_ISR_IDLE);
  if((tmp_flag != RESET))
  {

	  	  __HAL_UART_CLEAR_IDLEFLAG(&(stUartProperty[UART_IF_DEV_4].hUardHandle));
			temp = stUartProperty[UART_IF_DEV_4].hUardHandle.Instance->ISR;
			temp = stUartProperty[UART_IF_DEV_4].hUardHandle.Instance->RDR;
			HAL_UART_DMAStop(&(stUartProperty[UART_IF_DEV_4].hUardHandle));
			//__HAL_DMA_DISABLE(UART1_Handler.hdmarx);

			stUartProperty[UART_IF_DEV_4].u32UARTRxDataLength = UART_RX_BUFFERSIZE - hdma_usart3_rx.Instance->CNDTR;
			if(stUartProperty[UART_IF_DEV_4].u32UARTRxDataLength == UART_RX_BUFFERSIZE)
			{
				//let HAL_UART_RxCpltCallback function do the job.
				return ;
			}

			for(i = 0; i<stUartProperty[UART_IF_DEV_4].u32UARTRxDataLength;i++)
			{
				if(IsQueueFull(&(stUartProperty[UART_IF_DEV_4].qUartRxDataQ)))
					{
						ClearQueue(&(stUartProperty[UART_IF_DEV_4].qUartRxDataQ));
					}
				EnQueue(&(stUartProperty[UART_IF_DEV_4].qUartRxDataQ),(char)stUartProperty[UART_IF_DEV_4].u8UARTDmaRX_Buffer[i]);
			}
			memset(stUartProperty[UART_IF_DEV_4].u8UARTDmaRX_Buffer,0,UART_RX_BUFFERSIZE);
			HAL_UART_Receive_DMA(&stUartProperty[UART_IF_DEV_4].hUardHandle, stUartProperty[UART_IF_DEV_4].u8UARTDmaRX_Buffer, UART_RX_BUFFERSIZE);

  }

  UNUSED(temp);
}
void UART5_IRQHandler(void)
{
  volatile uint32_t temp = 0,tmp_flag;

  int i = 0;

  tmp_flag = READ_BIT(stUartProperty[UART_IF_DEV_5].hUardHandle.Instance->ISR, USART_ISR_IDLE);
  if((tmp_flag != RESET))
  {

	  	  __HAL_UART_CLEAR_IDLEFLAG(&(stUartProperty[UART_IF_DEV_5].hUardHandle));
			temp = stUartProperty[UART_IF_DEV_5].hUardHandle.Instance->ISR;
			temp = stUartProperty[UART_IF_DEV_5].hUardHandle.Instance->RDR;
			HAL_UART_DMAStop(&(stUartProperty[UART_IF_DEV_5].hUardHandle));
			//__HAL_DMA_DISABLE(UART1_Handler.hdmarx);

			stUartProperty[UART_IF_DEV_5].u32UARTRxDataLength = UART_RX_BUFFERSIZE - hdma_usart3_rx.Instance->CNDTR;
			if(stUartProperty[UART_IF_DEV_5].u32UARTRxDataLength == UART_RX_BUFFERSIZE)
			{
				//let HAL_UART_RxCpltCallback function do the job.
				return ;
			}

			for(i = 0; i<stUartProperty[UART_IF_DEV_5].u32UARTRxDataLength;i++)
			{
				if(IsQueueFull(&(stUartProperty[UART_IF_DEV_5].qUartRxDataQ)))
					{
						ClearQueue(&(stUartProperty[UART_IF_DEV_5].qUartRxDataQ));
					}
				EnQueue(&(stUartProperty[UART_IF_DEV_5].qUartRxDataQ),(char)stUartProperty[UART_IF_DEV_5].u8UARTDmaRX_Buffer[i]);
			}
			memset(stUartProperty[UART_IF_DEV_5].u8UARTDmaRX_Buffer,0,UART_RX_BUFFERSIZE);

			HAL_UART_Receive_DMA(&stUartProperty[UART_IF_DEV_5].hUardHandle, stUartProperty[UART_IF_DEV_5].u8UARTDmaRX_Buffer, UART_RX_BUFFERSIZE);

  }

  UNUSED(temp);

}

UART_IF_ErrorCode_t tszUART_Init(
	  UART_IF_DeviceId_t	enuDevice,
	  uint32_t				u32BaudRate,
	  UART_IF_HwFlowCtl_t	enuHWFlowCtl,
	  UART_IF_StopBits_t	enuStopBits,
	  UART_IF_Parity_t		enuParity
	  )
{
	UART_IF_ErrorCode_t reCode = UART_IF_ERR_OK;
	//to do, should judge other parameter 
	if((u32BaudRate > 1152000)||(u32BaudRate < 2400))
	{
		reCode = UART_IF_ERR_PARAM;
		return reCode;
	}
	if((enuDevice > UART_IF_DEV_5)||(enuDevice < UART_IF_DEV_1))
	{
		reCode = UART_IF_ERR_PARAM;
		return reCode;
	}
	if(stUartProperty[enuDevice].bInit == false)
		{
			memset(&stUartProperty[enuDevice],0,sizeof(UartProperty));
		}
	else
		{
			reCode = UART_IF_ERR_INIT;
			return reCode;
		}
	stUartProperty[enuDevice].enUartId = enuDevice;
	if(enuDevice == UART_IF_DEV_1)
	{
		stUartProperty[enuDevice].hUardHandle.Instance = USART1;
	}
	else if (enuDevice == UART_IF_DEV_2)
	{
		stUartProperty[enuDevice].hUardHandle.Instance = USART2;
	}
	else if (enuDevice == UART_IF_DEV_3)
	{
		stUartProperty[enuDevice].hUardHandle.Instance = USART3;
	}
	else if (enuDevice == UART_IF_DEV_4)
	{
		stUartProperty[enuDevice].hUardHandle.Instance = UART4;
	}
	else if (enuDevice == UART_IF_DEV_5)
	{
		stUartProperty[enuDevice].hUardHandle.Instance = UART5;
	}
	else
	{
		reCode = UART_IF_ERR_PARAM;
		return reCode;
	}
	stUartProperty[enuDevice].hUardHandle.Init.BaudRate = u32BaudRate;				    //bound
	stUartProperty[enuDevice].hUardHandle.Init.WordLength = UART_WORDLENGTH_8B;   //8bit
	stUartProperty[enuDevice].hUardHandle.Init.StopBits = enuStopBits;	    //stop
	stUartProperty[enuDevice].hUardHandle.Init.Parity = enuParity;		    //Parity check
	stUartProperty[enuDevice].hUardHandle.Init.HwFlowCtl = enuHWFlowCtl;   //HwFlow
	stUartProperty[enuDevice].hUardHandle.Init.Mode = UART_MODE_TX_RX;		    //Mode
	stUartProperty[enuDevice].hUardHandle.Init.OverSampling = UART_OVERSAMPLING_16;
	InitQueue(&(stUartProperty[enuDevice].qUartRxDataQ));
	HAL_UART_Init(&(stUartProperty[enuDevice].hUardHandle));	
	HAL_UART_Receive_DMA(&(stUartProperty[enuDevice].hUardHandle), stUartProperty[enuDevice].u8UARTDmaRX_Buffer, UART_RX_BUFFERSIZE);
//	HAL_UART_Receive_DMA(&(stUartProperty[enuDevice].hUardHandle),UartDma_Init(uart_dma_send,USART1),RECE_BUF_MAX_LEN);
//	HAL_UART_Receive_DMA(&(stUartProperty[enuDevice].hUardHandle),stUartProperty[UART_IF_DEV_1].u8UARTDmaRX_Buffer,UART_RX_BUFFERSIZE);
	//	stUartProperty[enuDevice].xUart_Tx_Muxtex = xSemaphoreCreateBinary();
//	if(NULL == stUartProperty[enuDevice].xUart_Tx_Muxtex)
//		{
//			reCode = UART_IF_ERR_UNKNOWN;
//			return reCode;
//		}
//	else
//		{
//			xSemaphoreGive( stUartProperty[enuDevice].xUart_Tx_Muxtex);
//		}
//
//	stUartProperty[enuDevice].xUart_Rx_Muxtex = xSemaphoreCreateBinary();
//	if(NULL == stUartProperty[enuDevice].xUart_Rx_Muxtex)
//		{
//			reCode = UART_IF_ERR_UNKNOWN;
//			return reCode;
//		}
//	else
//		{
//			xSemaphoreGive( stUartProperty[enuDevice].xUart_Rx_Muxtex);
//		}
	
	FREERTOS_CUSTOMIZE_RESET_EVENT(stUartProperty[enuDevice].eUARTDMATx_Done_Event);	
    stUartProperty[enuDevice].bInit = true;
	return reCode;
}

UART_IF_ErrorCode_t tszUART_DeInit(UART_IF_DeviceId_t	enuDevice)
{
	//

	uint8_t Id = 0;
	UART_IF_ErrorCode_t enuRetVal = UART_IF_ERR_OK;
	if(enuDevice == UART_IF_DEV_1)
	{
		Id = 0;
	}
	else if(enuDevice == UART_IF_DEV_2)
	{
		Id = 1;
	}
	else if(enuDevice == UART_IF_DEV_3)
	{
		Id = 2;
	}
	else if(enuDevice == UART_IF_DEV_4)
	{
		Id = 3;
	}
	else if(enuDevice == UART_IF_DEV_5)
	{
		Id = 4;
	}
	else
	{
		enuRetVal = UART_IF_ERR_PARAM;
		return enuRetVal;
	}
//	if(NULL != stUartProperty[Id].xUart_Rx_Muxtex)
//		{
//			vSemaphoreDelete(stUartProperty[Id].xUart_Rx_Muxtex);
//		}
//	if(NULL != stUartProperty[Id].xUart_Tx_Muxtex)
//		{
//			vSemaphoreDelete(stUartProperty[Id].xUart_Tx_Muxtex);
//		}

	HAL_UART_DeInit(&(stUartProperty[Id].hUardHandle));
	memset(&stUartProperty[Id],0,sizeof(UartProperty));

	return enuRetVal;
}
UART_IF_ErrorCode_t tszUART_Read(
	  UART_IF_DeviceId_t	enuDevice,
	  uint8_t			   *pu8Buffer,
	  uint32_t				u32ReadLength,
	  uint32_t			   *pu32BytesReceived,
	  uint32_t				u32Timeout
	  )
{
	UART_IF_ErrorCode_t enuRetVal = UART_IF_ERR_OK;
	uint8_t Id = 0;		
	uint32_t u32DeleyCount = 0 ;//in MS
	uint32_t u32RecCount = 0 ;
	bool bReady = false;

	if((NULL == pu8Buffer)||(u32ReadLength <=0)||(NULL == pu32BytesReceived))
	{
		enuRetVal = UART_IF_ERR_PARAM;
		return enuRetVal;
	}
	
	if(enuDevice == UART_IF_DEV_1)
	{
		Id = 0;
	}
	else if(enuDevice == UART_IF_DEV_2)
	{
		Id = 1;
	}
	else if(enuDevice == UART_IF_DEV_3)
	{
		Id = 2;
	}
	else if(enuDevice == UART_IF_DEV_4)
	{
		Id = 3;
	}
	else if(enuDevice == UART_IF_DEV_5)
	{
		Id = 4;
	}
	else
	{
		enuRetVal = UART_IF_ERR_PARAM;
		return enuRetVal;
	}

	if(stUartProperty[Id].bInit == false)
	{
		enuRetVal = UART_IF_ERR_INIT;
		return enuRetVal;
	}
	
	
//	if(xSemaphoreTake( stUartProperty[Id].xUart_Rx_Muxtex, ( TickType_t ) 0 ) == pdTRUE)
//	{
			do{
			//if there is data and receive not finished
			while((GetLength(&(stUartProperty[Id].qUartRxDataQ))>0)&&(!bReady))
//			while((!bReady))
			{
				//put into read buffer
				pu8Buffer[u32RecCount] = (uint8_t)DeQueue(&(stUartProperty[Id].qUartRxDataQ));
				u32RecCount++;
				//if read buffer if full
				if(u32RecCount == u32ReadLength)
				{
				bReady = true;
				}
			}
			if(!bReady)
			{
				u32DeleyCount++;
				//delay_ms(1);
				if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
				{
					osDelay(1);
				}
				else
				{
					HAL_Delay(1);
				}

			}
		}while((!bReady)&&(u32DeleyCount <= u32Timeout));

		*pu32BytesReceived = u32RecCount ;
		if(bReady)
		{
			enuRetVal = UART_IF_ERR_OK;
		}
		else if((u32RecCount > 0)&&(!bReady))
		{
			enuRetVal = UART_IF_ERR_TIMEOUT;
		}
		else
			enuRetVal = UART_IF_ERR_UNKNOWN;

//	xSemaphoreGive(stUartProperty[Id].xUart_Rx_Muxtex);
//	}
//	else
//	{
//		enuRetVal = UART_IF_ERR_TIMEOUT;
//		//ivan, for humidity adjust and probe reset, some time the system can NOT
//		//give out the semaphore, so at there give is manually.
////		if(bgUpProbeResetHumidityAdjOngoing)
//		{
//			xSemaphoreGive( stUartProperty[Id].xUart_Rx_Muxtex);
//		}
//
//		return enuRetVal;
//	}
	return enuRetVal;

}
static UART_IF_ErrorCode_t tszUART_Write_Reg(
	  UART_HandleTypeDef	*huart,
	  const uint8_t		   *pu8Data,
	  uint32_t				u32DataLength
	 
	  )
{
	UART_IF_ErrorCode_t enuRetVal = UART_IF_ERR_OK;
	HAL_UART_Transmit(huart, (uint8_t *)pu8Data, u32DataLength, 0xFFFF);
	return enuRetVal;
}

static UART_IF_ErrorCode_t tszUART_Write_Dma(
	  UART_IF_DeviceId_t	enuDevice,
	  UART_HandleTypeDef	*huart,
	  const uint8_t		   *pu8Data,
	  uint32_t				u32DataLength
	  )
{
	UART_IF_ErrorCode_t enuRetVal = UART_IF_ERR_OK;
	uint8_t  *p8UartTxData = NULL;
	uint32_t u32UartTxInteg=0, u32UartTxResidue=0;
	uint32_t u32UartSendCountDma=0;
	u32UartTxInteg = u32DataLength / UART_TX_BUFFERSIZE ;
	u32UartTxResidue = u32DataLength % UART_TX_BUFFERSIZE ;
	p8UartTxData = (uint8_t*)pu8Data;
	if(u32UartTxInteg > 0)
	{
		
		for (u32UartSendCountDma = 0; u32UartSendCountDma < u32UartTxInteg; u32UartSendCountDma++)
		{
			memset(stUartProperty[enuDevice].u8UARTDmaTX_Buffer,0,UART_TX_BUFFERSIZE);
			memcpy(stUartProperty[enuDevice].u8UARTDmaTX_Buffer,p8UartTxData,UART_TX_BUFFERSIZE);

			HAL_UART_Transmit_DMA(huart,stUartProperty[enuDevice].u8UARTDmaTX_Buffer,UART_TX_BUFFERSIZE);
			//wait DMA transmit complete
			//xSemaphoreTake(xUart1_Tx_DoneEvent,( TickType_t )0xFFFFFFFFUL );
			
			FREERTOS_CUSTOMIZE_WAIT_EVENT(stUartProperty[enuDevice].eUARTDMATx_Done_Event);
			FREERTOS_CUSTOMIZE_RESET_EVENT(stUartProperty[enuDevice].eUARTDMATx_Done_Event);
			//remove point
			p8UartTxData = p8UartTxData + UART_TX_BUFFERSIZE;
		}
	}

	if(u32UartTxResidue > 0)
	{
		memset(stUartProperty[enuDevice].u8UARTDmaTX_Buffer,0,UART_TX_BUFFERSIZE);
		for (u32UartSendCountDma = 0; u32UartSendCountDma < u32UartTxResidue; u32UartSendCountDma++)
		{
			stUartProperty[enuDevice].u8UARTDmaTX_Buffer[u32UartSendCountDma] = (uint8_t)(*p8UartTxData++ & (uint8_t)0x00FF);
		}

		HAL_UART_Transmit_DMA(huart,stUartProperty[enuDevice].u8UARTDmaTX_Buffer,(uint16_t)u32UartTxResidue);
        //HAL_UART_Transmit_DMA(huart,stUartProperty[enuDevice].u8UARTDmaTX_Buffer,strlen(stUartProperty[enuDevice].u8UARTDmaTX_Buffer));
		//wait DMA transmit complete	
		//xSemaphoreTake(xUart1_Tx_DoneEvent,( TickType_t )0xFFFFFFFFUL );
		FREERTOS_CUSTOMIZE_WAIT_EVENT(stUartProperty[enuDevice].eUARTDMATx_Done_Event);

		FREERTOS_CUSTOMIZE_RESET_EVENT(stUartProperty[enuDevice].eUARTDMATx_Done_Event);
	}

	return enuRetVal;
}
UART_IF_ErrorCode_t tszUART_Write(
	  UART_IF_DeviceId_t	enuDevice,
	  const uint8_t		   *pu8Data,
	  uint32_t				u32DataLength,
	  uint32_t				u32Timeout,
	  UART_IF_SENDWAY_t     enuWay
	  )
{
	UART_IF_ErrorCode_t enuRetVal = UART_IF_ERR_OK;
	uint8_t Id = 0;

	if(enuDevice == UART_IF_DEV_1)
	{
		Id = 0;
	}
	else if(enuDevice == UART_IF_DEV_2)
	{
		Id = 1;
	}
	else if(enuDevice == UART_IF_DEV_3)
	{
		Id = 2;
	}
	else if(enuDevice == UART_IF_DEV_4)
	{
		Id = 3;
	}
	else if(enuDevice == UART_IF_DEV_5)
	{
		Id = 4;
	}
	else
	{
		enuRetVal = UART_IF_ERR_PARAM;
		return enuRetVal;
	}

	if(stUartProperty[Id].bInit == false)
	{
		enuRetVal = UART_IF_ERR_INIT;
		return enuRetVal;
	}
//	HAL_NVIC_DisableIRQ(EXTI4_IRQn);
//	if(xSemaphoreTake( stUartProperty[Id].xUart_Tx_Muxtex, ( TickType_t ) u32Timeout ) == pdTRUE)
//	{
		if(enuWay == UART_IF_SEND_WAY_REG)
		 	{
		 		tszUART_Write_Reg(&(stUartProperty[Id].hUardHandle),pu8Data,u32DataLength);
		 	}
		else if(enuWay == UART_IF_SEND_WAY_INT)
		 	{
		 		//ivan ,no time to implement it
		 	}
		else if(enuWay == UART_IF_SEND_WAY_DMA)
		 	{
		 		tszUART_Write_Dma(Id,&(stUartProperty[Id].hUardHandle),pu8Data,u32DataLength);
		 	}		
//		xSemaphoreGive( stUartProperty[Id].xUart_Tx_Muxtex);
//
//	}
//	else
//	{
//		enuRetVal = UART_IF_ERR_TIMEOUT;
//		//ivan, for humidity adjust and probe reset, some time the system can NOT
//		//give out the semaphore, so at there give is manually.
////		if(bgUpProbeResetHumidityAdjOngoing)
//		{
//			xSemaphoreGive( stUartProperty[Id].xUart_Tx_Muxtex);
//		}
//
//	}

//	HAL_NVIC_EnableIRQ(EXTI4_IRQn);
	return enuRetVal;
}


UART_IF_ErrorCode_t tszUART_ClearRxData(UART_IF_DeviceId_t enuDevice)
{
	UART_IF_ErrorCode_t enuRetVal = UART_IF_ERR_OK;
	uint8_t Id = 0;		

	if(enuDevice == UART_IF_DEV_1)
	{
		Id = 0;
	}
	else if(enuDevice == UART_IF_DEV_2)
	{
		Id = 1;
	}
	else if(enuDevice == UART_IF_DEV_3)
	{
		Id = 2;
	}
	else if(enuDevice == UART_IF_DEV_4)
	{
		Id = 3;
	}
	else if(enuDevice == UART_IF_DEV_5)
	{
		Id = 3;
	}
	else
	{
		enuRetVal = UART_IF_ERR_PARAM;
		return enuRetVal;
	}

	if(stUartProperty[Id].bInit == false)
	{
		enuRetVal = UART_IF_ERR_INIT;
		return enuRetVal;
	}
	
//	if(xSemaphoreTake(stUartProperty[Id].xUart_Rx_Muxtex, (TickType_t) 100) == pdTRUE)
//	{
        ClearQueue(&(stUartProperty[Id].qUartRxDataQ));
//    	xSemaphoreGive(stUartProperty[Id].xUart_Rx_Muxtex);
//	}
//	else
//	{
//		enuRetVal = UART_IF_ERR_UNKNOWN;
//		return enuRetVal;
//	}
	
	return enuRetVal;
}

#endif
