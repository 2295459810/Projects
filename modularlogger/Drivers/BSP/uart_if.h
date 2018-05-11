/***************************************************************************
* Copyright: Testo AG, 79849 Lenzkirch, Postfach 1140
***************************************************************************/
/** @uart_if.h
@brief<b>Description: </b>
@author:

\ingroup
***************************************************************************/


#ifndef _UART_IF_H
#define _UART_IF_H
/****************************************************************************
*  INCLUDES
*****************************************************************************/
/* stdint, stddef, stdbool will be included in module header file by config,
 * hence this file has to be included at first */
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "stm32l4xx_hal_uart.h"
//#include "legacy/stm32_hal_legacy.h"
//#include "stm32l4xx_hal_gpio.h"
//#include "stm32l4xx_hal_gpio_ex.h"
//#include "stm32l4xx_hal_rcc.h"
//#include "stm32l4xx_hal_cortex.h"
#include "appQueue.h"
/* for use in C++ code C-file header must use extern C define! */
#if defined (__cplusplus)
extern "C"
{
#endif
/****************************************************************************
* DEFINES
*****************************************************************************/
/* as few as posible */


#if 1
#define UART_MAX_NUM	5
#define UART_RX_BUFFERSIZE     40
#define UART_TX_BUFFERSIZE     128

/****************************************************************************
*  TYPE DECLARATION
*****************************************************************************/

/**
*****************************************************************************
@typedef Error Codes
*****************************************************************************/
typedef enum
{
  UART_IF_ERR_OK = 0,                 //!< Operation has completed successfully
  UART_IF_ERR_INIT,                 //!< Module is not initialized
  UART_IF_ERR_PARAM,                  //!< Invalid parameter
  UART_IF_ERR_DRIVER,                 //!< Internal driver failure
  UART_IF_ERR_CONFIG,                 //!< Failure to configure UART or query configuration
  UART_IF_ERR_TIMEOUT,                //!< Operation has timed out
  UART_IF_ERR_UNKNOWN                   //!< Receive line has been pulled low for too long
} UART_IF_ErrorCode_t;

/**
*****************************************************************************
@typedef Device identifier

The device identifier corresponds to the UART numbering scheme
*****************************************************************************/
typedef enum
{
  UART_IF_DEV_1 = 0,
  UART_IF_DEV_2,
  UART_IF_DEV_3,
    UART_IF_DEV_4 = 3,
	UART_IF_DEV_5 = 4,
//  UART_IF_DEV_6,
//	UART_IF_DEV_7 = 6,
//	UART_IF_DEV_8 = 7
} UART_IF_DeviceId_t;

typedef enum
{
	  UART_IF_WORDLENGTH_8B = UART_WORDLENGTH_8B,
	  UART_IF_WORDLENGTH_9B = UART_WORDLENGTH_9B
}UART_IF_WordLength_t;
typedef enum
{
	  UART_IF_HWFLOWCTL_NONE = UART_HWCONTROL_NONE,
	  UART_IF_HWFLOWCTL_RTS = UART_HWCONTROL_RTS,
	  UART_IF_HWFLOWCTL_CTS = UART_HWCONTROL_CTS,
	  UART_IF_HWFLOWCTL_RTS_CTS = UART_HWCONTROL_RTS_CTS
}UART_IF_HwFlowCtl_t;

typedef enum
{
	  UART_IF_SEND_WAY_REG = 0,
	  UART_IF_SEND_WAY_INT ,
	  UART_IF_SEND_WAY_DMA ,
	  UART_IF_SEND_WAY_UNKNOWN = 0xFF
}UART_IF_SENDWAY_t;


/**
*****************************************************************************
@typedef Number of stop bits
*****************************************************************************/
typedef enum
{
  UART_IF_STOP_BITS_1 = UART_STOPBITS_1,              //!< 1 stop bit
  UART_IF_STOP_BITS_2 = UART_STOPBITS_2               //!< 2 stop bits
} UART_IF_StopBits_t;


/**
*****************************************************************************
@typedef Parity
*****************************************************************************/
typedef enum
{
  UART_IF_PARITY_NONE = UART_PARITY_NONE,
	UART_IF_PARITY_EVEN = UART_PARITY_EVEN,
  UART_IF_PARITY_ODD = UART_PARITY_ODD
} UART_IF_Parity_t;

typedef struct _UART_Tag
{
	UART_IF_DeviceId_t enUartId;
	UART_HandleTypeDef hUardHandle;
	uint8_t u8UARTDmaRX_Buffer[UART_RX_BUFFERSIZE];
	uint8_t u8UARTDmaTX_Buffer[UART_TX_BUFFERSIZE];
	Circlequeue  qUartRxDataQ;
	volatile uint32_t u32UARTRxDataLength ;  //data length of current receive data, one frame
	volatile uint32_t u32UARTTxDataLength ;	//total send length
	volatile uint32_t u32UARTTxResidueDataLength ;
//	SemaphoreHandle_t xUart_Rx_Muxtex;
//	SemaphoreHandle_t xUart_Tx_Muxtex;
	volatile uint8_t eUARTDMATx_Done_Event;
	bool bInit;
}UartProperty,*PUartProperty;

extern UartProperty  stUartProperty[UART_MAX_NUM];
/****************************************************************************
*  FUNCTION DECLARATION
*****************************************************************************/
/**
 *****************************************************************************
 Initialize UART

 This function initializes the UART by setting the transmission parameters,
 setting up a receive buffer, and registering interrupt service routines.

 @param[in]   enuDevice        UART to be initialized. Each UART must be
                               initialized separately.
 @param[in]   u32BaudRate      Data rate in bits/second.
 @param[in]   enuHWFlowCtl     HW Flow Control
 @param[in]   enuStopBits      Number of stop bits
 @param[in]   enuParity        Parity

 @return error code
 *****************************************************************************/
UART_IF_ErrorCode_t tszUART_Init(
	  UART_IF_DeviceId_t	enuDevice,
	  uint32_t				u32BaudRate,
	  UART_IF_HwFlowCtl_t	enuHWFlowCtl,
	  UART_IF_StopBits_t	enuStopBits,
	  UART_IF_Parity_t		enuParity
	  );
UART_IF_ErrorCode_t tszUART_DeInit(
	  UART_IF_DeviceId_t	enuDevice
	  );
/**
*****************************************************************************
Read from UART

@param[in]   enuDevice         UART to be used
@param[out]  pu8Buffer         Target buffer for data
@param[in]   u32ReadLength     Number of bytes to be read
@param[out]  pu32BytesReceived Number of bytes that have actually been received
@param[in]   u32Timeout        Timeout for reading [milliseconds]

@return error code
*****************************************************************************/
UART_IF_ErrorCode_t tszUART_Read(
		  UART_IF_DeviceId_t	enuDevice,
		  uint8_t			   *pu8Buffer,
		  uint32_t				u32ReadLength,
		  uint32_t			   *pu32BytesReceived,
		  uint32_t				u32Timeout
	  	  );

/**
*****************************************************************************
Write to UART

@param[in]   enuDevice         UART to be used
@param[out]  pu8Data           Source buffer for data
@param[in]   u32DataLength     Number of bytes to be write
@param[in]   u32Timeout        Timeout for writing [milliseconds]

@return error code
*****************************************************************************/
UART_IF_ErrorCode_t tszUART_Write(
		  UART_IF_DeviceId_t	enuDevice,
		  const uint8_t		   *pu8Data,
		  uint32_t				u32DataLength,
		  uint32_t				u32Timeout,
		  UART_IF_SENDWAY_t     enuWay
	  	  );

UART_IF_ErrorCode_t tszUART_ClearRxData(UART_IF_DeviceId_t enuDevice);

#endif
#if defined (__cplusplus)
}
#endif
#endif
