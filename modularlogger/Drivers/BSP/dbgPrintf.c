/***************************************************************************
* Copyright: Testo AG, 79849 Lenzkirch, Postfach 1140
***************************************************************************/

/***************************************************************************
*  INCLUDES
***************************************************************************/
/* fist include must be the header */
//#include "single_bsp_header.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dbgPrintf.h"
#include <stdarg.h>
#include <stdbool.h>

//#include "..\modules\sacpServer\sacpServerSingle.h"
/***************************************************************************
* DEFINES
***************************************************************************/
/* as few as posible */

/***************************************************************************
* MODULE GLOBALS
***************************************************************************/
static char mg_szBuff[PRINTF_BUFFERSIZE];
/***************************************************************************
* LOCAL FUNCTION DECLARATIONS
***************************************************************************/
extern uint8_t guint8EnableLogOutPut ;
/* only static local functions are declared here */
bool writeString(const char* szText);
/***************************************************************************
* FUNCTION IMPLEMENTATIONS
***************************************************************************/

bool writeString(const char* szText)
{
	bool bSuccess = false;

	if(UART_IF_ERR_OK == tszUART_Write(CONSOLE_UART, (const uint8_t*)szText, strlen(szText), 500,UART_IF_SEND_WAY_REG))
	{
		bSuccess = true;
	}
	return bSuccess;
}

void Console_dbgPrintf(uint8_t uint8LogMode, const char* szText, ...)
{
	va_list argl;

#if(SacpServerViaComEnable)	// add by will su
	return;
#endif
//	switch(uint8LogMode)
//	{
//	case LOG_MODE_SYS:
//		if((guint8EnableLogOutPut & LOG_SWITCH_ON_SYS) == LOG_SWITCH_OFF)
//		{
//			return;
//		}
//		break;
//	case LOG_MODE_UI:
//		if((guint8EnableLogOutPut & LOG_SWITCH_ON_UI) == LOG_SWITCH_OFF)
//		{
//			return;
//		}
//		break;
//	case LOG_MODE_APP:
//		if((guint8EnableLogOutPut & LOG_SWITCH_ON_APP) == LOG_SWITCH_OFF)
//		{
//			return;
//		}
//		break;
//	case LOG_MODE_PRINT:
//		if((guint8EnableLogOutPut & LOG_SWITCH_ON_PRINT) == LOG_SWITCH_OFF)
//		{
//			return;
//		}
//		break;
//	default:
//		return;
//	}

	va_start(argl, szText);
	if (PRINTF_BUFFERSIZE <= vsnprintf(mg_szBuff, PRINTF_BUFFERSIZE, szText, argl))
	{
	  writeString("ERROR @dbgprintf: Buffer too small!\n\r");
	  va_end(argl);
	  return;
	}
	va_end(argl);

	writeString(mg_szBuff);
}

