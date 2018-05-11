/***************************************************************************
* Copyright: Testo AG, 79849 Lenzkirch, Postfach 1140
***************************************************************************/

/****************************************************************************
*  INCLUDES
*****************************************************************************/
/* fist include must be the header */
#include <stdio.h>  
#include <stdlib.h>
#include <stdbool.h>
#include "appQueue.h"
/***************************************************************************
* DEFINES
***************************************************************************/
/* as few as posible */

/***************************************************************************
* MODULE GLOBALS
***************************************************************************/

/* some static variable */


/***************************************************************************
* LOCAL FUNCTION DECLARATIONS
***************************************************************************/

/* only static local functions are declared here */

/***************************************************************************
* FUNCTION IMPLEMENTATIONS
***************************************************************************/
bool InitQueue( Circlequeue *pQueue)  
{  
	pQueue->u32Front = pQueue->u32Rear = 0;  
	pQueue->u32Count = 0;  
	return true;  
  
}  

/*************************************************/  
bool IsQueueEmpty( Circlequeue *pQueue)  
{  
	if(pQueue->u32Front == pQueue->u32Rear)//(pQueue->u32Count == 0)
		return true;  
	else  
		return false;  
  
  
}  
  
/*************************************************/  
bool IsQueueFull( Circlequeue *pQueue)  
{  
	if(((pQueue->u32Rear +1) % MAX_QUEUE_SIZE ) == pQueue->u32Front)//(pQueue->u32Count == MAX_QUEUE_SIZE)
		return true;  
	else  
		return false;  
  
  
}   
/*************************************************/  
bool EnQueue( Circlequeue *pQueue, char e)  
{  
	//
	if(IsQueueFull(pQueue))//if(pQueue->u32Count == MAX_QUEUE_SIZE)
	{  
		//Console_dbgPrintf("The pQueue is full\r\n");
		return false;  
	}  
	//
	pQueue->data[pQueue->u32Rear] = (uint8_t)e;
	//
	pQueue->u32Rear = (pQueue->u32Rear + 1) % MAX_QUEUE_SIZE;  
	//
	pQueue->u32Count++;  
	return true;  
  
}  
  
/*************************************************/ 
char DeQueue( Circlequeue *pQueue)  
{  
	//
	if(IsQueueEmpty(pQueue))//if(pQueue->u32Count == 0)
	{  
		//Console_dbgPrintf("The pQueue is empty!\r\n");
		//exit(EXIT_FAILURE);  
	}  
  
	//
	char e = (char)pQueue->data[pQueue->u32Front];
	//
	pQueue->u32Front = (pQueue->u32Front + 1) % MAX_QUEUE_SIZE;  
	//
	pQueue->u32Count--;  
  
	return e;  
  
}  
  

/*************************************************/ 
bool ClearQueue( Circlequeue *pQueue )  
{  
	pQueue->u32Front = pQueue->u32Rear = 0;  
	pQueue->u32Count = 0;  
	return true;    
}  
  
/*************************************************/  
unsigned int GetLength(Circlequeue *pQueue)  
{  
	return pQueue->u32Count;    
}  
