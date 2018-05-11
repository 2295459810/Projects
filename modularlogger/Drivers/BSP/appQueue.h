/****************************************************************************
* Copyright: Testo AG, 79849 Lenzkirch, Postfach 1140
*****************************************************************************/
/**@file
@author someone
@brief <b>Description: </b> Some module for something. The brief description
                           ends at an emtpy line

       This is where the detailed description starts. To describe the 
       usage you are encouraged to use coding examples:

       @code
         int var= 22;
       @endcode

set the module group for doxygen generation
\ingroup module_embedded_templates
*****************************************************************************/

#ifndef __HEADER_PROBE_QUEUE__
#define __HEADER_PROBE_QUEUE__

/****************************************************************************
*  INCLUDES
*****************************************************************************/

/* module generated config file must be incuded first!
   This include will define all cmake options and include
   <stdbool.h> to define the type bool (values true/false)
   <stdint.h> to define int types like uint8_t, uint32_t
   <stddef.h> to define size_t (sizeof(size_t) depends on the architecture)
   it will also define the decl. used for shared builds
   */
/*<<<<< REPLACE THIS INCLUDE WITH OUR OWN!!!*/

/*... other includes */
#include <stdint.h>
#include <stdbool.h>
/* for use in C++ code C-file header must use extern C define! */
#if defined (__cplusplus)
extern "C"
{
#endif

  /****************************************************************************
  * DEFINES
  *****************************************************************************/

  /* as few as possible */

#define MAX_QUEUE_SIZE 2000//128
  /****************************************************************************
  *  TYPE DECLARATIONS
  *****************************************************************************/

  /**
  *****************************************************************************
  @typedef SomeStruct
  some structure for something
  *****************************************************************************/
  //Circle queue  
  typedef struct _Circlequeue  
  {  
	  uint8_t data[MAX_QUEUE_SIZE];//
	  volatile uint32_t u32Front;//
	  volatile uint32_t u32Rear;//
	 volatile uint32_t u32Count;// volatile
  }Circlequeue; 

  /****************************************************************************
  *  FUNCTION DECLARATIONS
  *****************************************************************************/

  /**
  *****************************************************************************
  @brief Short descripion of some function doing something

  Detailed description ......
  ...... ........ ...... ......... .........
  ...... ......... ..... ... ....... .......... ..

  @param [in]  some parameter in
  @param [in]  some parameter in
  @param [out]  some parameter out

  @return some return value
  *****************************************************************************/

  /* note the decl is defined in the config file and must be used for shared builds! */

bool InitQueue( Circlequeue *pQueue);  
bool IsQueueEmpty( Circlequeue *pQueue); 
bool IsQueueFull( Circlequeue *pQueue);  
bool EnQueue( Circlequeue *pQueue, char e); 
char DeQueue( Circlequeue *pQueue);  
bool ClearQueue( Circlequeue *pQueue );
unsigned int GetLength( Circlequeue *pQueue);  

#if defined (__cplusplus)
}
#endif

#endif /* __HEADER_PROBE_QUEUE__ */


 
