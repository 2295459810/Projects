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

#ifndef __DBG_PRINTF_H___
# define __DBG_PRINTF_H___

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
#include "uart_if.h"

/*... other includes */

/* for use in C++ code C-file header must use extern C define! */
#if defined (__cplusplus)
extern "C"
{
#endif

  /****************************************************************************
  * DEFINES
  *****************************************************************************/

  #define CONSOLE_UART	UART_IF_DEV_1
  #define PRINTF_BUFFERSIZE	256
  /* as few as possible */

#define LOG_MODE_UNKOWN			0
#define LOG_MODE_SYS			1
#define LOG_MODE_UI				2
#define LOG_MODE_APP 			3
#define LOG_MODE_PRINT 			4
#define LOG_MODE_OnBoardMea 	5

#define LOG_SWITCH_OFF			0
#define LOG_SWITCH_ON_SYS		1
#define LOG_SWITCH_ON_UI		2
#define LOG_SWITCH_ON_APP 		4
#define LOG_SWITCH_ON_PRINT		8
#define LOG_SWITCH_ON_OnBoardMea 16//will add

#define SYS_dbgprintf(...)  Console_dbgPrintf(LOG_MODE_SYS, __VA_ARGS__)
#define UI_dbgprintf(...)  Console_dbgPrintf(LOG_MODE_UI, __VA_ARGS__)
#define APP_dbgprintf(...)  Console_dbgPrintf(LOG_MODE_APP, __VA_ARGS__)
#define PRINT_dbgprintf(...)  Console_dbgPrintf(LOG_MODE_PRINT, __VA_ARGS__)
#define OnBoardMea_dbgprintf(...)  Console_dbgPrintf(LOG_MODE_OnBoardMea, __VA_ARGS__)//will add
  /****************************************************************************
  *  TYPE DECLARATIONS
  *****************************************************************************/

  /**
  *****************************************************************************
  @typedef SomeStruct
  some structure for something
  *****************************************************************************/


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
  void Console_dbgPrintf(uint8_t uint8LogMode, const char* szText, ...);

#if defined (__cplusplus)
}
#endif

#endif /* HEADER_GUARD */
