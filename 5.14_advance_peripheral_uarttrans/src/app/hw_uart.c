#include "hw_uart.h"

#include "board.h"
#include <ti/drivers/uart/UARTCC26XX.h>

#include <stdio.h>
#include <string.h>
#include <stdarg.h>

#if USE_UART_DEBUG

static UART_Handle UARTHandle;
static UART_Params UARTparams;

static void Uart_ReadCallback(UART_Handle handle, void *rxBuf, size_t size)
{ 
}

static void Uart_WriteCallback(UART_Handle handle, void *txBuf, size_t size)
{
  
}

void HWUART_Init()
{
  
  UART_init();                                      //Initialize UART module
  UART_Params_init(&UARTparams);                    //Initialize UART parameters
  UARTparams.baudRate = 115200;                     //Baud rate 115200
  UARTparams.dataLength = UART_LEN_8;               //Data length 8 bits
  UARTparams.stopBits = UART_STOP_ONE;              //Stop bits 1
  UARTparams.readDataMode = UART_DATA_BINARY;       //RX data mode binary
  UARTparams.writeDataMode = UART_DATA_TEXT;        //TX data mode text
  UARTparams.readMode = UART_MODE_CALLBACK;         //Read mode async
  UARTparams.writeMode = UART_MODE_BLOCKING;        //Write mode blocking
  UARTparams.readEcho = UART_ECHO_OFF;              //Read echo off
  UARTparams.readReturnMode = UART_RETURN_NEWLINE;  //Return on newline
  UARTparams.readCallback = Uart_ReadCallback;      //
  UARTparams.writeCallback = Uart_WriteCallback;    //
  
  UARTHandle = UART_open(Board_UART0, &UARTparams); //Open UART channel
 // UART_control(UARTHandle, UARTCC26XX_RETURN_PARTIAL_ENABLE,  NULL);   //Enable partial read callback
  
}

void HWUART_Printf(const char* format, ...)
{
      va_list arg;
  va_start(arg,format);
  uint8_t buf[108];
  uint16_t len;
  len = vsprintf((char*)buf, format, arg);
  UART_write(UARTHandle, buf, len);
}

void HWUART_Close(void)
{
  if (UARTHandle) {
    UART_close(UARTHandle);
    UARTHandle = NULL;
  }
}

#endif // USE_UART_DEBUG
