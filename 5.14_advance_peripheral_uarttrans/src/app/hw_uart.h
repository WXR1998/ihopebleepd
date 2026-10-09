#ifndef HW_UART_H
#define HW_UART_H

#include <stddef.h>

// 诊断 UART：1 = 追捕期开启（会阻止 Standby，勿用于功耗测量）；0 = 正式版省电
#define USE_UART_DEBUG 0



#ifdef USE_UART_DEBUG
void HWUART_Init(void);
void HWUART_Printf(const char* format, ...);
void HWUART_Close(void);
#else
// Power saving: disable UART entirely
#define HWUART_Init()  ((void)0)
#define HWUART_Printf(format, ...)  ((void)0)
#define HWUART_Close()  ((void)0)
#endif // USE_UART_DEBUG

#endif // HW_UART_H
