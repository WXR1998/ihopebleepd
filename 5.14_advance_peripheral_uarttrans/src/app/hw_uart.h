#ifndef HW_UART_H
#define HW_UART_H

#include <stddef.h>

// 诊断构建：1.14 临时点亮（EPD 卡死追捕），完成后移除
#define USE_UART_DEBUG 1



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
