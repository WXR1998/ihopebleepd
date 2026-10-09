#ifndef HW_UART_H
#define HW_UART_H

#include <stddef.h>

// 调试构建：串口探针定位阶段2 切换。定版后移除此行。
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
