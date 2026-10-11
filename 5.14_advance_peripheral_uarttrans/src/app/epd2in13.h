
#ifndef EPD_2IN13_H
#define EPD_2IN13_H

#include <stdint.h>

// Display resolution
#ifdef SIZE_4IN2
#define EPD_2IN13_WIDTH       400
#define EPD_2IN13_HEIGHT      300
#else
#define EPD_2IN13_WIDTH       104
#define EPD_2IN13_HEIGHT      212
#endif

#define EPD_2IN13_BUFFER_WIDTH      ((EPD_2IN13_WIDTH % 8 == 0)? (EPD_2IN13_WIDTH / 8 ): (EPD_2IN13_WIDTH / 8 + 1))
#define EPD_2IN13_BUFFER_HEIGHT     EPD_2IN13_HEIGHT
#define EPD_2IN13_BUFFER_SIZE     (EPD_2IN13_BUFFER_HEIGHT * EPD_2IN13_BUFFER_WIDTH)

static const uint16_t EPD_Buffer_Width = EPD_2IN13_BUFFER_WIDTH;
static const uint16_t EPD_Buffer_Height = EPD_2IN13_BUFFER_HEIGHT; 
static const uint16_t EPD_Buffer_Size = EPD_2IN13_BUFFER_SIZE; 

// 睡眠收尾模式（褪色实验，BLE 0x08 切换；复位/重新上电回到 EPD_SLEEP_DEFAULT）
enum {
    EPD_SLEEP_DEFAULT = 0x0,   /* C3 收尾后不等待 BUSY（历史行为，基线） */
    EPD_SLEEP_WAIT_BUSY = 0x1, /* C3 收尾后等待 BUSY 再深睡断电（实验 A） */
    EPD_SLEEP_SKIP_C3 = 0x2,   /* 跳过 C3 收尾直接深睡断电（实验 C，诊断） */
    EPD_SLEEP_NONE = 0x3,      /* 完全不休眠，面板持续供电（诊断，LED 常亮） */
};

enum {
    EPD_MODE_BEGIN = 0x0,

    EPD_MODE_BW = 0x1,
    EPD_MODE_BWR = 0x2,
    EPD_MODE_GRAY = 0x3,
    EPD_MODE_BW3 = 0x4,   /* BW·三刷：灰度 LUT 单帧三叠刷（褪色实验 B） */
    EPD_MODE_FACTORY = 0x5, /* 原厂初始化序列实验（B1 分支） */
    EPD_MODE_FACTORY_TEMP91 = 0x6, /* 历史固定 25℃分支（仅复现） */
    EPD_MODE_FACTORY_TEMP_READ91 = 0x7, /* 原厂共享数据线读回温度，单刷 */
    EPD_MODE_FACTORY_TEMP_READ91_X2 = 0x8, /* 同一 0x91 帧连续激活两次 */
    EPD_MODE_FACTORY_TEMP_READ91_PARTIAL = 0x9, /* C7 后使用 0x0C 局部续刷 */

    EPD_MODE_END = 0xA,
};
// #define EPD_2IN13_FULL			0
// #define EPD_2IN13_PART			1

extern uint8_t epd_sleep_mode;
extern uint8_t epd_display_mode;
extern uint8_t epd_last_temperature;
extern uint8_t epd_last_status;

void epd_hw_init();

void EPD_2IN13_Init();
void EPD_2IN13_Init_With_LUT(const unsigned char* lut);
void EPD_2IN13_Clear(void);
void EPD_2IN13_Display(const uint8_t *Image);

void EPD_2IN13_PrepareBlkRAM(void);
void EPD_2IN13_PrepareRedRAM(void);
void EPD_2IN13_WriteRAM(const uint8_t *buf, const int len);

void EPD_2IN13_UpdateDisplay(void);

// void EPD_2IN13_DisplayPart(UBYTE *Image);
// void EPD_2IN13_DisplayPartBaseImage(UBYTE *Image);

void EPD_2IN13_Sleep(void);

// funcs
void EPD_Clear(uint8_t tofill);
void EPD_Display();
void EPD_Init_With_Mode(uint8_t mode);

#endif
