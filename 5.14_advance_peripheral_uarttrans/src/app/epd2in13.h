
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
    EPD_MODE_FACTORY_TEMP_READ91_NO1A = 0xA, /* 91 激活但不写 0x1A 温度——隔离温度写入对黑度的影响 */
    EPD_MODE_FACTORY_TEMP_READ91_PARTIAL_X3 = 0xB, /* C7 后局部续刷激活 ×3——无清白相加深实验 */
    EPD_MODE_SSD1675B = 0xC, /* DEPG0213/SSD1675B reference init + LUT */
    EPD_MODE_T1_GATE296 = 0xE, /* T1: 296 门极 + Y 窗口 [84..295] 底部锚定（213ICE 原表） */
    EPD_MODE_T2_PRE40 = 0xF,   /* T2: 写 RAM 后 C7 前 0x22 0x40 预激活（213ICE 必发） */
    EPD_MODE_T4_21DUAL = 0x10, /* T4: init 尾 0x21 双字节 00 80（bw213_m3n 形式） */
    EPD_MODE_T5_ACT_C4 = 0x11, /* T5: 显示激活 C4 替代 C7（bw213_m OTP+Mode1） */
    /* T17: 中途打断锁峰值——激活后 (mode-0x13+2) 秒强制深睡断电（跳过 ReadBusy） */
    EPD_MODE_T17_I2 = 0x13,
    EPD_MODE_T17_I3 = 0x14,
    EPD_MODE_T17_I4 = 0x15,
    EPD_MODE_T17_I5 = 0x16,
    EPD_MODE_T17_I6 = 0x17,
    EPD_MODE_T17_I7 = 0x18,
    EPD_MODE_T17_I8 = 0x19,

    EPD_MODE_END = 0x1A,
};

/* Non-refresh diagnostics: mode 0 reads 0x2F and 0x1B; mode 1 writes a
 * tiny RAM pattern and reads via the controller's readback path. */
#define EPD_PROBE_REGISTERS 0x0
#define EPD_PROBE_RAM       0x1
#define EPD_PROBE_OTP       0x2  /* 0x2D readback */
#define EPD_PROBE_USER_ID   0x3  /* 0x2E readback */
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
void EPD_2IN13_ClearRedRAM(void);   /* E2: 0x26 全 0 清红 RAM（BW 数据保留） */
extern uint8_t epd_clear_red_on_update; /* E2: 0B 切换；1=下次全刷前清红 */
extern uint8_t epd_gate296;    /* T1: 296 门极窗口参数（init+prepare 共用） */
extern uint8_t epd_pre_act_40; /* T2: 0x40 预激活开关 */
extern uint8_t epd_act_c4;     /* T5: C4 激活开关 */
extern uint8_t epd_interrupt_delay_s; /* T17: >0=激活后延时秒数打断（跳过 ReadBusy 直接深睡断电） */
void EPD_2IN13_ReadBusy(void);

/* 波形实验：写入自定义 LUT（n=76 全表或 70 仅波形），调用方随后 0x22 激活 */
void EPD_2IN13_LoadLutConfig(const uint8_t *cfg, uint8_t n);
/* 波形实验入口：载入 LUT 并按指定激活值驱动一次（RAM 内容不变） */
void EPD_2IN13_ActivateWithLut(const uint8_t *cfg, uint8_t n, uint8_t act);

// void EPD_2IN13_DisplayPart(UBYTE *Image);
// void EPD_2IN13_DisplayPartBaseImage(UBYTE *Image);

void EPD_2IN13_Sleep(void);

// funcs
void EPD_Clear(uint8_t tofill);
void EPD_Display();
void EPD_Init_With_Mode(uint8_t mode);

/* Probe without activating a display update or loading a waveform. Returns
 * 0 on success and fills four bytes of raw controller readback. */
uint8_t EPD_2IN13_Probe(uint8_t mode, uint8_t out[4]);

#endif
