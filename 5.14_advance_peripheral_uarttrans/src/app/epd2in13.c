
#include "epd2in13.h"
#include "hw_uart.h"

// SPI/EPD error flag: set by SPI layer on failure, read by handle_cmd()
// to propagate errors to the web response. 0 = no error.
uint8_t epd_spi_error = 0;

// 睡眠收尾模式（见 epd2in13.h 枚举；默认 = 历史行为，褪色实验用）
uint8_t epd_sleep_mode = EPD_SLEEP_DEFAULT;

// 当前显示模式（EPD_Display 据此决定 BW·三刷的激活次数）
uint8_t epd_display_mode = EPD_MODE_BW;
uint8_t epd_last_temperature = 0xFF;
uint8_t epd_last_status = 0xFF;

static void epd_spi_ensure_open(void);


#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC26XXDMA.h>
#include <ti/sysbios/knl/Task.h>
#include <string.h>

#include "board.h"

 // for debug
//#include "inc/sdi_task.h"
#include "util.h"
 
#include <ti/drivers/PIN.h>


#define BLUE_LED_PIN             IOID_0     // low active

#define REED_PIN    IOID_13     // reed switch, ground when there is magnet
#define TEST_PIN    IOID_15     // on back of pcb with 'test'

#define EPD_POWER_PIN             IOID_20       // low active

#define EPD_RST_PIN             IOID_10
#define EPD_DC_PIN              IOID_11
#define EPD_BUSY_PIN            IOID_9
#define EPD_CS_PIN              IOID_12
#define EPD_BB_CLK_PIN          IOID_18
#define EPD_BB_DATA_PIN         IOID_19



/*********************************************************************
 * LOCAL PARAMETER
 */   
static PIN_Handle GPIOHandle = NULL;
static PIN_State GPIOState;
static PIN_Config GPIOTable[] =
{
  EPD_BUSY_PIN          | PIN_GPIO_OUTPUT_DIS  | PIN_INPUT_EN  |  PIN_PULLUP,
  
  EPD_POWER_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  BLUE_LED_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  
  EPD_DC_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_RST_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
  EPD_CS_PIN | PIN_GPIO_OUTPUT_EN | PIN_GPIO_HIGH | PIN_PUSHPULL | PIN_DRVSTR_MIN,
 
 
  PIN_TERMINATE
};

static SPI_Handle      SPIHandle = NULL;
static SPI_Params      SPIparams;

const unsigned char EPD_2IN13_lut_gray_update[]= {
    0x40,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT0: BB:     VS 0 ~7
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT1: BW:     VS 0 ~7
    0x40,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT2: WB:     VS 0 ~7
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT3: WW:     VS 0 ~7
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT4: VCOM:   VS 0 ~7

    0x03,0x05,0x00,0x00,0x00,                       // TP0 A~D RP0
    0x00,0x00,0x00,0x00,0x00,                       // TP0 A~D RP0
    0x00,0x00,0x00,0x00,0x00,                       // TP0 A~D RP0
    0x00,0x00,0x00,0x00,0x00,                       // TP0 A~D RP0
    0x00,0x00,0x00,0x00,0x00,                       // TP1 A~D RP1
    0x00,0x00,0x00,0x00,0x00,                       // TP2 A~D RP2
    0x00,0x00,0x00,0x00,0x00,                       // TP6 A~D RP6

    0x17,0x41,0xAC,0x32,0x02,0x0D,
    // 0x15,0x41,0xA8,0x32,0x30,0x0A,
};

const unsigned char EPD_2IN13_lut_bw_update[]= {
    0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT0: BB:     VS 0 ~7
    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT1: BW:     VS 0 ~7
    0x80,0x60,0x40,0x00,0x00,0x00,0x00,             //LUT2: WB:     VS 0 ~7
    0x10,0x60,0x20,0x00,0x00,0x00,0x00,             //LUT3: WW:     VS 0 ~7
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,             //LUT4: VCOM:   VS 0 ~7

    0x03,0x03,0x00,0x00,0x02,                       // TP0 A~D RP0
    0x09,0x09,0x00,0x00,0x02,                       // TP1 A~D RP1
    0x09,0x09,0x00,0x00,0x02,                       // TP2 A~D RP2
    0x00,0x00,0x00,0x00,0x00,                       // TP0 A~D RP0
    0x00,0x00,0x00,0x00,0x00,                       // TP1 A~D RP1
    0x00,0x00,0x00,0x00,0x00,                       // TP2 A~D RP2
    0x00,0x00,0x00,0x00,0x00,                       // TP6 A~D RP6

    0x15,0x41,0xA8,0x32,0x30,0x0A,
};

/* 同类 2.13 控制器局部更新 LUT；仅由 91→partial 诊断模式使用。 */
const unsigned char EPD_2IN13_lut_partial_update[] = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x80,0x00,0x00,0x00,0x00,0x00,0x00,
  0x40,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0A,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,
  0x15,0x41,0xA8,0x32,0x30,0x0A,
};

// quick and dirty way
// Task_sleep defined in <ti/sysbios/knl/Task.h>
static void Util_delay_ms(uint16_t t)
{  
  Task_sleep( ((t) * 1000) / Clock_tickPeriod );
}

static void HwUARTPrintf(const char *str)
{
  HWUART_Printf(str);
}

static void DEV_Digital_Write(uint32_t pin, uint8_t value)
{    
    PIN_setOutputValue(GPIOHandle, pin, value);    
}

static int DEV_Digital_Read(uint32_t pin)
{
    int ret;
  ret = PIN_getInputValue(pin);
  return ret;
}

static void DEV_Delay_ms(uint16_t t)
{
    Util_delay_ms(t);
}

static void DEV_SPI_WriteByte(uint8_t byte)
{
    uint8_t txbuf[2];
    uint8_t rxbuf[2];
    
    txbuf[0] = byte;
    
  epd_spi_ensure_open();

  SPI_Transaction spiTransaction;
  spiTransaction.arg = NULL;
  spiTransaction.count = 1;
  spiTransaction.txBuf = txbuf;
  spiTransaction.rxBuf = rxbuf;
    
  bool ok =  SPI_transfer(SPIHandle, &spiTransaction);
 
  if (!ok) {
    HwUARTPrintf("spi transf fail\r\n");
    epd_spi_error = 0xf0;
  }
  
}

// static uint8_t DEV_SPI_ReadByte(uint8_t byte)
// {
//     uint8_t txbuf[2];
//     uint8_t rxbuf[2];
//     
//   SPI_Transaction spiTransaction;
//   spiTransaction.arg = NULL;
//   spiTransaction.count = 1;
//   spiTransaction.txBuf = txbuf;
//   spiTransaction.rxBuf = rxbuf;
//     
//   bool ok =  SPI_transfer(SPIHandle, &spiTransaction);
//  
//   if (!ok) {
//     HwUARTPrintf("spi transf fail\r\n");
//   }
// 
//   return rxbuf[0];
//   
// }

/* 原厂 ed00/ed50 的最小读回等价路径；仅由读回诊断模式调用。 */
static uint8_t EPD_2IN13_ReadSharedByte(void)
{
  uint8_t value = 0;
  uint8_t second = 0;
  epd_last_temperature = 0xFF;
  epd_last_status = 0xFF;
  if (SPIHandle) { SPI_close(SPIHandle); SPIHandle = NULL; }
  if (PIN_add(GPIOHandle, EPD_BB_CLK_PIN | PIN_INPUT_EN | PIN_PULLDOWN) != PIN_SUCCESS ||
      PIN_add(GPIOHandle, EPD_BB_DATA_PIN | PIN_INPUT_EN | PIN_PULLDOWN) != PIN_SUCCESS) {
    PIN_remove(GPIOHandle, EPD_BB_CLK_PIN);
    PIN_remove(GPIOHandle, EPD_BB_DATA_PIN);
    epd_spi_ensure_open();
    epd_spi_error = 0xf1;
    return 0;
  }
  PIN_setConfig(GPIOHandle, PIN_BM_ALL,
                EPD_BB_CLK_PIN | PIN_GPIO_OUTPUT_EN | PIN_PUSHPULL | PIN_GPIO_LOW);
  PIN_setConfig(GPIOHandle, PIN_BM_ALL,
                EPD_BB_DATA_PIN | PIN_INPUT_EN | PIN_PULLDOWN);
  DEV_Digital_Write(EPD_CS_PIN, 0);
  DEV_Digital_Write(EPD_DC_PIN, 1);
  DEV_Digital_Write(EPD_BB_CLK_PIN, 0);
  // ed00 读第一字节（温度）；ed50 在同一 CS 周期再读第二字节并收尾。
  for (uint8_t byte = 0; byte < 2; byte++) {
    for (uint8_t bit = 0; bit < 8; bit++) {
      uint8_t sample = PIN_getInputValue(EPD_BB_DATA_PIN) ? 1 : 0;
      if (byte == 0) value = (uint8_t)((value << 1) | sample);
      else second = (uint8_t)((second << 1) | sample);
      DEV_Digital_Write(EPD_BB_CLK_PIN, 1);
      DEV_Digital_Write(EPD_BB_CLK_PIN, 0);
    }
  }
  DEV_Digital_Write(EPD_CS_PIN, 1);
  PIN_setConfig(GPIOHandle, PIN_BM_ALL, EPD_BB_CLK_PIN | PIN_INPUT_EN | PIN_PULLDOWN);
  PIN_setConfig(GPIOHandle, PIN_BM_ALL, EPD_BB_DATA_PIN | PIN_INPUT_EN | PIN_PULLDOWN);
  PIN_remove(GPIOHandle, EPD_BB_CLK_PIN);
  PIN_remove(GPIOHandle, EPD_BB_DATA_PIN);
  epd_spi_ensure_open();
  epd_last_temperature = value;
  epd_last_status = second;
  return value;
}

// should be only called once!
void epd_hw_init()
{

    HwUARTPrintf("setup epd gpio\r\n");
   GPIOHandle = PIN_open(&GPIOState, GPIOTable);      

     HwUARTPrintf("setup epd spi\r\n");
    
       SPI_init();
  SPI_Params_init(&SPIparams);
  SPIparams.bitRate  = 1000000;                    //1MHz
  SPIparams.dataSize = 8; 
  SPIparams.frameFormat = SPI_POL0_PHA0;           //
  SPIparams.mode = SPI_MASTER;                     //SPI master
  SPIparams.transferCallbackFxn = NULL;
  SPIparams.transferMode = SPI_MODE_BLOCKING;      // blocking
  SPIparams.transferTimeout = SPI_WAIT_FOREVER;
  
  SPIHandle = SPI_open(CC2640R2_LAUNCHXL_SPI0, &SPIparams);
  if (NULL == SPIHandle) {
    HwUARTPrintf("spi open fail\r\n");
  }
  
}

// EPD_2IN13_Sleep 每次刷屏后关闭 SPI（省电）；此处按需重开，
// 否则第二次刷屏会拿 NULL 句柄调 SPI_transfer 直接硬故障。
static void epd_spi_ensure_open(void)
{
  if (SPIHandle) {
    return;
  }
  SPI_Handle h = SPI_open(CC2640R2_LAUNCHXL_SPI0, &SPIparams);
  if (NULL == h) {
    HwUARTPrintf("spi reopen fail\r\n");
    epd_spi_error = 0xf0;
    return;
  }
  SPIHandle = h;
  HwUARTPrintf("spi reopen\r\n");
}

/******************************************************************************
function :	Software reset
parameter:
******************************************************************************/
static void EPD_2IN13_Reset(void)
{
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
    DEV_Digital_Write(EPD_RST_PIN, 0);
    DEV_Delay_ms(2);
    DEV_Digital_Write(EPD_RST_PIN, 1);
    DEV_Delay_ms(200);
}

/******************************************************************************
function :	send command
parameter:
     Reg : Command register
******************************************************************************/
static void EPD_2IN13_SendCommand(uint8_t Reg)
{
    DEV_Digital_Write(EPD_DC_PIN, 0);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Reg);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

/******************************************************************************
function :	send data
parameter:
    Data : Write data
******************************************************************************/
static void EPD_2IN13_SendData(uint8_t Data)
{
    DEV_Digital_Write(EPD_DC_PIN, 1);
    DEV_Digital_Write(EPD_CS_PIN, 0);
    DEV_SPI_WriteByte(Data);
    DEV_Digital_Write(EPD_CS_PIN, 1);
}

// /******************************************************************************
// function :	recv data
// parameter:
//     Data : Read data
// ******************************************************************************/
// static uint8_t EPD_2IN13_RecvData(uint8_t Data)
// {
//   uint8_t ret;
//     DEV_Digital_Write(EPD_DC_PIN, 1);
//     DEV_Digital_Write(EPD_CS_PIN, 0);
//     ret = DEV_SPI_ReadByte(Data);
//     DEV_Digital_Write(EPD_CS_PIN, 1);
//     return ret;
// }

/******************************************************************************
function :	Wait until the busy_pin goes LOW
parameter:
******************************************************************************/
void EPD_2IN13_ReadBusy(void)
{
    int busy_wait = 0;
    HwUARTPrintf("e-Paper busy\r\n");

    // 电源脚是输出，必须读取输出锁存值；输入缓冲在输出模式下可能返回不确定值。
    if (PIN_getOutputValue(EPD_POWER_PIN) == 1) {
        HwUARTPrintf("e-Paper busy: PANEL POWER OFF!\r\n");
        epd_spi_error = 0xf2;
        return;
    }

    while(DEV_Digital_Read(EPD_BUSY_PIN) == 1) {      //LOW: idle, HIGH: busy
        DEV_Delay_ms(100);
        if (++busy_wait % 20 == 0) {
            HWUART_Printf("e-Paper busy %ds\r\n", busy_wait / 10);
        }
        if (busy_wait >= 200) {  // 防卡死：20s 超时，宁可本次刷屏失败也不拖死整机
            HwUARTPrintf("e-Paper busy TIMEOUT!\r\n");
            epd_spi_error = 0xf1;  // busy 超时：本次刷屏未完成，通过应答上报
            break;
        }
    }
    HWUART_Printf("e-Paper busy release %ds\r\n", busy_wait / 10);
}


/******************************************************************************
function :	Initialize the e-Paper register
parameter:
******************************************************************************/
void EPD_2IN13_Init() {
  EPD_2IN13_Init_With_LUT(NULL);
}

void EPD_2IN13_Init_With_LUT(const unsigned char* lut)
{
  // power on
  DEV_Digital_Write(EPD_POWER_PIN, 0);
  DEV_Digital_Write(BLUE_LED_PIN, 0);
  DEV_Delay_ms(100);
  
  EPD_2IN13_Reset();
  EPD_2IN13_ReadBusy();

  EPD_2IN13_SendCommand(0x12); // soft reset
  EPD_2IN13_ReadBusy();

  EPD_2IN13_SendCommand(0x74); //set analog block control
  EPD_2IN13_SendData(0x54);
  EPD_2IN13_SendCommand(0x7E); //set digital block control
  EPD_2IN13_SendData(0x3B);

  EPD_2IN13_SendCommand(0x01); //Driver output control
  // EPD_2IN13_SendData(0xD3);
  //EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)&0xff);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)>>8);
  EPD_2IN13_SendData(0x00);

  EPD_2IN13_SendCommand(0x11); //data entry mode
  EPD_2IN13_SendData(0x01);

  EPD_2IN13_SendCommand(0x44); //set Ram-X address start/end position
  EPD_2IN13_SendData(0x00);
  // EPD_2IN13_SendData(0x0C);    //0x0C-->(15+1)*8=128
  EPD_2IN13_SendData((EPD_2IN13_WIDTH/8)-1);

  EPD_2IN13_SendCommand(0x45); //set Ram-Y address start/end position
  // EPD_2IN13_SendData(0xD3);   //0xF9-->(249+1)=250
  // EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)&0xff);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)>>8);
  EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData(0x00);

  EPD_2IN13_SendCommand(0x3C); //BorderWavefrom
  EPD_2IN13_SendData(0x01);
  
  // OTP 路径不手工覆盖 VCOM；上游原始 OTP 初始化直接从 0x18/0x22/0x20 加载。
  // 自定义 LUT 路径仍使用参考驱动的 VCOM=0x55。
  if(lut) {
    EPD_2IN13_SendCommand(0x2C); //set vcom value
    EPD_2IN13_SendData(0x55);
    EPD_2IN13_SendCommand(0x03);
    EPD_2IN13_SendData(lut[70]);

    EPD_2IN13_SendCommand(0x04); //;
    EPD_2IN13_SendData(lut[71]);
    EPD_2IN13_SendData(lut[72]);
    EPD_2IN13_SendData(lut[73]);

    EPD_2IN13_SendCommand(0x3A);     //Dummy Line;
    EPD_2IN13_SendData(lut[74]);
    EPD_2IN13_SendCommand(0x3B);     //Gate time;
    EPD_2IN13_SendData(lut[75]);

    EPD_2IN13_SendCommand(0x32);
    for(int i=0; i<70; i++)
        EPD_2IN13_SendData(lut[i]);
  } else {
    // load lut
    EPD_2IN13_SendCommand(0x18); // set built in temperature sensor
    EPD_2IN13_SendData(0x80); //
    
    EPD_2IN13_SendCommand(0x22); // 
    EPD_2IN13_SendData(0xB1); //
    // EPD_2IN13_SendData(0xC0); //
    
    EPD_2IN13_SendCommand(0x20); // load LUT from OTP
    EPD_2IN13_ReadBusy();
  }

  DEV_Delay_ms(100);

  HwUARTPrintf("epd initialized\r\n");
}

/******************************************************************************
function :	Clear screen
parameter:
******************************************************************************/
void EPD_2IN13_Clear(void)
{
  HwUARTPrintf("epd clear\r\n");
  uint16_t Width, Height;
  Width = (EPD_2IN13_WIDTH % 8 == 0)? (EPD_2IN13_WIDTH / 8 ): (EPD_2IN13_WIDTH / 8 + 1);
  Height = EPD_2IN13_HEIGHT;

  EPD_2IN13_SendCommand(0x24);
  for (uint16_t j = 0; j < Height; j++) {
      for (uint16_t i = 0; i < Width; i++) {
          EPD_2IN13_SendData(0XFF);
      }
  }

  EPD_2IN13_UpdateDisplay();
}

/******************************************************************************
function :	Sends the image buffer in RAM to e-Paper and displays
parameter:
******************************************************************************/
void EPD_2IN13_Display(const uint8_t *Image)
{
    uint16_t Width, Height;
    Width = (EPD_2IN13_WIDTH % 8 == 0)? (EPD_2IN13_WIDTH / 8 ): (EPD_2IN13_WIDTH / 8 + 1);
    Height = EPD_2IN13_HEIGHT;

    EPD_2IN13_SendCommand(0x24);
    for (uint16_t j = 0; j < Height; j++) {
        for (uint16_t i = 0; i < Width; i++) {
            EPD_2IN13_SendData(Image[i + j * Width]);
        }
    }
    EPD_2IN13_UpdateDisplay();
}

void EPD_2IN13_PrepareBlkRAM(void)
{
  EPD_2IN13_SendCommand(0x4E);  // set RAM x address counter
  EPD_2IN13_SendData(0x00);
  
  EPD_2IN13_SendCommand(0x4F);  // set RAM Y address counter
  // EPD_2IN13_SendData(0xD3);
  // EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)&0xff);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)>>8);
  
    // black white image
    EPD_2IN13_SendCommand(0x24);
    
}

void EPD_2IN13_PrepareRedRAM(void)
{
   EPD_2IN13_SendCommand(0x4E);  // set RAM x address counter
  EPD_2IN13_SendData(0x00);
  
  EPD_2IN13_SendCommand(0x4F);  // set RAM Y address counter
  // EPD_2IN13_SendData(0xD3);
  // EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)&0xff);
  EPD_2IN13_SendData((EPD_2IN13_HEIGHT-1)>>8);
  
    // red white image
    EPD_2IN13_SendCommand(0x26);
}

void EPD_2IN13_WriteRAM(const uint8_t *buf, const int len)
{
    for (int i = 0; i < len; i++) {
        EPD_2IN13_SendData(buf[i]);
    }
}

static void EPD_2IN13_PreparePartialUpdate(void)
{
  EPD_2IN13_SendCommand(0x2C); EPD_2IN13_SendData(0x26);
  EPD_2IN13_ReadBusy();
  EPD_2IN13_SendCommand(0x32);
  for (uint8_t i = 0; i < 70; i++) EPD_2IN13_SendData(EPD_2IN13_lut_partial_update[i]);
  EPD_2IN13_SendCommand(0x37);
  EPD_2IN13_SendData(0x00); EPD_2IN13_SendData(0x00); EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData(0x00); EPD_2IN13_SendData(0x40); EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendCommand(0x22); EPD_2IN13_SendData(0xC0);
  EPD_2IN13_SendCommand(0x20); EPD_2IN13_ReadBusy();
  EPD_2IN13_SendCommand(0x3C); EPD_2IN13_SendData(0x01);
}

static void EPD_2IN13_UpdateDisplayPartial(void)
{
  EPD_2IN13_SendCommand(0x22); EPD_2IN13_SendData(0x0C);
  EPD_2IN13_SendCommand(0x20); EPD_2IN13_ReadBusy();
  DEV_Delay_ms(200);
}

void EPD_2IN13_UpdateDisplay(void)
{
     HwUARTPrintf("turn on display\r\n");
  
    EPD_2IN13_SendCommand(0x22);
    EPD_2IN13_SendData(0xC7);
    EPD_2IN13_SendCommand(0x20);
    EPD_2IN13_ReadBusy();
    DEV_Delay_ms(200);
}

/******************************************************************************
function :	Enter sleep mode
parameter:
******************************************************************************/
void EPD_2IN13_Sleep(void)
{
    if (epd_sleep_mode == EPD_SLEEP_NONE) {
        // 诊断：完全不休眠，面板持续供电（LED 常亮提示），只关 SPI 便于下轮重开
        if (SPIHandle) {
            SPI_close(SPIHandle);
            SPIHandle = NULL;
        }
        return;
    }
    if (epd_sleep_mode == EPD_SLEEP_SKIP_C3) {
        // 实验 C：跳过 C3 收尾，直接深睡断电（验证 C3 是否引起褪色）
        EPD_2IN13_SendCommand(0x10); //enter deep sleep
        EPD_2IN13_SendData(0x01);
        DEV_Delay_ms(100);

        DEV_Digital_Write(EPD_POWER_PIN, 1);
        DEV_Digital_Write(BLUE_LED_PIN, 1);
        DEV_Delay_ms(100);

        if (SPIHandle) {
            SPI_close(SPIHandle);
            SPIHandle = NULL;
        }
        return;
    }
    EPD_2IN13_SendCommand(0x22); //POWER OFF
    EPD_2IN13_SendData(0xC3);
    EPD_2IN13_SendCommand(0x20);

    if (epd_sleep_mode == EPD_SLEEP_WAIT_BUSY) {
        // 实验 A：等电源收尾序列真正结束再深睡断电
        EPD_2IN13_ReadBusy();
    }

    EPD_2IN13_SendCommand(0x10); //enter deep sleep
    EPD_2IN13_SendData(0x01);
    DEV_Delay_ms(100);
    
    // power off
  DEV_Digital_Write(EPD_POWER_PIN, 1);
  DEV_Digital_Write(BLUE_LED_PIN, 1);
  DEV_Delay_ms(100);

  // Power saving: close SPI after screen update to release power dependency
  if (SPIHandle) {
    SPI_close(SPIHandle);
    SPIHandle = NULL;
  }
}


// funcs
void EPD_Clear(uint8_t tofill) {
  HwUARTPrintf("work: epd clear\n");

  EPD_Init_With_Mode(EPD_MODE_BW);

  EPD_2IN13_PrepareBlkRAM();
  for(int i=0; i<EPD_Buffer_Size; i++) {
    EPD_2IN13_SendData(tofill);
  }

  EPD_2IN13_PrepareRedRAM();
  for(int i=0; i<EPD_Buffer_Size; i++) {
    EPD_2IN13_SendData(0x00);
  }

  EPD_Display(EPD_MODE_BW);
}

void EPD_Display() {
  // BW·三刷：同一帧 RAM 连续激活 3 次（第 2/3 次走 BB 类再驱动，等效灰度三叠刷）
  uint8_t passes = (epd_display_mode == EPD_MODE_BW3) ? 3 :
                   ((epd_display_mode == EPD_MODE_FACTORY_TEMP_READ91_X2 ||
                     epd_display_mode == EPD_MODE_FACTORY_TEMP_READ91_PARTIAL) ? 2 : 1);
  for (uint8_t i = 0; i < passes; i++) {
    if (epd_display_mode == EPD_MODE_FACTORY_TEMP_READ91_PARTIAL && i == 1) {
      EPD_2IN13_PreparePartialUpdate();
      EPD_2IN13_UpdateDisplayPartial();
    } else {
      EPD_2IN13_UpdateDisplay();
    }
  }
  EPD_2IN13_Sleep();
}

/*
 * 原厂初始化序列实验：从 original_backup.bin ARM Thumb 反汇编得到的
 * EPD 命令顺序。它不替换 OTP 默认路径，只由 mode=EPD_MODE_FACTORY 选择。
 * 温度分支固定采用原厂常见的 0x22=B1 路径；实机结果用于验证
 * 0x2B/0x3D/0x3E/0x3F 与 A1→B1 两阶段是否影响锐度。
 */
static void EPD_2IN13_Init_FactorySequence(uint8_t tempMode)
{
  DEV_Digital_Write(EPD_POWER_PIN, 0);
  DEV_Digital_Write(BLUE_LED_PIN, 0);
  DEV_Delay_ms(100);
  EPD_2IN13_Reset();
  EPD_2IN13_ReadBusy();

  EPD_2IN13_SendCommand(0x74); EPD_2IN13_SendData(0x54);
  EPD_2IN13_SendCommand(0x7E); EPD_2IN13_SendData(0x3B);

  // 原厂反汇编 @0xEB7A：0x2B + 04 63；随后 0x0C + 8B 9C 96 0F
  EPD_2IN13_SendCommand(0x2B);
  EPD_2IN13_SendData(0x04); EPD_2IN13_SendData(0x63);
  EPD_2IN13_SendCommand(0x0C);
  EPD_2IN13_SendData(0x8B); EPD_2IN13_SendData(0x9C);
  EPD_2IN13_SendData(0x96); EPD_2IN13_SendData(0x0F);

  EPD_2IN13_SendCommand(0x01);
  EPD_2IN13_SendData(0xD3); EPD_2IN13_SendData(0x00); EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendCommand(0x11); EPD_2IN13_SendData(0x01);
  EPD_2IN13_SendCommand(0x18); EPD_2IN13_SendData(0x80);

  EPD_2IN13_SendCommand(0x44);
  EPD_2IN13_SendData(0x00); EPD_2IN13_SendData(0x0C);
  EPD_2IN13_SendCommand(0x45);
  EPD_2IN13_SendData(0xD3); EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendData(0x00); EPD_2IN13_SendData(0x00);
  EPD_2IN13_SendCommand(0x3C); EPD_2IN13_SendData(0x01);

  // 原厂第一阶段：0x22=A1 → 0x20
  EPD_2IN13_SendCommand(0x22); EPD_2IN13_SendData(0xA1);
  EPD_2IN13_SendCommand(0x20); EPD_2IN13_ReadBusy();

  // 原厂 0x1B 读取发生在 0x3D/0x3E/0x3F 之前。
  uint8_t temperature = 0;
  if (tempMode == 1) {
    EPD_2IN13_SendCommand(0x1B);
  } else if (tempMode == 2) {
    EPD_2IN13_SendCommand(0x1B);
    temperature = EPD_2IN13_ReadSharedByte();
  }

  // 原厂温度/时序参数：反汇编中的 0x3D/0x3E/0x3F
  EPD_2IN13_SendCommand(0x3D); EPD_2IN13_SendData(0x09); EPD_2IN13_SendData(0x09);
  EPD_2IN13_SendCommand(0x3E);
  EPD_2IN13_SendData(0x01); EPD_2IN13_SendData(0x11); EPD_2IN13_SendData(0x0C);
  EPD_2IN13_SendCommand(0x3F); EPD_2IN13_SendData(0x07);

  if (tempMode == 1) {
    // 固定值分支，仅复现历史构建；网页入口已隐藏。
    EPD_2IN13_SendCommand(0x1B);
    EPD_2IN13_SendCommand(0x1A); EPD_2IN13_SendData(0x55); EPD_2IN13_SendData(25);
    EPD_2IN13_SendCommand(0x22); EPD_2IN13_SendData(0x91);
  } else if (tempMode == 2) {
    if (temperature >= 10 && temperature <= 127) {
      EPD_2IN13_SendCommand(0x1A); EPD_2IN13_SendData(0x55); EPD_2IN13_SendData(temperature);
      EPD_2IN13_SendCommand(0x22); EPD_2IN13_SendData(0x91);
    } else {
      EPD_2IN13_SendCommand(0x22); EPD_2IN13_SendData(0xB1);
    }
  } else {
    EPD_2IN13_SendCommand(0x22); EPD_2IN13_SendData(0xB1);
  }
  EPD_2IN13_SendCommand(0x20); EPD_2IN13_ReadBusy();
  EPD_2IN13_SendCommand(0x21); EPD_2IN13_SendData(0x03);
  DEV_Delay_ms(100);
}

void EPD_Init_With_Mode(uint8_t mode) {
  epd_display_mode = mode;
  switch(mode) {
    case EPD_MODE_BW:
      // 2026-10-10 实测（1.17-diag 检测图）：自定义 lut_bw_update 在 DEPG0213RH
      // 三色屏上驱动力严重不足（含 16x16 大块在内全图极淡，波形驱动仅 ~3s，
      // OTP 全程 ~10-15s）。BW 改用出厂 OTP 波形；红色 RAM 填 0
      // （本板红色数据反相：1=红），纯黑白内容时 OTP 红相位无可见变化。
      // 自定义 LUT 保留在源码中，作为后续 LUT 调参实验的起点。
      EPD_2IN13_Init_With_LUT(NULL);
      EPD_2IN13_PrepareRedRAM();
      for (uint16_t i = 0; i < EPD_Buffer_Size; i++) {
        EPD_2IN13_SendData(0x00);
      }
      break;
    case EPD_MODE_BWR:
      EPD_2IN13_Init_With_LUT(NULL);
      break;
    case EPD_MODE_GRAY:
      EPD_2IN13_Init_With_LUT(EPD_2IN13_lut_gray_update);
      break;
    case EPD_MODE_BW3:
      // 褪色实验 B：灰度 LUT + 同帧 RAM 三叠刷（EPD_Display 放行 3 次激活）。
      // 流程约束：网页须先 01FF 清白（灰度 LUT 无「黑→白」相）。
      EPD_2IN13_Init_With_LUT(EPD_2IN13_lut_gray_update);
      break;
    case EPD_MODE_FACTORY:
    case EPD_MODE_FACTORY_TEMP91:
    case EPD_MODE_FACTORY_TEMP_READ91:
    case EPD_MODE_FACTORY_TEMP_READ91_X2:
      EPD_2IN13_Init_FactorySequence(mode == EPD_MODE_FACTORY_TEMP91 ? 1 :
                                     ((mode == EPD_MODE_FACTORY_TEMP_READ91 ||
                                       mode == EPD_MODE_FACTORY_TEMP_READ91_X2) ? 2 : 0));
      // 原厂实验当前网页只写 BW RAM；清除红色 RAM，避免上一次红层/随机 RAM 污染画面。
      EPD_2IN13_PrepareRedRAM();
      for (uint16_t i = 0; i < EPD_Buffer_Size; i++) {
        EPD_2IN13_SendData(0x00);
      }
      break;
    default:
      HwUARTPrintf("unknown Update mode\n");
      return;
  }
}
