
#include "epd2in13.h"
#include "hw_uart.h"

// SPI/EPD error flag: set by SPI layer on failure, read by handle_cmd()
// to propagate errors to the web response. 0 = no error.
uint8_t epd_spi_error = 0;

// 睡眠收尾模式（见 epd2in13.h 枚举；默认 = 历史行为，褪色实验用）
uint8_t epd_sleep_mode = EPD_SLEEP_DEFAULT;

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

    // 面板断电时 BUSY 被上拉读成 HIGH，会白等满 20s——快速失败并标记
    if (DEV_Digital_Read(EPD_POWER_PIN) == 1) {
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
  
  EPD_2IN13_SendCommand(0x2C); //set vcom value
  EPD_2IN13_SendData(0x5A);

  if(lut) {
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
  EPD_2IN13_UpdateDisplay();
  EPD_2IN13_Sleep();
}

void EPD_Init_With_Mode(uint8_t mode) {
  switch(mode) {
    case EPD_MODE_BW:
      EPD_2IN13_Init_With_LUT(EPD_2IN13_lut_bw_update);
      break;
    case EPD_MODE_BWR:
      EPD_2IN13_Init_With_LUT(NULL);
      break;
    case EPD_MODE_GRAY:
      EPD_2IN13_Init_With_LUT(EPD_2IN13_lut_gray_update);
      break;
    default:
      HwUARTPrintf("unknown Update mode\n");
      return;
  }
}
