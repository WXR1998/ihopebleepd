#include <stdio.h>
#include <string.h>

#include <xdc/std.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Semaphore.h>
#include <ti/sysbios/knl/Queue.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Clock.h>
#include <ti/sysbios/knl/Event.h>

// 网页手动复位（RST 指令）：SysCtrlSystemReset 来自 driverlib，
// 写 AON_SYSCTL SYSRESET 位，整芯片软复位，等效拉 NRST
#include <ti/devices/DeviceFamily.h>
#include DeviceFamily_constructPath(driverlib/sys_ctrl.h)

#include "board.h"
//#include <ti/drivers/uart/UARTCC26XX.h>
#include "task_epd.h"


// if defined, not calling actual epd function, just test protocol
//#define EPD_DRY_RUN

#ifndef EPD_DRY_RUN
#include "epd2in13.h"
#endif
 
 // for debug
//#include "inc/sdi_task.h"

#include "hw_uart.h"

#include "util.h"

// SPI/EPD error flag from epd2in13.c
extern uint8_t epd_spi_error;

uint8_t VERSION_MAJOR = 0;
uint8_t VERSION_MINOR = 2;

#define ASSERT_INIT \
  if (!epd_initialized) { \
    epd_resp_frame[1] = 0xfc; \
    return; \
  }
 
#define ASSERT_MIN_LEN(val, min_len) \
	if (val < min_len)               \
	{                                \
    epd_resp_frame[1] = 0xfe; \
		return;                    \
	}
 
enum {
  EPD_CMD_INIT = 0x0,
  EPD_CMD_CLEAR = 0x1,
  EPD_CMD_WRITE_RAM = 0x2,
  EPD_CMD_UPDATE = 0x3,
  EPD_CMD_PREPARE_BLK = 0x4,
  EPD_CMD_PREPARE_RED = 0x5,
  EPD_CMD_SHUTDOWN = 0x6,
  EPD_CMD_RESET = 0x7,
  EPD_CMD_SLEEP_CFG = 0x8,
};
 
 
 
#define EPD_TASK_PRIORITY                     2
#define EPD_TASK_STACK_SIZE                   900
Task_Struct EPDTask;
Char EPDTaskStack[EPD_TASK_STACK_SIZE];

// cc2640 received epd command frame from andoid app
// first byte is length, second is command, third and follow are command data if any
static uint8_t epd_rx_frame[256];
uint8_t rx_fram_len = 0;

// Bounded command queue: prevents GATT write callback from
// overwriting epd_rx_frame while EPD task is processing it.
#define EPD_CMD_QUEUE_SIZE 4
typedef struct {
    uint8_t len;
    uint8_t data[256];
} epd_cmd_slot_t;
static epd_cmd_slot_t epd_cmd_queue[EPD_CMD_QUEUE_SIZE];
static volatile uint8_t epd_cmd_head = 0;
static volatile uint8_t epd_cmd_tail = 0;

static uint8_t epd_resp_frame[6];
uint8_t resp_fram_len = 1;

EpdResponseCallback respCallback = NULL;

// Event used to control the EPD thread
Event_Struct EPDEvent;
Event_Handle hEPDEvent;

#define EPDTASK_EVENT_RX_REQUEST      Event_Id_00 


#define EPDTASK_EVENT_ALL ( EPDTASK_EVENT_RX_REQUEST  )
                            
                                         

static void handle_cmd();       
void TaskEPD_taskFxn(UArg a0, UArg a1);
static void post_epd_response(uint8_t *buf, uint16_t len);


void TaskEPD_createTask(void)
{
  Task_Params taskParams;

  // Configure task
  Task_Params_init(&taskParams);
  taskParams.stack = EPDTaskStack;
  taskParams.stackSize = EPD_TASK_STACK_SIZE;
  taskParams.priority = EPD_TASK_PRIORITY;

  Task_construct(&EPDTask, TaskEPD_taskFxn, &taskParams, NULL);
}

/*********************************************************************
 * @fn      TaskEPD_taskInit
 *
 * @brief   ���ڳ�ʼ��
 *
 * @param   None
 *
 * @return  None.
 */
void TaskEPD_taskInit(void)
{
    HWUART_Printf("TaskEPD_taskInit\r\n");
    
#ifndef EPD_DRY_RUN
     epd_hw_init();
#endif
    
    
}

/*********************************************************************
 * @fn      TaskEPD_taskFxn
 *
 * @brief   ����������
 *
 * @param   None
 *
 * @return  None.
 */
void TaskEPD_taskFxn(UArg a0, UArg a1)
{ 
  Event_Params evParams;
  Event_Params_init(&evParams);
  Event_construct(&EPDEvent, &evParams);
  hEPDEvent = Event_handle(&EPDEvent);
  
  TaskEPD_taskInit();

  while(1)
  {
    UInt events;
    events = Event_pend(hEPDEvent,Event_Id_NONE, EPDTASK_EVENT_ALL, BIOS_WAIT_FOREVER);
    
    if(events & EPDTASK_EVENT_RX_REQUEST)
    {
      // Drain command queue: process all pending commands
      while (epd_cmd_head != epd_cmd_tail)
      {
        epd_cmd_slot_t *slot = &epd_cmd_queue[epd_cmd_head];

        // Copy queued command to working buffer
        rx_fram_len = slot->len;
        memcpy(epd_rx_frame, slot->data, slot->len);

        // Advance head (consumes the slot)
        epd_cmd_head = (epd_cmd_head + 1) % EPD_CMD_QUEUE_SIZE;

        // Process this command
        handle_cmd();

        // Send response for this command
        if (resp_fram_len)
        {
          post_epd_response(epd_resp_frame, resp_fram_len);
        }
      }
    }
  }
}


void EPDTask_RegisterResponseCallback(EpdResponseCallback callback)
{
    respCallback = callback;
}

//void epd_on_rx_cmd(const uint8_t *buf, int len)
//{
//    memcpy(epd_rx_frame, buf, len);
//    Event_post(hEPDEvent, EPDTASK_EVENT_RX_REQUEST);
//}



void post_epd_response(uint8_t *buf, uint16_t len)
{
    
    if (respCallback != NULL) {
        // 0x10 is the event id , should be the same in peripheral_uarttrans.c
        respCallback(0x10, buf, len);
    }

}

bool epd_initialized = false;
void handle_cmd()
{
  HWUART_Printf("[EPD] cmd enter\r\n");
  uint8_t cmd = epd_rx_frame[0];
  
  resp_fram_len  = 2;
  epd_resp_frame[0] = cmd;
  epd_resp_frame[1] = 0;
  epd_spi_error = 0;

  switch(cmd) {
    case EPD_CMD_INIT:
      ASSERT_MIN_LEN(rx_fram_len, 2);
      if (epd_rx_frame[1] <= EPD_MODE_BEGIN || epd_rx_frame[1] >= EPD_MODE_END) {
        epd_resp_frame[1] = 0xff;
        return;
      }
      EPD_Init_With_Mode(epd_rx_frame[1]);
      epd_initialized = true;
      if (epd_rx_frame[1] == EPD_MODE_FACTORY_TEMP_READ91 ||
          epd_rx_frame[1] == EPD_MODE_FACTORY_TEMP_READ91_X2) {
        resp_fram_len = 4;
        epd_resp_frame[2] = epd_last_temperature;
        epd_resp_frame[3] = epd_last_status;
      }
      break;
    case EPD_CMD_CLEAR:
      ASSERT_MIN_LEN(rx_fram_len, 2);
      EPD_Clear(epd_rx_frame[1]);
      break;
    case EPD_CMD_PREPARE_BLK:
      ASSERT_INIT;
      EPD_2IN13_PrepareBlkRAM();
      break;
    case EPD_CMD_PREPARE_RED:
      ASSERT_INIT;
      EPD_2IN13_PrepareRedRAM();
      break;
    case EPD_CMD_WRITE_RAM:
      ASSERT_INIT;
      EPD_2IN13_WriteRAM(epd_rx_frame + 1, rx_fram_len - 1);
      break;
    case EPD_CMD_UPDATE:
      ASSERT_INIT;
      EPD_Display();
      epd_initialized = false;
      break;
    case EPD_CMD_RESET:
      // 不回 ACK——复位后链路即刻消失，网页以断开事件为确认
      HWUART_Printf("RST: system reset (NRST-equivalent)\r\n");
      SysCtrlSystemReset();
      break;
    case EPD_CMD_SLEEP_CFG:
      // 褪色实验：0x08 + mode（0=默认 1=C3+等BUSY 2=跳过C3 3=不休眠）
      ASSERT_MIN_LEN(rx_fram_len, 2);
      if (epd_rx_frame[1] > EPD_SLEEP_NONE) {
        epd_resp_frame[1] = 0xff;
        break;
      }
      epd_sleep_mode = epd_rx_frame[1];
      HWUART_Printf("[EPD] sleep mode=%d\r\n", epd_sleep_mode);
      break;
    default:
      HWUART_Printf("unknown cmd\r\n");
      epd_resp_frame[1] = 0xfd;
      break;
  }

  // Propagate SPI/EPD errors to the web response
  if (epd_spi_error) {
    epd_resp_frame[1] = epd_spi_error;
  }
}

void EPDTask_parseCommand(uint8_t *pMsg, uint8_t length)
{
  // Enqueue command into bounded queue (SPSC ring buffer).
  // If queue is full, command is dropped (web should retry after timeout).
  uint8_t next_tail = (epd_cmd_tail + 1) % EPD_CMD_QUEUE_SIZE;

  if (next_tail == epd_cmd_head) {
    HWUART_Printf("[EPD] queue full, drop cmd\r\n");
    return;
  }

  epd_cmd_slot_t *slot = &epd_cmd_queue[epd_cmd_tail];
  memcpy(slot->data, pMsg, length);
  slot->len = length;

  epd_cmd_tail = next_tail;
  Event_post(hEPDEvent, EPDTASK_EVENT_RX_REQUEST);
}