#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "UART_Cfg.h"

#define UART_CHANNELS_NUMBER 8U



typedef enum {UART_OK = 0, UART_NOK = 1} UART_ChkType;
typedef enum {OneStopBit = 0, TwoStopBit = 1} UART_StopBitsType;
typedef enum {Data_5 = 0, Data_6 = 1, Data_7 = 2, Data_8 = 3} UART_WordLength;
typedef enum {Disabled = 0, Enabled = 1} UART_EnableType;
typedef enum {Idle = 0, TxInProg = 1, TxDone = 2, RxInProg = 3, RxDone = 4} UART_StatusChkType;
typedef enum {FIFO_2=2, FIFO_4=4, FIFO_8=8, FIFO_12=12, FIFO_14=14}FIFO_INT_Level ;
typedef enum {High_Speed = 8, Regular_Speed = 16} BaudRate_Speed;

typedef void(*UART_PtrToCallbackType)(void);


/* A structure type contains all the required configurations */

typedef struct
{

    /* UART port number */
    uint8_t UARTPortID;

    /* Tx pin number */
    uint8_t Rx_Pin_GPIO_groupID;

    /* Rx pin number */
    uint8_t Tx_Pin_GPIO_groupID;

    /* Baud rate */
    uint32_t BaudRate;

    /* Regular or high speed */
    BaudRate_Speed BR_Speed;

    /* Number of stop bits */
    UART_StopBitsType StopBits;

    /* Word length */
    UART_WordLength WordLen;

    /* Parity */
    UART_EnableType Parity;

    /* Enabling FIFO */
    UART_EnableType FIFOEN;

    /* Tx FIFO size */
    uint8_t TxFIFOSize;

    /* Rx FIFO size */
    uint8_t RxFIFOSize;

    /* TxDone Callback function pointer */
    UART_PtrToCallbackType TxDoneCallbackPtr;

    /* RxDone Callback function pointer */
    UART_PtrToCallbackType RxDoneCallbackPtr;

    /* Enable and Disable interrupt*/
    UART_EnableType INT_Enable;

    /* Interrupt FIFo level*/
    FIFO_INT_Level FIFO_Level;


} UART_ConfigType;

extern const UART_ConfigType UART_ConfigParam[UART_GROUPS_NUMBER];

typedef struct
{

    uint8_t TxChVal;
    uint8_t RxChVal;

} GPIO_AlternFunc;

extern const GPIO_AlternFunc UART_AlternFunc[UART_CHANNELS_NUMBER];

/* Declaration of global functions */
UART_ChkType UART_Init (void);
UART_ChkType UART_Tx_Init(const uint8_t* TxBuffPtr, uint8_t TxLen, uint8_t groupID);
UART_ChkType UART_Tx (uint8_t groupID);
UART_ChkType UART_Rx_Init (uint8_t* RxBuffPtr, uint8_t RxLen, uint8_t groupId);
UART_ChkType UART_Rx (uint8_t groupId);

static uint8_t groupID_Search (uint8_t ChannelID);

void TxDone_Func (void);
void RxDone_Func (void);



#endif
