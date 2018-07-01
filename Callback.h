#ifndef CALLBACK_H
#define CALLBACK_H

#include <stdint.h>

extern uint8_t UART_Tx_IntBuff[];
extern uint8_t UART_Rx_IntBuff[];

extern uint32_t XXX,Y1,Y2;

void TxDone_Func (void);
void RxDone_Func (void);

#endif
