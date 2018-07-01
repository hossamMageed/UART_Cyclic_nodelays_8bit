#include "UART.h"
#include "Callback.h"


void TxDone_Func (void)
{
  Y1++;

  /*UART_Rx_Init(UART_Rx_IntBuff,1,0);
  while(1)
  {
      UART_Rx(0);
  }*/
//  UART_Rx_Init(UART_Rx_IntBuff,25,0);

}

void RxDone_Func (void)
{

  Y2++;
  XXX++;
  //UART_Tx_Init(UART_Tx_IntBuff,50,0);
  //UART_Tx_Init(UART_Tx_IntBuff,4,1);
}
