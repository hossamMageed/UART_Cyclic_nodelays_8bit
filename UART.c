#include "GPIO.h"
#include "UART.h"
#include "UART_Cfg.h"
#include "Common_Macros.h"
#include "M4MemMap.h"
#include <stdint.h>

typedef volatile uint32_t* const UART_RegAddType;


#define LOOPBACKMODE 0


/* Registers memory map */


#define UART0_BASE_ADDRESS 0x4000C000
#define UART1_BASE_ADDRESS 0x4000D000
#define UART2_BASE_ADDRESS 0x4000E000
#define UART3_BASE_ADDRESS 0x4000F000
#define UART4_BASE_ADDRESS 0x40010000
#define UART5_BASE_ADDRESS 0x40011000
#define UART6_BASE_ADDRESS 0x40012000
#define UART7_BASE_ADDRESS 0x40013000



static const uint32_t PortsBaseAddressLut[UART_CHANNELS_NUMBER]
={
  UART0_BASE_ADDRESS,
  UART1_BASE_ADDRESS,
  UART2_BASE_ADDRESS,
  UART3_BASE_ADDRESS,
  UART4_BASE_ADDRESS,
  UART5_BASE_ADDRESS,
  UART6_BASE_ADDRESS,
  UART7_BASE_ADDRESS
};


#define UART_REG_ADDRESS(ID,REG_OFFSET) (PortsBaseAddressLut[ID] + REG_OFFSET)


/* Data Control */
#define UARTDR_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x000))


/* Clock Control */

#define UARTCC_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0xFC8))
#define UARTCTL_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x030))


/* DMA Control */

#define UARTDMACTL_REG(PORT_ID)         *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x048))


/* Interrupt Control */

#define UARTIFLS_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x034))
#define UARTIM_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x038))
#define UARTRIS_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x03C))
#define UARTMIS_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x040))
#define UARTICR_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x044))


/* Status Control */

#define UARTRSR_REG(PORT_ID)            *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x004))
#define UARTFR_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x018))
#define UARTLCRH_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x02C))
#define UARTILPR_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x020))
#define UART9BITADDR_REG(PORT_ID)       *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x0A4))
#define UART9BITAMASK_REG(PORT_ID)      *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x0A8))
#define UARTPP_REG(PORT_ID)             *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0xFC0))


/* Baud Rate Control */

#define UARTIBRD_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x024))
#define UARTFBRD_REG(PORT_ID)           *((UART_RegAddType) UART_REG_ADDRESS(PORT_ID,0x028))


/* Defining some constants */


#define PEN_BIT_NO      1U
#define STP2_BIT_NO     3U
#define FEN_BIT_NO      4U
#define WLEN_BIT_NO     5U

#define UARTEN_BIT_NO   0U
#define LBE_BIT_NO      7U
#define TXE_BIT_NO      8U
#define RXE_BIT_NO      9U

#define BUSY_BIT_NO     3U
#define RXFE_BIT_NO     4U
#define TXFF_BIT_NO     5U
#define RXFF_BIT_NO     6U
#define TXFE_BIT_NO     7U

//Interrupt bits definition
#define RXIM_BIT_NO     4U
#define RXIFLSEL_BIT_NO 3U
#define RXIC_BIT_NO     4U

#define TXIM_BIT_NO     5U
#define TXIFLSEL_BIT_NO 0U
#define TXIC_BIT_NO     5U



#define INT_BUFF_LEN 19
uint8_t INT_msg_rx_buff[INT_BUFF_LEN];


// Defining some variables

static uint8_t UART_Driver_State[UART_GROUPS_NUMBER];

#define UART_INIT_DONE 0U
#define UART_TX_INIT_DONE 1U
#define UART_RX_INIT_DONE 3U


static uint8_t UART_TxLength[UART_GROUPS_NUMBER];
static uint8_t UART_TxCount[UART_GROUPS_NUMBER];
static const uint8_t* UART_TxBuffPtr[UART_GROUPS_NUMBER];

static uint8_t UART_RxLength[UART_GROUPS_NUMBER];
static uint8_t UART_RxCount[UART_GROUPS_NUMBER];
static uint8_t* UART_RxBuffPtr[UART_GROUPS_NUMBER];



/* Initialization Function */
UART_ChkType UART_Init (void)
{

    UART_ChkType RetVar;
    uint8_t LoopIndex;
    uint8_t TempVar1;
    uint16_t TempVar2;
    const UART_ConfigType* CfgPtr;
    const GPIO_AlternFunc* GPIO_AlternFunc_CfgPtr;
    double BaudRateDiv = 0;

    for (LoopIndex = 0; LoopIndex < UART_GROUPS_NUMBER; LoopIndex++)
    {
        CfgPtr = &UART_ConfigParam[LoopIndex];
        if(((CfgPtr->StopBits)   <=  TwoStopBit)&&
           ((CfgPtr->WordLen)    <=  Data_8)&&
           ((CfgPtr->TxFIFOSize) <=   16)&&
           ((CfgPtr->RxFIFOSize) <=   16)&&
           ((CfgPtr->UARTPortID)  <=    7)&&
           ((CfgPtr->INT_Enable) <= Enabled)&&
           ((CfgPtr->FIFO_Level) <= FIFO_14)
           )
          {
            /* Configure GPIO port's alternate function as UART*/

            /* Enable UART port clock */
            RCGCUART_REG |= 1 << (CfgPtr->UARTPortID);

            /* Disable the UART port*/
            UARTCTL_REG(CfgPtr->UARTPortID) = 0x00;

            /* BaudRate value */
            BaudRateDiv = SYSCLK /(uint64_t)((CfgPtr->BR_Speed)* (CfgPtr->BaudRate));

            // The integer part
            UARTIBRD_REG(CfgPtr->UARTPortID) = (uint32_t) BaudRateDiv;

            // The floating part
            BaudRateDiv -= (uint32_t) BaudRateDiv;
            UARTFBRD_REG(CfgPtr->UARTPortID) = (uint32_t)((BaudRateDiv * 64) + 0.5);

            /* Selecting the system clock */
            UARTCC_REG(CfgPtr->UARTPortID) = 0x0;

            /* Configuring the UART Line Control register */
            // Initializing the temporary variable
            TempVar1 = 0x00;
            // Word Length = 8 bits (11 in binary)
            TempVar1 |= (CfgPtr->WordLen) << WLEN_BIT_NO;
            // One stop bit
            TempVar1 |= (CfgPtr->StopBits) << STP2_BIT_NO;
            // Parity
            TempVar1 |= (CfgPtr->Parity) << PEN_BIT_NO;
            // FIFO
            if(((CfgPtr->TxFIFOSize) > 0) ||
               ((CfgPtr->RxFIFOSize) > 0))
            {
            TempVar1 |= (CfgPtr->FIFOEN) << FEN_BIT_NO;
            }
            // Configuring the register
            UARTLCRH_REG(CfgPtr->UARTPortID) = TempVar1;

            //Enabling UART interrupts
            if((CfgPtr->INT_Enable))
            {
                UARTIM_REG(CfgPtr->UARTPortID)  |= 1 <<RXIM_BIT_NO;     // setting the interrupt mask bit
                UARTIM_REG(CfgPtr->UARTPortID)  |= 1 <<TXIM_BIT_NO;
            }
            if((CfgPtr->FIFOEN)==1)
            {
                if((CfgPtr->FIFO_Level)!=FIFO_14)
                {
            TempVar1=( (CfgPtr->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
            UARTIFLS_REG(CfgPtr->UARTPortID)=TempVar1;
            UARTIFLS_REG(CfgPtr->UARTPortID)|=( (CfgPtr->FIFO_Level) >>2 );

                }
                else
                {
                    TempVar1=4 << RXIFLSEL_BIT_NO;
                    UARTIFLS_REG(CfgPtr->UARTPortID)=TempVar1;
                    UARTIFLS_REG(CfgPtr->UARTPortID)|=( (CfgPtr->FIFO_Level) >>2 );
                }
            }
            /* Enabling the UART port */
            TempVar2 = 0x00;
            TempVar2 |= 1 << UARTEN_BIT_NO;
            TempVar2 |= 1 << TXE_BIT_NO;

#if LOOPBACKMODE
            TempVar2 |= 1 << LBE_BIT_NO; // Loopback mode
#endif
            UARTCTL_REG(CfgPtr->UARTPortID) = TempVar2;

            if (CfgPtr->INT_Enable )
            RetVar = UART_OK;


            GPIO_AlternFunc_CfgPtr = &UART_AlternFunc[CfgPtr->UARTPortID];
            GPIO_SetAlternFunction(CfgPtr->Rx_Pin_GPIO_groupID,GPIO_AlternFunc_CfgPtr->RxChVal);
            GPIO_SetAlternFunction(CfgPtr->Tx_Pin_GPIO_groupID,GPIO_AlternFunc_CfgPtr->TxChVal);

            UART_Driver_State[LoopIndex] |= ( 1U <<UART_INIT_DONE ) ;
        }
        else
        {
            RetVar = UART_NOK;
        }

    }
    return RetVar;

}




UART_ChkType UART_Tx_Init(const uint8_t* TxBuffPtr, uint8_t TxLen, uint8_t groupID)
{
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;
    static uint8_t Transmitted_Bytes;
    static uint8_t FIFO_State_Tx;



    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if ( ( ( UART_Driver_State[groupID] & (1U <<UART_INIT_DONE ) ) != 0 ) && ( ( UART_Driver_State[groupID] & (1U <<UART_TX_INIT_DONE ) ) == 0 ) ) // UART was initialized successfully and there is no pending transmission
        {
            UART_TxLength[groupID] = TxLen;
            UART_TxBuffPtr[groupID] = TxBuffPtr;
            UART_TxCount[groupID] = 0;
            FIFO_State_Tx =(uint8_t)(CfgPtr->FIFOEN);

            UARTCTL_REG(CfgPtr->UARTPortID) &= ~(1U << TXE_BIT_NO);  // Disable the transmitter pin to prevent sending while filling the FIFO
           // UARTIM_REG(CfgPtr->UARTPortID)  &= ~(1 <<TXIM_BIT_NO);  //Disable the interrupts to prevent the ISR from double transmitting

            if((TxLen <= (uint8_t)(CfgPtr->FIFO_Level)) && ( (CfgPtr->INT_Enable)!= 0 )  )
                // bytes count not enough to trigger an interrupt
            {
                UARTLCRH_REG(CfgPtr->UARTPortID) &= ~(1U<<FEN_BIT_NO); //Disable the FIFO so the ISR will be Triggered after transmitting the bytes
                FIFO_State_Tx=0;
            }
            if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << TXFF_BIT_NO)) == 1) // FIFO is full no place to write, the function returns
                {
                    RetVar=UART_OK;
                    return RetVar;
                }

            else if ( ( (UARTFR_REG(CfgPtr->UARTPortID)) & (1 << TXFE_BIT_NO) ) !=0 )// FIFO is empty, we can write 16 bytes
            {
               if( (FIFO_State_Tx) != 0)   // FIFO is enabled so we write to it
               {
                if(UART_TxLength[groupID] >= 16) // the data is larger or equal to 16 bytes
                {
                    for(Transmitted_Bytes=0;Transmitted_Bytes<16;Transmitted_Bytes++)
                        {
                            UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                            UART_TxCount[groupID]++;
                        }
                }
                else // the data is less than 16 bytes
                {
                    for(Transmitted_Bytes=0;Transmitted_Bytes<UART_TxLength[groupID];Transmitted_Bytes++)
                        {
                            UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                            UART_TxCount[groupID]++;
                        }
                    }
               }
               else // FIFO is not enabled so we write one byte
               {
                   UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                   UART_TxCount[groupID]++;
               }


            }
            else // FIFO is neither totally empty nor totally full
                {
                if((CfgPtr->INT_Enable) == 1)// if interrupts are enabled
                {
                        if ( ( (UARTRIS_REG(CfgPtr->UARTPortID)) & (1U <<5) ) != 0 ) // the number of free bytes in FIFO is equal to the interrupt fifo size
                            {
                                if((UART_TxLength[groupID]) >= (CfgPtr->FIFO_Level)) // the data is larger or equal to the FIFO interrupt level
                                    {
                                        for(Transmitted_Bytes=0;Transmitted_Bytes<(CfgPtr->FIFO_Level);Transmitted_Bytes++) // Fill the FIFO with a number of bytes enough to trigger an ISR
                                            {
                                                UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                                                UART_TxCount[groupID]++;
                                            }
                                    }
                                else // the Transmitted data is less than the FIFO interrupt level
                                    {
                                        for(Transmitted_Bytes=0;Transmitted_Bytes<UART_TxLength[groupID];Transmitted_Bytes++)// fit the whole number of bytes in the FIFO
                                            {
                                                UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                                                UART_TxCount[groupID]++;
                                            }
                                    }
                            }
                        else // number is not sufficient to trigger the raw interrupt flag so we send byte by byte
                        {
                            UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                            UART_TxCount[groupID]++;
                        }
                }
                else // we don't know exactly how many places are empty so we send byte by byte
                {
                    UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                    UART_TxCount[groupID]++;
                }
            }
        }
        else // UART is not initialized yet
            {
                RetVar = UART_NOK;
                return RetVar;
            }
    }

    else //Wrong group ID
    {
        RetVar = UART_NOK;
        return RetVar;
    }
    UART_Driver_State[groupID] |= (1U << UART_TX_INIT_DONE);
    UARTCTL_REG(CfgPtr->UARTPortID) |= (1U << TXE_BIT_NO);
    RetVar = UART_OK;
    /*if((CfgPtr->INT_Enable) ==Enabled)
    {
        UARTIM_REG(CfgPtr->UARTPortID)  |= 1 <<TXIM_BIT_NO;
    }
*/

    return RetVar;
}



UART_ChkType UART_Tx (uint8_t groupID)
{

        UART_ChkType RetVar;
        const UART_ConfigType* CfgPtr;
        static uint8_t Transmitted_Bytes,Remaining_Bytes;

        if(groupID < UART_GROUPS_NUMBER)
        {

            if ( ( ( UART_Driver_State[groupID] & (1U <<UART_TX_INIT_DONE ) ) != 0 ) ) //UART transmission in progress
            {
                CfgPtr = &UART_ConfigParam[groupID];
              if(UART_TxCount[groupID] < UART_TxLength[groupID] )
              {
                if((UARTFR_REG(CfgPtr->UARTPortID) & (1 << TXFF_BIT_NO)) == 1) // FIFO is full no place to write, the function returns
                    {
                        RetVar=UART_OK;
                        return RetVar;
                    }
                else if ( ( (UARTFR_REG(CfgPtr->UARTPortID)) & (1 << TXFE_BIT_NO) ) !=0 )// FIFO is empty we can write 16 bytes
                {
                    if( ((uint8_t)(CfgPtr->FIFOEN)) !=0 )
                    {
                    if((UART_TxLength[groupID]-UART_TxCount[groupID]) >= 16) // the data is larger or equal to 16 bytes
                    {
                        for(Transmitted_Bytes=0;Transmitted_Bytes<16;Transmitted_Bytes++)
                            {
                                UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                                UART_TxCount[groupID]++;
                            }
                        RetVar=UART_OK;
                    }
                    else // the data is less than 16 bytes
                    {Remaining_Bytes= UART_TxLength[groupID]-UART_TxCount[groupID];
                        for(Transmitted_Bytes=0;Transmitted_Bytes<Remaining_Bytes;Transmitted_Bytes++)
                        {
                            UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                            UART_TxCount[groupID]++;
                        }
                        RetVar = UART_OK;
                    }

                }
                    else // FIFO is not enabled so we write one byte
                    {
                        UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                        UART_TxCount[groupID]++;
                        RetVar = UART_OK;
                    }
                }/*
                else // FIFO is neither totally empty nor totally full
                {
                    if(((uint8_t)(CfgPtr->INT_Enable))==1)
                        {
                        if ( ( (UARTRIS_REG(CfgPtr->UARTPortID)) & (1U <<5)) !=0 ) // the number of free bytes in FIFO is equal to the interrupt fifo size
                            {
                                if((UART_TxLength[groupID]-UART_TxCount[groupID]) >= (CfgPtr->FIFO_Level)) // the data is larger or equal to the FIFO interrupt level
                                    {
                                        for(Transmitted_Bytes=0;Transmitted_Bytes<(CfgPtr->FIFO_Level);Transmitted_Bytes++)
                                        {
                                            UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                                            UART_TxCount[groupID]++;
                                        }
                                        RetVar=UART_OK;
                                    }
                                else // the remaining data is less than the FIFO interrupt level
                                {
                                    Remaining_Bytes= UART_TxLength[groupID]-UART_TxCount[groupID];
                                    for(Transmitted_Bytes=0;Transmitted_Bytes<Remaining_Bytes;Transmitted_Bytes++)
                                    {
                                        UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                                        UART_TxCount[groupID]++;
                                    }
                                    RetVar=UART_OK;
                                }
                            }
                        else // free slots in FIFO is not sufficient to trigger the raw interrupt flag so we Send byte by byte
                        {
                            UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                            UART_TxCount[groupID]++;
                            RetVar=UART_OK;
                        }
                        }
                    else // we don't know exactly how many places are empty so we send byte by byte
                        {
                            UARTDR_REG(CfgPtr->UARTPortID) = *(UART_TxBuffPtr[groupID] + UART_TxCount[groupID]);
                            UART_TxCount[groupID]++;
                            RetVar=UART_OK;
                        }
                }*/
              }
              if( ( UART_TxCount[groupID] == UART_TxLength[groupID] ) && ( (UARTFR_REG(CfgPtr->UARTPortID)) & (1 << TXFE_BIT_NO) ) ) // Transmitted data is complete and the FIFO is empty
              {

                  RetVar=UART_OK;
                 // UARTCTL_REG(CfgPtr->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                  UART_Driver_State[groupID] &= ~(1U<< UART_TX_INIT_DONE);

                  //Callback function is called here
                  CfgPtr->TxDoneCallbackPtr();

              }

        }
            else // UART transmitter is not initialized
            {
                RetVar= UART_NOK;
            }
        }

        else //Wrong group ID
        {
            RetVar = UART_NOK;
        }

        return RetVar;

}





UART_ChkType UART_Rx_Init (uint8_t* RxBuffPtr, uint8_t RxLen, uint8_t groupID)
{
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;

    if (groupID < UART_GROUPS_NUMBER)
        {
            CfgPtr = &UART_ConfigParam[groupID];


            if ( ( ( UART_Driver_State[groupID] & (1U <<UART_INIT_DONE ) ) != 0 ) && ( ( UART_Driver_State[groupID] & (1U <<UART_RX_INIT_DONE ) ) == 0 ) ) // UART was initialized successfully and there is no pending receiving
            {
                if( ((CfgPtr->FIFOEN) != 0) && ( (CfgPtr->INT_Enable)!= 0 ) )
                   {
                     // UARTLCRH_REG(CfgPtr->UARTPortID) |= ( 1U<<FEN_BIT_NO );
                   }
                UART_RxLength[groupID] = RxLen;
                UART_RxCount[groupID] = 0;
                UART_RxBuffPtr[groupID] = RxBuffPtr;
                if((RxLen < (uint8_t)(CfgPtr->FIFO_Level)) && ( (CfgPtr->INT_Enable)!= 0 )  )
                    // bytes count not enough to trigger an interrupt
                    {
                    UARTLCRH_REG(CfgPtr->UARTPortID) &= ~(1U<<FEN_BIT_NO); //disable the FIFO
                    }
                UARTCTL_REG(CfgPtr->UARTPortID) |= (1U << RXE_BIT_NO);  // enable the receiver pin
                UART_Driver_State[groupID] |=(1U << UART_RX_INIT_DONE);
                RetVar=UART_OK;

            }
            else
                RetVar=UART_NOK;

        }
    else
    {
        RetVar=UART_NOK;
    }

    return RetVar;
}


UART_ChkType UART_Rx (uint8_t groupID)
{
    static uint8_t Received_Bytes,Remaining_Bytes;
    UART_ChkType RetVar;
    const UART_ConfigType* CfgPtr;


    if(groupID < UART_GROUPS_NUMBER)
    {
        CfgPtr = &UART_ConfigParam[groupID];

        if ( ( ( UART_Driver_State[groupID] & (1U <<UART_RX_INIT_DONE ) ) != 0 ) ) //UART Receiving in progress
        {
            if(UART_RxCount[groupID] < UART_RxLength[groupID])
            {
                if( (UARTFR_REG(CfgPtr->UARTPortID) & (1 << RXFE_BIT_NO) )!= 0 )// nothing to receive yet so we return from the function
                {
                    RetVar=UART_OK;
                    return RetVar;
                }
                else if (( (UARTFR_REG(CfgPtr->UARTPortID)) & (0X40) ) != 0) // FIFO is full
                {
                    if (( (uint8_t)(CfgPtr->FIFOEN) ) != 0) // FIFO is enabled so the UART receives 16 bytes at once
                    {  Remaining_Bytes=  UART_RxLength[groupID] - UART_RxCount[groupID];
                        if(Remaining_Bytes >= 16)
                            {
                            for(Received_Bytes=0;Received_Bytes<16;Received_Bytes++)
                                {
                                    *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                                    UART_RxCount[groupID]++;
                                }
                            }
                        else
                            for(Received_Bytes=0;Received_Bytes<Remaining_Bytes;Received_Bytes++)
                                {
                                    *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                                    UART_RxCount[groupID]++;
                                }
                        RetVar=UART_OK;
                    }
                    else // FIFO is disabled so we receive one byte at a time
                    {
                        *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                        UART_RxCount[groupID]++;
                        RetVar=UART_OK;
                    }

                }
                else // this condition implies that the FIFO is enabled, but the number of bytes in it is less than 16 bytes
                { if ( ((UARTRIS_REG(CfgPtr->UARTPortID))&(1U <<4) )!=0 ) // the number of bytes in FIFO is equal to the interrupt fifo size
                    {
                        Remaining_Bytes=  UART_RxLength[groupID] - UART_RxCount[groupID];
                    if( Remaining_Bytes >= ((uint8_t) (CfgPtr->FIFO_Level)) )
                        {
                            for(Received_Bytes=0;Received_Bytes<((uint8_t) (CfgPtr->FIFO_Level));Received_Bytes++)
                            {
                                *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                                UART_RxCount[groupID]++;
                            }
                        }
                    else
                        for(Received_Bytes=0; Received_Bytes < Remaining_Bytes ;Received_Bytes++)
                        {
                            *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                            UART_RxCount[groupID]++;
                        }

                    RetVar=UART_OK;
                    }
                else // number is not sufficient to trigger the raw interrupt flag so we receive byte by byte
                   {
                        *(UART_RxBuffPtr[groupID] + UART_RxCount[groupID]) = UARTDR_REG(CfgPtr->UARTPortID);
                        UART_RxCount[groupID]++;
                        RetVar=UART_OK;
                   }
                }

            }

            if(UART_RxCount[groupID] == UART_RxLength[groupID]) // received the correct amount of bytes
            {
                UART_Driver_State[groupID] &= ~(1U<< UART_RX_INIT_DONE);
                // Callback function
                UARTCTL_REG(CfgPtr->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                UART_ConfigParam[groupID].RxDoneCallbackPtr();
                //CfgPtr->RxDoneCallbackPtr();
            }

        }
        else // UART receiver wasn't initialized
        {
            RetVar = UART_NOK;
        }

    }
    else // wrong group ID
    {
        RetVar = UART_NOK;
    }

    return RetVar;


}


// This function is used to determine the groupID of each UART channel
static uint8_t groupID_Search (uint8_t ChannelID)
{
    uint8_t UART_groupID = 0;
    const UART_ConfigType* CfgPtr;
    uint8_t LoopIndex = 0;
    if(ChannelID < UART_CHANNELS_NUMBER)
    {
        for(LoopIndex = 0; LoopIndex < UART_GROUPS_NUMBER; LoopIndex++)
        {
            CfgPtr =&UART_ConfigParam[LoopIndex];
            //Checking if the configuration has the required channelID
            if((CfgPtr->UARTPortID) == ChannelID)
            {
                UART_groupID = LoopIndex;
            }
        }
    }
    return UART_groupID;
}


void UART0_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(0);
     uint8_t UART_TX_groupID = groupID_Search(0);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}


void UART1_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(1);
     uint8_t UART_TX_groupID = groupID_Search(1);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}




void UART2_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(2);
     uint8_t UART_TX_groupID = groupID_Search(2);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}





void UART3_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(3);
     uint8_t UART_TX_groupID = groupID_Search(3);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}


void UART4_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(4);
     uint8_t UART_TX_groupID = groupID_Search(4);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}


void UART5_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(5);
     uint8_t UART_TX_groupID = groupID_Search(5);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}




void UART6_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(6);
     uint8_t UART_TX_groupID = groupID_Search(6);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}



void UART7_ISR(void)
{
     uint8_t UART_RX_groupID = groupID_Search(7);
     uint8_t UART_TX_groupID = groupID_Search(7);
     uint8_t Processed_Bytes;
     uint8_t Remaining_Bytes;
     const  UART_ConfigType *CfgPtr_Rx, *CfgPtr_Tx;
     static uint8_t Int_Level,FIFO_State_Rx,FIFO_State_Tx;
     CfgPtr_Rx = &UART_ConfigParam[UART_RX_groupID];
     CfgPtr_Tx = &UART_ConfigParam[UART_TX_groupID];



     if(UART_RxCount[UART_RX_groupID]==0) // Store the starting values of the FIFO and interrupts only at the beginning
     {
         Int_Level = (uint8_t) (CfgPtr_Rx->FIFO_Level);
         FIFO_State_Rx =(uint8_t)(CfgPtr_Rx->FIFOEN);
     }

     if((UARTLCRH_REG(CfgPtr_Rx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO is disabled
      {
          FIFO_State_Rx = 0;
      }

    if((UARTMIS_REG(CfgPtr_Rx->UARTPortID))&(1U <<4))       // the interrupt is caused by a receiving operation
    {



            if(UART_RxCount[UART_RX_groupID] < UART_RxLength[UART_RX_groupID]) // Number of received bytes is less than the length that should be received
            {
                if( (FIFO_State_Rx) != 0 )
                {
                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) >= Int_Level)
// In case the remaining bytes are more than the interrupt level receive a number of bytes equal to the interrupt level
                    {
                        for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                        {
                            *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);
                            UART_RxCount[UART_RX_groupID]++;
                        }
                    }

                    if((UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]) < Int_Level)   // The number of remaining bytes is less than the FIFO so we resize the FIFO accordingly
                    {
                        Remaining_Bytes = (UART_RxLength[UART_RX_groupID]-UART_RxCount[UART_RX_groupID]);
                        if(Remaining_Bytes <2) // Only one byte is remaining. Disable the FIFO.
                        {
                            UARTLCRH_REG(CfgPtr_Rx->UARTPortID) &= ~(1U<<FEN_BIT_NO);
                            FIFO_State_Rx=0;
                        }
                        else if(Remaining_Bytes <4)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x0 << RXIFLSEL_BIT_NO);
                            Int_Level=2;
                        }
                        else if(Remaining_Bytes <8)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x1 << RXIFLSEL_BIT_NO);
                            Int_Level=4;
                        }
                        else if(Remaining_Bytes <12)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x2 << RXIFLSEL_BIT_NO);
                            Int_Level=8;
                        }
                        else if(Remaining_Bytes <14)
                        {
                            UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=(0x3 << RXIFLSEL_BIT_NO);
                            Int_Level=12;
                        }

                    }

                }
                else // receive without FIFO
                {
                    *(UART_RxBuffPtr[UART_RX_groupID] + UART_RxCount[UART_RX_groupID]) = UARTDR_REG(CfgPtr_Rx->UARTPortID);

                    UART_RxCount[UART_RX_groupID]++;
                }
            }

            if(UART_RxCount[UART_RX_groupID] == UART_RxLength[UART_RX_groupID]) // received the right amount of bytes
            {

                UARTLCRH_REG(CfgPtr_Rx->UARTPortID)|= (CfgPtr_Rx->FIFOEN) << FEN_BIT_NO; //return the FIFO enable to it's orignal status
                if((CfgPtr_Rx->FIFO_Level)!=FIFO_14) // return the fifo interrupt level to it's orignal status
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=( (CfgPtr_Rx->FIFO_Level) >>2 ) << RXIFLSEL_BIT_NO;
                }
                else
                {
                    UARTIFLS_REG(CfgPtr_Rx->UARTPortID)=4 << RXIFLSEL_BIT_NO;
                }
                UART_Driver_State[UART_RX_groupID] &= ~(1U<< UART_RX_INIT_DONE);// the driver is back to the init status
                UARTCTL_REG(CfgPtr_Rx->UARTPortID) &= ~((1U << RXE_BIT_NO));  // Disable the receiver pin
                CfgPtr_Rx->RxDoneCallbackPtr();
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                // CALL BACK FUNCTION
                //UART_ConfigParam[UART_RX_groupID].RxDoneCallbackPtr();


            }

            else // there is still remaining bytes to be received
            {
                UARTICR_REG(CfgPtr_Rx->UARTPortID)|= (1 << RXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
            }



    }



            /***********************************************************************/



    else if((UARTMIS_REG(CfgPtr_Tx->UARTPortID))&(1U <<5)) // the interrupt is caused by a transmitting operation
    {

        FIFO_State_Tx =(uint8_t)(CfgPtr_Tx->FIFOEN); // We save the FIFO state here

        if((UARTLCRH_REG(CfgPtr_Tx->UARTPortID)&(1U<<FEN_BIT_NO)) == 0) // FIFO was disabled in the init function
              {
                  FIFO_State_Tx = 0;
              }


                if(UART_TxCount[UART_TX_groupID] < UART_TxLength[UART_TX_groupID])
                // Didn't transmit all the bytes yet
                   {
                    Remaining_Bytes= UART_TxLength[UART_TX_groupID]- UART_TxCount[UART_TX_groupID];
                    if( (FIFO_State_Tx) != 0 ) //FIFO is enabled
                        {

                            if(Remaining_Bytes < Int_Level ) // Remaining bytes are less than the interrupt level
                                for(Processed_Bytes=0 ; Processed_Bytes < Remaining_Bytes;Processed_Bytes++)
                                    // Receive a number of bytes equal to the remaining bytes only
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            else // Remaining bytes are more than the interrupt level
                            {
                                for(Processed_Bytes=0;Processed_Bytes < Int_Level;Processed_Bytes++)
                                 //Receive a number of bytes equal to the interrupt level
                                {
                                    UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                                    UART_TxCount[UART_TX_groupID]++;

                                }
                            }
                        }
                        else // Transmit without FIFO
                        {
                            UARTDR_REG(CfgPtr_Tx->UARTPortID) = *(UART_TxBuffPtr[UART_TX_groupID] + UART_TxCount[UART_TX_groupID]);
                            UART_TxCount[UART_TX_groupID]++;
                        }
                    }

                else  if((UART_TxCount[UART_TX_groupID] == UART_TxLength[UART_TX_groupID])) // received the right amount of bytes
                    {

                        UART_Driver_State[UART_TX_groupID] &= ~(1U<< UART_TX_INIT_DONE);// the driver is back to the init status
                        //UARTCTL_REG(CfgPtr_Tx->UARTPortID) &= ~(1U << TXE_BIT_NO); // Disable the transmitter pin
                        CfgPtr_Tx->TxDoneCallbackPtr();
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                        //CallBack function

                    }

                    else // there is still remaining bytes to be Transmited
                    {
                        UARTICR_REG(CfgPtr_Tx->UARTPortID)|= (1 << TXIC_BIT_NO); // clearing the interrupt bit so the program can get out of the ISR
                    }



    }
}
