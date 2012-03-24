/*******************************************************************************
 * @file
 * @purpose        
 * @version        0.1
 *------------------------------------------------------------------------------
 * Copyright (C) 2011 Gumstix Inc.
 * All rights reserved.
 *
 * Contributer(s):
 *   Neil MacMunn   <neil@gumstix.com>
 *------------------------------------------------------------------------------
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 * Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 * 
 * Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 ******************************************************************************/

#include "return.h"
#include "table.h"

#include "LPC17xx.h"
#include "lpc_types.h"
#include "lpc17xx_gpio.h"
#include "lpc17xx_pinsel.h"
#include "lpc17xx_can.h"
#include "lpc17xx_adc.h"

/*****************************************************************************

    Generic Interrupt Service Routine

*****************************************************************************/
void IntHandler(void)
{
    unsigned int int_num;
    uint8_t int_str[8];

    /*
     * Get the interrupt number
     */
    __asm("mrs %0, ipsr;" : "=r"(int_num) );
    int_num -= 16;

    /*
     * Disable the interrupt
     */
    NVIC_DisableIRQ(int_num);

    /*   
     * Send the interrupt signal and number
     */
    sprintf((char*) int_str, "\r\n%x\r\n", int_num);
    writeUSBOutString(int_str);
}

/*****************************************************************************

        Hardware Initialization Routine

*****************************************************************************/

void hwInit(void)
{
    /*
     * make the LED pin an output and turn it on
     */
    GPIO_SetDir(3, (1 << 25), 1);
    GPIO_ClearValue(3, (1 << 25));

    /*
     * start the usb device wait until configuration completes before proceeding
     */
    USB_Init();
    USB_Connect(TRUE);
    while (!USB_Configuration);

}

int BatteryVoltage(void)
{
	uint8_t start_mode = 1; //Start now
	uint8_t adc_data_done = 1; //Data_done
	unsigned value = 0;
	
	ADC_Init(LPC_ADC, 200000);
	ADC_ChannelCmd(LPC_ADC, 2, ENABLE);
	ADC_StartCmd(LPC_ADC, start_mode);
	while( ! ADC_ChannelGetStatus(LPC_ADC, 2, adc_data_done))
		{;}
	value = ADC_ChannelGetData(LPC_ADC, 2);
	if (value < 2709)//battery empty
	{
		return 0;
	}
	else//battery full
	{
		return 1;
	}
}

void heartbeat(int time)
{
    unsigned long i;
    unsigned long time_off1,time_off2,time_on;
		
		if(time == 1)
		{
			time_off1= 400000;
			time_off2= 400000;
			time_on= 200000;
		}
		else
		{
			time_off1= 1200000;
			time_off2= 3200000;
			time_on= 800000;
		}
		
    for (i = 0; i < time_off1; i++);
    GPIO_ClearValue(3, (1 << 25));
    for (i = 0; i < time_on; i++);
    GPIO_SetValue(3, (1 << 25));
    for (i = 0; i < time_off2; i++);
    GPIO_ClearValue(3, (1 << 25));
    for (i = 0; i < time_on; i++);
    GPIO_SetValue(3, (1 << 25));
}

extern int heartbeat_on;

int main(void)
{
    hwInit();
		_roboveroConfig((uint8_t *) NULL);

    /*
     * let usbuser/robovero handle the rest
     */
    while(1)
    {
        if(heartbeat_on)
        {
        	if(BattertyVoltage()==1)
        		heartbeat(0);
        	else 
        		heartbeat(1);   
				}
		}

    return 0;
}

// CAN TEST
/*PINSEL_CFG_Type PinCfg;
CAN_MSG_Type TXMsg;
PinCfg.Funcnum = 3;
PinCfg.OpenDrain = 0;
PinCfg.Pinmode = 0;
PinCfg.Pinnum = 21;
PinCfg.Portnum = 0;
PINSEL_ConfigPin(&PinCfg);
PinCfg.Pinnum = 22;
PINSEL_ConfigPin(&PinCfg);
CAN_Init(LPC_CAN1, 100000);
CAN_SetAFMode(LPC_CANAF,CAN_AccBP);
TXMsg.format = EXT_ID_FORMAT;
TXMsg.id = 0x00001234;
TXMsg.len = 8;
TXMsg.type = DATA_FRAME;
TXMsg.dataA[0] = TXMsg.dataA[1] = TXMsg.dataA[2] = TXMsg.dataA[3] = 0x01234567;
TXMsg.dataB[0] = TXMsg.dataB[1] = TXMsg.dataB[2] = TXMsg.dataB[3] = 0x89ABCDEF;
while (1) {
    CAN_SendMsg(LPC_CAN1, &TXMsg);
    TXMsg.id ++;
}*/

// MASTER CLOCK TEST
/*//Initialize clockout pin
PINSEL_CFG_Type PinCfg;
PinCfg.Portnum = 1;
PinCfg.Pinnum = 27;
PinCfg.Pinmode = 0;
PinCfg.OpenDrain = 0;
PinCfg.Funcnum = 1;
PINSEL_ConfigPin(&PinCfg);*/

///* Initialize UART1 */
//PINSEL_CFG_Type PinCfg;
//PinCfg.Portnum = 2;
//PinCfg.Pinnum = 0;
//PinCfg.Pinmode = 0;
//PinCfg.OpenDrain = 0;
//PinCfg.Funcnum = 2;
//PINSEL_ConfigPin(&PinCfg);
//PinCfg.Pinnum = 1;
//PINSEL_ConfigPin(&PinCfg);

//    UART_CFG_Type * UARTConfigStruct_ptr;
//    UARTConfigStruct_ptr = _UART_CFG_Type_malloc();
//    if (UARTConfigStruct_ptr == NULL)
//        while (1);
//    UART_ConfigStructInit(UARTConfigStruct_ptr);
//    _UART_CFG_Type_set_Baud_rate(UARTConfigStruct_ptr, 115200);
//    UART_Init(LPC_UART1, UARTConfigStruct_ptr);
//    UART_TxCmd(LPC_UART1, ENABLE);

