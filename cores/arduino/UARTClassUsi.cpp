/*
  Copyright (c) 2011 Arduino.  All right reserved.

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "Arduino.h"

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "UARTClassUsi.h"

#ifdef __cplusplus
extern "C" {
#endif
#include "rtl8721d_usi.h"
#include "serial_api.h"
#include "rtl8721d_vector.h"

#ifdef __cplusplus
}
#endif

#define USI_UART_TX    _PB_20
#define USI_UART_RX    _PB_21
#define USI_DEV        USI0_DEV

RingBuffer rx_buffer3;

UARTClassUsi::UARTClassUsi(RingBuffer* pRx_buffer)
{
    _rx_buffer = pRx_buffer;
    //_dwIrq = dwIrq;
}

// Protected Methods //////////////////////////////////////////////////////////////

// Public Methods //////////////////////////////////////////////////////////////

void usi_uart_irq()
{
	u32 IntId;
    u8 data;
	IntId = USI_UARTIntStatus(USI_DEV);

	if((IntId&USI_RXFIFO_ALMOST_FULL_INTS)||(IntId&USI_RXFIFO_TM_OUT_INTS)){
		while(!USI_UARTReadable(USI_DEV));
		USI_UARTCharGet(USI_DEV, &data);
        // printf("usi_uart_irq:0x%x \r\n",data);

        rx_buffer3.store_char(data);
	}

	if((IntId&USI_TXFIFO_ALMOST_EMTY_INTS)){
        USI_UARTINTConfig(USI_DEV,USI_TX_FIFO_ALMOST_EMPTY_INTER, DISABLE);
	}
}

void UARTClassUsi::begin(const uint32_t dwBaudRate)
{
    USI_UARTInitTypeDef USI_UARTInitStruct;
    u32 LineSts=0;

    //printf("UARTClassUsi::begin \r\n");

	Pinmux_Config(USI_UART_TX, PINMUX_FUNCTION_UART);
	Pinmux_Config(USI_UART_RX, PINMUX_FUNCTION_UART);
	PAD_PullCtrl(USI_UART_TX, GPIO_PuPd_UP);   //Tx/Rx pin should pull up
	PAD_PullCtrl(USI_UART_RX, GPIO_PuPd_UP);

	RCC_PeriphClockCmd(APBPeriph_USI_REG, APBPeriph_USI_CLOCK, ENABLE);

	USI_UARTStructInit(&USI_UARTInitStruct);
	USI_UARTInitStruct.USI_UARTParity=USI_RUART_PARITY_DISABLE;
	USI_UARTInitStruct.USI_UARTRxFifoTrigLevel=1;
	USI_UARTInit(USI_DEV, &USI_UARTInitStruct);
	USI_UARTSetBaud(USI_DEV, dwBaudRate);
	USI_UARTRxCmd(USI_DEV, ENABLE);

    InterruptRegister((IRQ_FUN)usi_uart_irq, USI_DEV_TABLE[0].IrqNum, (u32)USI_DEV, 10);
	InterruptEn(USI_DEV_TABLE[0].IrqNum, 10);
	LineSts = USI_TX_FIFO_OVERFLOW_INTER | USI_RX_FIFO_OVERFLOW_INTER | USI_UART_PARITY_ERROR_INTER | USI_UART_STOP_ERROR_INTER;
	USI_UARTINTConfig(USI_DEV, USI_TX_FIFO_ALMOST_EMPTY_INTER|USI_RX_FIFO_ALMOST_FULL_INTER | USI_RX_FIFO_TIMEOUT_INTER|LineSts, ENABLE);
}

void UARTClassUsi::end(void)
{
    // clear any received data
    _rx_buffer->_iHead = _rx_buffer->_iTail;
}

int UARTClassUsi::available(void)
{
  return (uint32_t)(SERIAL_BUFFER_SIZE + _rx_buffer->_iHead - _rx_buffer->_iTail) % SERIAL_BUFFER_SIZE;
}

int UARTClassUsi::peek(void)
{
    if (_rx_buffer->_iHead == _rx_buffer->_iTail)
        return -1;

    return _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
}

int UARTClassUsi::read(void)
{
    // if the head isn't ahead of the tail, we don't have any characters
    if (_rx_buffer->_iHead == _rx_buffer->_iTail)
        return -1;

    uint8_t uc = _rx_buffer->_aucBuffer[_rx_buffer->_iTail];
    _rx_buffer->_iTail = (unsigned int)(_rx_buffer->_iTail + 1) % SERIAL_BUFFER_SIZE;
    return uc;
}

void UARTClassUsi::flush(void)
{
    // TODO: 
    // while ( serial_writable(&(this->sobj)) != 1 );
    /*
    // Wait for transmission to complete
    while ((_pUart->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY)
        ;
    */
}

size_t UARTClassUsi::write(const uint8_t uc_data)
{
    // printf("UARTClassUsi::write:%x \r\n",uc_data);
    while (!USI_UARTWritable(USI_DEV));
    USI_UARTCharPut(USI_DEV,uc_data);
    return 1;
}

UARTClassUsi Serial3(&rx_buffer3);

bool Serial3_available() {
    return Serial3.available() > 0;
}
