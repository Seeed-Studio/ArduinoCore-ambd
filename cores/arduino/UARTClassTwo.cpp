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
#include "UARTClassTwo.h"

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
void UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::ArduinoUartIrqHandler(uint32_t id, SerialIrq event)
{
    if (event == RxIrq)
    {
        auto instance = (UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>*)id;
        auto uartObj = &instance->UartObj_;
        auto rxBuffer = &instance->RxBuffer_;

        while (serial_readable(uartObj))
        {
            const uint8_t c = serial_getc(uartObj);
            rxBuffer->store_char(c);
        }
    }
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::UARTClassTwo(void)
{
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
void UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::begin(const uint32_t baudRate)
{
    serial_init(&UartObj_, TX_PIN, RX_PIN);
    serial_format(&UartObj_, 8, ParityNone, 1);
    serial_baud(&UartObj_, baudRate);

    serial_irq_set(&UartObj_, RxIrq, 1);
    serial_irq_handler(&UartObj_, ArduinoUartIrqHandler, (uint32_t)this);
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
void UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::end(void)
{
    serial_free(&UartObj_);
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
int UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::available(void)
{
    return RxBuffer_.available();
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
int UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::peek(void)
{
    return RxBuffer_.peek();
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
int UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::read(void)
{
    return RxBuffer_.read_char();
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
void UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::flush(void)
{
// TODO: 
// while ( serial_writable(&(this->sobj)) != 1 );
/*
  // Wait for transmission to complete
  while ((_pUart->UART_SR & UART_SR_TXRDY) != UART_SR_TXRDY)
    ;
*/
}

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
size_t UARTClassTwo<TX_PIN, RX_PIN, RX_BUFFER_SIZE>::write(const uint8_t uc_data)
{
    serial_putc(&UartObj_, (int)(uc_data));
    return 1;
}

UARTClassTwo<PA_26, PA_25, 4096> Serial2;
