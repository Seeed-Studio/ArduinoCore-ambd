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

#ifndef _UART_CLASS_TWO_
#define _UART_CLASS_TWO_

#include "HardwareSerial.h"
#include "RingBuffer.h"
#ifdef __cplusplus
extern "C" {
#endif
#include "PinNames.h"
#include "serial_api.h"
#ifdef __cplusplus
}
#endif

template <PinName TX_PIN, PinName RX_PIN, int RX_BUFFER_SIZE>
class UARTClassTwo : public HardwareSerial
{
    public:
        UARTClassTwo(void);
        void begin(const uint32_t baudRate);
        void end(void);
        int available(void);
        int peek(void);
        int read(void);
        void flush(void);
        size_t write(const uint8_t c);
        using Print::write; // pull in write(str) and write(buf, size) from Print
        operator bool() { return true; }; // UART always active

    private:
        RingBufferN<RX_BUFFER_SIZE> RxBuffer_;
        serial_t UartObj_;

    private:
        static void ArduinoUartIrqHandler(uint32_t id, SerialIrq event);

};

extern UARTClassTwo<PA_26, PA_25, 4096> Serial2;

#endif // _UART_CLASS_TWO_
