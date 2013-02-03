//
// Copyright (c) 2013 Jon Escombe
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include "BudgetTrainer.h"

void USART_Setup(void)
{
   UCSR0B |= (1 << TXEN0) | (1 << RXEN0);       // Turn on the transmit / receive circuitry

   UBRR0L = USART_BAUD_PRESCALE;                // Load lower 8-bits of the baud rate register
   UBRR0H = (USART_BAUD_PRESCALE >> 8);         // Load upper 8-bits of the baud rate register
}

uint8_t USART_GetChar(void)
{
    while (!(UCSR0A & (1<<RXC0)));
    return UDR0;
}

void USART_SendChar(char ByteToSend)
{
      while ((UCSR0A & (1 << UDRE0)) == 0) {};  // Wait until UDR is ready for more data
      UDR0 = ByteToSend;                        // Write the current byte
}

void USART_SendBuffer(uint8_t* BuffToSend, uint8_t BuffSize)
{
	uint8_t i;

   for (i=0; i<BuffSize; i++)
   {
      USART_SendChar(BuffToSend[i]);
   }

   // wait for transmit to complete before returning.
   while (!(UCSR0A & (1 << TXC0)));
}

void USART_ReadBuffer(uint8_t* BuffToRead, uint8_t BuffSize)
{
	uint8_t i;

	for (i=0 ; i < BuffSize; i++)
	{
		BuffToRead[i] = USART_GetChar();
	}
}


void SetupHardware()
{
	clock_prescale_set(clock_div_1);					// Disable clock prescaler (for 8MHz operation)

	USART_Setup();
}

void ReadData(uint8_t *buf, uint8_t size)
{
	USART_ReadBuffer(buf, size);
}


void WriteData(uint8_t *buf, uint8_t size)
{
	USART_SendBuffer(buf, size);
}

void GetButtonStatus(TrainerData *data)
{
	data->buttons = 0;
}

void CalculatePosition()
{

}

void MotorController()
{

}

void ProcessControlMessage(uint8_t *buf, TrainerData *data)
{
	// pull the inbound telemetry data out of the packet buffer and into
	// our internal data structure..

	data->mode = buf[2];
	data->target_gradient = buf[4];

	data->target_load = buf[6];
	data->target_load <<= 8;
	data->target_load |= buf[5];

	data->current_speed = buf[8];
	data->current_speed <<= 8;
	data->current_speed |= buf[7];

	data->current_power = buf[10];
	data->current_power <<= 8;
	data->current_power |= buf[9];
}

void PrepareStatusMessage(uint8_t *buf, TrainerData *data)
{
	// format the outbound status response packet buffer with current data
	buf[0] = 0xAA;
	buf[1] = 0x01;
	buf[2] = data->buttons;
	buf[3] = data->target_position;
	buf[4] = data->current_position;
}

int main()
{

	uint8_t RequestBuffer[BT_REQUEST_SIZE];
	uint8_t ResponseBuffer[BT_RESPONSE_SIZE];

	TrainerData data;

	SetupHardware();

	while(1)
	{
		// read the control message from the pc
		ReadData(RequestBuffer, BT_REQUEST_SIZE);

		ProcessControlMessage(RequestBuffer, &data);

		// calculate required motor position
		CalculatePosition();

		// move motor towards required position
		MotorController();

		// get the current button status
		GetButtonStatus(&data);

		// update response packet with button and position data
		PrepareStatusMessage(ResponseBuffer, &data);

		// update pc with current status
		WriteData(ResponseBuffer, BT_RESPONSE_SIZE);
	}
}
