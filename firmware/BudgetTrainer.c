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
    UCSR0B |= (1 << TXEN0) | (1 << RXEN0);              // Turn on the transmit / receive circuitry

    UBRR0L = USART_BAUD_PRESCALE;                       // Load lower 8-bits of the baud rate register
    UBRR0H = (USART_BAUD_PRESCALE >> 8);                // Load upper 8-bits of the baud rate register
}

void TimerSetup(void)
{
    TCCR1B |= (1 << WGM13);                             // Configure timer for PWM Mode 8 (phase and frequency correct)
    ICR1 = SERVO_INTERVAL;                              // Set TOP value for the wave form to 20ms
    OCR1A = SERVO_MIDPOINT;                             // Set output compare value to 1.5ms pulse (mid point)
    TCCR1A |= ((1 << COM1A1));                          // Clear OC1A on upcount compare and set on downcount compare
    TIMSK1 |= (1 << TOIE1);                             // Interrupt at bottom - TIMER1_OVF_vect

    TCCR1B |= (1 << CS11);                              // Start timer with prescaler of 8 = 1MHz timer
}

void PortSetup()
{
    DDRB |= (1 << 0);                                   // Set port B0 as output for debug LED
    DDRB |= (1 << PB1);                                 // Enable output on PINB1 (OC1A)
}

uint8_t USART_GetChar(void)
{
    while (!(UCSR0A & (1 << RXC0)));
    return UDR0;
}

void USART_SendChar(char ByteToSend)
{
    while ((UCSR0A & (1 << UDRE0)) == 0);               // Wait until UDR is ready for more data
    UDR0 = ByteToSend;                                  // Write the current byte
}

void USART_SendBuffer(uint8_t* BuffToSend, uint8_t BuffSize)
{
    uint8_t i;

    for (i = 0; i < BuffSize; i++)
    {
        USART_SendChar(BuffToSend[i]);
    }

    // wait for transmit to complete before returning.
    while (!(UCSR0A & (1 << TXC0)));
}

void USART_ReadBuffer(uint8_t* BuffToRead, uint8_t BuffSize)
{
    uint8_t i;

    for (i = 0; i < BuffSize; i++)
    {
        BuffToRead[i] = USART_GetChar();
    }
}

void SetupHardware()
{
    clock_prescale_set(clock_div_1);                    // Disable clock prescaler (for 8MHz operation)

    USART_Setup();
    TimerSetup();
    PortSetup();

    sei();                                              // Enable global interrupts
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

void MotorController(TrainerData *data)
{
    uint8_t target_position;
    uint8_t current_position;

    double x_axis, angle_rad, angle_deg;

    angle_rad = asin(X_AXIS_MAX);                       // Maximum rotational angle in radians
    angle_deg = angle_rad * 180 / M_PI;                 // Convert radians to degrees

    target_position = data->target_position;
    current_position = data->current_position;

    printf("max x_axis %f\n", X_AXIS_MAX);
    printf("max angle_rad %f\n", angle_rad);
    printf("max angle_deg %f\n", angle_deg);

    if ((target_position >= 1) && (target_position <= SERVO_RES))
    {
        printf("Target position %i\n", target_position);

        // CHECK MY MATHS :)
        x_axis = (X_AXIS_MAX *
                 (target_position - SERVO_MIDSTEP))     // position should be signed plus/minus around the mid-point,
                 / (SERVO_MIDSTEP - 1);                 // divided by the number of steps each side
                                                        // Check also correct for even numbers of steps?

        angle_rad = asin(x_axis);                       // Required rotational angle in radians
        angle_deg = angle_rad * 180 / M_PI;             // Convert radians to degrees

        OCR1A = SERVO_MIDPOINT - (angle_deg *           // Timing value to get servo rotation to
                SERVO_DEGREE);                          //   required angle (in given direction)

        printf("Setting x_axis to %f, arm angle to %f, servo pulse to %i\n",
                x_axis, angle_deg, OCR1A);
        current_position = target_position;
        data->target_position = target_position;
        data->current_position = current_position;
    }
    else
    {
        printf("Invalid entry, please try again...\n");
    }
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

ISR( TIMER1_OVF_vect)
{
    PORTB ^= (1 << 0);                                  // Toggle the debug LED on port B0
}

static int uart_putc(char c, FILE *unused)
{
    while (!(UCSR0A & (1 << UDRE0)));
    UDR0 = c;
    return 0;
}
FILE uart_str = FDEV_SETUP_STREAM(uart_putc, NULL, _FDEV_SETUP_WRITE);

int main()
{
    uint8_t RequestBuffer[BT_REQUEST_SIZE];
    uint8_t ResponseBuffer[BT_RESPONSE_SIZE];

    TrainerData data;

    SetupHardware();

    stdout = &uart_str;                                 // redirect printf and scanf to UART for debug output

    while (1)
    {
        // read the control message from the pc
        ReadData(RequestBuffer, BT_REQUEST_SIZE);

        ProcessControlMessage(RequestBuffer, &data);

        // calculate required motor position
        CalculatePosition();

        // move motor towards required position
        MotorController(&data);

        // get the current button status
        GetButtonStatus(&data);

        // update response packet with button and position data
        PrepareStatusMessage(ResponseBuffer, &data);

        // update pc with current status
        WriteData(ResponseBuffer, BT_RESPONSE_SIZE);
    }
}
