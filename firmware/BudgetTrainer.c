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
#include "usb_serial.h"
#include "lookup.h"

// volatile globals - accessed from interrupt handlers
volatile uint8_t buttons = 0;

uint8_t Interpolate(TableEntry *t0, TableEntry *t1, TableEntry *t2, TableEntry *t3, TableEntry *t4)
{
    uint8_t i = 0;
    double x, x1, x2, y, y1, y2, z11, z12, z21, z22;

    // bilinear interpolation
    //
    //      x1     x     x2
    //       |     |      |
    // y1--t1/z11-------t2/z21--
    //       |     |      |
    // y  ---|----t0------|----
    //       |     |      |
    // y2--t3/z12-------t4/z22--
    //       |     |      |
    //

    x   = t0->speed;
    x1  = t1->speed;
    x2  = t2->speed;
    y   = t0->power;
    y1  = t1->power;
    y2  = t3->power;
    z11 = t1->res;
    z12 = t3->res;
    z21 = t2->res;
    z22 = t4->res;

    i = ((((x2-x)*(y2-y)) / ((x2-x1)*(y2-y1))) * z11) +
        ((((x-x1)*(y2-y)) / ((x2-x1)*(y2-y1))) * z21) +
        ((((x2-x)*(y-y1)) / ((x2-x1)*(y2-y1))) * z12) +
        ((((x-x1)*(y-y1)) / ((x2-x1)*(y2-y1))) * z22);

    return i;
}

double LookupResistance(double x_speed, double y_power)
{
    uint8_t i, col, row;
    double resistance;
    TableEntry t0, t1, t2, t3, t4;

    // find the last row/col that we are greater than or equal to
    // using the provided x (speed) and y (power) values
    col = 0;
    for (i = 0; i < SPEED_COLS; i++)
    {
        if (x_speed >= speed_index[i])
            col = i;
    }

    row = 0;
    for (i = 0; i < POWER_ROWS; i++)
    {
        if (y_power >= power_index[i])
            row = i;
    }

    // find the four surrounding data points in the table (t1 - t4)
    if (row < POWER_ROWS-1)
    {
        t1.power = t2.power = power_index[row];
        t3.power = t4.power = power_index[row+1];
        t1.row = t2.row = row;
        t3.row = t4.row = row+1;
    }
    else
    {
        t1.power = t2.power = power_index[row-1];
        t3.power = t4.power = power_index[row];
        t1.row = t2.row = row-1;
        t3.row = t4.row = row;
    }

    if (col < SPEED_COLS-1)
    {
        t1.speed = t3.speed = speed_index[col];
        t2.speed = t4.speed = speed_index[col+1];
        t1.col = t3.col = col;
        t2.col = t4.col = col+1;
    }
    else
    {
        t1.speed = t3.speed = speed_index[col-1];
        t2.speed = t4.speed = speed_index[col];
        t1.col = t3.col = col-1;
        t2.col = t4.col = col;
    }

    // set the current speed and power point (t0)
    t0.speed = x_speed;
    t0.power = y_power;

    // ensure current speed and power values are not outside of the
    // surrounding data points, this should only be possible at edge of table
    if (t0.speed < min(t1.speed,t2.speed))
        t0.speed = min(t1.speed,t2.speed);

    if (t0.speed > max(t1.speed,t2.speed))
        t0.speed = max(t1.speed,t2.speed);

    if (t0.power < min(t1.power,t3.power))
        t0.power = min(t1.power,t3.power);

    if (t0.power > max(t1.power,t3.power))
        t0.power = max(t1.power,t3.power);

    // lookup the resistance values for each surrounding point
    t1.res = lookup_table_1d[((SPEED_COLS*t1.row) + t1.col)];
    t2.res = lookup_table_1d[((SPEED_COLS*t2.row) + t2.col)];
    t3.res = lookup_table_1d[((SPEED_COLS*t3.row) + t3.col)];
    t4.res = lookup_table_1d[((SPEED_COLS*t4.row) + t4.col)];

    // get the interpolated resistance value based on these points..
    resistance = Interpolate(&t0, &t1, &t2, &t3, &t4);

    // ensure final resistance value falls within our expected 1 - 100 range
    if (resistance < 51)
        resistance = 51;
    if (resistance > 150)
        resistance = 150;
    resistance = resistance - 50;

    return resistance;
}

double GetVirtualPower(double speed, double slope)
{
    //
    // Estimate the power required for a given speed & slope,
    // equations and constants from analyticcycling.com;
    //
    // Wind Resistance     Fw = 1/2 A Cw Rho Vmps2
    // Rolling Resistance  Frl = Wkg 9.8 Crr
    // Gravity Forces      Fsl = Wkg 9.8 GradHill
    // Power        RiderPower = (Fw + Frl + Fsl) Vmps
    //
    double A = 0.5;             // Frontal area
    double Cw = 0.5;            // Drag coefficient
    double Rho = 1.226;         // Air density
    double crr = 0.004;         // Coefficient of rolling resistance
    double g = 9.81;            // Gravity
    double Wkg = 85;            // 85kg combined rider+bike weight. TODO: move rider weight to config data
    double Vmps = speed / 3.6;  // Speed in meters per sec

    double power, Fw, Frl, Fsl;

    // calculate wind resistance
    Fw = 0.5 * A * Cw * Rho * pow(Vmps,2);

    // calculate rolling resistance
    Frl = Wkg * g * crr;

    // calculate gravity forces
    Fsl = Wkg * g * slope;

    // calculate required power
    power = (Fw + Frl + Fsl) * Vmps;

    return (power);
}

// The source code for this function comes from the zunzun.com online curve fitting
// site. The input to the curve fitting algorithms was derived from the power curves
// published at tacx.com.
//
// Fitting function: Simple_SimpleEquation_33_model()
// Fitting target: lowest sum of squared absolute error
// Fitting target value = 7584.58195696
double GetResistance(double x_in, double y_in)
{

    double temp;
    temp = 0.0;

    // coefficients
    double a = -1.5551745362491399E+01;
    double b = -1.4121261534412761E+00;
    double c = 1.9523034800130676E-01;

    temp = (a+y_in)/(b+c*x_in);
    return temp;
}

void MotorController(TrainerData *data)
{
    uint8_t target_position;
    uint8_t current_position;

    double x_axis, angle_rad, angle_deg;

#ifdef DEBUG_OUTPUT
    angle_rad = asin(X_AXIS_MAX);                       // Maximum rotational angle in radians
    angle_deg = angle_rad * 180 / M_PI;                 // Convert radians to degrees
    printf("max x_axis %f\n", X_AXIS_MAX);
    printf("max angle_rad %f\n", angle_rad);
    printf("max angle_deg %f\n", angle_deg);
#endif

    target_position = data->target_position;
    current_position = data->current_position;

    if ((target_position >= 1) && (target_position <= SERVO_RES))
    {
#ifdef DEBUG_OUTPUT
        printf("Target position %i\n", target_position);
#endif

        // basic rate limiting code to slow large arm movements,
        // reducing stress & load on the physical components
        if (target_position > current_position)
        {
            if (target_position - current_position > 10 )
                current_position = current_position + 3;
            else
                current_position++;
        }
        if (target_position < current_position)
        {
            if (current_position - target_position > 10 )
                current_position = current_position - 3;
            else
                current_position--;
        }

        // CHECK MY MATHS :)
        x_axis = (X_AXIS_MAX *
                 (current_position - SERVO_MIDSTEP))    // position should be signed plus/minus around the mid-point,
                 / (SERVO_MIDSTEP - 1);                 // divided by the number of steps each side
                                                        // Check also correct for even numbers of steps?

        angle_rad = asin(x_axis);                       // Required rotational angle in radians
        angle_deg = angle_rad * 180 / M_PI;             // Convert radians to degrees

        OCR1A = SERVO_MIDPOINT - (angle_deg *           // Timing value to get servo rotation to
                SERVO_DEGREE*2);                        //   required angle (in given direction)

#ifdef DEBUG_OUTPUT
        printf("Setting x_axis to %f, arm angle to %f, servo pulse to %i\n",
                x_axis, angle_deg, OCR1A);
#endif

        data->current_position = current_position;
    }
    else
    {
#ifdef DEBUG_OUTPUT
        printf("Invalid entry, please try again...\n");
#endif
    }
}

void USART_Setup(void)
{
    UCSR1B |= (1 << TXEN1) | (1 << RXEN1);              // Turn on the transmit / receive circuitry

    UBRR1L = USART_BAUD_PRESCALE;                       // Load lower 8-bits of the baud rate register
    UBRR1H = (USART_BAUD_PRESCALE >> 8);                // Load upper 8-bits of the baud rate register
}

void TimerSetup(void)
{
    // Configure timer3 for receive data timeout
    TCCR3B |= (1 << WGM32);                             // Configure timer for CTC mode 2
    OCR3A = RECEIVE_INTERVAL;                           // Set TOP value to 100ms
    TIMSK3 |= (1 << TOIE3);                             // Enable overflow interrupt

    // Configure timer1 for servo control output
    TCCR1B |= (1 << WGM13);                             // Configure timer for PWM Mode 8 (phase and frequency correct)
    ICR1 = SERVO_INTERVAL;                              // Set TOP value for the wave form to 20ms
    TCCR1A |= ((1 << COM1A1));                          // Clear OC1A on upcount compare and set on downcount compare
    TIMSK1 |= (1 << TOIE1);                             // Interrupt at bottom - TIMER1_OVF_vect

    TCCR1B |= (1 << CS11);                              // Start timer with prescaler of 8 = 2MHz timer
}

void PortSetup()
{
    DDRE |= (1 << 6);                                   // Set port E6 as output for debug LED
    DDRB |= (1 << PB5);                                 // Enable output on PINB5 (OC1A)

    PORTF |= (1 << 4);                                  // Enable pullup resistor on port F4 (Enter)
    PORTF |= (1 << 5);                                  // Enable pullup resistor on port F5 (Minus)
    PORTF |= (1 << 6);                                  // Enable pullup resistor on port F6 (Plus)
    PORTF |= (1 << 7);                                  // Enable pullup resistor on port F7 (Cancel)
}

uint8_t USART_GetChar(void)
{
    while (!(UCSR1A & (1 << RXC1)));
    return UDR1;
}

void USART_SendChar(char ByteToSend)
{
    while ((UCSR1A & (1 << UDRE1)) == 0);               // Wait until UDR is ready for more data
    UDR1 = ByteToSend;                                  // Write the current byte
}

void USB_SendBuffer(uint8_t* BuffToSend, uint8_t BuffSize)
{
    uint8_t i;

    for (i = 0; i < BuffSize; i++)
    {
        usb_serial_putchar(BuffToSend[i]);
    }

    // wait for transmit to complete before returning.
}

void USB_ReadBuffer(uint8_t* BuffToRead, uint8_t BuffSize)
{
    uint8_t i;
    int16_t c;

    // set the initial timer value to 1 in case we get to the check before the first timer tick
    TCNT3 = 1;

    // (re)start the receive timer with prescaler of 256 (timeout the request after 100ms)
    TCCR3B |= (1 << CS32);

    for (i = 0; i < BuffSize; i++)
    {
        do {
            c = usb_serial_getchar();

            // Invalidate the data header and abort the receive if the timer has expired
            if (TCNT3 == 0)
            {
#ifdef DEBUG_OUTPUT
                printf("timeout while reading data buffer\n");
#endif
                BuffToRead[0] = 0x00;
                return;
            }
        } while (c == -1);

        BuffToRead[i] = c;
    }
}

void SetupHardware(TrainerData *data)
{
    clock_prescale_set(clock_div_1);                    // Disable clock prescaler (for 8MHz operation)

    usb_init();

    USART_Setup();
    PortSetup();

    data->target_position = 1;                          // Set the initial arm position to minimum before we
    data->current_position = 1;                         // enable the timer for servo control
    MotorController(data);                              // Set the OCR1A timer interval to control the servo arm position

    TimerSetup();

    sei();                                              // Enable global interrupts
}

void ReadData(uint8_t *buf, uint8_t size)
{
    USB_ReadBuffer(buf, size);
    if ((buf[0] != 0xAA) || (buf[1] != 0x01))
    {
        // if we're here then the packet contains unexpected data, just log for now
#ifdef DEBUG_OUTPUT
        printf("packet contains unexpected data\n");
#endif

        // todo: handle this situation, possibly means we're out of sync
        //       and receiving part way though a packet? read in the remaining
        //       bytes until we get back in sync?
    }
}

void WriteData(uint8_t *buf, uint8_t size)
{
    USB_SendBuffer(buf, size);
}

void GetButtonStatus(TrainerData *data)
{
    static uint8_t last_buttons = 0;
    uint8_t cur_buttons;

    PORTE ^= (1 << 6);                                  // Toggle the debug LED on port E6

    // in lieu of debounce support, just make sure we don't send
    // multiple lap button presses in a row, as probably not what
    // anybody wants
    cur_buttons = buttons;
    data->buttons = cur_buttons;

    if (last_buttons & BT_ENTER)
        data->buttons &= ~BT_ENTER;

    last_buttons = cur_buttons;
    buttons = 0;
}

void CalculatePosition(TrainerData *data)
{
    uint8_t position;
    double speed, load, slope;
    double avg_speed = 0;
    static double total_speed;

#if 1
static int total_speed_init;

    // correctly initialise total_speed on first pass.
    // do we really want to do this? will ramp up from 0 over
    // a couple of seconds without - which may be preferable,
    // and what about the start of subsequent erg files?
    if (total_speed_init == 0)
    {
        total_speed = data->current_speed / 10.0 * SPEED_SAMPLES;
        total_speed_init = 1;
    }
#endif

    if (data->mode == BT_CALIBRATE)
    {
        // in calibration mode
        // for initial testing, just linear mapping between slope and position
        // todo: assumes gradient data is populated, need to confirm in GC
        avg_speed = data->current_speed / 10.0;
        position = data->target_gradient / 2.5;

        if (position < 1)
            position = 1;
        if (position > SERVO_RES)
            position = SERVO_RES;

        data->target_position = position;
    }

    if (data->mode == BT_SSMODE)
    {
        // in slope mode
        // Convert gradient representation (percentage + 10 * 10, i.e. -5% = 50, 0% = 100, 10% = 200)
        // into a fractional slope (i.e. -5% = -0.05, 0% = 0.0, 10% = 0.1)
        slope = (((data->target_gradient / 10.0) - 10) / 100);

        // Convert realtime speed into kph
        speed = data->current_speed / 10.0;

        // Track average values for the realtime speed
        total_speed -= total_speed / SPEED_SAMPLES;     // keep 9/10
        total_speed += speed;                           // and add in 1/10 from the new sample
        avg_speed = total_speed / SPEED_SAMPLES;        // and use composite rolling average

        // Estimate the required power to achieve current speed & slope
        load = GetVirtualPower(avg_speed, slope);

        // watch out for those downhills! ;)
        // the current resistance model appears to break down at low wattage
        if (load < 50)
            load = 50;

        // Look up the required resistance level for current speed and estimated power
        position = LookupResistance(avg_speed, load);

        if (position < 1)
            position = 1;
        if (position > SERVO_RES)
            position = SERVO_RES;

        data->target_position = position;
    }
    if (data->mode == BT_ERGOMODE)
    {
        // in ergo mode

        // Convert realtime speed into kph
        speed = data->current_speed / 10.0;

        // Track average values for the realtime speed
        total_speed -= total_speed / SPEED_SAMPLES;     // keep 9/10
        total_speed += speed;                           // and add in 1/10 from the new sample
        avg_speed = total_speed / SPEED_SAMPLES;        // and use composite rolling average

        // convert load into watts
        load = data->target_load / 10.0;

        // Look up the required resistance level for current speed and estimated power
        position = LookupResistance(avg_speed, load);

        if (position < 1)
            position = 1;
        if (position > SERVO_RES)
            position = SERVO_RES;

        data->target_position = position;
    }

    // speed override, if lower than 5kph set resistance to minimum
    if (avg_speed < 5)
        data->target_position = 1;
}

void ProcessControlMessage(uint8_t *buf, TrainerData *data)
{
    // make sure it's a valid packet before retrieving data
    if ((buf[0] == 0xAA) && (buf[1] == 0x01))
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

ISR(TIMER3_OVF_vect)
{
    // timer3 overflow interrupt, will get here 100ms after the start of a read operation.
    // Setting the timer value to 0 will abort the read if it's still in progress
    TCCR3B |= 0;                                        // Stop the timer
    TCNT3 = 0;                                          // Reset timer value to 0
}

ISR(TIMER1_OVF_vect)
{
    static uint8_t count = 0;
    uint8_t i;

    // we get here every 20ms, so process buttons every 5th interval for 10Hz
    // - just ensuring it's more frequent than the comms with the PC to ensure
    //   we can pass new button data every time..
    if (++count == 5)
    {
        count = 0;
//        PORTE ^= (1 << 6);                                  // Toggle the debug LED on port E6

        // save the state of PINS F4 to F7
        // todo: add debouncing
        //
        // F4 - Enter
        // F5 - Minus
        // F6 - Plus
        // F7 - Cancel
        //
        for (i=4 ; i<8 ; i++)
        {
            if (!(PINF & (1<<i)))
                buttons |= (1<<i);
        }
    }
}

#ifdef DEBUG_OUTPUT
static int uart_putc(char c, FILE *unused)
{
    while (!(UCSR1A & (1 << UDRE1)));
    UDR1 = c;
    return 0;
}

FILE uart_str = FDEV_SETUP_STREAM(uart_putc, NULL, _FDEV_SETUP_WRITE);
#endif

int main()
{
    uint8_t RequestBuffer[BT_REQUEST_SIZE];
    uint8_t ResponseBuffer[BT_RESPONSE_SIZE];

    TrainerData data;

    SetupHardware(&data);

#ifdef DEBUG_OUTPUT
    stdout = &uart_str;                                 // redirect printf and scanf to UART for debug output
#endif

    while (1)
    {
        // read the control message from the pc
        // todo: currently blocks forever, add timeout
        ReadData(RequestBuffer, BT_REQUEST_SIZE);

        ProcessControlMessage(RequestBuffer, &data);

        // calculate required motor position
        CalculatePosition(&data);

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
