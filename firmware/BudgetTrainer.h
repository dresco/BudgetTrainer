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

//
// Inbound control (request) message has the format:
//
// Byte          Value / Meaning
//
// 0             0xAA CONSTANT (device code)
// 1             0x01 CONSTANT (version code)
// 2             Mode - 0x01 = ergo, 0x02 = slope, 0x4 = calibrate -- ever used inbound?
// 3             Buttons - 0x01 = Enter, 0x02 = Minus, 0x04 = Plus, 0x08 = Cancel
// 4             Target gradient - (percentage + 10 * 10, i.e. -5% = 50, 0% = 100, 10% = 200)
// 5             Target load - Lo Byte
// 6             Target load - Hi Byte
// 7             Realtime speed - Lo Byte
// 8             Realtime speed - Hi Byte
// 9             Realtime power - Lo Byte
// 10            Realtime power - Hi Byte
// 11            0x00 -- UNUSED
// 12            0x00 -- UNUSED
// 13            0x00 -- UNUSED
// 14            0x00 -- UNUSED
// 15            0x00 -- UNUSED
//
//
// Outbound status (response) message has the format:
//
// Byte          Value / Meaning
//
// 0             0xAA CONSTANT
// 1             0x01 CONSTANT
// 2             Buttons - 0x01 = Enter, 0x02 = Minus, 0x04 = Plus, 0x08 = Cancel
// 3             Target motor position (1 to 100)
// 4             Current motor position (1 to 100)
// 5             0x00 -- UNUSED

#include <stdio.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <math.h>
//#include <stdlib.h>
//#include <string.h>
//#include <avr/sleep.h>
//#include <util/delay.h>

//#define DEBUG_OUTPUT

#define USART_BAUDRATE 9600
#define USART_BAUD_PRESCALE (((F_CPU / (USART_BAUDRATE * 16UL))) - 1)

// Buttons
#define BT_PLUS           0x04
#define BT_MINUS          0x02
#define BT_CANCEL         0x08
#define BT_ENTER          0x01

// Device operation modes
#define BT_ERGOMODE       0x01
#define BT_SSMODE         0x02
#define BT_CALIBRATE      0x04

// Define message sizes
#define BT_REQUEST_SIZE 16
#define BT_RESPONSE_SIZE 6

typedef struct TrainerData
{
    uint8_t mode;
    uint8_t buttons;

    uint8_t target_gradient;
    uint16_t target_load;

    uint8_t target_position;
    uint8_t current_position;

    uint16_t current_speed;
    uint16_t current_power;
} TrainerData;

typedef struct TableEntry
{
    uint8_t row;
    uint8_t col;
    uint8_t res;
    double speed;
    double power;
} TableEntry;

#define SERVO_INTERVAL      20000                       // 20ms (2MHz timer, 10ms up & 10ms down)
#define SERVO_MIDPOINT      1500                        // 1.5ms output pulse (1,500 us) for centre angle
#define SERVO_MAX_DIFF      600                         // 0.6ms change in output pulse for max angle
#define SERVO_DEGREE        6.25                        // Nominally 5.555us per degree of rotation (1000/180),
                                                        // but measured at ~6.25us (625us per 100 degrees rotation)
#define ARM_RADIUS          19.0                        // Effective radius of the servo arm in mm
#define LINEAR_TRAVEL       30                          // Required linear travel in mm
#define X_AXIS_MAX          (LINEAR_TRAVEL/ARM_RADIUS)/2// Maximum point on x-axis (unit circle)
#define SERVO_RES           100.00                      // Target resolution of 100 positions (99 steps between so make it a double)
#define SERVO_MIDSTEP       (SERVO_RES+1)/2             // Assuming starting at 1, there are 99 steps, mid-way is 50.5

#define SPEED_SAMPLES       10                          // How many speed samples to average

#define RECEIVE_INTERVAL    6250                        // 100ms receive timeout (clk/256 prescaler)
