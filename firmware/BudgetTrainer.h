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
// 6             0x00 -- UNUSED
// 7             0x00 -- UNUSED

#include <stdio.h>
#include <stdlib.h>
#include <avr/io.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <stdbool.h>
#include <math.h>

#include "Descriptors.h"
#include "LUFA/Drivers/Board/LEDs.h"
#include "LUFA/Drivers/USB/USB.h"

#define DEBUG_LEVEL_NONE  0
#define DEBUG_LEVEL_MIN   1
#define DEBUG_LEVEL_MID   2
#define DEBUG_LEVEL_MAX   3
#define DEBUG_OUTPUT      DEBUG_LEVEL_MIN

#define LOOKUP_TABLE
//#define SERVO_ARM                       // Using a servo arm with linkage instead of a pulley

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
#define BT_RESPONSE_SIZE 8

// Define offline operation modes
#define OFFLINE_STATUS_IDLE     0       // Offline - no controller buttons pressed (or cancel)
#define OFFLINE_STATUS_MANUAL   1       // Offline - buttons pressed to enable manual control
#define OFFLINE_STATUS_ONLINE   2       // Online  - GC training mode underway

typedef struct TrainerData
{
    uint8_t mode;
    uint8_t offline_mode;
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

#define LINEAR_TRAVEL       30                          // Required linear travel in mm

#define ARM_RADIUS          19.0                        // Effective radius of the servo arm in mm
#define X_AXIS_MAX          (LINEAR_TRAVEL/ARM_RADIUS)/2// Maximum point on x-axis (unit circle)

#define PULLEY_RADIUS       11.2                        // Radius of the servo pulley in mm
#define ROTATION_MAX        (LINEAR_TRAVEL \
                            / PULLEY_RADIUS)/2          // Maximum rotation from centre angle in radians

#define SERVO_MIN           1                           // Minimum motor position
#define SERVO_MAX           100.00                      // Target resolution of 100 positions (99 steps between so make it a double)
#define SERVO_MIDSTEP       (SERVO_MAX+SERVO_MIN)/2     // Assuming starting at 1, there are 99 steps, mid-way is 50.5

#define SPEED_SAMPLES       8                           // How many speed samples to average (note: sampled at 2x update rate)
#define POWER_SAMPLES       20                          // How many power samples to average (note: sampled at 10x update rate)
#define MAX_TRIM            10                          // Maximum change to calculated resistance, based on real-time power vs load
#define TRIM_DELAY          50                          // How many samples at constant load before attempting to trim resistance
#define TRIM_WAIT           25                          // How many samples since last trim value update before adjusting again
#define TRIM_THRESHOLD      2.5                         // Percentage difference between power & load before trim is attempted

#define RECEIVE_INTERVAL    6250                        // 100ms receive timeout (clk/256 prescaler)
#define MISSED_MSG_TIMEOUT  50                          // Timeout the session after 50 missed control messages (set minimum resistance etc)

// LUFA LED support macros
#define LEDMASK_USB_NOTREADY     LEDS_LED1              // Interface not ready
#define LEDMASK_USB_ENUMERATING (LEDS_LED2 | LEDS_LED3) // Interface is enumerating
#define LEDMASK_USB_READY       (LEDS_LED2 | LEDS_LED4) // Interface is ready
#define LEDMASK_USB_ERROR       (LEDS_LED1 | LEDS_LED3) // Error has occurred

// LUFA Function Prototypes
void EVENT_USB_Device_Connect(void);
void EVENT_USB_Device_Disconnect(void);
void EVENT_USB_Device_ConfigurationChanged(void);
void EVENT_USB_Device_ControlRequest(void);


