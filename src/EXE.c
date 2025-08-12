//
// Smpl_Timer_SR04x2 : using two SR04 Ultrasound Sensors
//
// Timer Capture :
// GPB2 / RTS0 / T2EX (NUC140 pin34)
// GPB3 / CTS0 / T3EX (NUC140 pin35)
// GPB4 / RX1         (NUC140 pin19)
// GPB5 / TX1         (NCU140 pin20)

// SR04 Ultrasound Sensor A pitch
// pin1 Vcc : to Vcc5
// pin2 Trig: to GPB4      (NUC140VE3xN pin19)
// pin3 ECHO: to GPB2/T2EX (NUC140VE3xN pin34)
// pin4 Gnd : to GND

// SR04 Ultrasound Sensor B volume
// pin1 Vcc : to Vcc5
// pin2 Trig: to GPB5      (NUC140VE3xN pin20)
// pin3 ECHO: to GPB3/T3EX (NUC140VE3xN pin35)
// pin4 Gnd : to GND

// Buzzer: GND and GPA15

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "NUC1xx.h"
#include "DrvGPIO.h"
#include "DrvSYS.h"
#include "LCD_Driver.h"
#include "note_freq.h"
#include "Driver_PWM.h"
#include "scankey.h"

// Global definition
#define _SR04A_ECHO (GPB_2) // NUC140VE3xN, Pin19
#define _SR04A_TRIG (GPB_4) // NUC140VE3xN, Pin34
#define _SR04A_TRIG_Low (GPB_4 = 0)
#define _SR04A_TRIG_High (GPB_4 = 1)

#define _SR04B_ECHO (GPB_3) // NUC140VE3xN, Pin20
#define _SR04B_TRIG (GPB_5) // NUC140VE3xN, Pin35
#define _SR04B_TRIG_Low (GPB_5 = 0)
#define _SR04B_TRIG_High (GPB_5 = 1)
#define  ONESHOT  0   // counting and interrupt when reach TCMPR number, then stop
#define  PERIODIC 1   // counting and interrupt when reach TCMPR number, then counting from 0 again
#define  MUTE_VOLUME 0
#define  MID_VOLUME 99
#define  MAX_VOLUME 5
// Global variables
volatile uint32_t SR04A_Echo_Width = 0;
volatile uint32_t SR04A_Echo_Flag = FALSE;
volatile uint32_t SR04B_Echo_Width = 0;
volatile uint32_t SR04B_Echo_Flag = FALSE;

char TextTimer[17] = "                ";
static int h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 0, s2 = 0;
unsigned char CleanC4[16 * 8];
unsigned char CleanD4[16 * 8];
unsigned char CleanE4[16 * 8];
unsigned char CleanF4[16 * 8];
unsigned char CleanG4[16 * 8];
unsigned char CleanA4[16 * 8];
unsigned char CleanB4[16 * 8];
unsigned char CleanC5[16 * 8];

uint32_t Timing_Flag_1000ms = 0;

uint32_t Value_1 = 0;

unsigned char lines[128*8] = {
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
	0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
	0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,0x01,
	0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,0x81,
	0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

unsigned char DO[16 * 8] = {
    0xff, 0x81, 0x81, 0x81, 0x7e, 0x00, 0x7e, 0x81, 0x81, 0x81, 0x7e, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char RE[16 * 8] = {
        0xff, 0x11, 0x31, 0x51, 0x8e, 0x00, 0xff, 0x91, 0x91, 0x91, 0x91, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char MI[16 * 8] = { 
    0xff, 0x02, 0x04, 0x02, 0xff, 0x00, 0x81, 0x81, 0xff, 0x81, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char FA[16 * 8] = { 
    0xff, 0x11, 0x11, 0x11, 0x11, 0x00, 0xfe, 0x11, 0x11, 0x11, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char SOL[24 * 8] = { 
    0x9f, 0x91, 0x91, 0x91, 0xf1, 0x00, 0x7e, 0x81, 0x81, 0x81, 0x7e, 0x00, 0xff, 0x80, 0x80, 0x80};

unsigned char LA[16 * 8] = { 
    0xff, 0x80, 0x80, 0x80, 0x80, 0x00, 0xfe, 0x11, 0x11, 0x11, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char SI[16 * 8] = { 
    0x9f, 0x91, 0x91, 0x91, 0xf1, 0x00, 0x81, 0x81, 0xff, 0x81, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00};
    
unsigned char lines2[128*8] = {
    0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
	0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
	0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
	0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
	0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,0x10,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
};

unsigned char full[16 * 8] = {
    0x81, 0x81, 0x81, 0x81, 0x99, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x99, 0x81, 0x81, 0x81, 0x81};

unsigned char ufull[16 * 8] = {
    0x01, 0x01, 0x01, 0x01, 0x19, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x7f, 0x19, 0x01, 0x01, 0x01, 0x01};

unsigned char dfull[16 * 8] = {
    0x80, 0x80, 0x80, 0x80, 0x98, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0xfe, 0x98, 0x80, 0x80, 0x80, 0x80};

unsigned char midfull2[16 * 8] = {
    0x10, 0x10, 0x10, 0x10, 0x18, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x18, 0x10, 0x10, 0x10, 0x10};

unsigned char emptfull[16 * 8] = {
    0x00, 0x00, 0x00, 0x00, 0x18, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x7e, 0x18, 0x00, 0x00, 0x00, 0x00};

unsigned char b[16 * 8] = {
    0xff, 0x90, 0x90, 0x90, 0x60, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

unsigned char empty[16 * 8] = { 
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};



// Timer1 initialize to tick every 1ms
void InitTIMER1(void)
{
    /* Step 1. Enable and Select Timer clock source */
    SYSCLK->CLKSEL1.TMR1_S = 0; // Select 12Mhz for Timer1 clock source
    SYSCLK->APBCLK.TMR1_EN = 1; // Enable Timer1 clock source

    /* Step 2. Select Operation mode */
    TIMER1->TCSR.MODE = PERIODIC; // Select periodic mode for operation mode

    /* Step 3. Select Time out period = (Period of timer clock input) * (8-bit Prescale + 1) * (24-bit TCMP)*/
    TIMER1->TCSR.PRESCALE = 255; // Set Prescale [0~255]
    TIMER1->TCMPR = 46875;       // Set TCMPR [0~16777215]
                                 // (1/12000000)*(255+1)*46875 = 1 sec / 1 Hz

    /* Step 4. Enable interrupt */
    TIMER1->TCSR.IE = 1;
    TIMER1->TISR.TIF = 1;      // Write 1 to clear for safty
    NVIC_EnableIRQ(TMR1_IRQn); // Enable Timer1 Interrupt

    /* Step 5. Enable Timer module */
    TIMER1->TCSR.CRST = 1; // Reset up counter
    TIMER1->TCSR.CEN = 1;  // Enable Timer1
}

void TMR1_IRQHandler(void) // Timer1 interrupt subroutine
{
    {
        s1++;
        if (s1 == 10)
        {
            s1 = 0;
            s2++;
        }
        if (s1 == 0 && s2 == 6)
        {
            m1++;
            s1 = s2 = 0;
        }
        if (m1 == 10)
        {
            m2++;
            m1 = 0;
        }
        if (m1 == 0 && m2 == 6)
        {
            h1++;
            m1 = m2 = 0;
        }
        if (h1 == 10)
        {
            h2++;
            h1 = 0;
        }
        if (h1 == 4 && h2 == 2)
        {
            h1 = h2 = 0;
        }
        sprintf(TextTimer, " %d%d : %d%d : %d%d ", h2, h1, m2, m1, s2, s1);
        print_lcd(0, TextTimer);
        TIMER1->TISR.TIF = 1;
    }
}




// Initial TMR2
//
// Timer Clock:	12 MHz
// Prescale:		11
// Compare:		0xffffff
// Mode:			One-Shot mode
// Capture:		Enable, Capture with Falling Edge
void Init_TMR2(void)
{
    // Step 1: T2EX pin Enable (PB.2, Pin34)
    SYS->GPBMFP.UART0_nRTS_nWRL = 1;
    SYS->ALTMFP.PB2_T2EX = 1;

    // Step 2: Timer Controller Reset and Setting Clock Source
    SYS->IPRSTC2.TMR2_RST = 1;  // Timer Controller: Reset
    SYS->IPRSTC2.TMR2_RST = 0;  // Timer Controller: Normal
    SYSCLK->CLKSEL1.TMR2_S = 0; // Timer Clock = 12 MHz
    SYSCLK->APBCLK.TMR2_EN = 1; // Timer C lock Enable

    // Step 3: Timer Controller Setting
    //   TMR0_T = (12 MHz / (11+1) / 1000000)^-1 = 1.000 Second
    TIMER2->TCMPR = 0xffffff;   // Timer Compare Value:  [0~16777215]
    TIMER2->TCSR.PRESCALE = 11; // Timer Prescale:       [0~255]
    TIMER2->TCSR.MODE = ONESHOT;      // Timer Operation Mode: One-Shot

    // Step 4: External Capture Mode Setting
    TIMER2->TEXCON.TEXEN = 1;     // External Capture Function Enable
    TIMER2->TEXCON.RSTCAPSEL = 0; // Capture Mode Select: Capture Mode
    TIMER2->TEXCON.TEX_EDGE = 2;  // Capture Edge: Rising & Falling

    // Step 5: Timer Interrupt Setting
    TIMER2->TEXCON.TEXIEN = 1; // Capture Interrupt Enable
    TIMER2->u32TEXISR |= 0x01; // Clear Capture Flag (TEXIF)
    NVIC_EnableIRQ(TMR2_IRQn); // Timer NVIC IRQ Enable
}


void TMR2_IRQHandler(void)// Timer2 interrupt subroutine
{
    TIMER2->TEXCON.RSTCAPSEL = 0; // set back for falling edge to capture
    TIMER2->TCSR.CEN = 1;         // Timer Start

    if (TIMER2->TEXISR.TEXIF == 1) // Capture Flag (TEXIF)
    {
        TIMER2->u32TEXISR |= 0x01;       // Clear Capture Flag (TEXIF)
        SR04A_Echo_Width = TIMER2->TCAP; // Load Capture Value (Unit: us)
        SR04A_Echo_Flag = TRUE;
    }
}


// Initial TMR3
//
// Timer Clock:	12 MHz
// Prescale:		11
// Compare:		0xffffff
// Mode:			One-Shot mode
// Capture:		Enable, Capture with Falling Edge
void Init_TMR3(void)
{
    // Step 1: T3EX pin Enable (PB.3, Pin35)
    SYS->GPBMFP.UART0_nCTS_nWRH = 1;
    SYS->ALTMFP.PB3_T3EX = 1;

    // Step 2: Timer Controller Reset and Setting Clock Source
    SYS->IPRSTC2.TMR3_RST = 1;  // Timer Controller: Reset
    SYS->IPRSTC2.TMR3_RST = 0;  // Timer Controller: Normal
    SYSCLK->CLKSEL1.TMR3_S = 0; // Timer Clock = 12 MHz
    SYSCLK->APBCLK.TMR3_EN = 1; // Timer C lock Enable

    // Step 3: Timer Controller Setting
    //   TMR3_T = (12 MHz / (11+1) / 1000000)^-1 = 1.000 Second
    TIMER3->TCMPR = 0xffffff;   // Timer Compare Value:  [0~16777215]
    TIMER3->TCSR.PRESCALE = 11; // Timer Prescale:       [0~255]
    TIMER3->TCSR.MODE = ONESHOT;      // Timer Operation Mode: One-Shot

    // Step 4: External Capture Mode Setting
    TIMER3->TEXCON.TEXEN = 1;     // External Capture Function Enable
    TIMER3->TEXCON.RSTCAPSEL = 0; // Capture Mode Select: Capture Mode
    TIMER3->TEXCON.TEX_EDGE = 2;  // Capture Edge: Rising & Falling

    // Step 5: Timer Interrupt Setting
    TIMER3->TEXCON.TEXIEN = 1; // Capture Interrupt Enable
    TIMER3->u32TEXISR |= 0x01; // Clear Capture Flag (TEXIF)
    NVIC_EnableIRQ(TMR3_IRQn); // Timer NVIC IRQ Enable
}


void TMR3_IRQHandler(void)// Timer3 interrupt subroutine
{
    TIMER3->TEXCON.RSTCAPSEL = 0; // set back for falling edge to capture
    TIMER3->TCSR.CEN = 1;         // Timer Start

    if (TIMER3->TEXISR.TEXIF == 1) // Capture Flag (TEXIF)
    {
        TIMER3->u32TEXISR |= 0x01;       // Clear Capture Flag (TEXIF)
        SR04B_Echo_Width = TIMER3->TCAP; // Load Capture Value (Unit: us)
        SR04B_Echo_Flag = TRUE;
    }
}

// Ultrasonic Trigger
void SR04_Trigger(void)
{
    // Trigger of Ultrasonic Sensor
    _SR04A_TRIG_High;
    _SR04B_TRIG_High;
    DrvSYS_Delay(10); // 10us for TRIG width
    _SR04A_TRIG_Low;
    _SR04B_TRIG_Low;

    TIMER2->TEXCON.RSTCAPSEL = 1; // set for rising edge trigger to reset counter
    TIMER3->TEXCON.RSTCAPSEL = 1;
}

void Init_GPIO_SR04(void)
{
    // Ultrasonic I/O Pins Initial
    GPIOB->PMD.PMD2 = 0; //_SR04_ECHO pin, Input
    GPIOB->PMD.PMD3 = 0;
    GPIOB->PMD.PMD4 = 1; //_SR04_TRIG pin, Output
    GPIOB->PMD.PMD5 = 1;
    _SR04A_TRIG_Low; // set Trig output to Low
    _SR04B_TRIG_Low;
}

uint16_t dist_to_note(uint32_t distance_mm)//Setting musical note by measured distance
{
    if (distance_mm < 50)
        return C5;
    else if (distance_mm < 100)
        return B4;
    else if (distance_mm < 150)
        return B4b;
    else if (distance_mm < 200)
        return A4;
    else if (distance_mm < 250)
        return A4b;
    else if (distance_mm < 300)
        return G4;
    else if (distance_mm < 350)
        return G4b;
    else if (distance_mm < 400)
        return F4;
    else if (distance_mm < 450)
        return E4;
    else if (distance_mm < 500)
        return E4b;
    else if (distance_mm < 550)
        return D4;
    else if (distance_mm < 600)
        return D4b;
    return C4;
}

const char *getNoteName(int note)//Converting int to char
{
    switch (note)
    {
    case C5:
        return "C5 ";
    case B4:
        return "B4 ";
    case B4b:
        return "B4b";
    case A4:
        return "A4 ";
    case A4b:
        return "A4b";
    case G4:
        return "G4 ";
    case G4b:
        return "G4b";
    case F4:
        return "F4 ";
    case E4:
        return "E4 ";
    case D4:
        return "D4 ";
    case D4b:
        return "D4b";
    case C4:
        return "C4 ";
    case B3:
        return "B3 ";
    case B3b:
        return "B3b";
    case A3:
        return "A3 ";
    case A3b:
        return "A3b";
    case G3:
        return "G3 ";
    case G3b:
        return "G3b";
    }
}

uint8_t dist_to_vol(uint32_t distance_mm)//Setting on timre dudty cycle by measured distance
{
    if (distance_mm < 50)
        return MUTE_VOLUME;
    else if (distance_mm < 100)
        return MID_VOLUME;
    // else if (distance_mm < 150)
    //     return 5;
    // return 5;
    else
        return MAX_VOLUME;
}


//------------------------------
// MAIN function
//------------------------------
int main(void)
{

    uint32_t distance_mm, distanceVol_mm, distancePitch_mm;
    uint8_t i, duty_cycle;
    uint16_t note;
    int8_t number;

    char TextJonNotes[] = "G4 E4 E4 F4 D4 D4 C4 D4 E4 F4 G4 G4 G4 G4";
    char TextFreq[17]  = "Freq :         Hz";
    char TextEmpty[17] = "                 ";
    char TextNotes[17] = "notes:          ";
    char TextCMajorNotes[] = "C4 D4 E4 F4 G4 A4 B4 C5";
    char TempNote[3];

    int INTArrCMajorNotes[] = {C4, D4, E4, F4, G4, A4, B4, C5};
    int NoteToPlay;
    int j, w, k, a;
    int INTArrJonNotes[] = {G4, E4, E4, F4, D4, D4, C4, D4, E4, F4, G4, G4, G4, G4};
    int record[50] = { 0 };
    int FlagMatchNoteAndMute;
    int FlagRecord;
    // int flag3 = 0;  -----------delete--------------

    // System Clock Initial
    UNLOCKREG();
    DrvSYS_SetOscCtrl(E_SYS_XTL12M, ENABLE);
    SYSCLK->PWRCON.XTL12M_EN = 1;
    SYSCLK->CLKSEL0.HCLK_S = 0;
    while (DrvSYS_GetChipClockSourceStatus(E_SYS_XTL12M) == 0);
    DrvSYS_Open(50000000);
    LOCKREG();

    Initial_panel(); // initialize LCD
    clr_all_panel(); // clear LCD display
    InitTIMER1(); // initialize Timer1
    Init_TMR2(); // initialize Timer2 Capture
    Init_TMR3(); // initialize Timer3 Capture

    InitPWM(3); // initialize PWM3, output pin = GPA15

    OpenKeyPad(); // initialize 3x3 keypad


    // for (w = 0; w < 50; w++)//
    // {
    //     record[w] = 0;
    // }
    //w = 0; RUN to check

    for (i = 0; i < 16; i++)
    {
        CleanC4[i] = lines[825 + i];
        CleanD4[i] = lines[695 + i];
        CleanE4[i] = lines2[570 + i];
        CleanF4[i] = lines[570 + i];
        CleanG4[i] = lines2[438 + i];
        CleanA4[i] = lines[438 + i];
        CleanB4[i] = lines2[312 + i];
        CleanC5[i] = lines[312 + i];
    }

    while (1)
    {
        //Showing Main Menu on LCD
        print_lcd(0, "Theremin        "); 
        print_lcd(1, "1)Learning Mode ");
        print_lcd(2, "2)Freestyle     ");
        print_lcd(3, "3)Record        ");
        PWM_Freq(3, note, MUTE_VOLUME); // Mute Buzzer - set PWM3 with 0% duty cycle

        number = Scankey(); // Scan keypad to input
        switch (number)
        {
        case 1://Learning Mode
            TIMER1->TCSR.CEN = 0;
            //Showing Learning Mode Menu on LCD
            print_lcd(0, "Learning Mode   ");
            print_lcd(1, "1)C Major       ");
            print_lcd(2, "2)YonatanAkatan ");
            print_lcd(3, TextEmpty);//CLean up text line
            number = 0;//Ensuring no entrance to first option in next menu
            DrvSYS_Delay(1000000);

            while (number != 9)//Learning Mode
            {
                number = Scankey();
                switch (number)
                { 
                case 1://C Major

                    TIMER1->TCSR.CEN = 0;
                    //Clean up LCD & Reset flag and indexes
                    print_lcd(0, TextEmpty);
                    print_lcd(1, TextEmpty);
                    print_lcd(2, TextEmpty);
                    print_lcd(3, TextEmpty);
                    j = 0;
                    k = 0;
                    FlagMatchNoteAndMute = 0;

                    //flag3 = 0;---------delete------------------

                    number = Scankey();
                    while (number != 9)//C Major is running until press 9
                    {

                        number = Scankey();
                        SR04_Trigger();      // Trigger Ultrasound Sensor for 10us
                        DrvSYS_Delay(40000); // Wait 40ms for Echo to trigger interrupt

                        if (SR04A_Echo_Flag == TRUE)
                        {
                            SR04A_Echo_Flag = FALSE;

                            distance_mm = SR04A_Echo_Width * (340 / 2) / 1000;
                            distancePitch_mm = distance_mm;
                        }

                        if (SR04B_Echo_Flag == TRUE)
                        {
                            SR04B_Echo_Flag = FALSE;

                            distance_mm = SR04B_Echo_Width * (340 / 2) / 1000;
                            distanceVol_mm = distance_mm;

                        }
                        DrvSYS_Delay(10000); // 10ms from Echo to next Trigger

                        note = dist_to_note(distancePitch_mm);//Inserting note

                        duty_cycle = dist_to_vol(distanceVol_mm);//Inserting duty cycle on time

                        strncpy(TempNote, TextCMajorNotes + k, 2);//Inserting note from TextCMajorNotes into TempNode
                        TempNote[2] = NULL;
                        PWM_Freq(3, note, duty_cycle);// set PWM3 with frequency & duty cycle
                        
                        NoteToPlay = INTArrCMajorNotes[j];

                        if (j < 8)//Size of 8 notes in TextCMajorNotes
                            sprintf(TextNotes + 7, "%s  ", TempNote);//Note to play
                        else
                            sprintf(TextNotes + 7, "%s  ", "C5");//Last note in TextCMajorNotes

                        //Showing next action with the hands according their position on second line of LCD
                        if (note < NoteToPlay)
                            print_lcd(1, "Move Hand: <<<   ");
                        else if (note > NoteToPlay)
                            print_lcd(1, "Move Hand: >>>   ");
                        else if (note == NoteToPlay && duty_cycle != 0)
                            print_lcd(1, "Mute The Note    ");
                        else if (note == NoteToPlay && duty_cycle == 0)
                            print_lcd(1, "Turn Volume Up   ");
                        
                        //Showing volume on fourth line of LCD
                        if (duty_cycle == 0)
                            print_lcd(3, "Volume: _______  ");
                        else if (duty_cycle == 99)
                            print_lcd(3, "Volume: | |      ");
                        else
                            print_lcd(3, "Volume: | | | |  ");

                        print_lcd(0, TextNotes);//Showing note to play on first line of LCD 
                        
                        sprintf(TextFreq + 7, "%s", getNoteName(note));
                        print_lcd(2, TextFreq);//Showing a note playing on third line of LCD
                        
                        //If there is a match between note and note for playing and muting - flag goes up
                        if (note == NoteToPlay && duty_cycle == MUTE_VOLUME)
                        {
                            FlagMatchNoteAndMute = 1;
                        }

                        //If there is a match between note and note for playing and flag up and full volume - flag goes down, increment indexes 
                        if (note == NoteToPlay && FlagMatchNoteAndMute == 1 && duty_cycle == MAX_VOLUME)
                        {
                            FlagMatchNoteAndMute = 0;
                            j++;
                            k += 3;
                            if (note == C5)//If finished resetting indexes and NoteToPlay to run again
                            {
                                NoteToPlay = INTArrCMajorNotes[0];
                                k = 0;
                                j = 0;
                            }
                        }
                    }
                    if (number == 9)
                        break;

                case 2://YonatanAkatan
                    TIMER1->TCSR.CEN = 0;

                    //Clean up LCD & Reset flag and indexes
                    print_lcd(0, TextEmpty);
                    print_lcd(1, TextEmpty);
                    print_lcd(2, TextEmpty);
                    print_lcd(3, TextEmpty);
                    j = 0;
                    k = 0;
                    FlagMatchNoteAndMute = 0;

                    while (number != 9)//YonatanAkatan is running until press 9 or finish
                    {

                        number = Scankey();
                        SR04_Trigger();      // Trigger Ultrasound Sensor for 10us
                        DrvSYS_Delay(40000); // Wait 40ms for Echo to trigger interrupt

                        if (SR04A_Echo_Flag == TRUE)
                        {
                            SR04A_Echo_Flag = FALSE;

                            distance_mm = SR04A_Echo_Width * (340 / 2) / 1000;
                            distancePitch_mm = distance_mm;
                        }

                        if (SR04B_Echo_Flag == TRUE)
                        {
                            SR04B_Echo_Flag = FALSE;

                            distance_mm = SR04B_Echo_Width * (340 / 2) / 1000;
                            distanceVol_mm = distance_mm;
                        }
                        DrvSYS_Delay(10000); // 10ms from Echo to next Trigger

                        note = dist_to_note(distancePitch_mm);//Inserting note

                        duty_cycle = dist_to_vol(distanceVol_mm);//Inserting duty cycle on time

                        strncpy(TempNote, TextJonNotes + k, 2);//Inserting note from TextCMajorNotes into TempNode
                        TempNote[2] = NULL;
                        PWM_Freq(3, note, duty_cycle);// set PWM3 with frequency & duty cycle

                        NoteToPlay = INTArrJonNotes[j];

                        if (j < 14)//Size of 14 notes in TextJonNotes
                            sprintf(TextNotes + 7, "%s  ", TempNote);//Note to play
                        else
                            sprintf(TextNotes + 7, "%s  ", "G4");//Last note in TextCMajorNotes

                        //Showing next action with the hands according their position on second line of LCD
                        if (note < NoteToPlay)
                            print_lcd(1, "Move Hand: <<<   ");
                        else
                            print_lcd(1, "Move Hand: >>>   ");

                        //Showing volume on fourth line of LCD
                        if (duty_cycle == 0)
                            print_lcd(3, "Volume: _______  ");
                        else if (duty_cycle == 99)
                            print_lcd(3, "Volume: | |      ");
                        else
                            print_lcd(3, "Volume: | | | |  ");
                        
                        print_lcd(0, TextNotes);//Showing note to play on first line of LCD 
                        
                        sprintf(TextFreq + 7, "%s", getNoteName(note));					
						print_lcd(2, TextFreq);//Showing a note playing on third line of LCD
                        
                        //If there is a match between note and note for playing and muting - flag goes up
                        if (note == NoteToPlay && duty_cycle == 0)
                        {
                            FlagMatchNoteAndMute = 1;
                        }

                        //If there is a match between note and note for playing and flag up and full volume - flag goes down, increment indexes 
                        if (note == NoteToPlay && FlagMatchNoteAndMute == 1 && duty_cycle == MAX_VOLUME)
                        {
                            FlagMatchNoteAndMute = 0;
                            j++;
                            k += 3;
                            if (j == 14)//If completed so number = 9 for return to main menu
                            {
                                DrvSYS_Delay(40000);
                                number = 9;
                                DrvSYS_Delay(40000);
                            }
                        }
                    }
                    if (number == 9)
                        break;
                }
            }
            if (number == 9)//Return to main menu
                break;

        case 2://Freestyle
            TIMER1->TCSR.CEN = 0;
            //Clean up LCD and mute buzzer
            print_lcd(0, TextEmpty);
            print_lcd(1, TextEmpty);
            print_lcd(2, TextEmpty);
            print_lcd(3, TextEmpty);
            PWM_Freq(3, note, MUTE_VOLUME);

            //Showing Freestyle Mode Menu on LCD
            print_lcd(0, "Freestyle       ");
            print_lcd(1, "1)Freestyle     ");
            print_lcd(2, "2)With Notes    "); 
            

            number = 0;//Ensuring no entrance to second option in next menu
            DrvSYS_Delay(1000000);
            while (number != 9)//Freestyle Mode is running until press 9
            {
                number = Scankey();
                switch (number)
                {

                case 1: // freestyle

                    TIMER1->TCSR.CEN = 1;// Enable Timer1
                    //Clean up LCD and resetting timer
                    print_lcd(0, TextEmpty);
                    print_lcd(1, TextEmpty);
                    print_lcd(2, TextEmpty);
                    print_lcd(3, TextEmpty);
                    h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 0, s2 = 0;
                    sprintf(TextTimer, " %d%d : %d%d : %d%d ", h2, h1, m2, m1, s2, s1);

                    print_lcd(0, TextTimer);//Showing timer on first line of LCD

                    while (number != 9)//Freestyle is running until press 9
                    {
                        number = Scankey();
                        SR04_Trigger();      // Trigger Ultrasound Sensor for 10us
                        DrvSYS_Delay(40000); // Wait 40ms for Echo to trigger interrupt

                        if (SR04A_Echo_Flag == TRUE)
                        {
                            SR04A_Echo_Flag = FALSE;

                            distance_mm = SR04A_Echo_Width * (340 / 2) / 1000;
                            distancePitch_mm = distance_mm;
                        }

                        if (SR04B_Echo_Flag == TRUE)
                        {
                            SR04B_Echo_Flag = FALSE;

                            distance_mm = SR04B_Echo_Width * (340 / 2) / 1000;
                            distanceVol_mm = distance_mm;
                        }
                        DrvSYS_Delay(10000); // 10ms from Echo to next Trigger

                        note = dist_to_note(distancePitch_mm);//Inserting note

                        duty_cycle = dist_to_vol(distanceVol_mm);//Inserting duty cycle on time

                        PWM_Freq(3, note, duty_cycle); // set PWM3 with frequency & duty cycle

                        sprintf(TextFreq + 7, "%s", getNoteName(note));
                        print_lcd(2, TextFreq);//Showing a note playing on third line of LCD

                        //Showing volume on fourth line of LCD
                        if (duty_cycle == 0)
                            print_lcd(3, "Volume: _______  ");
                        else if (duty_cycle == 99)
                            print_lcd(3, "Volume: | |      ");
                        else
                            print_lcd(3, "Volume: | | | |  ");
                    }

                    if (number == 9)
                    {
                        TIMER1->TCSR.CEN = 0;// Disable timer
                        break;
                    }
                        

                case 2://With Notes
                    TIMER1->TCSR.CEN = 0;
                    //Clean up LCD 
                    print_lcd(0, TextEmpty);
                    print_lcd(1, TextEmpty);
                    print_lcd(2, TextEmpty);
                    print_lcd(3, TextEmpty);

                    //drawing lines on LCD 
                    draw_LCD(lines);

                    while (number != 9)//With Notes is running until press 9
                    {
                        number = Scankey();
                        switch (note)
                        {
                        case C4:
                            //Clean up LCD 
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }
                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines[825 + i] = midfull2[0 + i];
                                lines[896 + i] = DO[i];
                                lines[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines);
                            break;

                        case D4:
                            //Clean up LCD 
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines[695 + i] = emptfull[0 + i];
                                lines[896 + i] = RE[i];
                                lines[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines);
                            break;

                        case E4:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }
                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines2[570 + i] = midfull2[0 + i];
                                lines2[896 + i] = MI[i];
                                lines2[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines2);
                            break;

                        case F4:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines[570 + i] = dfull[0 + i];
                                lines[896 + i] = FA[i];
                                lines[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines);
                            break;

                        case G4:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines2[438 + i] = midfull2[0 + i];
                                lines2[896 + i] = SOL[i];
                                lines2[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines2);
                            break;

                        case A4:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines[438 + i] = full[0 + i];
                                lines[896 + i] = LA[i];
                                lines[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines);
                            break;

                        case B4:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines2[312 + i] = midfull2[0 + i];
                                lines2[896 + i] = SI[i];
                                lines2[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines2);
                            break;

                        case C5:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines[312 + i] = ufull[0 + i];
                                lines[896 + i] = DO[i];
                                lines[915 + i] = empty[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines);
                            break;

                        case D4b:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines[695 + i] = emptfull[0 + i];
                                lines[896 + i] = RE[i];
                                lines[915 + i] = b[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines);
                            break;

                        case E4b:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }
                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines2[570 + i] = midfull2[0 + i];
                                lines2[896 + i] = MI[i];
                                lines2[915 + i] = b[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines2);
                            break;

                        case G4b:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines2[438 + i] = midfull2[0 + i];
                                lines2[896 + i] = SOL[i];
                                lines2[915 + i] = b[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines2);
                            break;

                        case A4b:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines[438 + i] = full[0 + i];
                                lines[896 + i] = LA[i];
                                lines[915 + i] = b[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines);
                            break;

                        case B4b:
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (i = 0; i < 16; i++){//Clean lines
                                lines[825 + i] = CleanC4[i];
                                lines[695 + i] = CleanD4[i];
                                lines2[570 + i] = CleanE4[i];
                                lines[570 + i] = CleanF4[i];
                                lines2[438 + i] = CleanG4[i];
                                lines[438 + i] = CleanA4[i];
                                lines2[312 + i] = CleanB4[i];
                                lines[312 + i] = CleanC5[i];
                            }

                            for (i = 0; i < 16; i++){//Inserting note and label to line
                                lines2[312 + i] = midfull2[0 + i];
                                lines2[896 + i] = SI[i];
                                lines2[915 + i] = b[i];
                            }

                            //drawing lines with note and label on LCD
                            draw_LCD(lines2);
                            break;
                        }

                        SR04_Trigger();      // Trigger Ultrasound Sensor for 10us
                        DrvSYS_Delay(40000); // Wait 40ms for Echo to trigger interrupt

                        if (SR04A_Echo_Flag == TRUE)
                        {
                            SR04A_Echo_Flag = FALSE;

                            distance_mm = SR04A_Echo_Width * (340 / 2) / 1000;
                            distancePitch_mm = distance_mm;
                        }

                        if (SR04B_Echo_Flag == TRUE)
                        {
                            SR04B_Echo_Flag = FALSE;

                            distance_mm = SR04B_Echo_Width * (340 / 2) / 1000;
                            distanceVol_mm = distance_mm;
                        }
                        DrvSYS_Delay(10000); // 10ms from Echo to next Trigger

                        note = dist_to_note(distancePitch_mm);//Inserting note

                        duty_cycle = dist_to_vol(distanceVol_mm);//Inserting duty cycle on time

                        PWM_Freq(3, note, duty_cycle); // set PWM3 with frequency & duty cycle
                    }
                    if (number == 9)
                        break;
                }
            }
            if (number == 9)//Return to main menu
                break;
        case 3: //Record
            TIMER1->TCSR.CEN = 0;
            //Clean up LCD and mute buzzer
            print_lcd(0, TextEmpty);
            print_lcd(1, TextEmpty);
            print_lcd(2, TextEmpty);
            print_lcd(3, TextEmpty);
            PWM_Freq(3, note, MUTE_VOLUME);   

            //Showing Record Mode Menu on LCD
            print_lcd(0, "Record          "); 
            print_lcd(1, "1)New Record    "); 
            print_lcd(2, "2)Play Record   "); 
                     
            number = 0;//Not necessary here but if we add more options in Record Mode in future 
            DrvSYS_Delay(1000000);
            while (number != 9)//Record Mode
            {
                number = Scankey();

                switch (number)
                {
                case 1: //New Record

                    TIMER1->TCSR.CEN = 1;// Enable Timer1

                    //Clean up LCD resetting timer index and flag
                    print_lcd(0, TextEmpty);
                    print_lcd(1, TextEmpty);
                    print_lcd(2, TextEmpty);
                    print_lcd(3, TextEmpty);
                    FlagRecord = 0;
                    w = 0;
                    h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 0, s2 = 0;
                    sprintf(TextTimer, " %d%d : %d%d : %d%d ", h2, h1, m2, m1, s2, s1);

                    print_lcd(0, TextTimer);//Showing timer on first line of LCD
                    while (number != 9)//New Record is running until press 9
                    {
                        number = Scankey();
                        SR04_Trigger();      // Trigger Ultrasound Sensor for 10us
                        DrvSYS_Delay(40000); // Wait 40ms for Echo to trigger interrupt

                        if (SR04A_Echo_Flag == TRUE)
                        {
                            SR04A_Echo_Flag = FALSE;

                            distance_mm = SR04A_Echo_Width * (340 / 2) / 1000;
                            distancePitch_mm = distance_mm;
                        }

                        if (SR04B_Echo_Flag == TRUE)
                        {
                            SR04B_Echo_Flag = FALSE;

                            distance_mm = SR04B_Echo_Width * (340 / 2) / 1000;
                            distanceVol_mm = distance_mm;
                        }
                        DrvSYS_Delay(10000); // 10ms from Echo to next Trigger

                        note = dist_to_note(distancePitch_mm);//Inserting note

                        duty_cycle = dist_to_vol(distanceVol_mm);//Inserting duty cycle on time

                        PWM_Freq(3, note, duty_cycle);// set PWM3 with frequency & duty cycle

                        sprintf(TextFreq + 7, "%s", getNoteName(note));
                        print_lcd(2, TextFreq);//Showing a note playing on third line of LCD

                        //Showing volume on fourth line of LCD
                        if (duty_cycle == 0)
                            print_lcd(3, "Volume: _______  ");
                        else if (duty_cycle == 99)
                            print_lcd(3, "Volume: | |      ");
                        else
                            print_lcd(3, "Volume: | | | |  ");

                        //If flag down and silence then Recording note
                        if (duty_cycle == MUTE_VOLUME && FlagRecord == 0)
                        {
                            FlagRecord = 1;

                            record[w] = note;
                            w++;
                            if (w == 50)
                            {
                                TIMER1->TCSR.CEN = 0;// Disable Timer1
                                print_lcd(0, TextEmpty);
                                print_lcd(1, TextEmpty);
                                print_lcd(2, TextEmpty);
                                print_lcd(3, TextEmpty);
                                print_lcd(0, "FullMemory");
                                print_lcd(1, "Record press 1 ");
                                print_lcd(2, "Exit press 9 ");
                                PWM_Freq(3, 0, 0);
                                while (number != 9 && number != 1)
                                {
                                    DrvSYS_Delay(100000);
                                    number = Scankey();
                                }

                                if (number == 1)
                                {
                                    DrvSYS_Delay(100000);
                                    number = 0;
                                    DrvSYS_Delay(100000);
                                    h1 = 0, h2 = 0, m1 = 0, m2 = 0, s1 = 0, s2 = 0;
                                    sprintf(TextTimer, " %d%d : %d%d : %d%d ", h2, h1, m2, m1, s2, s1);
                                    TIMER1->TCSR.CEN = 1;// Enable Timer1
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
																	  print_lcd(2, TextEmpty);
                                    print_lcd(0, TextTimer);
																	  w=0;
                                }
                            }
                        }
                            
                        //If flag up and unmute then flag goes down for ready to record another note
                        if (duty_cycle != MUTE_VOLUME && FlagRecord == 1)
                        {
                            FlagRecord = 0;
                        }                        
                 
                    }
                    if (number == 9)
                    {
                        TIMER1->TCSR.CEN = 0;// Disable timer
                        break;
                    }
                        

                case 2: // Play Record
                    TIMER1->TCSR.CEN = 0;
                    //Clean up LCD
                    print_lcd(0, TextEmpty);
                    print_lcd(1, TextEmpty);
                    print_lcd(2, TextEmpty);
                    print_lcd(3, TextEmpty);

                    while (number != 9)
                    {
                        number = Scankey();
                        if (record[0] == 0)//Checking if recording was made
                        {

                            print_lcd(0, "EMPTY");
                        }
                        else
                        {
                            //Clean up LCD
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            for (a = 0; a < w; a++)
                            {

                                PWM_Freq(3, (record[a]), MAX_VOLUME);// set PWM3 with record's frequency & duty cycle


                                DrvSYS_Delay(100000 / 3);
                                number = Scankey();
                                if (number == 9)
                                    break;
                                DrvSYS_Delay(100000 / 3);
                                number = Scankey();
                                if (number == 9)
                                    break;
                                DrvSYS_Delay(100000 / 3);
                                number = Scankey();
                                if (number == 9)
                                    break;

                                switch (record[a])
                                {
                                case C4:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }
                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines[825 + i] = midfull2[0 + i];
                                        lines[896 + i] = DO[i];
                                        lines[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines);
                                    break;

                                case D4:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines[695 + i] = emptfull[0 + i];
                                        lines[896 + i] = RE[i];
                                        lines[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines);
                                    break;

                                case E4:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }
                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines2[570 + i] = midfull2[0 + i];
                                        lines2[896 + i] = MI[i];
                                        lines2[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines2);
                                    break;

                                case F4:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines[570 + i] = dfull[0 + i];
                                        lines[896 + i] = FA[i];
                                        lines[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines);
                                    break;

                                case G4:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines2[438 + i] = midfull2[0 + i];
                                        lines2[896 + i] = SOL[i];
                                        lines2[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines2);
                                    break;

                                case A4:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines[438 + i] = full[0 + i];
                                        lines[896 + i] = LA[i];
                                        lines[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines);
                                    break;

                                case B4:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines2[312 + i] = midfull2[0 + i];
                                        lines2[896 + i] = SI[i];
                                        lines2[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines2);
                                    break;

                                case C5:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines[312 + i] = ufull[0 + i];
                                        lines[896 + i] = DO[i];
                                        lines[915 + i] = empty[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines);
                                    break;

                                case D4b:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines[695 + i] = emptfull[0 + i];
                                        lines[896 + i] = RE[i];
                                        lines[915 + i] = b[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines);
                                    break;

                                case E4b:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }
                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines2[570 + i] = midfull2[0 + i];
                                        lines2[896 + i] = MI[i];
                                        lines2[915 + i] = b[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines2);
                                    break;

                                case G4b:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines2[438 + i] = midfull2[0 + i];
                                        lines2[896 + i] = SOL[i];
                                        lines2[915 + i] = b[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines2);
                                    break;

                                case A4b:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines[438 + i] = full[0 + i];
                                        lines[896 + i] = LA[i];
                                        lines[915 + i] = b[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines);
                                    break;

                                case B4b:
                                    //Clean up LCD 
                                    print_lcd(0, TextEmpty);
                                    print_lcd(1, TextEmpty);
                                    print_lcd(2, TextEmpty);
                                    print_lcd(3, TextEmpty);

                                    for (i = 0; i < 16; i++){//Clean lines
                                        lines[825 + i] = CleanC4[i];
                                        lines[695 + i] = CleanD4[i];
                                        lines2[570 + i] = CleanE4[i];
                                        lines[570 + i] = CleanF4[i];
                                        lines2[438 + i] = CleanG4[i];
                                        lines[438 + i] = CleanA4[i];
                                        lines2[312 + i] = CleanB4[i];
                                        lines[312 + i] = CleanC5[i];
                                    }

                                    for (i = 0; i < 16; i++){//Inserting note and label to line
                                        lines2[312 + i] = midfull2[0 + i];
                                        lines2[896 + i] = SI[i];
                                        lines2[915 + i] = b[i];
                                    }

                                    //drawing lines with note and label on LCD
                                    draw_LCD(lines2);
                                    break;
                                }
                            }
                            //Clean up LCD 
                            print_lcd(0, TextEmpty);
                            print_lcd(1, TextEmpty);
                            print_lcd(2, TextEmpty);
                            print_lcd(3, TextEmpty);

                            //Showing this options on LCD untill press 1 or 9
                            print_lcd(0, "Again? press 1 ");
                            print_lcd(1, "Exit press 9 ");
                            PWM_Freq(3, note, MUTE_VOLUME);
                            while (number != 9 && number != 1)
                            {
                                DrvSYS_Delay(100000);//Delay to input pressing
                                number = Scankey();
                            }

                        }
                    }
                    if (number == 9)
                        break;
                }
            }
        }
        DrvSYS_Delay(5000);	
    }
}



