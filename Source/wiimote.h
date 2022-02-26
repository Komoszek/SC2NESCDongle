/*
 * Copyright (c) 2011 Peter Brinkmann (peter.brinkmann@gmail.com)
 * http://gitorious.org/randomstuff/arduino-wiimote
 * For information on usage and redistribution, and for a DISCLAIMER OF ALL
 * WARRANTIES, see the file, "LICENSE_wiimote.txt," in this distribution.
 * 
 * Adjustments for CC2541 were made by Łukasz Komoszyński
 */

#ifndef __WIIMOTE_H__
#define __WIIMOTE_H__

#include "wm_crypto.h"
#include "hal_uart.h"

// Include Name definitions of individual bits and bit-fields in the CC254x device registers.
#include "ioCC254x_bitdef.h"

// Identification sequence for Classic Controller
const static uint8 idbuf[] = { 0x00, 0x00, 0xa4, 0x20, 0x01, 0x01 };

// Classic Controller calibration data
const static uint8 calbuf[] = { 0xFC, 0x04, 0x7E, 0xFC, 0x04, 0x7E, 0xFC, 0x04, 0x7E,
		0xFC, 0x04, 0x7E, 0x00, 0x00, 0x55, 0xAA }; // Checksum will be calculated later...

static uint8 outbuf[8] = {0};
static uint8 state = 0;
static uint8 crypt_setup_done = 0;

/*
 * Callback for streaming uint8s from the Arduino to the Wiimote.
 * If this function pointer is not null, then it will be called
 * right _after_ the current output bufferIn has been sent to the
 * Wiimote, and the implementing function has the choice of either
 * overwriting the given bufferIn and returning it or returning
 * another bufferIn that contains the next six uint8s to be sent.
 *
 * The return value of this function becomes the new output bufferIn.
 * The idea is much like the one behind wiimote_swap_bufferIns.  You
 * probably want to use only one of them; either wiimote_stream
 * for streaming data, or wiimote_swap_bufferIns for more sporadic
 * updates.
 *
 * There is one exception to this rule: Since this function is
 * called after the current output bufferIn has been sent, the very
 * first bufferIn will be all zero unless you initialize it with
 * wiimote_set_uint8 or wiimote_swap_bufferIns.  (This approach may
 * seem awkward, but it minimizes delays in I2C communication, and
 * in most cases an all-zero initial bufferIn is not a problem.)
 */
void (*wiimote_stream)(uint8 *buffer) = NULL;

/*
 * Global array for storing and accessing register values written
 * by the console.  Those are the 256 registers corresponding to
 * the Extension, starting at 04a40000.
 */
uint8 wiimote_registers[0x100];

/*
 * Start Console <-> Extension communication encryption,
 * if requested by the application.
 */

static void setup_encryption() {
    uint8 i;

    for(i = 0x40; i <= 0x49; i++){
        wm_rand[9 - (i - 0x40)] = wiimote_registers[i];
    }

    for(i = 0x4A; i <= 0x4F; i++){
        wm_key[5 - (i - 0x4A)] = wiimote_registers[i];
    }

    wm_gentabs();

    crypt_setup_done = 1;
}

/*
 * Takes joystick, and button values and encodes them
 * into a buffer.
 *
 * Classic Controller
 *
 * Buffer encoding details:
 * https://github.com/ClusterM/nes2wii/blob/master/main.c
 */


void wiimote_write_buffer(uint8 *buffer, uint8 lx, uint8 rx, uint8 ly, uint8 ry, uint8 lt, uint8 rt, uint8 bg1, uint8 bg2) {
  buffer[0] = lx;
  buffer[1] = rx;
  buffer[2] = ly;
  buffer[3] = ry;
  buffer[4] = lt;
  buffer[5] = rt;

  buffer[6] = bg1;
  buffer[7] = bg2;

  buffer[6] = ~buffer[6];
  buffer[7] = ~buffer[7];
}



/***********************************************************************************
* CONSTANTS
*/

// Define size of buffer and number of bytes to send

#define SLAVE_ADDRESS 0x52    // 7-bit addressing
#define MAX_I2C_MSG_SIZE 21

/***********************************************************************************
* LOCAL VARIABLES
*/

static uint8 bufferIn[2];
static uint8 bufferInIndex = 0;

static uint8 bufferOut[8];
static uint8 bufferOutIndex = 0;


/***********************************************************************************
* LOCAL FUNCTIONS
*/

static void send_data(uint8 * data, uint8 size, uint8 addr){
    uint8 i;

    if (wiimote_registers[0xF0] == 0xAA && crypt_setup_done){
        for(i = 0; i < size; i++) {
            bufferOut[i] = (data[i] - wm_ft[(addr + i) % 8]) ^ wm_sb[(addr + i) % 8];
        }
    } else {
        for(i = 0; i < size; i++) {
            bufferOut[i] = data[i];
        }
    }

    for(i = size; i < 8; i++)
      bufferOut[i] = 0;

    bufferOutIndex = 0;

}

/***********************************************************************************
* @fn          I2C_ISR
*
* @brief       Function which sends the next I2C data byte.
*
* @param       none
*
* @return      0
*/

#pragma vector = I2C_VECTOR
__interrupt void I2C_ISR(void)
{
    // Clear I2C CPU interrupt flag.
    P2IF = 0;

    // If own SLA+R has been received or a Data byte has been transmitted,
    // and ACK has been returned ...
    if (I2CSTAT == 0xA8 || I2CSTAT == 0xB8){
        if(I2CSTAT == 0xA8){
          bufferOutIndex = 0;
        }
        // Load Data byte and increment index.
        if (bufferOutIndex >= 8){
            I2CDATA = 0x00;
        } else {
            I2CDATA = bufferOut[bufferOutIndex++];
        }

    }
    // If previously addressed with own Slave address, and a Data byte has been
    // received and acknowledged ...
    else if (I2CSTAT == 0x80)
    {
        // Read the I2C byte.
        bufferIn[bufferInIndex++] = I2CDATA;
    }

    // If a Stop condition is detected ...
    else if (I2CSTAT == 0xA0)
    {
        if (bufferInIndex){
            uint8 crypt_keys_received = 0;

            if (bufferInIndex == 1) {
                state = bufferIn[0];
                static uint8 last_state = 0xFF;
            static uint8 offset = 0;

                switch (state) {

                case 0x00:
                    send_data(outbuf, 8, 0x00);

                    if (wiimote_stream)
                        wiimote_stream(outbuf);

                    break;
                case 0xFA:
                    send_data(wiimote_registers + state, 6, state);
                    break;
                default:
                    if (last_state == state){
                        offset += 8;
                    } else {
                        last_state = state;
                        offset = 0;
                    }

                    send_data(wiimote_registers + state + offset, 8, state + offset);
                }

            } else if (bufferInIndex > 1){
                uint8 addr = bufferIn[0];
                uint8 iter = addr;
                uint8 i;

                for(i = 1; i < bufferInIndex; i++){
                    uint8 data = bufferIn[i];

                    // Wii is trying to disable encryption...
                    if (addr == 0xF0 && data == 0x55)
                        crypt_setup_done = 0;

                    if (wiimote_registers[0xF0] == 0xAA && crypt_setup_done){
                        // Decrypt
                        wiimote_registers[iter] = (data ^ wm_sb[iter % 8]) + wm_ft[iter
                                        % 8];
                        iter++;
                    } else {
                      wiimote_registers[iter] = data;

                      if (iter == 0xFE && data == 0x03)
                          wiimote_stream(outbuf);

                      iter++;
                    }

                    // Check if last crypt key setup uint8 was received...
                    if (iter == 0x50){
                        crypt_keys_received = 1;
                    }

                }
            }

            // Setup encryption
            if (crypt_keys_received) {
                    setup_encryption();
            }
            // Reset bufferIn index.

            bufferInIndex = 0;

        }
    }

    // I2CCFG.SI flag must be cleared by software at the end of the ISR.
    I2CCFG &= ~I2CCFG_SI;
}


/***********************************************************************************
* @fn          main
*
* @brief       Receive data from a Master using I2C in Slave mode
*
* @param       void
*
* @return      0
*/
void i2c_init(void)
{
    /****************************************************************************
    * Clock setup
    */

    // Set system clock source to HS XOSC, with no pre-scaling.
    CLKCONCMD = (CLKCONCMD & ~(CLKCON_OSC | CLKCON_CLKSPD)) | CLKCON_CLKSPD_32M;
    while (CLKCONSTA & CLKCON_OSC);   // Wait until clock source has changed

    // Note the 32 kHz RCOSC starts calibrating, if not disabled.
    /***************************************************************************
    * Setup I/O ports
    *
    * CC2541 has dedicated I2C ports
    * I2C SCL:   Pin 2   (Debug Connector P18_5)
    * I2C SDA:   Pin 3   (Debug Connector P18_3)
    */

    // Enable I2C on CC2541.
    I2CWC &= ~0x80;

    /***************************************************************************
    * Setup interrupt
    */

    // Clear I2C CPU interrupt flag.
    P2IF = 0;

     // Enable global interrupts.
    EA = 1;

    // Enable interrupt from I2C by setting [IEN2.I2CIE = 1].
    IEN2 |= IEN2_P2IE;

    /***************************************************************************
    * Configure I2C
    */

    // Set device address, with General-call address acknowledge.
    I2CADDR = (SLAVE_ADDRESS << 1) | I2CADDR_GC;

    // Enable the I2C module, the slave is clocked by the Master.
    // Enable Assert Acknowledge (AA bit).
    I2CCFG |= I2CCFG_ENS1 | I2CCFG_AA;
}


void wiimote_init() {
    uint8 i;

    osal_memset(wiimote_registers, 0xFF, 0x100);

    // Set extension id on registers
    for(i = 0; i < 6; i++){
        wiimote_registers[i + 0xFA] = idbuf[i];
    }

    // Set calibration data on registers
    for(i = 0x20; i <= 0x2F; i++){
        wiimote_registers[i] = calbuf[i - 0x20]; // 0x20
        wiimote_registers[i + 0x10] = calbuf[i - 0x20]; // 0x30
    }

    // Initialize outbuf, otherwise, "Up+Right locked" bug...
    wiimote_write_buffer(outbuf, calbuf[2]>>2, calbuf[8]>>3, calbuf[5]>>2, calbuf[11]>>3, 0, 0, 0, 0);

    // Encryption disabled by default
    wiimote_registers[0xF0] = 0x55;
    wiimote_registers[0xFB] = 0x00;

    // Join I2C bus
    i2c_init();
}


extern void wiimote_write_buffer(uint8 *buffer, uint8 lx, uint8 rx, uint8 ly, uint8 ry,
		uint8 lt, uint8 rt, uint8 bg1, uint8 bg2);

#endif
