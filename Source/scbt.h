/*
 * Based on work of kozec - https://github.com/kozec/sc-controller
 */

#ifndef SCBT_H
#define SCBT_H

#define LONG_PACKET 0x80
#define PACKET_SIZE 19
#define ZONE_CONST 10922
#define UART_PORT       HAL_UART_PORT_1

#define UART_RX_TEMP_BUF_SIZE 100

#include "hal_uart.h"


enum BtInPacketType {
    BUTTON   = 0x0010,
    TRIGGERS = 0x0020,
    STICK    = 0x0080,
    LPAD     = 0x0100,
    RPAD     = 0x0200,
    GYRO     = 0x1800,
    PING     = 0x5000,
};

enum SCButtons {
    SCB_RPADTOUCH   = 0x10000000,
    SCB_LPADTOUCH   = 0x8000000,
    SCB_RPAD        = 0x4000000,
    SCB_LPAD        = 0x2000000, // # Same for stick but without LPadTouch
    SCB_STICKPRESS  = 0x1, // # generated internally, not sent by controller
    SCB_RGRIP       = 0x1000000,
    SCB_LGRIP       = 0x800000,
    SCB_START       = 0x400000,
    SCB_C           = 0x200000,
    SCB_BACK        = 0x100000,
    SCB_A           = 0x8000,
    SCB_X           = 0x4000,
    SCB_B           = 0x2000,
    SCB_Y           = 0x1000,
    SCB_LB          = 0x800,
    SCB_RB          = 0x400,
    SCB_LT          = 0x200,
    SCB_RT          = 0x100,
};

struct SCByBtControllerInput {
    uint16 type;
    uint32 buttons;
    uint8 ltrig;
    uint8 rtrig;
    int16 stick_x;
    int16 stick_y;
    int16 lpad_x;
    int16 lpad_y;
    int16 rpad_x;
    int16 rpad_y;
    int16 gpitch;
    int16 groll;
    int16 gyaw;
    int16 q1;
    int16 q2;
    int16 q3;
    int16 q4;
};

struct wiicontrolls{
  union{
    struct {
      uint8 c: 1;
      uint8 brt: 1;
      uint8 bp: 1;
      uint8 bhome: 1;
      uint8 bm: 1;
      uint8 blt: 1;
      uint8 bdd: 1;
      uint8 bdr: 1;

      uint8 bdu: 1;
      uint8 bdl: 1;
      uint8 bzr: 1;
      uint8 bx: 1;
      uint8 ba: 1;
      uint8 by: 1;
      uint8 bb: 1;
      uint8 bzl: 1;

      uint8 lpad: 1; // 0x1
      uint8 rpad: 1; // 0x2
      uint8 lpadtouch: 1; // 0x4
      uint8 rpadtouch: 1; // 0x8
      uint8 stickclicked: 1;
    };

    uint8 raw[3];
  };

  uint8 ltrig;
  uint8 rtrig;
  uint8 lx;
  uint8 ly;
  uint8 rx;
  uint8 ry;

};

uint8 BUTTON_MAP[][2] = {
  {0, 0x2},
  {0, 0x20},
  {1, 0x4},
  {1, 0x80},
  {1, 0x20}, //y -> y
  {1, 0x10}, //b -> a
  {1, 0x8}, // x -> x
  {1, 0x40}, // a -> b
  {3, 0},
  {3, 0},
  {3, 0},
  {3, 0},
  {0, 0x10},
  {0, 0x8},
  {0, 0x4},
  {1, 0x10},
  {1, 0x40},
  {2, 0x1}, // lpad
  {2, 0x2}, // rpad
  {2, 0x4}, // lpadtouch
  {2, 0x8}, // rpadtouch
  {3, 0},
  {2, 0x10} // lpad - lpadtouch?
};

void serial_parser( uint8 port, uint8 events )
{
    uint8 bytesLeft = Hal_UART_RxBufLen(UART_PORT);
    //Allocate te packet
    uint8 *temp_buf;
    uint16 res, bytesRead;

    //Get the buffer
    // config loading

    if(Hal_UART_RxBufLen(UART_PORT)){

      temp_buf = osal_mem_alloc( UART_RX_TEMP_BUF_SIZE );


      HalUARTWrite(UART_PORT, "Rx: ", 4);
      do {
      bytesLeft = Hal_UART_RxBufLen(UART_PORT);
      if( bytesLeft + bytesRead > UART_RX_TEMP_BUF_SIZE)
        break;

      res = HalUARTRead( UART_PORT, temp_buf, bytesLeft );

      if( res ){
        //HalUARTWrite(UART_PORT, temp_buf, res);
        bytesRead += res;
      }

    } while(bytesLeft);


    // do something with it

    HalUARTWrite(UART_PORT, "\n", 1);
    osal_mem_free(temp_buf);

    }
}


struct SCByBtC {
    struct SCByBtControllerInput state;
    //struct SCByBtControllerInput old_state;
};

typedef struct SCByBtC* SCByBtCPtr;

#define BT_BUTTONS_BITS 23

const uint32 BT_BUTTONS[] = {
    // Bit to SCButton
    SCB_RT,         // 00
    SCB_LT,         // 01
    SCB_RB,         // 02
    SCB_LB,         // 03
    SCB_Y,          // 04
    SCB_B,          // 05
    SCB_X,          // 06
    SCB_A,          // 07
    0,              // 08 - dpad, ignored
    0,            // 09 - dpad, ignored
    0,            // 10 - dpad, ignored
    0,            // 11 - dpad, ignored
    SCB_BACK,       // 12
    SCB_C,          // 13
    SCB_START,        // 14
    SCB_LGRIP,        // 15
    SCB_RGRIP,        // 16
    SCB_LPAD,       // 17
    SCB_RPAD,       // 18
    SCB_LPADTOUCH,      // 19
    SCB_RPADTOUCH,      // 20
    0,            // 21 - nothing
    SCB_STICKPRESS,     // 22
};

uint8 alfa(int16 x, int16 y) {
    int8 res = 0;

    if (x > ZONE_CONST){
      res |= 0x1;
    } else if (x < -ZONE_CONST){
      res |= 0x2;
    }

    if (y > ZONE_CONST){
      res |= 0x4;
    } else if(y < -ZONE_CONST){
      res |= 0x8;
    }

    return res;
}

long map (long x, long in_min, long in_max, long out_min, long out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/** Returns 1 if state has changed, 2 on read error */
uint8 read_input(SCByBtCPtr ptr, uint8 * buffer) {
    struct SCByBtControllerInput* state = &(ptr->state);
    //struct SCByBtControllerInput* old_state = &(ptr->old_state);

    uint8 rv = 0;
    uint16 type = *((uint16*)(buffer + 1));
    uint8* data = &buffer[3];
    if (type & PING) // PING packet does nothing
        return 0;

    if (type & BUTTON){
        if (rv == 0){ //*old_state = *state;
            state->type = type; rv = 1; }

        state->buttons = *((uint32*)data);
        data += 3;
    }
    if (type & TRIGGERS) {
        if (rv == 0){ //*old_state = *state;
            state->type = type; rv = 1; }
        state->ltrig = *(((uint8*)data) + 0);
        state->rtrig = *(((uint8*)data) + 1);

        data += 2;
    }
    if (type & STICK){
        if (rv == 0){ //*old_state = *state;
            state->type = type; rv = 1; }
        state->stick_x = *(((int16*)data) + 0);
        state->stick_y = *(((int16*)data) + 1);
        data += 4;
    }
    if (type & LPAD){
        if (rv == 0){ //*old_state = *state;
            state->type = type; rv = 1; }
        state->lpad_x = *(((int16*)data) + 0);
        state->lpad_y = *(((int16*)data) + 1);
        data += 4;
    }

    if (type & RPAD){
        if (rv == 0){ //*old_state = *state;
            state->type = type; rv = 1; }
        state->rpad_x = *(((int16*)data) + 0);
        state->rpad_y = *(((int16*)data) + 1);

        data += 4;
    }
    if (type & GYRO){
        if (rv == 0){ //*old_state = *state;
            state->type = type; rv = 1; }
        state->gpitch = *(((int16*)data) + 0);
        state->groll = *(((int16*)data) + 1);
        state->gyaw = *(((int16*)data) + 2);
        state->q1 = *(((int16*)data) + 3);
        state->q2 = *(((int16*)data) + 4);
        state->q3 = *(((int16*)data) + 5);
        state->q4 = *(((int16*)data) + 6);
        data += 14;
    }

    return rv;
}

void translate_input(SCByBtCPtr ptr, struct wiicontrolls * wc){
    uint8 bit, sign;
    struct SCByBtControllerInput* state = &(ptr->state);
    uint32 buttons = state->buttons;

    osal_memset(wc, 0, sizeof(struct wiicontrolls));

    // buttons - button to button map

    for(bit = 0; bit < BT_BUTTONS_BITS; bit++){
      if ( (buttons & 1) && BUTTON_MAP[bit][0] != 3){
          wc->raw[BUTTON_MAP[bit][0]] |= BUTTON_MAP[bit][1];
      }
        buttons >>= 1;
    }

    // triggers - triggers or no triggers, analog value to button
    wc->ltrig = state->ltrig;
    wc->rtrig = state->rtrig;



    // stick, lpad and rpad - analog to analog or analog to button

    // stick
    wc->lx = map(state->stick_x, -32767, 32767, 0, 255);
    wc->ly = map(state->stick_y, -32767, 32767, 0, 255);

    // lpad
    sign = alfa( state->lpad_x, state->lpad_y);

    if (sign & 0x1)
      wc->raw[0] |= 0x80; // r
    if (sign & 0x2)
      wc->raw[1] |= 0x2; // l
    if (sign & 0x4)
      wc->raw[1] |= 0x1; // u
    if (sign & 0x8)
      wc->raw[0] |= 0x40; // d

    // rpad
    wc->rx = map(state->rpad_x, -32767, 32767, 0, 255);
    wc->ry = map(state->rpad_y, -32767, 32767, 0, 255);
}

#endif
