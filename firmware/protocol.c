/*
 * protocol.c - part of USBasp
 *
 * Description....: Shared state and common helpers of the USBasp vendor
 *                  request protocol. See protocol.h for details.
 * Licence........: GNU GPL v2 (see Readme.txt)
 */

#include "protocol.h"
#include "usbasp.h"
#include "isp.h"

/* ------------------------------------------------------------------ */
/* Protocol state                                                      */
/* ------------------------------------------------------------------ */

uint16_t prog_address = 0;            /* 16-bit: no 32-bit math on ATmega8 */
uint8_t  prog_address_high = 0;       /* raw bits 16..23 from SETLONGADDRESS */
volatile uint8_t prog_address_newmode = 0;

uint16_t prog_pagesize = 0;
uint16_t prog_pagecounter = 0;
uint16_t prog_nbytes = 0;

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

static uint8_t is_power_of_two16(uint16_t v) {
    return v != 0 && (v & (v - 1)) == 0;
}

uint16_t protoPacketLen(const uint8_t data[8]) {
    return ((uint16_t)data[7] << 8) | data[6];
}

void protoSetupTransfer(const uint8_t data[8], uint8_t new_state) {
    uint8_t *p_addr = (uint8_t *)&prog_address;

    /* Direct little-endian byte load: byte[0]=low, byte[1]=high. */
    p_addr[0] = data[2];
    p_addr[1] = data[3];

    /* In "new mode" avrdude sends the full transaction size in wIndex;
     * keep it as prog_nbytes only via wLength below (per-chunk count). */
    prog_nbytes = protoPacketLen(data);

    if (new_state == PROG_STATE_WRITEFLASH) {
        prog_pagesize = ((uint16_t)data[5] << 8) | data[4]; /* avrdude puts pagesize into wIndex */

        if (is_power_of_two16(prog_pagesize)) {
            uint16_t low_addr = ((uint16_t)data[3] << 8) | data[2];
            prog_pagecounter = prog_pagesize - (low_addr & (prog_pagesize - 1));
        } else {
            /* not a power of two (or zero): safe byte-wise mode */
            prog_pagesize = 0;
            prog_pagecounter = 0;
        }
    }
}

void protoSetupStream(uint8_t new_state, uint16_t nbytes) {
    prog_pagesize = 0;
    prog_pagecounter = 0;
    prog_nbytes = nbytes;
    prog_state = new_state;
}
