/*
 * protocol.h - part of USBasp
 *
 * Description....: Shared state and common helpers of the USBasp vendor
 *                  request protocol (ISP flash/eeprom transfers). Kept in a
 *                  separate module so that main.c only contains the request
 *                  dispatcher and the V-USB chunk callbacks.
 * Licence........: GNU GPL v2 (see Readme.txt)
 */

#ifndef __protocol_h_included__
#define __protocol_h_included__

#include <stdint.h>

/* ------------------------------------------------------------------ */
/* Protocol state (defined in protocol.c)                              */
/*                                                                     */
/* prog_address is 16-bit on purpose: this is an 8-bit CPU and the     */
/* USBasp wire protocol never carries more than 24 address bits, of    */
/* which only the low 16 are used for per-byte addressing. Bit 16      */
/* (A17, the "extended address" bit for >64K flash chips) lives        */
/* separately in prog_address_high and is consumed by SETLONGADDRESS   */
/* only -- no 32-bit arithmetic anywhere in the transfer loops!        */
/* Do not change the layout of prog_address: SETUP bytes are written   */
/* straight into its two halves (little-endian).                       */
/* ------------------------------------------------------------------ */
extern uint16_t prog_address;         /* current transfer address       */
extern uint8_t  prog_address_high;    /* bits 16..23, set by SETLONGADDRESS */
extern volatile uint8_t prog_address_newmode; /* 3-byte addressing active */
extern uint16_t prog_pagesize;        /* flash page size in BYTES, 0 = byte-wise */
extern uint16_t prog_pagecounter;     /* bytes left until page flush    */
extern uint16_t prog_nbytes;          /* bytes left in current USB chunk */

/* Number of bytes still pending in the current transfer. */
#define protoBytesLeft() prog_nbytes

/* ------------------------------------------------------------------ */
/* Helpers                                                             */
/* ------------------------------------------------------------------ */

/* Extract wLength (this packet's payload size) from a SETUP packet. */
uint16_t protoPacketLen(const uint8_t data[8]);

/* Load prog_address from SETUP bytes [2],[3] and set prog_nbytes from
 * wLength; initialise paged-write bookkeeping. Only meaningful for
 * new_state == PROG_STATE_WRITEFLASH: pagesize is taken from wIndex and
 * must be a power of two, otherwise the transfer silently falls back to
 * safe byte-wise mode. */
void protoSetupTransfer(const uint8_t data[8], uint8_t new_state);

/* Enter a plain streaming state (SPI/I2C/microwire): no address or
 * paging involved, just "prog_nbytes bytes follow in this packet". */
void protoSetupStream(uint8_t new_state, uint16_t nbytes);

#endif /* __protocol_h_included__ */
