/*
 * USBasp - USB in-circuit programmer for Atmel AVR controllers
 *
 * Thomas Fischl <tfischl@gmx.de>
 *
 * License........: GNU GPL v2 (see Readme.txt)
 * Target.........: ATMega8 at 12 MHz
 * Creation Date..: 2005-02-20
 *
 * This module is the protocol dispatcher: it maps USB vendor requests
 * onto the ISP / TPI / SPI / I2C / microwire back ends and implements
 * the V-USB chunked-transfer callbacks (usbFunctionRead/Write).
 *
 * NOTE on addressing: this is an 8-bit CPU. prog_address is 16-bit;
 * the extended-address bit A17 (>64K flash chips) travels separately
 * in prog_address_high and is pushed to the target with the ISP
 * "load extended address" command (0x4D) only when its page changes.
 * There is deliberately NO 32-bit arithmetic in the transfer loops.
 *
 * PC2 SCK speed option.
 * GND  -> slow (8khz SCK),
 * open -> software set speed (default is 375kHz SCK)
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include "usbasp.h"
#include "usbdrv.h"
#include "isp.h"
#include "clock.h"
#include "tpi.h"
#include "tpi_defs.h"
#include "i2c.h"
#include "microwire.h"
#include "protocol.h"

/* ---- transfer state shared with isp.c via isp.h ---- */
uint8_t prog_state = PROG_STATE_IDLE;
uint8_t prog_sck = USBASP_ISP_SCK_AUTO;
static uint8_t requested_sck = USBASP_ISP_SCK_AUTO;   /* raw host request, before resolve */

/* ---- per-interface flags not covered by protocol.c ---- */
static uint8_t spi_cs_hi = 1;         /* raise CS again at end of SPI transaction */
static uint8_t mw_cs_lo = 1;          /* drop CS (RST) at end of microwire transaction */
static uint8_t mw_bits_remaining = 0; /* bits left in current microwire chunk */
uint8_t user_speed_requested = 0;     /* flag: user explicitly pinned SCK speed */
static uint8_t i2c_stop_aw = 0;       /* I2C flow control: send STOP after the block? */
static uint8_t ext_page = 0;          /* last 8-byte flash page loaded into the chip */

static uint8_t replyBuffer[8] = {0};

/* Cross-module symbols referenced below. */
extern uint8_t sck_sw_delay;
extern void tpi_init(void);
extern void tpi_send_byte(uint8_t data);
extern uint8_t tpi_recv_byte(void);
extern void tpi_pr_update(uint16_t addr);
extern uint8_t last_success_speed;
extern uint8_t isp_hiaddr;
extern uint8_t tpi_parity_err;

/* Optimized min() instead of a macro: clamp an u16 count into an u8 chunk size. */
static inline uint8_t min_u8_u16(uint8_t a, uint16_t b) {
    return (b < a) ? (uint8_t)b : a;
}

/* Bytes left in the current transfer (protocol.h declares protoBytesLeft()). */
#define BYTES_LEFT() protoBytesLeft()

/* Abort the current multi-page/streaming transfer. */
static inline void endTransfer(void) {
    prog_state = PROG_STATE_IDLE;
    prog_nbytes = 0;
}

/* --------------------------------------------------------------------------------
 * The USBasp protocol carries metadata in the SETUP packet fields:
 *   data[4:5] -> wIndex:   FULL expected transaction size (or pagesize for
 *                          WRITEFLASH, or 0 for compatibility).
 *   data[6:7] -> wLength:  payload size of THIS particular USB packet (max 255 B).
 * Address and page bookkeeping lives in protocol.c; here we only enter the state.
 * -------------------------------------------------------------------------------- */
static void setupTransfer(const uint8_t *data, uint8_t new_state) {
    protoSetupTransfer(data, new_state);
    prog_state = new_state;
}

/* SPI transaction: data[2] asks us to raise CS when the block is done. */
static void setupSPIState(uint8_t mode, const uint8_t *data) {
    spi_cs_hi = data[2]; // read the flag first...
    CS_LOW();            // ...then poke the pin
    protoSetupStream(mode, protoPacketLen(data));
}

/* Plain streaming states (I2C): payload size comes straight from wLength. */
static void setupStreamState(uint8_t mode, const uint8_t *data) {
    protoSetupStream(mode, protoPacketLen(data));
}

/* ================================ TPI helpers ================================== */

static uint8_t tpi_waitbusy(void) {
    uint8_t t, csr;
    for (t = 0; t < 50; t++) {              /* 50 iterations, ~16 ms with clockWait */
        tpi_send_byte(TPI_OP_SIN(NVMCSR));
        csr = tpi_recv_byte();
        if (tpi_parity_err) return 1;
        if (!(csr & NVMCSR_BSY)) return 0;
        clockWait(1);
    }
    return 2;
}

void tpi_read_block_c(uint16_t addr, uint8_t *buf, uint16_t len) {
    tpi_pr_update(addr);
    while (len--) {
        tpi_send_byte(TPI_OP_SLD_INC);
        *buf = tpi_recv_byte();
        if (tpi_parity_err) {               /* don't ship corrupt bytes to the host silently */
            prog_state = PROG_STATE_IDLE;
            prog_nbytes = 0;
            return;
        }
        buf++;
    }
}

void tpi_write_block_c(uint16_t addr, uint8_t *buf, uint16_t len) {
    tpi_pr_update(addr);
    while (len--) {
        tpi_send_byte(TPI_OP_SOUT(NVMCMD));
        tpi_send_byte(NVMCMD_WORD_WRITE);
        tpi_send_byte(TPI_OP_SST_INC);
        tpi_send_byte(*buf++);
        if (tpi_waitbusy() != 0) {
            prog_state = PROG_STATE_IDLE;
            prog_nbytes = 0;                /* short transfer -> avrdude sees the error */
            return;
        }
    }
}

/* ============================ vendor request dispatch ========================== */
usbMsgLen_t usbFunctionSetup(uint8_t data[8]) {

    usbMsgLen_t len = 0;

    if (data[1] == USBASP_FUNC_CONNECT) {
        ispSetSCKOption(prog_sck);          /* speed must be set before pin init */
        ledRedOn();
        ispConnect();                       /* resets newmode/high byte/hiaddr */
        ext_page = 0xFF;                    /* force reload of page address */

    } else if (data[1] == USBASP_FUNC_DISCONNECT) {
        ispDisconnect();
        ledGreenOff();

    /* ------------------------------- plain ISP --------------------------------- */

    } else if (data[1] == USBASP_FUNC_TRANSMIT) {
        replyBuffer[0] = ispTransmit(data[2]);
        replyBuffer[1] = ispTransmit(data[3]);
        replyBuffer[2] = ispTransmit(data[4]);
        replyBuffer[3] = ispTransmit(data[5]);
        len = 4;

    } else if (data[1] == USBASP_FUNC_ENABLEPROG) {
        replyBuffer[0] = ispEnterProgrammingMode();
        len = 1;

    } else if (data[1] == USBASP_FUNC_READFLASH) {
        setupTransfer(data, PROG_STATE_READFLASH);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_READEEPROM) {
        setupTransfer(data, PROG_STATE_READEEPROM);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_WRITEFLASH) {
        setupTransfer(data, PROG_STATE_WRITEFLASH);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_WRITEEEPROM) {
        setupTransfer(data, PROG_STATE_WRITEEEPROM);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {
        prog_address_newmode = 1;

        /* 1. STORE WITHOUT SHIFT! These are the clean bits 16..23 of the
         * address; shifting them would break the address math. */
        prog_address_high = data[4];

        /* 2. SEND TO THE CHIP WITH THE SHIFT! Extract bit A17 into bit 0
         * as required by the AVR datasheet (load-ext-address, 0x4D). */
        ispUpdateExtended(data[4] >> 1);

        replyBuffer[0] = 0;
        len = 1;

    } else if (data[1] == USBASP_FUNC_SETISPSCK) {
        prog_state = PROG_STATE_IDLE;

        if (data[2] == USBASP_ISP_SCK_AUTO ||
            (data[2] >= USBASP_ISP_SCK_0_5 && data[2] <= USBASP_ISP_SCK_3000)) {
            prog_sck = data[2];
            requested_sck = prog_sck;
            user_speed_requested = (data[2] != USBASP_ISP_SCK_AUTO);
            replyBuffer[0] = 0;                      /* success */
        } else {
            prog_sck = USBASP_ISP_SCK_AUTO;
            requested_sck = prog_sck;
            user_speed_requested = 0;
            replyBuffer[0] = 1;                      /* error -> host shows "cannot set sck period" */
        }
        len = 1;

    } else if (data[1] == USBASP_FUNC_GETISPSCK) {
        replyBuffer[0] = 0;
        replyBuffer[1] = prog_sck;           /* currently configured speed */
        replyBuffer[2] = last_success_speed; /* previous successful speed  */
        replyBuffer[3] = sck_sw_delay;
        replyBuffer[4] = requested_sck;
        replyBuffer[5] = prog_state;
        len = 6;

    } else if (data[1] == USBASP_FUNC_GETCAPABILITIES) {
        replyBuffer[0] = USBASP_CAP_0_TPI | USBASP_CAP_0_I2C | USBASP_CAP_0_MW;
        replyBuffer[1] = USBASP_CAP_1_SCK_AUTO | USBASP_CAP_1_HW_SCK;
        replyBuffer[2] = 0;
        replyBuffer[3] = USBASP_CAP_3_FLASH | USBASP_CAP_3_EEPROM |
                         USBASP_CAP_3_FUSES | USBASP_CAP_3_LOCKBITS |
                         USBASP_CAP_3_EXTENDED_ADDR | USBASP_CAP_3MHZ;  /* 0x40 - no shifts */
        len = 4;

    /* ---------------------------------- SPI ------------------------------------ */

    } else if (data[1] == USBASP_FUNC_SPI_CONNECT) {
        ispSetSCKOption(prog_sck);
        ledRedOn();
        ispSPIConnect();

    } else if (data[1] == USBASP_FUNC_SPI_READ) {
        setupSPIState(PROG_STATE_SPI_READ, data);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_SPI_WRITE) {
        setupSPIState(PROG_STATE_SPI_WRITE, data);
        len = USB_NO_MSG;

    /* ------------------------------ I2C (24xx) --------------------------------- */

    } else if (data[1] == USBASP_FUNC_I2C_INIT) {
        ledRedOn();
        i2c_init();

    } else if (data[1] == USBASP_FUNC_I2C_START) {
        i2c_start();

    } else if (data[1] == USBASP_FUNC_I2C_STOP) {
        i2c_stop();

    } else if (data[1] == USBASP_FUNC_I2C_WRITE_BYTE) {
        replyBuffer[0] = i2c_send_byte(data[2]);
        len = 1;

    } else if (data[1] == USBASP_FUNC_I2C_READ_BYTE) {
        replyBuffer[0] = i2c_read_byte(data[2]);
        len = 1;

    } else if (data[1] == USBASP_FUNC_I2C_READ) {
        /* The host does not send I2C_START before reading; we do it ourselves. */
        i2c_start();
        i2c_address(data[2], I2C_READ);
        setupStreamState(PROG_STATE_I2C_READ, data);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_I2C_WRITE) {
        i2c_start();
        i2c_address(data[2], I2C_WRITE);       /* device address */
        i2c_stop_aw = data[4] & 0x01;          /* wIndex low: stop(1) or restart(0) afterwards */
        setupStreamState(PROG_STATE_I2C_WRITE, data);
        len = USB_NO_MSG;

    /* ---------------------------- microwire (93xx) ----------------------------- */

    } else if (data[1] == USBASP_FUNC_MW_WRITE) {
        CS_HI();                       /* RST high = start of microwire frame */
        mw_cs_lo = data[2];
        /* data[4] (wIndex low) = how many bits to shift in THIS chunk */
        mw_bits_remaining = data[4];
        setupStreamState(PROG_STATE_MW_WRITE, data);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_MW_READ) {
        mw_cs_lo = data[2];
        mw_bits_remaining = 0;         /* RESET THE FLAG */
        setupStreamState(PROG_STATE_MW_READ, data);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_MW_BUSY) {
        replyBuffer[0] = mwBusy() ? 1 : 0;
        len = 1;

    /* ---------------------------------- TPI ------------------------------------ */

    } else if (data[1] == USBASP_FUNC_TPI_CONNECT) {
        tpi_dly_cnt = data[2] | (data[3] << 8);

        /* RST high */
        ISP_OUT |= (1 << ISP_RST);
        ISP_DDR |= (1 << ISP_RST);

        clockWait(3);

        /* RST low */
        ISP_OUT &= ~(1 << ISP_RST);
        ledRedOn();

        clockWait(16);
        tpi_init();

    } else if (data[1] == USBASP_FUNC_TPI_DISCONNECT) {

        tpi_send_byte(TPI_OP_SSTCS(TPISR));
        tpi_send_byte(0);

        clockWait(10);

        /* pulse RST */
        ISP_OUT |= (1 << ISP_RST);
        clockWait(5);
        ISP_OUT &= ~(1 << ISP_RST);
        clockWait(5);

        /* set all ISP pins inputs */
        ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
        /* switch pullups off */
        ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));

    } else if (data[1] == USBASP_FUNC_TPI_RAWREAD) {
        replyBuffer[0] = tpi_recv_byte();
        len = 1;

    } else if (data[1] == USBASP_FUNC_TPI_RAWWRITE) {
        tpi_send_byte(data[2]);
        replyBuffer[0] = 0;            /* mandatory status byte */
        len = 1;

    } else if (data[1] == USBASP_FUNC_TPI_READBLOCK) {
        setupTransfer(data, PROG_STATE_TPI_READ);
        len = USB_NO_MSG;

    } else if (data[1] == USBASP_FUNC_TPI_WRITEBLOCK) {
        setupTransfer(data, PROG_STATE_TPI_WRITE);
        len = USB_NO_MSG; /* multiple out */

    }

    if (len != 0 && len != USB_NO_MSG) {
        usbMsgPtr = replyBuffer;
    }
    return len;
}

/* =========================== IN transfers (device -> host) ===================== */

/* Chunk producers, one per interface family. Each returns the number of
 * bytes actually placed into data[] and advances its own counters. */

static uint8_t tpiReadChunk(uint8_t *data, uint8_t len) {
    tpi_read_block_c(prog_address, data, len);
    prog_address += len;
    return len;
}

static uint8_t spiReadChunk(uint8_t *data, uint8_t len) {
    uint8_t count = len;
    uint8_t *dst = data;

    do {
        *dst++ = ispTransmit(0);
    } while (--count);

    BYTES_LEFT() -= len;
    if (BYTES_LEFT() == 0 && spi_cs_hi) CS_HI();
    return len;
}

static uint8_t i2cReadChunk(uint8_t *data, uint8_t len) {
    uint8_t count = len;
    uint8_t *dst = data;

    do {
        /* Decrement first: when it reaches 0 we are clocking out the very
         * last byte of the transaction -> send NACK. */
        BYTES_LEFT()--;
        *dst++ = i2c_read_byte(BYTES_LEFT() == 0 ? I2C_NACK : I2C_ACK);
    } while (--count);

    /* BYTES_LEFT() was already updated inside the loop. */
    if (BYTES_LEFT() == 0) i2c_stop();
    return len;
}

static uint8_t mwReadChunk(uint8_t *data, uint8_t len) {
    uint8_t count = len;
    uint8_t *dst = data;

    do {
        *dst++ = mwReadByte();
    } while (--count);

    BYTES_LEFT() -= len;
    if (BYTES_LEFT() == 0 && mw_cs_lo) mwEnd();
    return len;
}

static uint8_t flashReadChunk(uint8_t *data, uint8_t len) {
    uint16_t addr = prog_address;      /* 16-bit running address - no 32-bit math */
    uint8_t *dst = data;
    uint8_t count = len;

    /* Make sure the extended address is up to date at chunk start. */
    ispUpdateExtended(prog_address_high >> 1);

    do {
        /* Chips with >64K flash: load the 8-byte page containing addr
         * whenever the page changes (classic USBasp behaviour). */
        if (prog_address_newmode && ((addr >> 3) != ext_page)) {
            ext_page = addr >> 3;
            ispTransmit(0x46);             /* load page address command */
            ispTransmit(ext_page);
            ispTransmit(0);
            ispTransmit(0);
        }
        *dst++ = ispReadFlash16(addr);
        addr++;
    } while (--count);

    prog_address = addr;
    BYTES_LEFT() -= len;
    return len;
}

static uint8_t eepromReadChunk(uint8_t *data, uint8_t len) {
    /* EEPROM never needs the extended address (max 64 KB), so a fast
     * 16-bit running address is enough. */
    uint16_t addr = prog_address;
    uint8_t *dst = data;
    uint8_t count = len;

    do {
        *dst++ = ispReadEEPROM(addr++);
    } while (--count);

    prog_address = addr;
    BYTES_LEFT() -= len;
    return len;
}

uint8_t usbFunctionRead(uint8_t *data, uint8_t len)
{
    uint8_t n;

    /* Guard against zero-length packets. */
    if (len == 0) return 0;

    /* Green LED marks an active read transfer (one write per chunk). */
    ledGreenOn();

    /* Clamp to the number of bytes still pending in this transfer. */
    len = min_u8_u16(len, BYTES_LEFT());

    switch (prog_state) {
        case PROG_STATE_TPI_READ:    n = tpiReadChunk(data, len);    break;
        case PROG_STATE_SPI_READ:    n = spiReadChunk(data, len);    break;
        case PROG_STATE_I2C_READ:    n = i2cReadChunk(data, len);    break;
        case PROG_STATE_MW_READ:     n = mwReadChunk(data, len);     break;
        case PROG_STATE_READFLASH:   n = flashReadChunk(data, len);  break;
        case PROG_STATE_READEEPROM:  n = eepromReadChunk(data, len); break;

        /* Not a read state: unsupported vendor request. */
        default:
            ledGreenOff();
            return 0xFF;
    }

    /* Common post-handler: a finished transfer (or one that TPI aborted on
     * a parity error by zeroing BYTES_LEFT()) returns to idle and lets V-USB
     * terminate with a short packet. */
    if (BYTES_LEFT() == 0) {
        prog_state = PROG_STATE_IDLE;
    }

    ledGreenOff();
    ledRedOff();
    return n;
}

/* =========================== OUT transfers (host -> device) ==================== */

/* Returns 1 when the whole transaction is finished (possibly early, on an
 * abort), 0xFF on a hard error (STALL), 0 when more chunks are expected. */

static uint8_t tpiWriteChunk(uint8_t *data, uint8_t len) {
    tpi_write_block_c(prog_address, data, len);
    if (prog_state == PROG_STATE_IDLE) {
        /* NVM busy/parity error: block helper aborted the transfer. */
        return 0xFF;                    /* STALL -> avrdude sees the failure */
    }
    prog_address += len;
    BYTES_LEFT()  -= len;
    return 0;
}

static uint8_t spiWriteChunk(uint8_t *data, uint8_t len) {
    uint8_t count = len;
    uint8_t *src = data;

    do {
        ispTransmit(*src++);
    } while (--count);

    BYTES_LEFT() -= len;
    if (BYTES_LEFT() == 0 && spi_cs_hi) CS_HI();
    return 0;
}

static uint8_t i2cWriteChunk(uint8_t *data, uint8_t len) {
    uint8_t count = len;
    uint8_t *src = data;

    do {
        if (i2c_send_byte(*src++) == 0) {
            /* NACK from target: abort the whole transaction and report
             * completion so V-USB stops the OUT stream. */
            endTransfer();
            return 1;
        }
    } while (--count);

    BYTES_LEFT() -= len;
    if (BYTES_LEFT() == 0 && i2c_stop_aw) i2c_stop();
    return 0;
}

static uint8_t mwWriteChunk(uint8_t *data, uint8_t len) {
    uint8_t *src = data;
    uint8_t count = len;

    while (count--) {
        uint8_t bits = (mw_bits_remaining >= 8) ? 8 : mw_bits_remaining;
        if (bits == 0) break;           /* transaction bit-count exhausted */

        mwSendData(*src++, bits);
        mw_bits_remaining -= bits;
    }

    BYTES_LEFT() -= len;
    if (BYTES_LEFT() == 0 && mw_cs_lo) mwEnd();
    return 0;
}

static uint8_t flashWriteChunk(uint8_t *data, uint8_t len) {
    uint16_t addr = prog_address;      /* 16-bit running address - no 32-bit math */
    uint8_t *src = data;
    uint8_t count = len;
    uint8_t paged = (prog_pagesize != 0);

    /* Refresh the extended address (bit A17) at chunk start. */
    ispUpdateExtended(prog_address_high >> 1);

    do {
        /* Load the 8-byte page address when crossing into a new page
         * (needed by >64K flash chips; harmless elsewhere). */
        if (prog_address_newmode && ((addr >> 3) != ext_page)) {
            ext_page = addr >> 3;
            ispTransmit(0x46);             /* load page address command */
            ispTransmit(ext_page);
            ispTransmit(0);
            ispTransmit(0);
        }

        if (!paged) {
            /* Legacy chip: byte-wise write with busy-wait. */
            ispWriteFlash16(addr, *src++, 1);
        } else {
            /* Modern chip: load into page buffer. */
            ispWriteFlash16(addr, *src++, 0);

            /* Page boundary reached -> commit it. */
            if (--prog_pagecounter == 0) {
                ispFlushPage16(addr);
                prog_pagecounter = prog_pagesize;
            }
        }

        addr++;
    } while (--count);

    prog_address = addr;
    BYTES_LEFT() -= len;

    if (BYTES_LEFT() == 0) {
        /* Flush the tail page when the file is not page-aligned. */
        if (paged && prog_pagecounter != prog_pagesize) {
            ispFlushPage16(addr - 1);
        }
    }
    return 0;
}

static uint8_t eepromWriteChunk(uint8_t *data, uint8_t len) {
    uint8_t count = len;
    uint8_t *src = data;

    do {
        if (ispWriteEEPROM(prog_address, *src++) != 0) {
            /* Timeout waiting for the device: abort transaction. */
            endTransfer();
            return 1;
        }
        prog_address++;
    } while (--count);

    BYTES_LEFT() -= len;
    return 0;
}

uint8_t usbFunctionWrite(uint8_t *data, uint8_t len)
{
    uint8_t retVal;

    /* Guard against zero-length packets. */
    if (len == 0) return 0;

    /* Green LED marks an active write transfer (one write per chunk). */
    ledGreenOn();

    /* Clamp to the number of bytes still pending - once, up front. */
    len = min_u8_u16(len, BYTES_LEFT());

    switch (prog_state) {
        case PROG_STATE_TPI_WRITE:    retVal = tpiWriteChunk(data, len);   break;
        case PROG_STATE_SPI_WRITE:    retVal = spiWriteChunk(data, len);   break;
        case PROG_STATE_I2C_WRITE:    retVal = i2cWriteChunk(data, len);   break;
        case PROG_STATE_MW_WRITE:     retVal = mwWriteChunk(data, len);    break;
        case PROG_STATE_WRITEFLASH:   retVal = flashWriteChunk(data, len); break;
        case PROG_STATE_WRITEEEPROM:  retVal = eepromWriteChunk(data, len);break;

        /* Not a write state: unsupported vendor request. */
        default:
            ledGreenOff();
            return 0xFF;
    }

    if (retVal == 0xFF) {
        /* Hard error: the helper already tore down the transfer state.
         * Reset the LED before the abnormal exit. */
        ledGreenOff();
        return 0xFF;
    }

    /* Common completion check for all write states (success path). */
    if (retVal == 0 && BYTES_LEFT() == 0) {
        prog_state = PROG_STATE_IDLE;
        retVal = 1;   /* transfer done: V-USB ACKs the final packet */
    }
    /* retVal == 1: handler already called endTransfer(). */

    ledGreenOff();
    ledRedOff();
    return retVal;
}

/* =================================== main ====================================== */

int main(void) {

    /* 1. FIRST THING: start the timer! Lets us use clockWait() instead of
     * the heavy _delay_ms(). */
    clockInit();

    /* no pullups on USB and ISP pins */
    PORTD = 0;
    PORTB = 0;

    /* --- USB D+ (PD2) and D- (PD7) on ATmega8 ---
     * Drive both lines low (SE0) to force a fresh enumeration after
     * unplugging (no external reset circuit needed). */
    DDRD |= (1 << DDD2) | (1 << DDD7);   /* PD2 & PD7 output */
    PORTD &= ~((1 << DDD2) | (1 << DDD7));   /* D+ & D- LOW */

    clockWait(200);                        /* ~64 ms hold (USB 2.0 spec: >2.5 µs) */

    DDRD &= ~((1 << DDD2) | (1 << DDD7));  /* back to inputs */
    PORTD &= ~((1 << DDD2) | (1 << DDD7)); /* no pullups on USB lines */

    /* Port C init: LEDs on PC0/PC1, pull-ups for the remaining inputs */
    DDRC = (1 << PC0) | (1 << PC1);
    PORTC |= (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);

    /* ----------- power-on indication ----------- */

    ledRedOn();
    clockWait(200);
    clockWait(200);                      /* ~128 ms total */

    ledRedOff();
    ledGreenOn();
    clockWait(200);
    clockWait(200);

    ledGreenOff();

    /* ----------- USB ----------- */
    usbInit();

    sei();

    /* main event loop */
    for (;;) {
        usbPoll();
    }
    return 0;
}
