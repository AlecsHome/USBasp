/*
 * Tiny Microwire master bit-bang
 * Pins:  RST = CS,  MOSI = SI,  MISO = SO,  SCK = SK
 * Works on any AVR with any ISP port mapping
 */

#include <avr/io.h>
#include "clock.h"
#include <util/delay.h>
#include "isp.h"
#include "microwire.h"

#define ORG_OFFSET 6   /* Microwire организаци€ = найденный_0 + 6 */

/* ---------- local helpers ---------- */
static inline void mw_clk_hi(void)   { ISP_OUT |=  (1 << ISP_SCK); }
static inline void mw_clk_lo(void)   { ISP_OUT &= ~(1 << ISP_SCK); }
static inline void mw_si_hi(void)    { ISP_OUT |=  (1 << ISP_MOSI); }
static inline void mw_si_lo(void)    { ISP_OUT &= ~(1 << ISP_MOSI); }
static inline void mw_cs_hi(void)    { ISP_OUT |=  (1 << ISP_RST); }
static inline void mw_cs_lo(void)    { ISP_OUT &= ~(1 << ISP_RST); }
static inline uint8_t mw_so_read(void){ return (ISP_IN & (1 << ISP_MISO)) ? 1 : 0; }

/* ---------- public API ---------- */

void mwBegin(void)
{
    /* CS=RST must be output */
    ISP_DDR |= (1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI);   /* CLK + SI */
    ISP_DDR &= ~(1 << ISP_MISO);                   /* SO = input */
    mw_clk_lo();
    mw_si_lo();
    mw_cs_lo();  /* Ensure CS is low initially */
} 

void mwStart(void)
{
    // ¬ернуть оригинальную реализацию без параметра
    ISP_OUT &= ~(1 << ISP_RST);  // CS = 0
    ISP_OUT &= ~(1 << ISP_SCK);  // CLK = 0
    ispDelay();

    ISP_OUT |= (1 << ISP_RST);   // CS = 1
    
    // send start bit
    ISP_OUT |= (1 << ISP_MOSI);
    ispDelay();
    ISP_OUT |= (1 << ISP_SCK);
    ispDelay();
    ISP_OUT &= ~(1 << ISP_SCK);
    ispDelay();
}

void mwEnd(void)
{
    mw_cs_lo();   /* CS low  */
    mw_clk_lo();  /* CLK low */
}

/* -------------- core primitives -------------- */

uint8_t mwSendData(uint8_t data, uint8_t bits)
{
    for (uint8_t mask = 1 << (bits - 1); mask; mask >>= 1) {
        mw_clk_lo();
        if (data & mask) 
            mw_si_hi();
        else 
            mw_si_lo();
        ispDelay();          /* setup + hold */
        mw_clk_hi();
        ispDelay();          /* high time    */
        mw_clk_lo();         /* clock low for next bit */
    }
    return 0; // ¬озвращаем 0 в случае успеха
}

uint8_t mwReadByte(void)
{
    uint8_t val = 0;
    for (uint8_t i = 0; i < 8; ++i) {
        mw_clk_lo();
        ispDelay();
        mw_clk_hi();
        val = (val << 1) | mw_so_read();
        ispDelay();
        mw_clk_lo();        /* clock low for next bit */
    }
    return val;
}

/* -------------- higher level -------------- */

uint8_t mwBusy(void)
{
    mwBegin();
    mw_cs_hi();               /* CS high */
    _delay_us(10);
    ispDelay();
    uint8_t busy = mw_so_read() == 0;   /* SO = 0 > busy */
    mwEnd();
    return busy;
}

uint8_t mwGetAdrLen(void)
{
    mwBegin();
    mw_cs_hi();               /* CS high */
    _delay_us(10);
    mwSendData(0xC0, 8);      /* 1 start + 10b opcode + 5 zeros */
    
    uint8_t len = 0xFF;
    for (uint8_t i = 0; i < 7; ++i) {
        mw_clk_lo();
        ispDelay();
        mw_clk_hi();
        ispDelay();
        
        if (!mw_so_read()) {   /* первый 0 на SO > длина = i + 6 */
            len = i + ORG_OFFSET;
            mw_clk_lo();  /* set clock low before break */
            break;
        }
        mw_clk_lo();  /* clock low for next bit */
    }
    
    mwEnd();
    return len;
}
