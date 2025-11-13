/**
 * Size-optimized TPI driver
 * Works without external #ifdef blocks
 * PB5 Ц TPI CLK  (output)
 * PB3 Ц TPI DATA (open-drain, pull-up)
 *
 * Assembles with: avr-as -mmcu=atmega8 tpi.S -o tpi.o
 */

#include <avr/io.h>
#include "tpi_defs.h"

/* ---------- I/O mapping (hard-wired) ---------- */
#define TPI_CLK_PORT  PORTB
#define TPI_CLK_DDR   DDRB
#define TPI_CLK_PIN   5

#define TPI_DATA_PORT PORTB
#define TPI_DATA_DDR  DDRB
#define TPI_DATA_PIN  3
#define TPI_DATA_IN   PINB

/* ---------- RAM variable ---------- */
.comm tpi_dly_cnt, 2           ; uint16_t tpi_dly_cnt


/* =========================================================
 *              PUBLIC  API
 * =======================================================*/

.global tpi_init
.global tpi_send_byte
.global tpi_recv_byte
.global tpi_read_block
.global tpi_write_block
.global tpi_pr_update


/* ---------------------------------------------------------
 *  tpi_init  --  32 dummy clocks + configure pins
 * -------------------------------------------------------*/
tpi_init:
    /* CLK output, low */
    cbi _SFR_IO_ADDR(TPI_CLK_PORT), TPI_CLK_PIN
    sbi _SFR_IO_ADDR(TPI_CLK_DDR),  TPI_CLK_PIN

    /* DATA = pull-up (open-drain) */
    cbi _SFR_IO_ADDR(TPI_DATA_DDR), TPI_DATA_PIN
    sbi _SFR_IO_ADDR(TPI_DATA_PORT),TPI_DATA_PIN

    /* 32 dummy bits */
    ldi r21, 32
1:  rcall tpi_bit_h
    dec r21
    brne 1b
    ret


/* ---------------------------------------------------------
 *  tpi_send_byte  --  start + 8 data + parity + 2 stop
 *  in:  r24 = data
 * -------------------------------------------------------*/
tpi_send_byte:
    rcall tpi_bit_l          ; start bit
    ldi r18, 8               ; bit counter
    clr r19                  ; parity = 0
1:
    eor r19, r24             ; parity ^= data
    bst r24, 0               ; T = data[0]
    lsr r24                  ; data >>= 1
    rcall tpi_bit            ; send T
    dec r18
    brne 1b

    bst r19, 0               ; parity bit
    rcall tpi_bit
    rcall tpi_bit_h          ; stop 1
    rcall tpi_bit_h          ; stop 2
    ret


/* ---------------------------------------------------------
 *  tpi_recv_byte  --  wait start, 8 data, parity, 2 stop
 *  out: r24 = data (0 if timeout/break)
 * -------------------------------------------------------*/
tpi_recv_byte:
    ldi r18, 192             ; timeout ~1.5 byte
1:  rcall tpi_bit_h
    brtc 2f                  ; start detected
    dec r18
    brne 1b
    /* timeout Ц send break */
    ldi r24, 0
    ldi r18, 26
3:  rcall tpi_bit_l
    dec r18
    brne 3b
    rcall tpi_bit_h
    ret

2:  /* receive 8 data + parity */
    ldi r18, 8
    clr r19                  ; parity = 0
    clr r24                  ; data = 0
4:  rcall tpi_bit_h
    lsr r24
    bld r24, 7               ; data |= (T<<7)
    eor r19, r24             ; parity ^= data
    dec r18
    brne 4b

    rcall tpi_bit_h          ; parity bit
    bld r18, 7
    eor r19, r18
    brmi 3b                  ; parity error -> break

    rcall tpi_bit_h          ; stop 1
    rcall tpi_bit_h          ; stop 2
    ret


/* ---------------------------------------------------------
 *  LOW-LEVEL BIT I/O
 * -------------------------------------------------------*/
/* tpi_bit_h  --  send '1' bit */
tpi_bit_h:
    set                      ; T = 1
/* FALL-THROUGH */

/* tpi_bit_l  --  send '0' bit */
tpi_bit_l:
    clt                      ; T = 0
/* FALL-THROUGH */

/* tpi_bit  --  exchange 1 bit (open-drain) */
/* in:  T = bit to send */
/* out: T = bit received */
tpi_bit:
    /* drive DATA = 0 if T==0, else release (pull-up) */
    brtc 1f
    sbi _SFR_IO_ADDR(TPI_DATA_PORT), TPI_DATA_PIN ; release
    rjmp 2f
1:  cbi _SFR_IO_ADDR(TPI_DATA_PORT), TPI_DATA_PIN ; drive low
2:
    /* delay low half */
    rcall tpi_delay

    /* CLK high */
    sbi _SFR_IO_ADDR(TPI_CLK_PORT),  TPI_CLK_PIN

    /* sample DATA */
    in  r30, _SFR_IO_ADDR(TPI_DATA_IN)
    bst r30, TPI_DATA_PIN

    /* delay high half */
    rcall tpi_delay

    /* CLK low */
    cbi _SFR_IO_ADDR(TPI_CLK_PORT),  TPI_CLK_PIN
    ret


/* ---------------------------------------------------------
 *  DELAY  (t_low = t_high = tpi_dly_cnt cycles)
 * -------------------------------------------------------*/
tpi_delay:
    lds r30, tpi_dly_cnt
    lds r31, tpi_dly_cnt+1
1:  sbiw r30, 1
    brsh 1b
    ret


/* ---------------------------------------------------------
 *  BLOCK READ / WRITE
 * -------------------------------------------------------*/
/* void tpi_read_block(uint16_t length, uint8_t *buf) */
/* r20 = length, r22:r23 = buf */
tpi_read_block:
    rcall tpi_pr_update      ; PR = address (already in r24:r25)
1:  ldi r24, TPI_OP_SLD_INC
    rcall tpi_send_byte
    rcall tpi_recv_byte      ; r24 = data
    st  X+, r24
    dec r20
    brne 1b
    ret


/* void tpi_write_block(uint16_t length, uint8_t *buf) */
tpi_write_block:
    rcall tpi_pr_update
1:  ldi r24, TPI_OP_SOUT(NVMCMD)
    rcall tpi_send_byte
    ldi r24, NVMCMD_WORD_WRITE
    rcall tpi_send_byte
    ldi r24, TPI_OP_SST_INC
    rcall tpi_send_byte
    ld  r24, X+
    rcall tpi_send_byte
    /* wait NVM ready */
2:  ldi r24, TPI_OP_SIN(NVMCSR)
    rcall tpi_send_byte
    rcall tpi_recv_byte
    andi r24, NVMCSR_BSY
    brne 2b
    dec r20
    brne 1b
    ret


/* ---------------------------------------------------------
 *  UPDATE POINTER REGISTER  (PR = r25:r24)
 * -------------------------------------------------------*/
tpi_pr_update:
    movw r20, r24
    ldi r24, TPI_OP_SSTPR(0)
    rcall tpi_send_byte
    mov r24, r20
    rcall tpi_send_byte
    ldi r24, TPI_OP_SSTPR(1)
    rcall tpi_send_byte
    mov r24, r21
    rcall tpi_send_byte
    ret

; ------- старый ABI (addr, buf, len) Ц игнорируем addr -------
.global tpi_read_block_old
tpi_read_block_old:
    ; r22:r21 Ц addr (не нужен)
    ; r20:r19 Ц buf
    ; r18:r17 Ц len
    movw r22, r20        ; buf -> X
    movw r20, r18        ; len -> r20
    rjmp tpi_read_block  ; вызываем новую реализацию

.global tpi_write_block_old
tpi_write_block_old:
    movw r22, r20
    movw r20, r18
    rjmp tpi_write_block
