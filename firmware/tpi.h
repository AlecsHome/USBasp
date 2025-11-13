/**
 * \brief Header for tpi
 * \file tpi.h
 */
#ifndef __TPI_H__
#define __TPI_H__
#include <stdint.h>


/* Globals */
/** Number of iterations in tpi_delay loop */
extern uint16_t tpi_dly_cnt;


/* Functions */
/**
 * TPI init
 */
void tpi_init(void);
/**
 * Send raw byte by TPI
 * \param b Byte to send
 */
void tpi_send_byte(uint8_t b);
/**
 * Receive one raw byte from TPI
 * \return Received byte
 */
uint8_t tpi_recv_byte(void);
/**
 * Read block
 * \param addr Address of block
 * \param dptr Pointer to dest memory block
 * \param len Length of read
 */
/* новые Ц короткие */
void tpi_read_block (uint16_t len, uint8_t *buf);
void tpi_write_block(uint16_t len, uint8_t *buf);

/* старые Ц дл€ совместимости с main.c */
void tpi_read_block_old (uint16_t addr, uint8_t *buf, uint16_t len);
void tpi_write_block_old(uint16_t addr, uint8_t *buf, uint16_t len);

/* макросы-алиасы Ц чтобы не трогать main.c */
#define tpi_read_block  tpi_read_block_old
#define tpi_write_block tpi_write_block_old

void tpi_pr_update(uint16_t addr);

#endif /*__TPI_H__*/
