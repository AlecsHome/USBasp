/*
 * USBasp - USB in-circuit programmer for Atmel AVR controllers
 *
 * Thomas Fischl <tfischl@gmx.de>
 *
 * License........: GNU GPL v2 (see Readme.txt)
 * Target.........: ATMega8 at 12 MHz
 * Creation Date..: 2005-02-20
 * Last change....: 2009-02-28
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
#include <util/delay.h> 
#include "tpi.h"
#include "tpi_defs.h"
#include "I2c.h"
#include "microwire.h"
#include <stddef.h>

/* Макрос для быстрой проверки минимального значения */
//#define MIN(a, b) (((a) < (b)) ? (a) : (b))
// Правильный макрос MIN для разных типов
#define MIN(a, b) ({ \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a < _b ? _a : _b; \
})

// --- Перемещаем ОПРЕДЕЛЕНИЯ переменных ВВЕРХ ---
static uchar replyBuffer[8] = {0};
static uchar prog_state = PROG_STATE_IDLE;
uchar prog_sck = USBASP_ISP_SCK_AUTO;
static uint32_t prog_address = 0;
static uint16_t prog_nbytes = 0;
static uchar prog_pagecounter = 0;
static uchar spi_cs_hi = 1;
static uchar mw_cs_lo = 1;
static uchar mw_bitnum = 0;
static uint16_t mw_addr = 0;
static uint8_t mw_opcode = 0;
static uint16_t prog_pagesize = 0;
static uint8_t rc = 0;
static uint8_t i2c_dev_addr = 0xFF;
uint8_t user_speed_requested = 0;
uint8_t prog_address_sent = 0;

/* Глобальные переменные для I2C */
static uint8_t i2c_eeprom_mode = 0;
static uint8_t i2c_eeprom_device_addr = 0xA0;
static uint8_t i2c_eeprom_addr_size = 0;
static uint8_t i2c_stop_aw = 1;

extern uchar sck_sw_delay;

/* -------------------------------------------------------------------------------- */
static void setupTransfer(uint8_t *data, uint8_t new_state) {
    prog_address = (data[3] << 8) | data[2];
    prog_nbytes  = (data[7] << 8) | data[6];
    prog_state   = new_state;
}

static void setupSPIState(uint8_t mode, uint8_t *data) {
    CS_LOW();
    spi_cs_hi = data[2];
    prog_nbytes = (data[7] << 8) | data[6];
    prog_state = mode;
}

static void setupMicrowireOperation(uint8_t *data, uint8_t new_state) {
    mw_addr = (data[3] << 8) | data[2];
    mw_bitnum = data[4];
    mw_opcode = data[5];
    prog_nbytes = (data[7] << 8) | data[6];
    prog_state = new_state;
}

static void setupI2COperation(uint8_t *data, uint8_t new_state, usbMsgLen_t *len_out)
{
    i2c_eeprom_mode      = (data[3] > 0);  // 0 = обычный I2C, >0 = EEPROM
    i2c_eeprom_addr_size = data[3];        // 0, 1 или 2 байта адреса
    i2c_eeprom_device_addr = data[2];      // Базовый адрес устройства

    if (i2c_eeprom_mode) {
        uint32_t memory_address = 0;
        if (data[3] == 2) {
            memory_address = (data[5] << 8) | data[4];
        } else if (data[3] == 1) {
            memory_address = data[4];
        }
        prog_address = memory_address;
    }
    prog_nbytes = (data[7] << 8) | data[6];
    prog_state  = new_state;
    *len_out    = USB_NO_MSG;
}

static void clearReplyBuffer(void) {
	replyBuffer[0] = 0;
	replyBuffer[1] = 0;
	replyBuffer[2] = 0;
	replyBuffer[3] = 0;
	replyBuffer[4] = 0;
	replyBuffer[5] = 0;
	replyBuffer[6] = 0;
	replyBuffer[7] = 0;
}

/* -------------------------------------------------------------------------------- */
usbMsgLen_t usbFunctionSetup(uchar data[8]) {

 usbMsgLen_t len = 0;
        
        clearReplyBuffer(); // Очистить буфер ответа

	if (data[1] == USBASP_FUNC_CONNECT) {

    	  ispSetSCKOption(prog_sck);

    	  ledRedOn();
    	  ispConnect();

    	   rc = ispEnterProgrammingMode();
    	   if (rc != 0) {
           ispDisconnect();        // <-- критично
           last_success_speed = USBASP_ISP_SCK_AUTO;   // <-- сброс  
	    }

           replyBuffer[0] = rc;
	   len = 1;
								
//spi ----------------------------------------------------------------------------------------
	} else if (data[1] == USBASP_FUNC_SPI_CONNECT) {
		ispSetSCKOption(prog_sck);
		ledRedOn();
		isp25Connect();
			
	} else if (data[1] == USBASP_FUNC_SPI_READ) {
    		setupSPIState(PROG_STATE_SPI_READ, data);
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_SPI_WRITE) {
    		setupSPIState(PROG_STATE_SPI_WRITE, data);
    		len = USB_NO_MSG;
    	
//i2c 24xx ------------------------------------------------------------------------------------

	} else if (data[1] == USBASP_FUNC_I2C_INIT) {
    		ledRedOn();
    		i2c_init();

       	} else if (data[1] == USBASP_FUNC_I2C_START) {
		i2c_start();
		
	} else if (data[1] == USBASP_FUNC_I2C_STOP) {
		i2c_stop();

	/* ---------- WRITE_BYTE ---------- */
	} else if (data[1] == USBASP_FUNC_I2C_WRITE_BYTE) {
    		i2c_start(i2c_dev_addr & ~1);   // START + WRITE-бит
    		i2c_send_byte(data[2]);        // сам байт
    		i2c_stop();
    		replyBuffer[0] = 0;             // статус «ОК»
    		len = 1;

	/* ---------- READ_BYTE ---------- */
	} else if (data[1] == USBASP_FUNC_I2C_READ_BYTE) {
    		i2c_start(i2c_dev_addr | 1);    // START + READ-бит
    		replyBuffer[0] = i2c_read_byte(0); // 0 = NACK (последний байт)
    		i2c_stop();
    		len = 1;
    
	} else if (data[1] == USBASP_FUNC_I2C_READ) {
    		i2c_stop_aw = 0;
    		setupI2COperation(data, PROG_STATE_I2C_READ, &len);

	} else if (data[1] == USBASP_FUNC_I2C_WRITE) {
    		i2c_stop_aw = data[4];
    		prog_address_sent = 0;
    		setupI2COperation(data, PROG_STATE_I2C_WRITE, &len);

	} else if (data[1] == USBASP_FUNC_I2C_SETDEVICE) {   // 37
    		// data[2] должен содержать 7-битный адрес (0x00-0x7F)
    		uint8_t addr_7bit = data[2];
    
    		// Проверка, что адрес корректен (7 бит)
    		if (addr_7bit <= 0x7F) {
        	i2c_dev_addr = addr_7bit;  // Сохраняем 7-битный адрес
        	// При отправке на шину I2C будем делать: i2c_dev_addr << 1
        	replyBuffer[0] = 0;  // Успех
    		 } else {
        	   replyBuffer[0] = 1;  // Ошибка: неверный адрес
      		   }
    		 len = 1;

//microwire 93xx ---------------------------------------------------------------------------------------------

	} else if (data[1] == USBASP_FUNC_MW_WRITE) {
    		setupMicrowireOperation(data, PROG_STATE_MW_WRITE);
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_MW_READ) {
    		setupMicrowireOperation(data, PROG_STATE_MW_READ);
    		len = USB_NO_MSG;
			
	} else if (data[1] == USBASP_FUNC_MW_BUSY) {
        	replyBuffer[0] = mwBusy() ? 1 : 0;
        	len = 1;

	} else if (data[1] == USBASP_FUNC_MW_GETADRLEN) {	
		replyBuffer[0] = mwGetAdrLen();
		len = 1;

	 } else if (data[1] ==USBASP_FUNC_MW_TRANSMIT){
         	ledRedOn();
         	mwStart();
       		prog_address = *((unsigned long*) &data[2]);
        	mwSendData(prog_address, (data[6] & 0x1F));
        	data[6] >>= 5;
        	uint8_t n = data[6];
        	for (uint8_t i = 0; i < n && i < 4; i++) {
           	replyBuffer[i] = mwReadByte();
        	}
        	mwEnd();
        	len = (n > 4) ? 4 : n;
        	// replyBuffer уже заполнен

//------------------------------------------------------------------------------------------
	
	} else if (data[1] == USBASP_FUNC_DISCONNECT) {
		ispDisconnect();
		ledGreenOff();

	} else if (data[1] == USBASP_FUNC_TRANSMIT) {
		replyBuffer[0] = ispTransmit(data[2]);
		replyBuffer[1] = ispTransmit(data[3]);
		replyBuffer[2] = ispTransmit(data[4]);
		replyBuffer[3] = ispTransmit(data[5]);
		// ОЧИСТИТЬ остальные байты!
        	replyBuffer[4] = 0;
        	replyBuffer[5] = 0;
        	replyBuffer[6] = 0;
        	replyBuffer[7] = 0;
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
    		prog_address = (data[3] << 8) | data[2];
    		prog_pagesize = data[4] + (((unsigned int)data[5] & 0xF0) << 4);
    		prog_pagecounter = prog_pagesize;  // Всегда сбрасываем при начале записи
    		prog_nbytes = (data[7] << 8) | data[6];
    		prog_state = PROG_STATE_WRITEFLASH;
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_WRITEEEPROM) {
    		prog_address = (data[3] << 8) | data[2];
    		prog_pagesize = 0;
    		prog_nbytes = (data[7] << 8) | data[6];
    		prog_state = PROG_STATE_WRITEEEPROM;
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {  // Функция 9
    		/* 
     		* Стандартный формат USBasp:
     		* data[4], data[5] - wValue (младшие 16 бит)
     		* data[2], data[3] - wIndex (старшие 16 бит)
     		*/
    
    		uint32_t addr = ((uint32_t)data[3] << 24) |  // Старший байт wIndex
                    ((uint32_t)data[2] << 16) |  // Младший байт wIndex  
                    ((uint32_t)data[5] << 8)  |  // Старший байт wValue
                     (uint32_t)data[4];          // Младший байт wValue
    
    		// Вызываем вашу функцию extended addressing
    		// Ваша функция ожидает адреса ≥ 128KB
    		if (addr >= 0x20000) {  // 128KB
        	  ispUpdateExtended(addr);
    		}
    
    		replyBuffer[0] = 0;  // Успех
    		len = 1;

	} else if (data[1] == USBASP_FUNC_SETISPSCK) {
    	        prog_sck = data[2];
    	        // data[2] содержит значение скорости (0 = AUTO, >0 = конкретная скорость)
    		 if (prog_sck == USBASP_ISP_SCK_AUTO) {  // USBASP_ISP_SCK_AUTO = 0
        	  user_speed_requested = 0;  // AUTO режим
    		 } else {
        	  // user_speed_requested = 1 только если скорость > 0
        	  user_speed_requested = 1;  // Явно заданная скорость
    		 }
    		replyBuffer[0] = 0;
    		len = 1;

	} else if (data[1] == USBASP_FUNC_GETISPSCK) {
    		replyBuffer[0] = 0;
    		replyBuffer[1] = prog_sck;           // текущая установленная скорость
    		replyBuffer[2] = last_success_speed; // предыдущая успешная скорость
    		replyBuffer[3] = sck_sw_delay;
    		replyBuffer[4] = isp_hiaddr;
    		replyBuffer[5] = prog_state;
    		// ОЧИСТИТЬ остальные!
        	replyBuffer[6] = 0;
        	replyBuffer[7] = 0;
    		len = 6;

//------------------------------------------------------------------------------------------

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
	        replyBuffer[0] = 0;          // <-- обязательно!
    		len = 1;

	} else if (data[1] == USBASP_FUNC_TPI_READBLOCK) {
    		setupTransfer(data, PROG_STATE_TPI_READ);
    		len = USB_NO_MSG;
	
	} else if (data[1] == USBASP_FUNC_TPI_WRITEBLOCK) {
		setupTransfer(data, PROG_STATE_TPI_WRITE);
		len = USB_NO_MSG; /* multiple out */

//------------------------------------------------------------------------------------------
	} else if (data[1] == USBASP_FUNC_GETCAPABILITIES) {
		replyBuffer[0] = USBASP_CAP_0_TPI | USBASP_CAP_0_I2C | USBASP_CAP_0_MW;
		replyBuffer[1] = USBASP_CAP_1_SCK_AUTO | USBASP_CAP_1_HW_SCK;
		replyBuffer[2] = 0;
		replyBuffer[3] = USBASP_CAP_3_FLASH | USBASP_CAP_3_EEPROM |
                 		 USBASP_CAP_3_FUSES | USBASP_CAP_3_LOCKBITS |
                 		 USBASP_CAP_3_EXTENDED_ADDR | USBASP_CAP_3MHZ;          // 0x40 – никаких сдвигов    
    		len = 4;
	}

    usbMsgPtr = replyBuffer;
   return len;
}

uchar usbFunctionRead(uchar *data, uchar len)
{
    /* Быстрая проверка: если не «читаем» – сразу выход */
    if ((prog_state != PROG_STATE_READFLASH)  &&
        (prog_state != PROG_STATE_READEEPROM) &&
        (prog_state != PROG_STATE_TPI_READ)   &&
        (prog_state != PROG_STATE_SPI_READ)   &&
        (prog_state != PROG_STATE_MW_READ)    &&
        (prog_state != PROG_STATE_I2C_READ)) {
        goto exit_unsupported;
    }

    /* Оптимизация: определяем реальную длину один раз */
    len = MIN(len, prog_nbytes);
    ledGreenOn();

    /* TPI – быстро отдельно */
    if (prog_state == PROG_STATE_TPI_READ) {
        tpi_read_block(prog_address, data, len);
        prog_address += len;
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            prog_state = PROG_STATE_IDLE;
        }
        goto exit_success;
    }

    /* SPI – без буфера */
    if (prog_state == PROG_STATE_SPI_READ) {
        for (uint8_t i = 0; i < len; i++) {
            data[i] = ispTransmit(0);
            _delay_us(1);          // 1 мкс между байтами
        }
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            if (spi_cs_hi) {
                CS_HI();           // поднять CS
                _delay_us(1);      // tCS минимум 500 нс
            }
            prog_state = PROG_STATE_IDLE;
        }
        goto exit_success;
    }

       	/* ---------- I2C ---------- */
	if (prog_state == PROG_STATE_I2C_READ) {
	    /* ---------- Инициализация чтения (первый вызов) ---------- */
	    if (!prog_address_sent) {
	        uint8_t dev = i2c_eeprom_device_addr;
	        uint8_t addr_bytes = i2c_eeprom_addr_size;

	        if (i2c_eeprom_mode) dev |= (prog_address >> 15) & 0x0E; // Для EEPROM с адресами больше 64K

	        i2c_start();
	        if (i2c_send_byte(dev | I2C_WRITE) != I2C_ACK) goto nak;

	        if (addr_bytes >= 1) i2c_send_byte(prog_address >> 8);
	        if (addr_bytes >= 2) i2c_send_byte(prog_address & 0xFF);

	        i2c_start();  // повторный START для чтения
	        if (i2c_send_byte(dev | I2C_READ) != I2C_ACK) goto nak;

	        prog_address_sent = 1;
	    }

	    /* ---------- Чтение данных ---------- */
	    for (uint8_t i = 0; i < len; i++) {
	        data[i] = i2c_read_byte((i == len - 1) ? I2C_NACK : I2C_ACK);
	        prog_address++;
	    }

	    prog_nbytes -= len;
	    if (prog_nbytes == 0) {
	        i2c_stop();
	        prog_state = PROG_STATE_IDLE;
	        i2c_eeprom_mode = 0;
	        prog_address_sent = 0;
	    }
	    return len;

	nak:
	    i2c_stop();
	    prog_state = PROG_STATE_IDLE;
	    i2c_eeprom_mode = 0;
	    prog_address_sent = 0;
	    return 0; // ошибка
	}

    /* MW – без буфера */
    if (prog_state == PROG_STATE_MW_READ) {
        for (uint8_t i = 0; i < len; i++) {
            data[i] = mwReadByte();
            _delay_us(1);   // 1 мкс достаточно для 93С46/56/66
        }
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            if (mw_cs_lo) mwEnd();   // поднимаем CS
            prog_state = PROG_STATE_IDLE;
        }
        	goto exit_success;
    	}

	/* ---------- ГИБРИДНЫЙ ВАРИАНТ (финал) ---------- */
	if (prog_state == PROG_STATE_READFLASH)
	{
	    uint8_t  bytes_read = 0;
	    uint32_t addr = prog_address;
	    uint8_t  current_ext = 0x80;        // Невалидное (> макс. банка)
	    uint8_t  to_read = (len < prog_nbytes) ? len : prog_nbytes;

	    while (bytes_read < to_read)
	    {
	        uint8_t chunk = to_read - bytes_read;
	        if (chunk > 128) chunk = 255;   // Увеличили размер чанка до 255 байт

	        /* Обновляем extended только при необходимости */
	        if (addr >= EXTADDR_BLOCK) {
	            uint8_t ext = (uint8_t)(addr >> 17);
	            if (ext != current_ext) {
	                isp_hiaddr = ext;       // Сохраняем глобально
	                ispTransmit(0x4D);
	                ispTransmit(0x00);
	                ispTransmit(ext);
	                ispTransmit(0x00);
	                current_ext = ext;
	            }
	        }

	        /* Чтение чанка */
	        uint8_t *dst = data + bytes_read;
	        uint8_t n = chunk;
	        do {
	            *dst++ = ispReadFlashRaw(addr++);
	        } while (--n);

	        bytes_read += chunk;
	    }

	    prog_address = addr;
	    prog_nbytes -= bytes_read;
	    if (prog_nbytes == 0) prog_state = PROG_STATE_IDLE;

	    len = bytes_read;
	    goto exit_success;
	}

	/* ---------- Чтение EEPROM ---------- */
	if (prog_state == PROG_STATE_READEEPROM) {
	    for (uint8_t i = 0; i < len; i++) {
	        data[i] = (prog_address <= 0xFFF) ? ispReadEEPROM((uint16_t)prog_address) : 0xFF;
	        prog_address++;
	    }
	    prog_nbytes -= len;
	    if (prog_nbytes == 0) {
	        prog_state = PROG_STATE_IDLE;
	    }
    	goto exit_success;
   }

  exit_unsupported:
    ledGreenOff();
    return 0xFF;

  exit_success:
    ledGreenOff();
    ledRedOn();
    return len;
}

uint8_t mwSendDataBlock(uint8_t *buf, uint8_t len)
{
    	for (uint8_t i = 0; i < len; i++) {
          if (mwSendData(buf[i], 8) != 0) return 1; // ошибка
    	}
    	return 0; // OK
}

uchar usbFunctionWrite(uchar *data, uchar len)
{ 
    uint8_t i = 0;
    uchar retVal = 0;

	/* быстрая проверка режима */
    if (prog_state == PROG_STATE_IDLE)  return 0xFF;
    if ((prog_state != PROG_STATE_WRITEFLASH)  &&
        (prog_state != PROG_STATE_WRITEEEPROM) &&
        (prog_state != PROG_STATE_TPI_WRITE)   &&
        (prog_state != PROG_STATE_SPI_WRITE)   &&
        (prog_state != PROG_STATE_MW_WRITE)    &&
        (prog_state != PROG_STATE_I2C_WRITE)) return 0xFF;

    len = MIN(len, prog_nbytes);
    ledGreenOn();

	/* ---------- TPI ---------- */
	if (prog_state == PROG_STATE_TPI_WRITE) {
	        tpi_write_block(prog_address, data, len);
	        prog_address += len;
	        prog_nbytes  -= len;
	    if (prog_nbytes == 0) prog_state = PROG_STATE_IDLE;
        	goto exit;
    	}
    	/* ---------- SPI ---------- */
    	if (prog_state == PROG_STATE_SPI_WRITE) {
          for (uint8_t i = 0; i < len; i++) ispTransmit(data[i]);
        	prog_nbytes -= len;
          if (prog_nbytes == 0) {
             if (spi_cs_hi) CS_HI();
            	prog_state = PROG_STATE_IDLE;
        	}
        	goto exit;
    	}

	/* ---------- I2C Write (минималистичная) ---------- */
	if (prog_state == PROG_STATE_I2C_WRITE) {
	    /* ----- упрощённая реализация I2C-записи ----- */
	    uint8_t chunk = (len > prog_nbytes) ? prog_nbytes : len;

	    for (uint8_t i = 0; i < chunk; i++) {
	        if (i2c_send_byte(data[i]) != I2C_ACK) {
	            i2c_stop();
	            prog_state = PROG_STATE_IDLE;
	            retVal = 0xFE;
	            goto exit;
	        }
	        prog_nbytes--;
	    }

	    if (prog_nbytes == 0) {
	        i2c_stop();
	        prog_state = PROG_STATE_IDLE;
	        retVal = 1;
	    } else {
	        retVal = 0;
	    }
	    goto exit;
	}

	/* ---------- MW ---------- */
	    if (prog_state == PROG_STATE_MW_WRITE) {
        	if (mwSendDataBlock(data, len) != 0) return 0xFD;
	        prog_nbytes -= len;
        	if (prog_nbytes == 0) { if (mw_cs_lo) mwEnd(); prog_state = PROG_STATE_IDLE; }
	        goto exit;
    	}

	/* ---------- Flash – с extended addressing ---------- */
	if (prog_state == PROG_STATE_WRITEFLASH) {
	    // Локальная копия для оптимизации
	    uint32_t addr = prog_address;
	    uint8_t current_ext = 0x80;  // Гарантированно невалидное

	    for (i = 0; i < len; i++) {
	        // Обновляем extended адрес при переходе на новый блок 128KB
	        if (addr >= 0x20000UL) {
	            uint8_t ext = (uint8_t)(addr >> 17);
	            if (ext != current_ext) {
	                current_ext = ext;
	                isp_hiaddr = ext;
	                ispTransmit(0x4D);  // Команда установки extended адреса
	                ispTransmit(0x00);
	                ispTransmit(ext);
	                ispTransmit(0x00);
	            }
	        }

	        if (prog_pagesize == 0) {
	            // Нестраничная запись
	            if (ispWriteFlash(addr, data[i], 1) != 0) {
	                prog_state = PROG_STATE_IDLE;
	                prog_address = addr;
	                retVal = 0xFB;
	                goto exit;
	            }
	        } else {
	            // Страничная запись
	            if (ispWriteFlash(addr, data[i], 0) != 0) {
	                prog_state = PROG_STATE_IDLE;
	                prog_address = addr;
	                retVal = 0xFB;
	                goto exit;
	            }

	            // Уменьшаем счетчик байтов на странице
	            if (--prog_pagecounter == 0) {
	                // Страница заполнена, программируем её
	                uint32_t page_base = addr & ~(prog_pagesize - 1);

	                if (ispFlushPage(page_base) != 0) {
	                    prog_state = PROG_STATE_IDLE;
	                    prog_address = addr;
	                    retVal = 0xFA;
	                    goto exit;
	                }

	                prog_pagecounter = prog_pagesize;
	            }
	        }

	        addr++;  // Локальный инкремент адреса
	    }

	    prog_address = addr;  // Обновляем глобальную переменную адреса
	    prog_nbytes -= len;

	    if (prog_nbytes == 0) {
	        prog_state = PROG_STATE_IDLE;

	        // Обработка последней неполной страницы
	        if (prog_pagesize != 0 && prog_pagecounter != prog_pagesize) {
	            uint32_t last_page_base = (prog_address - 1) & ~(prog_pagesize - 1);

	            if (ispFlushPage(last_page_base) != 0) {
	                retVal = 0xFA;
	            } else {
	                retVal = 1;
	            }
	        } else {
	            retVal = 1;
	        }
	    } else {
	        retVal = 0;
	    }

	    goto exit;
	}

        /* ---------- EEPROM – без extended addressing и с обработкой ошибок ---------- */
	if (prog_state == PROG_STATE_WRITEEEPROM) {

	    for (i = 0; i < len; i++) {
	        if (ispWriteEEPROM((uint16_t)prog_address, data[i]) != 0) {
	            prog_state = PROG_STATE_IDLE;
	            retVal     = 0xFC;
	            goto exit;
	        }
	        prog_address++;
	    }
	    prog_nbytes -= len;

	    if (prog_nbytes == 0) {
	        prog_state = PROG_STATE_IDLE;
	        retVal     = 1;
	    } else {
	        retVal = 0;   // «ещё не всё»
	    }
	    goto exit;
	}

    /* Неизвестное состояние */
    retVal = 0xFF;
 
  exit:
    ledGreenOff();
    ledRedOn();
    return retVal;
}

int main(void) {

    /* no pullups on USB and ISP pins */
    PORTD = 0;
    PORTB = 0;
    
    /* Output SE0 for USB reset */
    /* aleh: i.e. both D+ and D- should be low. */
    PORTB &= ~((1 << PB1) | (1 << PB0)); // D+ и D- = 0
    DDRB |= (1 << PB1) | (1 << PB0); 	 // выходы, low	
    /* aleh: there was a delay loop here instead which probably would still work, I've put this when was debugging. */
    _delay_ms(63);           // >10 мс (USB 2.0 spec)
    DDRB = 0;                // возвращаем во входы

    /* Инициализация порта C: светодиоды и подтяжки для входов */
    DDRC = (1 << PC0) | (1 << PC1);  // Только PC0 и PC1 как выходы
    DDRC &= ~(1 << PC2);
    //   PORTC = (1 << PC0) | (1 << PC1); // Светодиоды выключены (общий анод)
    // Включим подтяжки для остальных пинов, включая PC2
    PORTC |= (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
   
    /* ----------- индикация ----------- */

    ledRedOn();
    _delay_ms(127);
    ledRedOff();
    ledGreenOn();  
    _delay_ms(127);  
    ledGreenOff();
    ledRedOn();
   
    /* ----------- USB ----------- */
    /* init timer */
    clockInit();
    
    /* main event loop */
    usbInit();

    sei();

  for (;;) {

        usbPoll();

    }
    return 0;
}
