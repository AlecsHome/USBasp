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

/* Макрос для быстрой проверки минимального значения */
//#define MIN(a, b) (((a) < (b)) ? (a) : (b))
// Правильный макрос MIN для разных типов
#define MIN(a, b) ({ \
    __typeof__(a) _a = (a); \
    __typeof__(b) _b = (b); \
    _a < _b ? _a : _b; \
})

// --- Перемещаем ОПРЕДЕЛЕНИЯ переменных ВВЕРХ ---
//static uint8_t flash_hiaddr_cache = 0xFF; // вместо current_hiaddr
static uchar replyBuffer[8];
static uchar prog_state = PROG_STATE_IDLE;
uchar prog_sck = USBASP_ISP_SCK_AUTO; // <-- prog_sck определен
static uchar prog_address_newmode = 0;
static unsigned long prog_address;
static uint16_t prog_nbytes = 0; // <-- prog_nbytes определен
static uchar prog_blockflags;
static uchar prog_pagecounter;
static uchar spi_cs_hi = 1;
static uchar mw_cs_lo = 1;
static uchar mw_bitnum = 0;
static uint16_t mw_addr;     // Добавьте эту строку
static uint8_t mw_opcode;    // И эту строку
static unsigned int prog_pagesize;
static uint8_t  rc;
/* Глобальные переменные для I2C */
static uint8_t i2c_eeprom_mode = 0;
static uint8_t i2c_eeprom_device_addr = 0xA0;
static uint8_t i2c_eeprom_addr_size = 0;
static uint8_t i2c_stop_aw = 1;
static uint8_t prog_address_sent = 0;  // Флаг отправки адреса

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

static void setupWriteOperation(uint8_t *data, uint8_t new_state,
                                uint8_t pagesize, uint8_t flags)
{
    if (!prog_address_newmode) {
        /* и для Flash и для EEPROM достаточно 16-битного адреса */
        prog_address = (data[3] << 8) | data[2];
    }

    prog_pagesize = pagesize;
    prog_blockflags = flags;
    if (flags & PROG_BLOCKFLAG_FIRST)
        prog_pagecounter = pagesize;

    prog_nbytes = (data[7] << 8) | data[6];
    prog_state = new_state;
}

static void setupMicrowireOperation(uint8_t *data, uint8_t new_state) {
    mw_addr = (data[3] << 8) | data[2];
    mw_bitnum = data[4];
    mw_opcode = data[5];
    prog_nbytes = (data[7] << 8) | data[6];
    prog_state = new_state;
}

/* -------------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8]) {

	usbMsgLen_t len = 0;
	
	if (data[1] == USBASP_FUNC_CONNECT) {

    	  ispSetSCKOption(prog_sck);
    	  prog_address_newmode = 0;

    	  ledRedOn();
    	  ispConnect();

 	  // Пытаемся войти в режим программирования
	    rc = ispEnterProgrammingMode();
	    if (rc != 0) {
	 // Ошибка - сбрасываем сохраненную скорость
	    last_success_speed = USBASP_ISP_SCK_AUTO;
            ispDisconnect();
            replyBuffer[0] = rc;
    	    len = 1;
           return len;
   	 }
    
    	    replyBuffer[0] = 0; // Успех
	    len = 1;								
//spi --------------------------------------------------------------
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

//i2c 24xx ---------------------------------------------------------

	} else if (data[1] == USBASP_FUNC_I2C_INIT) {
    		ledRedOn();
    		i2c_init();
    
	} else if (data[1] == USBASP_FUNC_I2C_START) {
    		i2c_start();
    
	} else if (data[1] == USBASP_FUNC_I2C_STOP) {
    		i2c_stop();
    
	} else if (data[1] == USBASP_FUNC_I2C_WRITEBYTE) {
    		replyBuffer[0] = i2c_send_byte(data[2]);
    		len = 1;
    
	} else if (data[1] == USBASP_FUNC_I2C_READBYTE) {
    		replyBuffer[0] = i2c_read_byte(data[2]);
    		len = 1;
    
	} else if (data[1] == USBASP_FUNC_I2C_READ) {
    		/* Автоматическое чтение I2C/EEPROM */
    		i2c_eeprom_mode = (data[3] > 0);
    		i2c_eeprom_addr_size = data[3];
    		i2c_eeprom_device_addr = data[2];
    
    		if (i2c_eeprom_mode) {
        	uint32_t memory_address = 0;
        	if (data[3] == 2) {
            	memory_address = (data[5] << 8) | data[4];
	        } else if (data[3] == 1) {
            	memory_address = data[4];
        	}
        	prog_address = memory_address;
        
        	// Установка указателя в EEPROM с правильной адресацией
        	i2c_start();
        
        	uint8_t dev = i2c_eeprom_device_addr;
        	if (memory_address >= 0x10000) {
            	dev |= ((memory_address >> 15) & 0x0E); // A16-A18 для 24C512+
        	}
        
        	if (i2c_send_byte(dev) != I2C_ACK) {
            	i2c_stop();
            	replyBuffer[0] = 0xFE;
            	len = 1;
            	return len;
        	}
        
        	// Отправка адреса памяти
        	if (memory_address >= 0x10000) {
            	i2c_send_byte((memory_address >> 8) & 0xFF);
            	i2c_send_byte(memory_address & 0xFF);
        	} else if (memory_address >= 0x100) {
            	i2c_send_byte((memory_address >> 8) & 0xFF);
            	i2c_send_byte(memory_address & 0xFF);
        	} else {
            	i2c_send_byte(memory_address & 0xFF);
        	}
        
        	// Повторный START для чтения
        	i2c_start();
        	i2c_send_byte(dev | 0x01);
    		} else {
        	// Обычный I2C
        	i2c_start();
        	i2c_address(data[2], I2C_READ);
    		}
    
    		prog_nbytes = (data[7] << 8) | data[6];
    		prog_state = PROG_STATE_I2C_READ;
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_I2C_WRITE) {
    		/* Автоматическая запись I2C/EEPROM */
    		i2c_eeprom_mode = (data[3] > 0);  // Если есть адрес - режим EEPROM
    		i2c_eeprom_addr_size = data[3];
    		i2c_eeprom_device_addr = data[2];
    		i2c_stop_aw = data[4];
    
    		// Устанавливаем начальный адрес для EEPROM
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
    		prog_state = PROG_STATE_I2C_WRITE;
    		prog_address_sent = 0;  // Сброс флага
    		len = USB_NO_MSG;					
			
//microwire 93xx ---------------------------------------------------------		

	} else if (data[1] == USBASP_FUNC_MW_WRITE) {
    		setupMicrowireOperation(data, PROG_STATE_MW_WRITE);
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_MW_READ) {
    		setupMicrowireOperation(data, PROG_STATE_MW_READ);
    		len = USB_NO_MSG;
			
	} else if (data[1] == USBASP_FUNC_MW_BUSY) {
		if (mwBusy() == 1) 
		{
			replyBuffer[0] = 1; //Линия занята
		}
		else
		{
			replyBuffer[0] = 0; 
		}
		
		len = 1;

	} else if (data[1] == USBASP_FUNC_MW_GETADRLEN) {	
		replyBuffer[0] = mwGetAdrLen();
		len = 1;


	 } else if (data[1] ==USBASP_FUNC_MW_TRANSMIT){
         	ledRedOn();
         	mwStart();
         	prog_address = *((unsigned long*) &data[2]);
         	mwSendData(prog_address,(data[6] & 0x1F));
         	data[6] = data[6]>>5;
         	if(data[6] > 0)
          	 replyBuffer[0] = mwReadByte();
         	if(data[6] > 1)
          	 replyBuffer[1] = mwReadByte();
         	if(data[6] > 2)
          	 replyBuffer[2] = mwReadByte();
         	if(data[6] > 3)
           	 replyBuffer[3] = mwReadByte();
         	mwEnd();
        	len = 4;

//------------------------------------------------------------------------
	
	} else if (data[1] == USBASP_FUNC_DISCONNECT) {
		ispDisconnect();
		ledGreenOff();
		
	} else if (data[1] == USBASP_FUNC_TRANSMIT) {
		replyBuffer[0] = ispTransmit(data[2]);
		replyBuffer[1] = ispTransmit(data[3]);
		replyBuffer[2] = ispTransmit(data[4]);
		replyBuffer[3] = ispTransmit(data[5]);
		len = 4;

	} else if (data[1] == USBASP_FUNC_READFLASH) {
    		setupTransfer(data, PROG_STATE_READFLASH);
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_READEEPROM) {
    		setupTransfer(data, PROG_STATE_READEEPROM);
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_ENABLEPROG) {

		replyBuffer[0] = ispEnterProgrammingMode();
		
		len = 1;

	} else if (data[1] == USBASP_FUNC_WRITEFLASH) {
    		uint8_t pagesize = data[4] + (((data[5] & 0xF0) << 4));
    		uint8_t flags = data[5] & 0x0F;
    		setupWriteOperation(data, PROG_STATE_WRITEFLASH, pagesize, flags);
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_WRITEEEPROM) {
    		setupWriteOperation(data, PROG_STATE_WRITEEEPROM, 0, 0);
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {

		/* set new mode of address delivering (ignore address delivered in commands) */
		prog_address_newmode = 1;
		/* set new address */
		prog_address = *((unsigned long*) &data[2]);

	} else if (data[1] == USBASP_FUNC_SETISPSCK) {
    		// Обычная установка скорости (без специальных кодов сброса)
    		prog_sck = data[2];
    		replyBuffer[0] = 0;
    		len = 1;

       	} else if (data[1] == USBASP_FUNC_GETISPSCK) {
    		replyBuffer[0] = 0;
    		replyBuffer[1] = prog_sck;
    		len = 2;

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
	
	} else if (data[1] == USBASP_FUNC_GETCAPABILITIES) {
		replyBuffer[0] = USBASP_CAP_0_TPI | USBASP_CAP_0_I2C | USBASP_CAP_0_MW;
		replyBuffer[1] = USBASP_CAP_1_SCK_AUTO | USBASP_CAP_1_HW_SCK;
		replyBuffer[2] = 0;
		replyBuffer[3] = USBASP_CAP_3_FLASH | USBASP_CAP_3_EEPROM | USBASP_CAP_3_FUSES | USBASP_CAP_3_LOCKBITS;
		len = 4;
	 
     	}

	usbMsgPtr = replyBuffer;

	return len;
}

uchar usbFunctionRead(uchar *data, uchar len)
{
    /* быстрая проверка: не «читаем» – сразу выход */
    if ((prog_state != PROG_STATE_READFLASH)  &&
        (prog_state != PROG_STATE_READEEPROM) &&
        (prog_state != PROG_STATE_TPI_READ)   &&
        (prog_state != PROG_STATE_SPI_READ)   &&
        (prog_state != PROG_STATE_MW_READ)    &&
        (prog_state != PROG_STATE_I2C_READ)) {
        goto exit_unsupported;
    }

    len = MIN(len, prog_nbytes);
    ledGreenOn();

    /* ---------- TPI ---------- */
    if (prog_state == PROG_STATE_TPI_READ) {
        tpi_read_block(prog_address, data, len);
        prog_address += len;
        prog_nbytes  -= len;
        if (prog_nbytes == 0) prog_state = PROG_STATE_IDLE;
        goto exit_success;
    }

    /* ---------- SPI ---------- */
    if (prog_state == PROG_STATE_SPI_READ) {
        for (uint8_t i = 0; i < len; i++) {
            data[i] = ispTransmit(0);
            _delay_us(1);
        }
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            if (spi_cs_hi) { CS_HI(); _delay_us(1); }
            prog_state = PROG_STATE_IDLE;
        }
        goto exit_success;
    }

    /* ---------- I2C ---------- */
    if (prog_state == PROG_STATE_I2C_READ) {
        for (uint8_t i = 0; i < len; i++) {
            data[i] = i2c_read_byte((i == len - 1) ? I2C_NACK : I2C_ACK);
            prog_address++;
        }
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            i2c_stop();
            prog_state      = PROG_STATE_IDLE;
            i2c_eeprom_mode = 0;
        }
        goto exit_success;
    }

    /* ---------- MW ---------- */
    if (prog_state == PROG_STATE_MW_READ) {
        for (uint8_t i = 0; i < len; i++) {
            data[i] = mwReadByte();
            _delay_us(1);
        }
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            if (mw_cs_lo) mwEnd();
            prog_state = PROG_STATE_IDLE;
        }
        goto exit_success;
    }

	/* ---------- ГИБРИДНЫЙ ВАРИАНТ - лучший компромисс ---------- */
	if (prog_state == PROG_STATE_READFLASH) {
    
	    // Быстрая проверка: если адрес <128K, extended адрес не нужен
	    if ((prog_address + len - 1) < 0x20000) {
	        // БЫСТРЫЙ ПУТЬ для ATmega328P
	        for (uint8_t i = 0; i < len; i++) {
	            data[i] = ispReadFlashRaw(prog_address);
	            prog_address++;
	        }
	    } else {
	        // МЕДЛЕННЫЙ ПУТЬ для ATmega2560 (упрощенный Вариант 3)
	        uint8_t bytes_read = 0;
        
	        while (bytes_read < len) {
	            // Установить extended адрес для текущей позиции
	            ispUpdateExtended(prog_address);
            
	            // Читаем до конца текущего блока или до len
	            uint32_t block_end = ((prog_address >> 17) + 1) << 17;
	            uint8_t remaining = len - bytes_read;   // 0...255
            
	            // ВАЖНО: block_end - prog_address может быть > 255, поэтому нужен uint32_t
	            uint32_t to_boundary = block_end - prog_address;  // 0...131071
	            uint8_t chunk = (to_boundary > remaining) ? remaining : (uint8_t)to_boundary;
            
	            // Читаем chunk байт
	            for (uint8_t i = 0; i < chunk; i++) {
	                data[bytes_read + i] = ispReadFlashRaw(prog_address);
	                prog_address++;
	            }
	            bytes_read += chunk;
	        }
	    }

	    prog_nbytes -= len;
	    if (prog_nbytes == 0) prog_state = PROG_STATE_IDLE;
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

static uint8_t eepromPageSize(uint32_t addr) 
{
  	return (addr >= 0x10000) ? 128 : 64;   // 24C512+ = 128, 24C256- = 64
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

	/* ---------- I2C (без отдельной функции) ---------- */
	if (prog_state == PROG_STATE_I2C_WRITE) {
	    uint8_t pageSize = eepromPageSize(prog_address);
	    uint8_t chunk    = pageSize - (prog_address % pageSize);
	    if (chunk > len) chunk = len;
	    if (chunk > prog_nbytes) chunk = prog_nbytes;

	    if (!prog_address_sent) {
	        i2c_start();
	        uint8_t dev = i2c_eeprom_device_addr;
	        if (prog_address >= 0x10000) dev |= ((prog_address >> 15) & 0x0E);
	        if (i2c_send_byte(dev | I2C_WRITE) != I2C_ACK) goto nak;
	        if (prog_address >= 0x100) {
	            if (i2c_send_byte(prog_address >> 8) != I2C_ACK) goto nak;
	        }
	        if (i2c_send_byte(prog_address & 0xFF) != I2C_ACK) goto nak;
	        prog_address_sent = 1;
	    }

	    for (uint8_t i = 0; i < chunk; i++) {
	        if (i2c_send_byte(data[i]) != I2C_ACK) goto nak;
	        prog_address++;
	    }

	    prog_nbytes -= chunk;

	    if (prog_nbytes == 0 || (prog_address % pageSize) == 0) {
	        i2c_stop(); _delay_ms(5);
	        if (prog_nbytes == 0) { prog_state = PROG_STATE_IDLE; i2c_eeprom_mode = 0; }
	    }
	    goto exit;

	nak:
	    i2c_stop(); prog_state = PROG_STATE_IDLE; i2c_eeprom_mode = 0;
	    return 0xFE;
	}
    /* ---------- MW ---------- */
    if (prog_state == PROG_STATE_MW_WRITE) {
        if (mwSendDataBlock(data, len) != 0) return 0xFD;
        prog_nbytes -= len;
        if (prog_nbytes == 0) { if (mw_cs_lo) mwEnd(); prog_state = PROG_STATE_IDLE; }
        goto exit;
    }

    /* ---------- Flash / EEPROM (общая часть) ---------- */
    if (prog_state == PROG_STATE_WRITEFLASH || prog_state == PROG_STATE_WRITEEEPROM) {

        /* защита EEPROM-адреса */
        if (prog_state == PROG_STATE_WRITEEEPROM && prog_address >= 0x10000UL)
            return 0xFC;

        uint32_t page_base = prog_pagesize ? (prog_address & ~(prog_pagesize - 1)) : 0;
        uint8_t  poll      = !prog_pagesize;

        for (uint8_t i = 0; i < len; i++) {
            uint8_t err = (prog_state == PROG_STATE_WRITEFLASH)
                        ? ispWriteFlash(prog_address, data[i], poll)
                        : ispWriteEEPROM((uint16_t)prog_address, data[i]);
            if (err) return (prog_state == PROG_STATE_WRITEFLASH) ? 0xFB : 0xFC;

            if (prog_pagesize && (--prog_pagecounter == 0)) {
                if (ispFlushPage(page_base)) return 0xFA;
                page_base += prog_pagesize;
                prog_pagecounter = prog_pagesize;
            }
            prog_address++;
        }

        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            if (prog_pagesize && prog_pagecounter != prog_pagesize) {
                if (ispFlushPage((prog_address - 1) & ~(prog_pagesize - 1))) return 0xFA;
            }
            prog_state = PROG_STATE_IDLE;
            return 1;
        }
        return 0; // ещё не всё
    }

    /* ни один режим не активен */
    return 0xFF;

 exit:
    ledGreenOff();
    ledRedOn();
    return (prog_nbytes == 0) ? 1 : 0;
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
    _delay_ms(64);           // >10 мс (USB 2.0 spec)
    DDRB = 0;                // возвращаем во входы

    /* Инициализация порта C: светодиоды и подтяжки для входов */
    DDRC = (1 << PC0) | (1 << PC1);  // Только PC0 и PC1 как выходы
    DDRC &= ~(1 << PC2);
    //   PORTC = (1 << PC0) | (1 << PC1); // Светодиоды выключены (общий анод)
    // Включим подтяжки для остальных пинов, включая PC2
    PORTC |= (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
   
    /* ----------- индикация ----------- */

    ledRedOn();
    _delay_ms(128);
    ledRedOff();
    ledGreenOn();  
    _delay_ms(128);  
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
