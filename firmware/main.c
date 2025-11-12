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
#define MIN(a, b) ((a) < (b) ? (a) : (b))

// --- Перемещаем ОПРЕДЕЛЕНИЯ переменных ВВЕРХ ---
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

/* -------------------------------------------------------------------------------- */

usbMsgLen_t usbFunctionSetup(uchar data[8]) {

	usbMsgLen_t len = 0;
	
	if (data[1] == USBASP_FUNC_CONNECT) {

		/* set SCK speed */
		
		ispSetSCKOption(prog_sck);
		
		/* set compatibility mode of address delivering */
		prog_address_newmode = 0;

		ledRedOn();
		ispConnect();
                replyBuffer[0] = ispEnterProgrammingMode(); // Потом пытаемся войти в режим программирования
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
    		// data[2] - младший байт адреса, data[3] - старший байт адреса
    		mw_addr = (data[3] << 8) | data[2];
    		mw_bitnum = data[4];  // количество бит для передачи
    		mw_opcode = data[5];  // опкод команды
    
    		prog_nbytes = (data[7] << 8) | data[6];
    		prog_state = PROG_STATE_MW_WRITE;
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_MW_READ) {
    		// data[2] - младший байт адреса, data[3] - старший байт адреса  
    		mw_addr = (data[3] << 8) | data[2];
    		mw_bitnum = data[4];  // количество бит для передачи
    
    		prog_nbytes = (data[7] << 8) | data[6];
    		prog_state = PROG_STATE_MW_READ;
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
		ledRedOff();

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

		if (!prog_address_newmode)
		prog_address = (data[3] << 8) | data[2];

		prog_pagesize = data[4];
		prog_blockflags = data[5] & 0x0F;
		prog_pagesize += (((unsigned int) data[5] & 0xF0) << 4);
		if (prog_blockflags & PROG_BLOCKFLAG_FIRST) {
  		  prog_pagecounter = prog_pagesize;
		}
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEFLASH;
		len = USB_NO_MSG; /* multiple out */

	} else if (data[1] == USBASP_FUNC_WRITEEEPROM) {

		if (!prog_address_newmode)
		prog_address = (data[3] << 8) | data[2];

		prog_pagesize = 0;
		prog_blockflags = 0;
		prog_nbytes = (data[7] << 8) | data[6];
		prog_state = PROG_STATE_WRITEEEPROM;
		len = USB_NO_MSG; /* multiple out */

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {

		/* set new mode of address delivering (ignore address delivered in commands) */
		prog_address_newmode = 1;
		/* set new address */
		prog_address = *((unsigned long*) &data[2]);

	} else if (data[1] == USBASP_FUNC_SETISPSCK) {

		/* set sck option */
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

    /* ---------- I2C – без буфера ---------- */
    /* В usbFunctionRead - расширенная обработка I2C */
	if (prog_state == PROG_STATE_I2C_READ) {
    	  if (i2c_eeprom_mode) {
          // EEPROM режим - уже установлен указатель, просто читаем данные
          for (uint8_t i = 0; i < len; i++) {
            data[i] = i2c_read_byte((i == len - 1) ? I2C_NACK : I2C_ACK);
            prog_address++;   // Обязательно для отслеживания позиции
         }
      } else {
          // Обычный I2C режим - уже отправлен адрес чтения, читаем данные
          for (uint8_t i = 0; i < len; i++) {
            data[i] = i2c_read_byte((i == len - 1) ? I2C_NACK : I2C_ACK);
           }
    	}
    
     	 prog_nbytes -= len;
    	if (prog_nbytes == 0) {
        	i2c_stop();
        	prog_state = PROG_STATE_IDLE;
        	i2c_eeprom_mode = 0;
         }
       goto exit_success;
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

    /* ISP (Flash/EEPROM) – читаем сразу в пакет */
    if (prog_state == PROG_STATE_READFLASH || prog_state == PROG_STATE_READEEPROM) {
        if (prog_state == PROG_STATE_READFLASH) {
            /* Чтение Flash памяти (поддерживает >64K) */
            for (uint8_t i = 0; i < len; i++) {
                data[i] = ispReadFlash(prog_address);
                prog_address++;
            }
        } else {
            /* Чтение EEPROM (ограничено 64K) с проверкой границ */
            for (uint8_t i = 0; i < len; i++) {
                if (prog_address <= 0xFFFF) {
                    data[i] = ispReadEEPROM((uint16_t)prog_address);
                } else {
                    // Ошибка: выход за пределы адресного пространства EEPROM
                    data[i] = 0xFF; // или другое значение ошибки
                }
                prog_address++;
            }
        }
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            prog_state = PROG_STATE_IDLE;
        }
        goto exit_success;
    }

    /* Неизвестное состояние */
    goto exit_unsupported;

   exit_success:
    ledGreenOff();
    ledRedOn();
    return len;

   exit_unsupported:
    ledGreenOff();
    return 0xFF;
}

static uint8_t eepromPageSize(uint32_t addr) 
{
  	return (addr >= 0x10000) ? 128 : 64;   // 24C512+ = 128, 24C256- = 64
}


uchar usbFunctionWrite(uchar *data, uchar len)
{
    uchar i;
    uchar retVal = 0;

    /* Быстрая проверка: если не «пишем» – сразу выход */
    if (prog_state == PROG_STATE_IDLE) {
        retVal = 0xFF;
        goto exit;
    }

    if ((prog_state != PROG_STATE_WRITEFLASH) &&
        (prog_state != PROG_STATE_WRITEEEPROM) &&
        (prog_state != PROG_STATE_TPI_WRITE) &&
        (prog_state != PROG_STATE_SPI_WRITE) &&
        (prog_state != PROG_STATE_MW_WRITE) &&
        (prog_state != PROG_STATE_I2C_WRITE)) {
        retVal = 0xFF;
        goto exit;
    }

    ledGreenOn();

    /* Оптимизация: определяем реальную длину */
    len = MIN(len, prog_nbytes);

    /* TPI – быстро отдельно */
    if (prog_state == PROG_STATE_TPI_WRITE) {
        tpi_write_block(prog_address, data, len);
        prog_address += len;
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            prog_state = PROG_STATE_IDLE;
            retVal = 1;
        }
        goto exit;
    }

    /* SPI – без буфера */
    if (prog_state == PROG_STATE_SPI_WRITE) {
        for (i = 0; i < len; i++) {
            ispTransmit(data[i]);
        }
        prog_nbytes -= len;
        if (prog_nbytes == 0) {
            if (spi_cs_hi) CS_HI();
            prog_state = PROG_STATE_IDLE;
            retVal = 1;
        }
        goto exit;
    }

        /* ---------- I2C – EEPROM page-write ( 64 байт за 5 мс) ---------- */
	/* В usbFunctionWrite - универсальная I2C запись */
	if (prog_state == PROG_STATE_I2C_WRITE) {
    
    	if (i2c_eeprom_mode) {
        
	/* ---------- EEPROM режим с постраничной записью ---------- */

        uint8_t page_size = eepromPageSize(prog_address);
	uint8_t chunk = page_size - (prog_address % page_size);
	if (chunk > len) chunk = len;
	if (chunk > prog_nbytes) chunk = prog_nbytes;

        /* ----- заголовок (один раз) ----- */
        if (!prog_address_sent) {
            i2c_start();
            
            uint8_t dev = i2c_eeprom_device_addr;
            // Для больших EEPROM добавляем старшие биты адреса в device address
            if (prog_address >= 0x10000) {
                // 24C512+: A16 в бит 1, A17 в бит 2 и т.д.
                dev |= ((prog_address >> 15) & 0x0E);
            }
            
            if (i2c_send_byte(dev | I2C_WRITE) != I2C_ACK) goto nak;
            
            // Отправляем адрес памяти
            if (prog_address >= 0x10000) {
                // 24C512+: 16-битный адрес (A0-A15)
                if (i2c_send_byte((prog_address >> 8) & 0xFF) != I2C_ACK) goto nak;
                if (i2c_send_byte(prog_address & 0xFF) != I2C_ACK) goto nak;
            } else if (prog_address >= 0x100) {
                // 24C256 и меньше: 16-битный адрес
                if (i2c_send_byte((prog_address >> 8) & 0xFF) != I2C_ACK) goto nak;
                if (i2c_send_byte(prog_address & 0xFF) != I2C_ACK) goto nak;
            } else {
                // Малые EEPROM: 8-битный адрес
                if (i2c_send_byte(prog_address & 0xFF) != I2C_ACK) goto nak;
            }
            
            prog_address_sent = 1;
        }

        /* ----- пишем данные ----- */
        for (uint8_t i = 0; i < chunk; i++) {
            if (i2c_send_byte(data[i]) != I2C_ACK) goto nak;
            prog_address++;
        }

        prog_nbytes -= chunk;

        /* ----- конец страницы или всей записи ----- */
        if (prog_nbytes == 0 || (prog_address % page_size) == 0) {
            i2c_stop();
            _delay_ms(5);  // Задержка записи EEPROM
       //     prog_address_sent = 0; // <-- уже сброшено в i2c_stop()
       }

        if (prog_nbytes == 0) {
            prog_state = PROG_STATE_IDLE;
            i2c_eeprom_mode = 0;
            retVal = 1;
        } else {
            retVal = chunk; // сколько байт обработано
        }
        
    	} else {
        /* ---------- Обычный I2C режим ---------- */
        for (i = 0; i < len; i++) {
            if (i2c_send_byte(data[i]) != I2C_ACK) {
                retVal = 0xFE;
                goto exit;
            }
        }
        prog_nbytes -= len;
        
        if (prog_nbytes == 0) {
            if (i2c_stop_aw == 1) i2c_stop();
            prog_state = PROG_STATE_IDLE;
            retVal = 1;
          }
    	}
     goto exit;

  nak: // Ошибка EEPROM
     i2c_stop();
     prog_state = PROG_STATE_IDLE;
     i2c_eeprom_mode = 0;
     prog_address_sent = 0;
     retVal = 0xFE;
     goto exit;
  }

    	/* ---------- MW – без буфера ----------*/
    	if (prog_state == PROG_STATE_MW_WRITE) {
     	  for (i = 0; i < len; i++) {
           if (mw_bitnum > 0) {
            uint8_t bits = (mw_bitnum < 8) ? mw_bitnum : 8;
            if (mwSendData(data[i], bits) != 0) {
                retVal = 0xFD; // Ошибка Microwire
                goto exit;
              }
              mw_bitnum -= bits;
        	}
    	}
    		prog_nbytes -= len;
    	if (prog_nbytes == 0) {
        	if (mw_cs_lo) mwEnd();
        	  prog_state = PROG_STATE_IDLE;
        	retVal = 1;
    	}
    	goto exit;
	}

	/* ---------- Flash / EEPROM – общий цикл с выбором функции ----- */

	if (prog_state == PROG_STATE_WRITEFLASH || prog_state == PROG_STATE_WRITEEEPROM) {

    	/* защита EEPROM-адреса (один раз – достаточно) */
    	if (prog_state == PROG_STATE_WRITEEEPROM && prog_address >= 0x10000UL) {
        retVal = 0xFC; goto exit;
    	}

    	uint32_t page_base = prog_pagesize ? (prog_address & ~(prog_pagesize - 1)) : 0;
    	uint8_t  poll      = !prog_pagesize;

    	for (uint8_t i = 0; i < len; i++) {
        /* запись */
        if ((prog_state == PROG_STATE_WRITEFLASH)
                ? ispWriteFlash(prog_address, data[i], poll)
                : ispWriteEEPROM((uint16_t)prog_address, data[i])) {
            retVal = (prog_state == PROG_STATE_WRITEFLASH) ? 0xFB : 0xFC;
            goto exit;
        }

        /* flush страницы */
        if (prog_pagesize && (--prog_pagecounter == 0)) {
            if (ispFlushPage(page_base)) { retVal = 0xFA; goto exit; }
            page_base += prog_pagesize;
            prog_pagecounter = prog_pagesize;
        	}
        	prog_address++;
    	}

    	prog_nbytes -= len;

    	if (prog_nbytes == 0) {
         if (prog_pagesize && prog_pagecounter != prog_pagesize) {
            if (ispFlushPage((prog_address - 1) & ~(prog_pagesize - 1))) {
                retVal = 0xFA; goto exit;
            }
        	}
        	prog_state = PROG_STATE_IDLE;
       	    retVal = 1;
    	} else {
            retVal = 0;
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

void init_frequency_generator(void) {
    // Сброс таймера1
    TCCR1A = 0;
    TCCR1B = 0;
    
       // Установить режим CTC
    TCCR1B |= (1 << WGM12);
    
    // Установить предделитель на 8
    TCCR1B |= (1 << CS11);
    
    // Установить значение для сравнения
    // Для 1 МГц с 16 МГц тактовой частотой: 16 МГц / 8 / 2 = 1000
    OCR1A = 1000; 
    
    // Включить выход на OC1A (PB1)
    TCCR1A |= (1 << COM1A0);

}

int main(void) {
    /* no pullups on USB and ISP pins */
    PORTD = 0;
    
    /* --- USB D+ (PD2) и D- (PD7) --- */
    PORTD &= ~((1 << PD2) | (1 << PD7)); // D+ и D- = 0
    DDRD  |= (1 << PD2) | (1 << PD7);    // выходы, low
    _delay_ms(25);;                       // >10 мс (USB 2.0 spec)
    DDRD  &= ~((1 << PD2) | (1 << PD7)); // возвращаем во входы

    // Теперь настраиваем порт B: все входы, кроме PB1
    DDRB = 0;   // все пины порта B как входы
    DDRB |= (1 << PB1);   // PB1 как выход

    /* Инициализация порта C: светодиоды и подтяжки для входов */
    DDRC = (1 << PC0) | (1 << PC1);  // Только PC0 и PC1 как выходы
    DDRC &= ~(1 << PC2);
    // Включим подтяжки для остальных пинов, включая PC2
    PORTC |= (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
   
    /* ----------- индикация ----------- */
    ledRedOn();
     _delay_ms(250);
    ledRedOff();
    ledGreenOn();  
    _delay_ms(250);
    ledGreenOff();
   
    /* ----------- USB ----------- */
    /* init timer */
    clockInit();

    /* Инициализация генератора частоты */
    init_frequency_generator();

    /* main event loop */
    usbInit();

    sei();
    
    for (;;) {
        usbPoll();
    }
    return 0;
}

