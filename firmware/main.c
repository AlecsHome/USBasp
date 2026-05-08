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
static uchar prog_address_newmode = 0;

/* Глобальные переменные для I2C */
uint8_t prog_stop_flag;  // Добавить эту строку
//static uint8_t i2c_eeprom_device_addr = 0xA0;
//static uint8_t i2c_eeprom_addr_size = 0;
//static uint8_t i2c_stop_aw = 1;

extern uchar sck_sw_delay;

/* -------------------------------------------------------------------------------- */
static void setupTransfer(uint8_t *data, uint8_t new_state) {
    
    if (!prog_address_newmode)
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
    mw_addr = (data[3] << 8) | data[2];  // Адрес (16 бит)
    mw_bitnum = data[4];                  // Длина адреса в битах
    mw_opcode = data[5];                  // Код операции
    prog_nbytes = (data[7] << 8) | data[6]; // Количество байт
    prog_state = new_state;
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
	  
	  /* set compatibility mode of address delivering */
	  prog_address_newmode = 0;

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
    
	} else if (data[1] == USBASP_FUNC_I2C_WRITE_BYTE) {
    		replyBuffer[0] = i2c_send_byte(data[2]);
    		len = 1;
    
	} else if (data[1] == USBASP_FUNC_I2C_READ_BYTE) {
    		replyBuffer[0] = i2c_read_byte(data[2]);
    		len = 1;
    
    	} else if (data[1] == USBASP_FUNC_I2C_READ) {
            	// Инициализация чтения блока данных
            	i2c_start();
            	i2c_address(data[2], I2C_WRITE); // Отправляем адрес устройства для записи

            	// Парсинг флагов и адреса (ТОЧНО ТАК ЖЕ, КАК В I2C_WRITE)
            	uint8_t flags = data[5];
            	uint8_t addr_size = (flags & 0x01) ? 2 : 1; // Бит 0: 1 = адрес 2 байта, 0 = 1 байт
    
            	if (addr_size == 2) {
                   i2c_send_byte(data[4]); // Старший байт адреса памяти из wIndex low
            	  }
            	i2c_send_byte(data[3]);     // Младший байт адреса памяти из wValue high

            	i2c_start(); // Повторный START
            	i2c_address(data[2], I2C_READ); // Адрес устройства для чтения
    
	} else if (data[1] == USBASP_FUNC_I2C_WRITE) {
    		i2c_start();
	    	i2c_address(data[2], I2C_WRITE); // Адрес устройства из wValue low

    		// Флаги из wIndex high (data[5])
    		uint8_t flags = data[5];
    		uint8_t addr_size = (flags & 0x01) ? 2 : 1; // Бит 0: 1 = адрес 2 байта, 0 = 1 байт
    
    		if (addr_size == 2) {
        	i2c_send_byte(data[4]); // Старший байт адреса памяти из wIndex low
    		}
    		i2c_send_byte(data[3]);     // Младший байт адреса памяти из wValue high
    
    		// Бит 1: флаг STOP (1 = отправить STOP в конце, 0 = Repeated START)
    		prog_stop_flag = (flags & 0x02) ? 1 : 0; 
    
    		prog_nbytes = (data[7] << 8) | data[6];
    		prog_state = PROG_STATE_I2C_WRITE;
    		len = USB_NO_MSG; // Будем принимать данные через usbFunctionWrite

	} else if (data[1] == USBASP_FUNC_I2C_SETDEVICE) {   // 37
    		// data[2] должен содержать 7-битный адрес (0x00-0x7F)
    		uint8_t addr_7bit = data[2];
    
    		// Проверка, что адрес корректен (7 бит)
    		if (addr_7bit <= 0x7F) {
 	           i2c_dev_addr = addr_7bit;  // Сохраняем 7-битный адрес
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
            	
            	if (!prog_address_newmode)
        	prog_address = (data[3] << 8) | data[2];
                            	
            	// СТАРАЯ ОШИБКА: prog_pagesize = data[4] + (((unsigned int)data[5] & 0xF0) << 4);
            
            	// ИСПРАВЛЕНО: Читаем чистые 16 бит из wIndex
            	prog_pagesize = data[4] | ((unsigned int)data[5] << 8);
            	prog_pagecounter = prog_pagesize;  
            	prog_nbytes = (data[7] << 8) | data[6];
            	prog_state = PROG_STATE_WRITEFLASH;
            	len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_WRITEEEPROM) {
    		
    		if (!prog_address_newmode)
        	 prog_address = (data[3] << 8) | data[2];
    		
    		prog_pagesize = 0;
    		prog_nbytes = (data[7] << 8) | data[6];
    		prog_state = PROG_STATE_WRITEEEPROM;
    		len = USB_NO_MSG;

	} else if (data[1] == USBASP_FUNC_SETLONGADDRESS) {  // Функция 9

    		prog_address_newmode = 1;

    		/*
    		* ПРАВИЛЬНЫЙ разбор V-USB:
    		* data[2], data[3] - wValue (МЛАДШИЕ 16 бит адреса)
    		* data[4], data[5] - wIndex (СТАРШИЕ 16 бит адреса)
    		*/
    		uint32_t addr = ((uint32_t)data[5] << 24) |  // wIndex high (старший байт)
                	    	((uint32_t)data[4] << 16) |  // wIndex low
                    		((uint32_t)data[3] << 8)  |  // wValue high
                    		(uint32_t)data[2];           // wValue low (младший байт)

    		// Сохраняем адрес
    		prog_address = addr;

    		// ИСПОЛЬЗУЕМ СУЩЕСТВУЮЩУЮ ФУНКЦИЮ!
    		ispUpdateExtended(addr);

    		replyBuffer[0] = 0;  // Успех
    		len = 1;

	/* Обработчик USB-команды SETISPSCK (уже есть, но проверим) */
	} else if (data[1] == USBASP_FUNC_SETISPSCK) {
    		prog_sck = data[2];  // data[2] содержит код скорости (1-16)
    
    		if (prog_sck == USBASP_ISP_SCK_AUTO) {  // 0 = AUTO
	        user_speed_requested = 0;
		    } else {
		        user_speed_requested = 1;
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
		prog_address_newmode = 0; // <-- Добавить это!

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
    uint8_t i = 0;
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
	        for (i = 0; i < len; i++) {
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
	    uint8_t bytes_to_read = (len < prog_nbytes) ? len : prog_nbytes;
	    for (i = 0; i < bytes_to_read; i++) {
	        uint8_t last = (i == bytes_to_read - 1);
        	data[i] = i2c_read_byte(last ? I2C_NACK : I2C_ACK); // <--- ИСПРАВЛЕНО
	    }
	    prog_nbytes -= bytes_to_read;
	    if (prog_nbytes == 0) {
	        i2c_stop();
	        prog_state = PROG_STATE_IDLE;
	    }
	    len = bytes_to_read;
	    goto exit_success;
	}	

    	/* -------- MW – без буфера ----------------------*/
	if (prog_state == PROG_STATE_MW_READ) {
    	// Чтение данных
	    for (i = 0; i < len; i++) {
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
	
	/* ---------- Чтение READFLASH ---------- */
        if (prog_state == PROG_STATE_READFLASH) {
	    uint8_t bytes_read = 0;
	    uint32_t addr = prog_address;
	    uint8_t current_ext = isp_hiaddr; // 
	    uint8_t to_read = (len < prog_nbytes) ? len : prog_nbytes;

	    while (bytes_read < to_read) {
	        uint8_t ext = (addr >= 0x20000UL) ? (uint8_t)(addr >> 17) : 0;
        
	        if (ext != current_ext) {
	            current_ext = ext;
	            // Отправляем SETLONGADDRESS только если адрес ≥ 128K
	            if (addr >= 0x20000UL) {
	                isp_hiaddr = ext;
	                ispTransmit(0x4D);
	                ispTransmit(0x00);
	                ispTransmit(ext);
	                ispTransmit(0x00);
	            }
	        }

	        	data[bytes_read++] = ispReadFlashRaw(addr++);
	    	}

	    	  prog_address = addr;
		  prog_nbytes -= bytes_read;
		  if (prog_nbytes == 0) prog_state = PROG_STATE_IDLE;

	    	len = bytes_read;
	     goto exit_success;
	}

	/* ---------- Чтение EEPROM ---------- */
		if (prog_state == PROG_STATE_READEEPROM) {
	        // uint8_t i достаточно, так как len имеет тип uint8_t (макс. 64 байта от V-USB)
	        for (uint8_t i = 0; i < len; i++) {
	            data[i] = ispReadEEPROM((uint16_t)prog_address);
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
          for (i = 0; i < len; i++) ispTransmit(data[i]);
        	prog_nbytes -= len;
          if (prog_nbytes == 0) {
             if (spi_cs_hi) CS_HI();
            	prog_state = PROG_STATE_IDLE;
        	}
        	goto exit;
    	}

	/* ---------- I2C Write (минималистичная) ---------- */
	if (prog_state == PROG_STATE_I2C_WRITE) {
	    uint8_t bytes_to_write = (len < prog_nbytes) ? len : prog_nbytes;

	    for (i = 0; i < bytes_to_write; i++) {
	        i2c_send_byte(data[i]);
	    }

	    prog_nbytes -= bytes_to_write;

	    if (prog_nbytes == 0) {              // всё передали
	        prog_stop_flag ? i2c_stop() : i2c_start();
	        prog_state = PROG_STATE_IDLE;
	    }

	    len = bytes_to_write;                // сколько записали
	    goto exit;
	}

	/* ---------- MW ---------- */
	if (prog_state == PROG_STATE_MW_WRITE) {
	    // Запись данных
	    // Формируем команду записи Microwire
	    uint32_t op = 1; // Start bit
	    op = (op << 2) | 1; // Opcode 01 (write)
	    op = (op << mw_bitnum) | mw_addr;
    
	    mwStart();
	    mwSendData(op, mw_bitnum + 3); // Start(1) + Opcode(2) + Addr
    
	    // Отправляем данные
	    if (mwSendDataBlock(data, len) != 0) {
	        mwEnd();
	        retVal = 0xFD; // Ошибка записи
	        goto exit;
	    }
    
	    prog_nbytes -= len;
	    if (prog_nbytes == 0) {
	        mwEnd();
	        prog_state = PROG_STATE_IDLE;
	    }
	    goto exit;
	  }

    	/* ---------- Flash – с extended addressing ---------- */
	if (prog_state == PROG_STATE_WRITEFLASH) {
        	uint32_t addr = prog_address;
	        uint8_t current_ext = isp_hiaddr; 

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
	                if (ispWriteFlash(addr, data[i], 1) != 0) {
	                    prog_state = PROG_STATE_IDLE;
	                    prog_address = addr;
	                    retVal = 0xFB;
	                    goto exit;
	                }
	            } else {
	                if (ispWriteFlash(addr, data[i], 0) != 0) {
	                    prog_state = PROG_STATE_IDLE;
	                    prog_address = addr;
	                    retVal = 0xFB;
	                    goto exit;
	                }

                if (--prog_pagecounter == 0) {
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

	            addr++;
	        }

	        prog_address = addr; 
	        prog_nbytes -= len;

	        if (prog_nbytes == 0) {
	            prog_state = PROG_STATE_IDLE;

	            // Защита от неполной страницы (хотя avrdude всегда выравнивает)
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

	/* ---------- EEPROM ---------- */
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
