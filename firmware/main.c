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
#include "tpi.h"
#include "tpi_defs.h"
#include "I2c.h"
#include "microwire.h"
#include <stddef.h>
#include <string.h>

/* Макрос для быстрой проверки минимального значения */
//#define MIN(a, b) (((a) < (b)) ? (a) : (b))
// Оптимизированная функция вместо макроса MIN
static inline uint8_t min_u8_u16(uint8_t a, uint16_t b) {
    return (b < a) ? (uint8_t)b : a;
}

// --- Перемещаем ОПРЕДЕЛЕНИЯ переменных ВВЕРХ ---
static uchar replyBuffer[8] = {0};
static uchar prog_state = PROG_STATE_IDLE;
uchar prog_sck = USBASP_ISP_SCK_AUTO;
static uint32_t prog_address = 0;
static uint16_t prog_nbytes = 0;
static uint16_t prog_pagecounter = 0;
static uchar spi_cs_hi = 1;
static uchar mw_bitnum = 0;
static uint16_t mw_addr = 0;
static uint8_t mw_opcode = 0;
static uint8_t mw_cmd_sent = 0; // <--- ДОБАВИТЬ
static uint16_t prog_pagesize = 0;
static uint8_t rc = 0;
static uint8_t i2c_dev_addr = 0xFF;
uint8_t user_speed_requested = 0;
uint8_t prog_address_sent = 0;
uchar prog_address_newmode = 0;
// Объявления ассемблерных функций
extern void tpi_init(void);
extern void tpi_send_byte(uint8_t data);
extern uint8_t tpi_recv_byte(void);
extern void tpi_pr_update(uint16_t addr);

/* Глобальные переменные для I2C */
uint8_t prog_stop_flag;  // Добавить эту строку

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
    mw_addr = (data[3] << 8) | data[2];  
    mw_bitnum = data[4];                  
    mw_opcode = data[5];                  
    prog_nbytes = (data[7] << 8) | data[6]; 
    mw_cmd_sent = 0; // <--- СБРАСЫВАЕМ ФЛАГ ПРИ НОВОЙ КОМАНДЕ
    prog_state = new_state;
}

static void clearReplyBuffer(void) {
    memset(replyBuffer, 0, 8); // Намного компактнее, чем 8 присваиваний
}


// Блоковое чтение TPI (написано на C для корректного ABI)
void tpi_read_block_c(uint16_t addr, uint8_t *buf, uint16_t len) {
    tpi_pr_update(addr);
    while (len--) {
        tpi_send_byte(TPI_OP_SLD_INC);
        *buf++ = tpi_recv_byte();
    }
}

// Блоковая запись TPI (написано на C для корректного ABI)
void tpi_write_block_c(uint16_t addr, uint8_t *buf, uint16_t len) {
    tpi_pr_update(addr);
    while (len--) {
        tpi_send_byte(TPI_OP_SOUT(NVMCMD));
        tpi_send_byte(NVMCMD_WORD_WRITE);
        tpi_send_byte(TPI_OP_SST_INC);
        tpi_send_byte(*buf++);
        
        // Ожидание готовности NVM
        do {
            tpi_send_byte(TPI_OP_SIN(NVMCSR));
        } while (tpi_recv_byte() & NVMCSR_BSY);
    }
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
		ispSPIConnect();
			
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
                
                // ВАЖНО: Передаем управление в usbFunctionRead!
                prog_nbytes = (data[7] << 8) | data[6];
                prog_state = PROG_STATE_I2C_READ;
                len = USB_NO_MSG; // Будем отдавать данные через usbFunctionRead
    
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
	// ТУТ ВСЕ ОСТАЕТСЯ КАК БЫЛО - после исправления mwSendData в microwire.c этот код работает корректно
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
            //	mwSendData(prog_address, (data[6] & 0x1F)); // Теперь работает с 32-бит адресом!
                mwSendData((uint16_t)prog_address, (data[6] & 0x1F)); // Теперь работает с 16-бит адресом!
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
    		//uint32_t addr = ((uint32_t)data[5] << 24) |  // wIndex high (старший байт)
                //	    	  ((uint32_t)data[4] << 16) |  // wIndex low
                //    		  ((uint32_t)data[3] << 8)  |  // wValue high
                //    		  (uint32_t)data[2];           // wValue low (младший байт)

    		// Сохраняем адрес
    		//prog_address = addr;
    		//в оригинальной прошке это делалось элегантнее одной строкой:
    		prog_address = *((unsigned long*) &data[2]);
    		// потому что архитектура AVR little-endian, и байты в памяти лежат ровно так, 
    		// как нужно для 32-битного числа.

    		// ИСПОЛЬЗУЕМ СУЩЕСТВУЮЩУЮ ФУНКЦИЮ!
    		ispUpdateExtended(prog_address);

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
//    uint8_t i = 0;
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
    len = min_u8_u16(len, prog_nbytes);
    ledGreenOn();

	/* ---------- TPI – быстро отдельно --- */
        if (prog_state == PROG_STATE_TPI_READ) {
            tpi_read_block_c(prog_address, data, len); // Используем C-функцию
            prog_address += len;
            prog_nbytes -= len;
            if (prog_nbytes == 0) {
                prog_state = PROG_STATE_IDLE;
            }
            goto exit_success;
        }

	/* ---------- SPI (Чтение) ----------- */
	if (prog_state == PROG_STATE_SPI_READ) {
            uint8_t count = len; // len уже посчитан!
            uint8_t *dst = data;
            
            do {
                *dst++ = ispTransmit(0);
            } while (--count);
            
            prog_nbytes -= len;
            // len уже готов к возврату
            goto exit_success;
        }
	
	/* ---------- I2C Read ---------- */
    	if (prog_state == PROG_STATE_I2C_READ) {
	        uint8_t count = len; // len уже посчитан как MIN(len, prog_nbytes)
	        uint8_t *dst = data;
        
	        do {
	            prog_nbytes--; // Сначала уменьшаем глобальный счетчик оставшихся байт
        
	            // Если после декремента prog_nbytes == 0, значит мы читаем САМЫЙ ПОСЛЕДНИЙ байт транзакции
	            uint8_t ack = (prog_nbytes == 0) ? I2C_NACK : I2C_ACK;
	            *dst++ = i2c_read_byte(ack);
            
	        } while (--count);
        
	        // prog_nbytes уже обновлен внутри цикла! Ничего вычитать не нужно.
        
	        if (prog_nbytes == 0) {
	            i2c_stop(); 
	            prog_state = PROG_STATE_IDLE;
	        }
    
	        // len уже содержит правильное количество байт для возврата
	        goto exit_success;
	}

	/* ---------- MW Read ---------- */
	if (prog_state == PROG_STATE_MW_READ) {
        	// Отправляем команду чтения только один раз
	        if (!mw_cmd_sent) {
	            uint16_t op = (1U << (mw_bitnum + 2)) | ((mw_opcode & 0x03) << mw_bitnum) | mw_addr; 
	            mwStart();
	            mwSendData(op, mw_bitnum + 3);
	            mwReadDummyBit(); 
	            mw_cmd_sent = 1;
	        }

        	// len уже ограничен сверху в начале функции usbFunctionRead!
	        uint8_t count = len;
	        uint8_t *dst = data; 
    
	        do {
	            *dst++ = mwReadByte();
	        } while (--count);
    
	        prog_nbytes -= len; // Вычитаем ровно столько, сколько прочитали
        
	        if (prog_nbytes == 0) {
	            mwEnd(); 
	            prog_state = PROG_STATE_IDLE;
	            mw_cmd_sent = 0;
	        }
    
        	// len уже содержит правильное количество байт для возврата.
	        goto exit_success;
	}	

        /* ---------- Чтение READFLASH ---------- */
        if (prog_state == PROG_STATE_READFLASH) {
            uint32_t addr = prog_address;
            uint8_t *dst = data;
            uint8_t count = len; // Используем len напрямую!
            
            ispUpdateExtended(addr);

            do {
                *dst++ = ispReadFlash(addr);
                addr++;
                
                if ((uint16_t)addr == 0) {
                   ispUpdateExtended(addr);
                }
            } while (--count);

            prog_address = addr;
            prog_nbytes -= len; // Используем len!
            
            if (prog_nbytes == 0) {
                prog_state = PROG_STATE_IDLE;
            }

            goto exit_success;
        }

    	/* ---------- Чтение EEPROM ---------- */
        if (prog_state == PROG_STATE_READEEPROM) {
            // EEPROM не нуждается в extended адресе (максимум 64К)
            // Используем быстрый 16-битный адрес в цикле
            uint16_t addr = (uint16_t)prog_address;
            uint8_t *dst = data;
            uint8_t count = len; // len уже посчитан как MIN(len, prog_nbytes) в начале функции!
        
            do {
                *dst++ = ispReadEEPROM(addr++);
            } while (--count);
        
            prog_address += len; // Обновляем глобальный 32-битный адрес
            prog_nbytes -= len;  // Вычитаем ровно столько, сколько прочитали
            
            if (prog_nbytes == 0) {
                prog_state = PROG_STATE_IDLE;
            }
        
            // len уже содержит правильное количество байт для возврата
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

uchar usbFunctionWrite(uchar *data, uchar len)
{ 
    uchar retVal = 0; // Убрали неиспользуемую 'i'

		/* быстрая проверка режима */
	    	if (prog_state == PROG_STATE_IDLE) return 0xFF;
    
	    	if ((prog_state != PROG_STATE_WRITEFLASH)  &&
	           (prog_state != PROG_STATE_WRITEEEPROM) &&
	           (prog_state != PROG_STATE_TPI_WRITE)   &&
	           (prog_state != PROG_STATE_SPI_WRITE)   &&
	           (prog_state != PROG_STATE_MW_WRITE)    &&
	           (prog_state != PROG_STATE_I2C_WRITE)) return 0xFF;

	    	// ОГРАНИЧИВАЕМ LEN ОДИН РАЗ В НАЧАЛЕ!
	    	len = min_u8_u16(len, prog_nbytes);
	    	ledGreenOn();

    		/* ---------- TPI ---------- */
	    	if (prog_state == PROG_STATE_TPI_WRITE) {
		   tpi_write_block_c(prog_address, data, len);
		   prog_address += len;
	           prog_nbytes  -= len;
	       
	        if (prog_nbytes == 0) {
	            prog_state = PROG_STATE_IDLE;
	            retVal = 1; // Важно!
	        } else {
	            retVal = 0;
	        }
	        goto exit;
	    }

	    /* ---------- SPI (Запись) ---------- */
	    if (prog_state == PROG_STATE_SPI_WRITE) {
	        uint8_t *src = data;
	        uint8_t count = len; // Используем len напрямую
        
	        do {
	            ispTransmit(*src++);
	        } while (--count);
        
	        prog_nbytes -= len;
	        if (prog_nbytes == 0) {
	            if (spi_cs_hi) CS_HI();
	            prog_state = PROG_STATE_IDLE;
	            retVal = 1;
	        } else {
	            retVal = 0;
	        }
	        goto exit;
	    }

	    /* ---------- I2C Write ---------- */
	    if (prog_state == PROG_STATE_I2C_WRITE) {
	        uint8_t *src = data;
	        uint8_t count = len; // len уже посчитан!
        
	        do {
	            i2c_send_byte(*src++);
	        } while (--count);

	        prog_nbytes -= len;

	        if (prog_nbytes == 0) {
	            if (prog_stop_flag) i2c_stop();
	            prog_state = PROG_STATE_IDLE;
	            retVal = 1;
	        } else {
	            retVal = 0;
	        }
	        goto exit;
	    }

	    /* ---------- MW Write ---------- */
	    if (prog_state == PROG_STATE_MW_WRITE) {
	        if (!mw_cmd_sent) {
	            uint16_t op = (1U << (mw_bitnum + 2)) | ((mw_opcode & 0x03) << mw_bitnum) | mw_addr;
	            mwStart();
	            mwSendData(op, mw_bitnum + 3);
	            mw_cmd_sent = 1;
	        }
    
	        uint8_t count = len; // len уже посчитан!
	        uint8_t *src = data;
    
	        do {
	            mwSendData(*src++, 8);
	        } while (--count);
    
	        prog_nbytes -= len;
    
	        if (prog_nbytes == 0) {
	            mwEnd();
	            prog_state = PROG_STATE_IDLE;
	            mw_cmd_sent = 0;
	            retVal = 1;
	        } else {
	            retVal = 0;
	        }
	        goto exit;
	    }

	    /* ---------- Flash – с extended addressing ---------- */
            if (prog_state == PROG_STATE_WRITEFLASH) {
            	uint32_t addr = prog_address;
            	uint8_t *src = data;
            	uint8_t count = len; 

            	ispUpdateExtended(addr);

            	do {
                 if (prog_pagesize == 0) {
                    if (ispWriteFlash(addr, *src++, 1) != 0) { 
                        prog_state = PROG_STATE_IDLE;
                        prog_address = addr;
                        retVal = 0xFB;
                        goto exit;
                    }
                } else {
                    if (ispWriteFlash(addr, *src++, 0) != 0) { 
                        prog_state = PROG_STATE_IDLE;
                        prog_address = addr;
                        retVal = 0xFB;
                        goto exit;
                    }

                    if (--prog_pagecounter == 0) {
                        // Передаем любой адрес страницы (например addr - 1). 
                        // Чип все равно проигнорирует младшие биты в команде 0x4C.
                        if (ispFlushPage(addr) != 0) {
                            prog_state = PROG_STATE_IDLE;
                            prog_address = addr;
                            retVal = 0xFA;
                            goto exit;
                        }
                        prog_pagecounter = prog_pagesize;
                    }
                }
        
                addr++;
        
                if ((uint16_t)addr == 0) {
                   ispUpdateExtended(addr);
                 }

            	} while (--count);
        
             	prog_address = addr; 
            	prog_nbytes -= len;


            	if (prog_nbytes == 0) {
                    prog_state = PROG_STATE_IDLE;

                if (prog_pagesize != 0 && prog_pagecounter != prog_pagesize) {

                    // Записываем остаток страницы
                    if (ispFlushPage(prog_address - 1) != 0) {
                        retVal = 0xFA;
                    } else {
                        retVal = 1; // Полный успех
                    }
                } else {
                    retVal = 1; // Успех (без поддержки страниц или страница кратна размеру)
                }
            } else {
                retVal = 0; // Блок отправлен, но это еще не конец (ждем следующие данные)
            }

            goto exit;
        } 
        
        /* ---------- EEPROM ---------- */
	if (prog_state == PROG_STATE_WRITEEEPROM) {
	        uint8_t *src = data;
	        uint8_t count = len; // V-USB передает максимум 255 байт, uint8_t достаточно!
        
	        do {
	            if (ispWriteEEPROM((uint16_t)prog_address, *src++) != 0) {
	                prog_state = PROG_STATE_IDLE;
	                retVal = 0xFC;
	                goto exit;
	            }
	            prog_address++;
	        } while (--count);
        
	        prog_nbytes -= len;
	        if (prog_nbytes == 0) {
	            prog_state = PROG_STATE_IDLE;
	            retVal = 1;
	        } else {
	            retVal = 0;
	        }
	        goto exit;
	    }

	    retVal = 0xFF;
 
   exit:
      ledGreenOff();
      ledRedOn();
      return retVal;
}

int main(void) {

    // 1. САМОЕ ПЕРВОЕ ДЕЛО: запускаем таймер! 
    // Это позволит нам использовать clockWait() вместо тяжелой _delay_ms()
    clockInit();

    /* no pullups on USB and ISP pins */
    PORTD = 0;
    PORTB = 0;
    
    /* Output SE0 for USB reset */
    PORTB &= ~((1 << PB1) | (1 << PB0)); // D+ и D- = 0
    DDRB |= (1 << PB1) | (1 << PB0); 	 // выходы, low	
    
    // Ждем 63 мс (200 * 320us = 64 мс)
    clockWait(200);           
    DDRB = 0;                // возвращаем во входы

    /* Инициализация порта C: светодиоды и подтяжки для входов */
    DDRC = (1 << PC0) | (1 << PC1);  
    DDRC &= ~(1 << PC2);
    PORTC |= (1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5);
   
    /* ----------- индикация ----------- */

    ledRedOn();
    // Ждем 127 мс (2 раза по 200 * 320us = 128 мс)
    clockWait(200);
    clockWait(200);
    
    ledRedOff();
    ledGreenOn();  
    clockWait(200);
    clockWait(200);
    
    ledGreenOff();
    ledRedOn();
   
    /* ----------- USB ----------- */
    usbInit();

    sei();

    for (;;) {
        usbPoll();
    }
    return 0;
}