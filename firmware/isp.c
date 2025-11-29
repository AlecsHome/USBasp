/*
 * isp.c - part of USBasp
 *
 * Autor..........: Thomas Fischl <tfischl@gmx.de>
 * Description....: Provides functions for communication/programming
 *                  over ISP interface
 * Licence........: GNU GPL v2 (see Readme.txt)
 * Creation Date..: 2005-02-23
 * Last change....: 2010-01-19
 */

#include <avr/io.h>
#include "isp.h"
#include "clock.h"
#include <util/delay.h> 
#include "usbasp.h"
#include <avr/pgmspace.h> //  для авто-подбора в программной памяти
#include <stddef.h>
#include <avr/eeprom.h>

// Любой свободный байт в EEPROM
#define EEPROM_SPEED_ADDR 0x10U   // просто 16-ый байт EEPROM

#define ISP_SPEED_CNT (sizeof(isp_retry_speeds)/sizeof(isp_retry_speeds[0]))
#define GET_SPEED(idx) pgm_read_byte(&isp_retry_speeds[(idx)])

extern uchar prog_sck;
static uchar last_success_speed = USBASP_ISP_SCK_AUTO;
uchar (*ispTransmit)(uchar) = NULL;

// Массив скоростей для авто-подбора в программной памяти
// Массив скоростей для авто-подбора (от БЫСТРОЙ к МЕДЛЕННОЙ)
static const uchar isp_retry_speeds[] PROGMEM = {

    USBASP_ISP_SCK_6000,   // 6.0 MHz
    USBASP_ISP_SCK_3000,   // 3.0 MHz
    USBASP_ISP_SCK_1500,   // 1.5 MHz
    USBASP_ISP_SCK_750,    // 750 kHz
    USBASP_ISP_SCK_375,    // 375 kHz
    USBASP_ISP_SCK_187_5,  // 187.5 kHz
    USBASP_ISP_SCK_93_75,  // 93.75 kHz
    USBASP_ISP_SCK_32,     // 32 kHz
    USBASP_ISP_SCK_16,     // 16 kHz
    USBASP_ISP_SCK_8,      // 8 kHz
    USBASP_ISP_SCK_4,      // 4 kHz
    USBASP_ISP_SCK_2,      // 2 kHz
    USBASP_ISP_SCK_1,      // 1 kHz
    USBASP_ISP_SCK_0_5     // 0.5 kHz
};

uchar sck_sw_delay;
uchar sck_spcr;
uchar sck_spsr;
uchar isp_hiaddr;

void spiHWenable() {
    SPCR = sck_spcr;   // загружаем пред-расчитанное значение
    SPSR = sck_spsr;   // и регистр двойной скорости
}

static inline void spiHWdisable() {
    SPCR = 0;
}

void ispSetSCKOption(uchar option) {
   // Всегда сбрасываем состояние SPI перед изменением скорости
    SPCR = 0;
	if (option == USBASP_ISP_SCK_AUTO)
 		option = USBASP_ISP_SCK_1500;  // Значение по умолчанию для авто-режима

	if (option >= USBASP_ISP_SCK_93_75) {
		ispTransmit = (uchar (*)(uchar))ispTransmit_hw;
		sck_spsr = 0;
		sck_sw_delay = 1;	/* force RST#/SCK pulse for 320us */

	switch (option) {
           case USBASP_ISP_SCK_6000:    // 6.0 MHz
    		sck_spcr = (1 << SPE) | (1 << MSTR);
    		sck_spsr = (1 << SPI2X); // Удвоение скорости
    		break;

	   case USBASP_ISP_SCK_3000:
		/* enable SPI, master, 3MHz, XTAL/4 */
		sck_spcr = (1 << SPE) | (1 << MSTR);
		sck_spsr = 0;       	
    	        break;
    	   
    	   case USBASP_ISP_SCK_1500:
        	/* enable SPI, master, 1.5MHz, f_osc/8 (SPR=01, SPI2X=1 for 12MHz) */
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR0);  // SPR1=0, SPR0=1
        	sck_spsr = (1 << SPI2X);     // делитель 8 > 1.5 MHz
        	break;

           case USBASP_ISP_SCK_750:
        	/* enable SPI, master, 0.75MHz, f_osc/16 (SPR=01, SPI2X=0 for 12MHz) */
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR0);   // SPR1=0, SPR0=1
        	sck_spsr = 0;                // делитель 16 > 0.75 MHz
        	break;

    	   case USBASP_ISP_SCK_375:
    	       	/* enable SPI, master, 0.375MHz, f_osc/32 (SPR=10, SPI2X=1 for 12MHz) */
       	 	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1);   // SPR1=1, SPR0=0
        	sck_spsr = (1 << SPI2X);     // делитель 32 > 0.375 MHz
        	break;

    	   case USBASP_ISP_SCK_187_5:
        	/* enable SPI, master, 0.1875MHz, f_osc/64 (SPR=10, SPI2X=0 for 12MHz) */
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1);    // SPR1=1, SPR0=0
        	sck_spsr = 0;                // делитель 64 > 0.1875 MHz
        	break;

    	   case USBASP_ISP_SCK_93_75:
        	/* enable SPI, master, 0.09375MHz, f_osc/128 (SPR=11, SPI2X=0 for 12MHz) */
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0); // SPR1=1, SPR0=1
        	sck_spsr = 0;                // делитель 128 > 0.09375 MHz
        	break;

	    }
                // Включаем аппаратный SPI
        	spiHWenable();

	} else {
		// Программный SPI
		ispTransmit = ispTransmit_sw;
	
	   switch (option) {
             case USBASP_ISP_SCK_32:
		sck_sw_delay = 3;
        	break;

 	      case USBASP_ISP_SCK_16:
		sck_sw_delay = 6;
                break;

  	      case USBASP_ISP_SCK_8:
		sck_sw_delay = 12;
        	break;

	      case USBASP_ISP_SCK_4:
		sck_sw_delay = 24;
        	break;

	      case USBASP_ISP_SCK_2:
		sck_sw_delay = 48;
                break;
	   	
	      case USBASP_ISP_SCK_1:
		sck_sw_delay = 96;
        	break;

	      case USBASP_ISP_SCK_0_5:
		sck_sw_delay = 192;
        	break;
		
	      }
	}
}

void ispDelay() {

	uint8_t starttime = TIMERVALUE;
	while ((uint8_t) (TIMERVALUE - starttime) < sck_sw_delay) {
	}
}

void ispConnect() {

	/* all ISP pins are inputs before */
	/* now set output pins */
	ISP_DDR |= (1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI);
        ISP_DDR &= ~(1 << ISP_MISO); // MISO всегда вход

	/* reset device */
	ISP_OUT &= ~(1 << ISP_RST); /* RST low */
	ISP_OUT &= ~(1 << ISP_SCK); /* SCK low */

	/* positive reset pulse > 2 SCK (target) */
	clockWait(1); /* ~320 µs */
	ISP_OUT |= (1 << ISP_RST); /* RST high */
	clockWait(1);		   /* 320us */
	ISP_OUT &= ~(1 << ISP_RST);/* RST low */

	if (ispTransmit == (uchar (*)(uchar))ispTransmit_hw) {

	spiHWenable();
	}
	
	/* Initial extended address value */
        isp_hiaddr = 0xff;  /* ensure that even 0x00000 causes a write of the extended address byte */
}

void isp25Connect() {
	/* all ISP pins are inputs before */
	/* now set output pins */
	ISP_DDR |= (1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI);
	
	if (ispTransmit == (uchar (*)(uchar))ispTransmit_hw) {
		spiHWenable();
	}
	isp_hiaddr = 0xff;  /* ensure that even 0x00000 causes a write of the extended address byte */
	CS_HI();
}

void ispDisconnect() {

	/* set all ISP pins inputs */
	ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
	/* switch pullups off */
	ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
        prog_sck = USBASP_ISP_SCK_AUTO;	
}

uchar ispTransmit_sw(uchar send_byte)
{
    uchar rec_byte = 0;

    ISP_OUT &= ~(1 << ISP_SCK);      // SCK = 0

    for (uchar bit = 8; bit; --bit) {
        /* MOSI */
        if (send_byte & 0x80)
            ISP_OUT |=  (1 << ISP_MOSI);
        else
            ISP_OUT &= ~(1 << ISP_MOSI);
        send_byte <<= 1;

        ispDelay();                  // tsetup

        /* rising edge */
        ISP_OUT |= (1 << ISP_SCK);
        ispDelay();                  // thold

        /* sample MISO */
        rec_byte = (rec_byte << 1) | ((ISP_IN >> ISP_MISO) & 1);

        /* falling edge + задержка */
        ISP_OUT &= ~(1 << ISP_SCK);
        ispDelay();                  // tlow — добавьте!
    }

    return rec_byte;
}

uchar ispTransmit_hw(uchar send_byte) {

	SPDR = send_byte;

	while (!(SPSR & (1 << SPIF)));

	return SPDR;
}

// Улучшенная версия с защитой от износа EEPROM
void ispSaveSpeedToEEPROM(uchar speed) {
    static uchar last_saved_speed = 0xFF;
    
    // Сохраняем только если скорость изменилась
    if (speed != last_saved_speed) {
        eeprom_update_byte((uint8_t *)EEPROM_SPEED_ADDR, speed);
        last_saved_speed = speed;
    }
}

void ispLoadLastSpeed(void) {
    uint8_t eep_speed = eeprom_read_byte((uint8_t *)EEPROM_SPEED_ADDR);
     // Проверяем, что значение из EEPROM валидно
    if (eep_speed >= USBASP_ISP_SCK_0_5 && eep_speed <= USBASP_ISP_SCK_6000) {
        last_success_speed = eep_speed;
    } else {
         // Невалидное значение - используем авторежим
        last_success_speed = USBASP_ISP_SCK_AUTO; // значение по умолчанию
    }
}

uchar ispEnterProgrammingMode(void) {
    uchar check, check2;
    uchar speed_idx;

    // Пробуем последнюю успешную скорость первой
    if (last_success_speed != USBASP_ISP_SCK_AUTO) {
        ispSetSCKOption(last_success_speed);
        for (uchar tries = 3; tries > 0; tries--) {
            // Сброс целевого устройства
            ISP_OUT |= (1 << ISP_RST); 
            clockWait(10);
            ISP_OUT &= ~(1 << ISP_RST); 
            clockWait(100);

            // Команда программирования enable
            ispTransmit(0xAC); 
            ispTransmit(0x53);
            check = ispTransmit(0); 
            check2 = ispTransmit(0);

            if (check == 0x53 && check2 == 0x00) {
                prog_sck = last_success_speed;
                return 0; // Успех
            }
            clockWait(10);
        }
        last_success_speed = USBASP_ISP_SCK_AUTO; // Сброс при неудаче
    }

    // Автоподбор: от быстрой к медленной
    for (speed_idx = 0; speed_idx < ISP_SPEED_CNT; speed_idx++) {
        uchar current_speed = GET_SPEED(speed_idx);
        ispSetSCKOption(current_speed);

        for (uchar tries = 3; tries > 0; tries--) {
            // Сброс целевого устройства
            ISP_OUT |= (1 << ISP_RST); 
            clockWait(10);
            ISP_OUT &= ~(1 << ISP_RST); 
            clockWait(100);

            // Команда программирования enable
            ispTransmit(0xAC); 
            ispTransmit(0x53);
            check = ispTransmit(0); 
            check2 = ispTransmit(0);

            if (check == 0x53 && check2 == 0x00) {
                prog_sck = current_speed;
                last_success_speed = current_speed; // Запоминаем успешную скорость
                // СОХРАНЯЕМ УСПЕШНУЮ СКОРОСТЬ В EEPROM
                ispSaveSpeedToEEPROM(current_speed);
		return 0; // Успех
            }
            clockWait(10);
        }
    }

    // Не удалось войти в режим программирования
    prog_sck = USBASP_ISP_SCK_AUTO;
    ispSetSCKOption(USBASP_ISP_SCK_187_5); // Безопасная скорость по умолчанию
    return 1; // Ошибка
}

static void ispUpdateExtended(uint32_t address)
{
    if (address < EXTADDR_BLOCK || address >= FLASH_MAX_BYTES)
        return;

    uint8_t curr_hiaddr = (uint8_t)(address >> 17);

    if (curr_hiaddr == isp_hiaddr)
        return;

    isp_hiaddr = curr_hiaddr;

    // Отправка команды стандартными вызовами
    ispTransmit(0x4D);
    ispTransmit(0x00);
    ispTransmit(isp_hiaddr);
    ispTransmit(0x00);
}

uchar ispReadFlash(uint32_t address) {

    ispUpdateExtended(address);
 
    ispTransmit(0x20 | ((address & 1) << 3));
    ispTransmit(address >> 9);
    ispTransmit(address >> 1);
 
    return ispTransmit(0);
}

uchar ispWriteFlash(uint32_t address, uint8_t data, uint8_t pollmode)
{
    ispUpdateExtended(address);

    /* ---------- 1. Собственно загрузка байта в буфер страницы ---------- */
    ispTransmit(0x40 | ((address & 1) << 3));
    ispTransmit(address >> 9);
    ispTransmit(address >> 1);
    ispTransmit(data);

    /* если страница ещё не полна – выходим сразу */
    if (!pollmode) return 0;

    /* ---------- 2. Проверка «уже 0xFF» (только для стирания) ---------- */
    if (data == 0xFF && ispReadFlash(address) == 0xFF) return 0;

    /* ---------- 3. Poll готовности ---------- */
    for (uint8_t t = 30; t; --t) {
        clockWait(t > 20 ? 1 : t > 10 ? 2 : 4);
        if (ispReadFlash(address) == data) return 0;
    }
    return 1;                 // timeout
}

uchar ispFlushPage(uint32_t address) {

    ispUpdateExtended(address);
    
    ispTransmit(0x4C);
    ispTransmit(address >> 9);
    ispTransmit(address >> 1);
    ispTransmit(0);

    /* Всегда проверяем запись */
    for (uint8_t t = 25; t > 0; t--) {
        clockWait(t > 15 ? 1 : (t > 5 ? 2 : 4));
        if (ispReadFlash(address) != 0xFF) {
            return 0; // Успех
        }
    }
    
    return 1; // Ошибка
}

uchar ispReadEEPROM(unsigned int address) {

    ispTransmit(0xA0);
    ispTransmit(address >> 8);
    ispTransmit(address & 0xFF);
    return ispTransmit(0);
    
}

uchar ispWriteEEPROM(unsigned int address, uchar data) {

    ispTransmit(0xC0);
    ispTransmit(address >> 8);    // Старший байт
    ispTransmit(address & 0xFF);  // Младший байт  
    ispTransmit(data);
    // Typical Wait Delay Before Writing
    // tWD_EEPROM min 3.6ms
    clockWait(11); // wait 3,52 ms
    return 0;

}

