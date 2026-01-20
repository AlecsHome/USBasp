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
#include <avr/interrupt.h>
#include <stddef.h>

extern uchar prog_sck;
uchar (*ispTransmit)(uchar) = NULL;

uint8_t last_success_speed = USBASP_ISP_SCK_1500;

// Явный массив скоростей от САМОЙ БЫСТРОЙ к САМОЙ МЕДЛЕННОЙ
// Порядок ВАЖЕН - от быстрой к медленной!
static const uchar isp_retry_speeds[] PROGMEM = {

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

#define ISP_SPEED_CNT (sizeof(isp_retry_speeds)/sizeof(isp_retry_speeds[0]))
// Макрос для удобного чтения из массива скоростей
#define GET_SPEED(idx) pgm_read_byte(&isp_retry_speeds[(idx)])

uchar sck_sw_delay;
uchar sck_spcr;
uchar sck_spsr;
uchar isp_hiaddr = 0xFF;   // 0xFF == «ещё не инициализировано»

void spiHWenable() {
	SPCR = sck_spcr;
	SPSR = sck_spsr;
}

static inline void spiHWdisable() {
    SPCR = 0;
}

void ispSetSCKOption(uchar option) {
    // Если option = AUTO, используем last_success или значение по умолчанию
    if (option == USBASP_ISP_SCK_AUTO) {
        if (last_success_speed != USBASP_ISP_SCK_AUTO) {
            option = last_success_speed;  // Используем последнюю успешную
        } else {
            option = USBASP_ISP_SCK_3000;  // Значение по умолчанию
        }
    }
    
    // Сохраняем оригинальное значение для GETISPSCK
     prog_sck = option;  // Теперь это реальная скорость, а не AUTO
        
        if (option >= USBASP_ISP_SCK_93_75) {
        ispTransmit = (uchar (*)(uchar))ispTransmit_hw;
        sck_spsr = 0;
        sck_sw_delay = 1;

    switch (option) {
    	
	case USBASP_ISP_SCK_3000:   // 4.0 MHz
        	sck_spcr = (1 << SPE) | (1 << MSTR);
        	sck_spsr = 0;
        	break;

    	case USBASP_ISP_SCK_1500:   // 2.0 MHz
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
        	sck_spsr = (1 << SPI2X);
        	break;

    	case USBASP_ISP_SCK_750:    // 1.0 MHz
          default:
		sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
        	sck_spsr = 0;
        	break;

    	case USBASP_ISP_SCK_375:    // 500 kHz
           
		sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1);
        	sck_spsr = (1 << SPI2X);      // /32 > 500 kHz
        	break;

    	case USBASP_ISP_SCK_187_5:  // 250 kHz
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1);
        	sck_spsr = 0;
        	break;

    	case USBASP_ISP_SCK_93_75:  // 125 kHz
        	sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0);
        	sck_spsr = 0;
        	break;
	}
    } else {
        ispTransmit = ispTransmit_sw;
        switch (option) {
            case USBASP_ISP_SCK_32:
                sck_sw_delay = 4;    // ~31.25 кГц
                break;
            case USBASP_ISP_SCK_16:
                sck_sw_delay = 8;    // ~15.625 кГц
                break;
            case USBASP_ISP_SCK_8:
                sck_sw_delay = 16;   // ~7.8 кГц
                break;
            case USBASP_ISP_SCK_4:
                sck_sw_delay = 31;   // ~4.03 кГц
                break;
            case USBASP_ISP_SCK_2:
                sck_sw_delay = 63;   // ~1.98 кГц
                break;
            case USBASP_ISP_SCK_1:
                sck_sw_delay = 125;  // 1 кГц
                break;
            case USBASP_ISP_SCK_0_5:
                sck_sw_delay = 250;  // 500 Гц
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

/* Попытка войти в режим программирования с заданной скоростью */
static uchar tryEnterProgMode(uchar speed)
{
    ispSetSCKOption(speed);

    /* Адаптивные тайминги сброса: длинные для SW ( 32 кГц), короткие для HW */
    uint8_t pulse = (speed <= USBASP_ISP_SCK_32) ? 15 : 1;
    uint8_t delay = (speed <= USBASP_ISP_SCK_32) ? 250 : 63;

    for (uchar tries = 3; tries > 0; tries--) {
        ISP_OUT |= (1 << ISP_RST);
        clockWait(pulse);
        ISP_OUT &= ~(1 << ISP_RST);
        clockWait(delay);

        ispTransmit(0xAC);
        ispTransmit(0x53);
        uchar check  = ispTransmit(0);
        uchar check2 = ispTransmit(0);

        if (check == 0x53 && check2 == 0x00) {
            return 0; /* успех */
        }
        _delay_ms(5);
    }
    return 1; /* не удалось */
}

uchar ispEnterProgrammingMode(void) {
    uchar rc;
    
    // 1. Если пользователь явно задал скорость - используем её
    if (user_speed_requested) {
        rc = tryEnterProgMode(prog_sck);
        if (rc == 0) {
            last_success_speed = prog_sck;
            return 0;
        }
        return 1;
    }
    
    // 2. Автоподбор от ВЫСОКОЙ скорости к НИЗКОЙ
    // Проверьте в вашем коде, как определена последовательность ISP_SPEED_CNT
    for (uchar i = 0; i < ISP_SPEED_CNT; i++) {
        uchar speed = GET_SPEED(i);  // Должно быть от 3 MHz к 500 Hz
        
        // Пропускаем если это была last_success_speed (уже пробовали)
        if (speed == last_success_speed) continue;
        
        rc = tryEnterProgMode(speed);
        if (rc == 0) {
            prog_sck = speed;
            last_success_speed = speed;
            return 0;
        }
    }
    
    return 1;
}

void ispUpdateExtended(uint32_t address) {

    // Если адрес < 64KB, extended адрес не нужен
    if (address < 0x10000) {
        return;
    }
    // Вычисляем новый hiaddr (биты 17:16 адреса)
    // address >> 17 = деление на 128КБ (0x20000)
    uint8_t new_hi = (uint8_t)(address >> 17);
    
    // Если не изменилось - ничего не делаем
    if (new_hi == isp_hiaddr) {
        return;
    }
    
    isp_hiaddr = new_hi;
    
    // Отправляем команду SETLONGADDRESS (0x4D)
    ispTransmit(0x4D);
    ispTransmit(0x00);
    ispTransmit(isp_hiaddr);
    ispTransmit(0x00);
}

uchar ispReadFlashRaw(uint32_t address) {
    // Команда чтения Flash (32-битный адрес)
    ispTransmit(0x20 | ((address & 1) << 3));
    ispTransmit(address >> 9);      // address[24:9] (16 бит)
    ispTransmit(address >> 1);      // address[8:1] (8 бит)
    return ispTransmit(0);
}

uchar ispReadFlash(uint32_t address) {
    ispUpdateExtended(address);
    return ispReadFlashRaw(address);
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
