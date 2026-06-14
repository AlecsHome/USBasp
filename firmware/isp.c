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
#include <avr/pgmspace.h>
#include <stddef.h>

extern uchar prog_sck;
uchar (*ispTransmit)(uchar) = NULL;

uint8_t last_success_speed = USBASP_ISP_SCK_3000;

// Быстрое вычисление расширенного адреса без 32-битной математики
// Эквивалент (uint8_t)(addr >> 17), но компилируется в 2 инструкции  
// Быстрое вычисление расширенного адреса без 32-битной математики
// Работает только на little-endian архитектурах (AVR - little-endian)
#define GET_EXT_ADDR(address) (((uint8_t*)&(address))[2] >> 1)
//#define GET_EXT_ADDR(address) (uint8_t)((((uint8_t*)&(address))[2] >> 1) & 0x7F) //лишняя операция AND старшие биты всегда равны 0

// Быстрое и безопасное вычисление расширенного адреса (банка 128K)
// Эквивалент (addr >> 17), но компилируется оптимально благодаря касту (uint8_t)
//#define GET_EXT_ADDR(address) ((uint8_t)((address) >> 17))      // +90 байт	

// Таблицы скоростей для аппаратного SPI (индекс = скорость - USBASP_ISP_SCK_3000)
static const uchar hw_spcr_table[] PROGMEM = {
    (1 << SPE) | (1 << MSTR),                                  // 3.0 MHz
    (1 << SPE) | (1 << MSTR) | (1 << SPR0),                    // 1.5 MHz
    (1 << SPE) | (1 << MSTR) | (1 << SPR0),                    // 0.75 MHz
    (1 << SPE) | (1 << MSTR) | (1 << SPR1),                    // 0.375 MHz
    (1 << SPE) | (1 << MSTR) | (1 << SPR1),                    // 0.1875 MHz
    (1 << SPE) | (1 << MSTR) | (1 << SPR1) | (1 << SPR0)      // 0.09375 MHz
};

static const uchar hw_spsr_table[] PROGMEM = {
    0,                // 3.0 MHz
    (1 << SPI2X),     // 1.5 MHz
    0,                // 0.75 MHz
    (1 << SPI2X),     // 0.375 MHz
    0,                // 0.1875 MHz
    0                 // 0.09375 MHz
};

// Таблицы для программного SPI (индекс = скорость - USBASP_ISP_SCK_32)
static const uchar sw_delay_table[] PROGMEM = { 3, 6, 12, 24, 48, 96, 192 };

static const uchar isp_retry_speeds[] PROGMEM = {
    USBASP_ISP_SCK_3000, USBASP_ISP_SCK_1500, USBASP_ISP_SCK_750,
    USBASP_ISP_SCK_375, USBASP_ISP_SCK_187_5, USBASP_ISP_SCK_93_75,
    USBASP_ISP_SCK_32, USBASP_ISP_SCK_16, USBASP_ISP_SCK_8,
    USBASP_ISP_SCK_4, USBASP_ISP_SCK_2, USBASP_ISP_SCK_1, USBASP_ISP_SCK_0_5
};

#define ISP_SPEED_CNT (sizeof(isp_retry_speeds)/sizeof(isp_retry_speeds[0]))
#define GET_SPEED(idx) pgm_read_byte(&isp_retry_speeds[(idx)])

uchar sck_sw_delay = 0;
uchar sck_spcr = 0;
uchar sck_spsr = 0;
uchar isp_hiaddr = 0xFF;

void spiHWenable() {
    SPCR = sck_spcr;
    SPSR = sck_spsr;
}

static inline void spiHWdisable() {
    SPCR = 0;
}

void ispSetSCKOption(uchar option) {
    if (option == USBASP_ISP_SCK_AUTO) {
        option = (last_success_speed != USBASP_ISP_SCK_AUTO && 
                  last_success_speed != 0) ? last_success_speed : USBASP_ISP_SCK_3000; // По умолчанию 13
    }
    
    // Защита от мусора
    if (option < USBASP_ISP_SCK_0_5 || option > USBASP_ISP_SCK_3000) {
        option = USBASP_ISP_SCK_3000;
    }
    
    prog_sck = option;

    if (option >= USBASP_ISP_SCK_93_75 && option <= USBASP_ISP_SCK_3000) {
        // Аппаратный SPI (option от 8 до 13)
        ispTransmit = (uchar (*)(uchar))ispTransmit_hw;
        sck_sw_delay = 1;
        
        // ИСПРАВЛЕНО: 13 - option дает 0, 1, 2... 5
        uint8_t idx = USBASP_ISP_SCK_3000 - option; 
        
        if (idx < sizeof(hw_spcr_table)) {
            sck_spcr = pgm_read_byte(&hw_spcr_table[idx]);
            sck_spsr = pgm_read_byte(&hw_spsr_table[idx]);
        } else {
            // Fallback
            sck_spcr = (1 << SPE) | (1 << MSTR) | (1 << SPR0);
            sck_spsr = (1 << SPI2X);
        }
        spiHWenable();
    } 
    else if (option >= USBASP_ISP_SCK_32 && option <= USBASP_ISP_SCK_16) { 
        // Программный SPI (option от 1 до 7)
        // Примечание: USBASP_ISP_SCK_16 = 6, USBASP_ISP_SCK_32 = 7
        // Ваше условие option <= USBASP_ISP_SCK_0_5 (1) пропускало бы 2,3,4,5,6
        // Правильное условие для Software SPI:
        ispTransmit = ispTransmit_sw;
        
        // ИСПРАВЛЕНО: 7 - option дает 0, 1, 2... 6
        uint8_t idx = USBASP_ISP_SCK_32 - option; 
        
        if (idx < sizeof(sw_delay_table)) {
            sck_sw_delay = pgm_read_byte(&sw_delay_table[idx]);
        } else {
            sck_sw_delay = 48;  // Значение по умолчанию
        }
    }
}

void ispDelay() {

	uint8_t starttime = TIMERVALUE;
	while ((uint8_t) (TIMERVALUE - starttime) < sck_sw_delay) {
	}
}

// Базовая инициализация пинов (inline, чтобы не тратить флеш на вызов функции)
static inline void ispInitPins(void) {
    ISP_DDR |= (1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI);
    ISP_DDR &= ~(1 << ISP_MISO); // Обязательно для обоих режимов!
    
    if (ispTransmit == (uchar (*)(uchar))ispTransmit_hw) {
        spiHWenable();
    }
    
    isp_hiaddr = 0xFF;
}

// Для AVR (с reset pulse)
void ispConnect(void) {
    ispInitPins();
    
    ISP_OUT &= ~(1 << ISP_RST); /* RST low */
    ISP_OUT &= ~(1 << ISP_SCK); /* SCK low */

    /* positive reset pulse > 2 SCK (target) */
    clockWait(1); /* ~320 µs */
    ISP_OUT |= (1 << ISP_RST); /* RST high */
    clockWait(1);               /* 320us */
    ISP_OUT &= ~(1 << ISP_RST);/* RST low */
}

// Для SPI Flash (без reset)
void ispSPIConnect(void) {
    ispInitPins();
    CS_HI(); // Просто снимаем CS (RST = 1)
}

void ispDisconnect(void) {
 
    ISP_OUT |= (1 << ISP_RST);
    clockWait(1);
    
    ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
    ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
    
    spiHWdisable();  // Всегда безопасно, даже если SPI уже отключён
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
    
    return SPDR;  // Это и есть полученный байт
    
}

static uchar tryEnterProgMode(uchar speed_code)
{
    ispSetSCKOption(speed_code);

    /* Адаптивные тайминги сброса: длинные для SW (≤ 32 кГц), короткие для HW */
    uint8_t pulse = (speed_code <= USBASP_ISP_SCK_32) ? 15 : 1;   // 15 или 1 тик * 320 мкс
    uint8_t delay = (speed_code <= USBASP_ISP_SCK_32) ? 250 : 63; // 250 или 63 тика
  
    for (uchar tries = 3; tries > 0; tries--) {
        // RST pulse
        ISP_OUT |= (1 << ISP_RST);
        clockWait(pulse);
        ISP_OUT &= ~(1 << ISP_RST);
        clockWait(delay);  // ~20 мс для HW, ~80 мс для SW

        // Programming Enable
        ispTransmit(0xAC);
        ispTransmit(0x53);
        uchar check  = ispTransmit(0);
        uchar check2 = ispTransmit(0);

        if (check == 0x53 && check2 == 0x00) {
            return 0; /* успех */
        }
        clockWait(3);
    }
    return 1; /* не удалось */
}

/* 
 * Функция входа в режим программирования
 * Важно: user_speed_requested устанавливается при получении команды SETISPSCK
 */
uchar ispEnterProgrammingMode(void) {
    uchar rc;

    // 1. Если пользователь явно задал скорость (например -B 0.33)
    if (user_speed_requested) {
        rc = tryEnterProgMode(prog_sck);
        if (rc == 0) {
            last_success_speed = prog_sck;
            return 0;
        }
        // Если не удалась - НЕ ВЫХОДИМ С ОШИБКОЙ! 
        // Идем в общий цикл подбора, начиная со СЛЕДУЮЩЕЙ более медленной скорости.
    } 
    
    // 2. Если режим AUTO (пришел 0)
    else {
        // Сначала пробуем последнюю успешную скорость
        if (last_success_speed != USBASP_ISP_SCK_AUTO) {
            rc = tryEnterProgMode(last_success_speed);
            if (rc == 0) {
                prog_sck = last_success_speed;
                return 0;
            }
            // Если не удалась, сбрасываем кэш
            last_success_speed = USBASP_ISP_SCK_AUTO;
        }
    }
    
    // 3. Единый цикл автоподбора от ВЫСОКОЙ к НИЗКОЙ
    // Ищем стартовый индекс для цикла
    uint8_t start_idx = 0;
    
    if (user_speed_requested) {
        // Ищем, на каком месте в массиве находится скорость, которую запросил юзер
        for (uint8_t i = 0; i < ISP_SPEED_CNT; i++) {
            if (GET_SPEED(i) == prog_sck) {
                start_idx = i + 1; // Пробовали её в шаге 1 и не получилось. Начинаем со следующей (более медленной)
                break;
            }
        }
    }
    
    for (uchar i = start_idx; i < ISP_SPEED_CNT; i++) {
        uchar speed = GET_SPEED(i);
        
        rc = tryEnterProgMode(speed);
        if (rc == 0) {
            prog_sck = speed;
            last_success_speed = speed;
            return 0;
        }
    }
    
    return 1; // Ни одна скорость не подошла
}

// Вспомогательная функция для обновления расширенного адреса
void ispUpdateExtended(uint32_t address) {
    // Если чип маленький (ATmega8 и т.д.), avrdude не шлет SETLONGADDRESS,
    // и prog_address_newmode остается 0. Команду 0x4D отправлять нельзя!
    if (!prog_address_newmode) return; 

    // Извлекаем номер 128K сегмента (0, 1, 2, 3)
    // Для 256K чипов это будут значения 0 и 1.
    //    uint8_t ext = (addr >> 17) & 0x03;
    
    uint8_t ext = GET_EXT_ADDR(address);  

    // Если кэш не совпадает, отправляем команду 0x4D
    if (ext != isp_hiaddr) {
        isp_hiaddr = ext;
        ispTransmit(0x4D);  // Команда Load Extended Address
        ispTransmit(0x00);
        ispTransmit(ext);   // Отправляем именно номер сегмента (биты 1:0)
        ispTransmit(0x00);
    }
}

// Аргумент ДОЛЖЕН быть 32-битным!
uchar ispReadFlash(uint32_t address)
{
    // Явно приводим результаты сдвигов к uint8_t!
    // Компилятор сгенерирует быстрый код, но 17-й бит (в address >> 9) не потеряется
    ispTransmit(0x20 | ((address & 1) << 3));
    ispTransmit((uint8_t)(address >> 9));  // Здесь находится 17-й бит!
    ispTransmit((uint8_t)(address >> 1));
    return ispTransmit(0);
}

uchar ispWriteFlash(uint32_t address, uint8_t data, uint8_t pollmode)
{
    // ВАЖНО: address теперь uint32_t! И кастуем результаты сдвигов к uint8_t
    ispTransmit(0x40 | ((address & 1) << 3));
    ispTransmit((uint8_t)(address >> 9)); // Здесь сидит 17-й бит!
    ispTransmit((uint8_t)(address >> 1));
    ispTransmit(data);

    if (!pollmode) return 0;
    if (ispReadFlash(address) == data) return 0; 

    for (uint8_t t = 16; t; --t) {
        clockWait(t > 10 ? 1 : t > 7 ? 2 : 4);
        if (ispReadFlash(address) == data) return 0; 
    }
    return 1;
}

uchar ispFlushPage(uint32_t address) {
    ispTransmit(0x4C);
    ispTransmit((uint8_t)(address >> 9)); // И здесь 17-й бит!
    ispTransmit((uint8_t)(address >> 1));
    ispTransmit(0);

    for (uint8_t t = 25; t > 0; t--) {
        clockWait(t > 15 ? 1 : (t > 5 ? 2 : 4));
        if (ispReadFlash(address) != 0xFF) return 0;
    }
    return 1;
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

    // Мгновенная проверка (иногда чип невероятно быстр)
    if (ispReadEEPROM(address) == data) return 0; 

    // Прогрессивный опрос готовности (ваш вариант)
    for (uint8_t t = 6; t; --t) {
        clockWait(t > 4 ? 1 : t > 2 ? 2 : 4);
        if (ispReadEEPROM(address) == data) return 0; 
    }
    
    return 1; // Таймаут (ошибка записи)
}

