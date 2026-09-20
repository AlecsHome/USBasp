/*
 * isp.c - part of USBasp
 * Optimized for speed and size.
 */
#include <avr/io.h>
#include "isp.h"
#include "clock.h"
#include "usbasp.h"
#include "i2c.h"
#include <avr/pgmspace.h>
#include <stddef.h>

// --- Безопасная заглушка для предотвращения Hard Fault ---
// Если ISP команды придут до инициализации скорости, 
// контроллер не зависнет, а вернет 0xFF (имитация пустой SPI шины).
static uint8_t dummy_transmit(uint8_t b) {
    (void) b; // <--- ДОБАВИТЬ ЭТУ СТРОКУ: говорим компилятору "я знаю, что не использую"
    return 0xFF;
}

// Private data / Globals
uint8_t (*ispTransmit)(uint8_t) = dummy_transmit; // <--- ИНИЦИАЛИЗИРУЕМ ЗАГЛУШКОЙ
uint8_t last_success_speed = USBASP_ISP_SCK_3000;


// Базовые биты для включения SPI в режиме Master
#define SPI_BASE ((1 << SPE) | (1 << MSTR))

// ИСПРАВЛЕННЫЕ таблицы скоростей для аппаратного SPI (для кварца 12 МГц)
// SPCR: биты SPR1, SPR0
static const uint8_t hw_spcr_table[] PROGMEM = {
    0,                                        // 3.0 MHz    f/4,   SPI2X=0
    (1 << SPR0),                              // 1.5 MHz    f/8,   SPI2X=1  ← БЫЛО 0
    (1 << SPR0),                              // 0.75 MHz   f/16,  SPI2X=0
    (1 << SPR1),                              // 0.375 MHz  f/32,  SPI2X=1  ← БЫЛО (1<<SPR0)
    (1 << SPR1),                              // 0.1875 MHz f/64,  SPI2X=0
    (1 << SPR1) | (1 << SPR0)                 // 0.09375 MHz f/128, SPI2X=0
};

static const uint8_t hw_spsr_table[] PROGMEM = {
    0,                                        // 3.0 MHz
    (1 << SPI2X),                             // 1.5 MHz
    0,                                        // 0.75 MHz
    (1 << SPI2X),                             // 0.375 MHz
    0,                                        // 0.1875 MHz
    0                                         // 0.09375 MHz
};

// Таблица задержек для Software SPI
static const uint8_t sw_delay_table[] PROGMEM = { 3, 6, 12, 24, 48, 96, 192 };

static const uint8_t isp_retry_speeds[] PROGMEM = {
    USBASP_ISP_SCK_3000, USBASP_ISP_SCK_1500, USBASP_ISP_SCK_750,
    USBASP_ISP_SCK_375, USBASP_ISP_SCK_187_5, USBASP_ISP_SCK_93_75,
    USBASP_ISP_SCK_32, USBASP_ISP_SCK_16, USBASP_ISP_SCK_8,
    USBASP_ISP_SCK_4, USBASP_ISP_SCK_2, USBASP_ISP_SCK_1, USBASP_ISP_SCK_0_5
};

#define ISP_SPEED_CNT (sizeof(isp_retry_speeds)/sizeof(isp_retry_speeds[0]))
#define GET_SPEED(idx) pgm_read_byte(&isp_retry_speeds[(idx)])

uint8_t sck_sw_delay = 0;
uint8_t isp_hiaddr = 0xFF;

static inline void spiHWdisable(void) {
    SPCR &= ~((1 << SPE) | (1 << MSTR)); // 1. Хирургическое отключение
    (void)SPSR;  // 2. Сброс флага прерывания
    (void)SPDR;  // 3. Сброс флага прерывания
}

void ispSetSCKOption(uint8_t option) {
    // 1. Обработка AUTO
    if (option == USBASP_ISP_SCK_AUTO) {
        option = (last_success_speed != USBASP_ISP_SCK_AUTO && 
                  last_success_speed != 0) ? last_success_speed : USBASP_ISP_SCK_3000;
    }
    
    // 2. Защита от мусора (отсекаем всё, кроме 1..13)
    if (option < USBASP_ISP_SCK_0_5 || option > USBASP_ISP_SCK_3000) {
        option = USBASP_ISP_SCK_3000;
    }

    prog_sck = option;

    // 3. Аппаратный SPI (опции 8..13)
    if (option >= USBASP_ISP_SCK_93_75) {
        ispTransmit = ispTransmit_hw;      // ← без каста
        sck_sw_delay = 0; 

        uint8_t idx = USBASP_ISP_SCK_3000 - option; 
        
        // SPCR можно писать напрямую
        SPCR = SPI_BASE | pgm_read_byte(&hw_spcr_table[idx]);
        
        // ИСПРАВЛЕНО: Элегантный Read-Modify-Write в одну строку.
        // ~(1 << SPI2X) создает маску 0xFE (11111110).
        // SPSR & 0xFE сбрасывает бит SPI2X, сохраняя SPIF и WCOL нетронутыми.
        // | spsr_mask записывает новое значение SPI2X из таблицы.
        uint8_t spsr_mask = pgm_read_byte(&hw_spsr_table[idx]);
        SPSR = (SPSR & ~(1 << SPI2X)) | spsr_mask;
        
        // Сброс флага SPIF
        (void)SPSR;
        (void)SPDR;
    }  
    // 4. Программный SPI (опции 1..7)
    else {
        ispTransmit = ispTransmit_sw; // ← без каста
        SPCR = 0; // Полностью отключаем HW SPI
        
        uint8_t idx = USBASP_ISP_SCK_32 - option; // Всегда 0..6
        sck_sw_delay = pgm_read_byte(&sw_delay_table[idx]);
    }
}

// --- Задержка ---
void ispDelay(void) {
    
    uint8_t starttime = TIMERVALUE;
    while ((uint8_t)(TIMERVALUE - starttime) < sck_sw_delay) {
    }
}

// --- Подключение ---
static inline void ispInitPins(void) {
    
    ISP_DDR |= (1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI);
    ISP_DDR &= ~(1 << ISP_MISO); 
    isp_hiaddr = 0xFF;
}

void ispResetState(void){
    prog_address_newmode = 0;
    prog_address_high    = 0;
    prog_pagesize 	 = 0;
    isp_hiaddr           = 0xFF;   // заставит первый 0x4D уйти гарантированно
}

void ispConnect(void) {
    
    ispInitPins();
    ispResetState();

    ISP_OUT &= ~(1 << ISP_RST);
    ISP_OUT &= ~(1 << ISP_SCK);
    clockWait(1); 
    ISP_OUT |= (1 << ISP_RST);
    clockWait(1);               
    ISP_OUT &= ~(1 << ISP_RST);

}

void ispSPIConnect(void) {
    ispInitPins();
    CS_HI(); 
}

void ispDisconnect(void) {
    
    ispResetState();

    ISP_OUT |= (1 << ISP_RST);
    clockWait(1);
    ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
    ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
    spiHWdisable();
    
    // ГАРАНТИРОВАННОЕ ОСВОБОЖДЕНИЕ ШИНЫ I2C
    i2c_stop();
    prog_sck = USBASP_ISP_SCK_AUTO;
    prog_state = PROG_STATE_IDLE;
    prog_pagecounter = 0;
    prog_nbytes = 0;
    
}

// --- Transmission ---
uint8_t ispTransmit_sw(uint8_t send_byte) {
    
    uint8_t rec_byte = 0;
    ISP_OUT &= ~(1 << ISP_SCK);

    for (uint8_t bit = 8; bit; --bit) {
        if (send_byte & 0x80)
            ISP_OUT |=  (1 << ISP_MOSI);
        else
            ISP_OUT &= ~(1 << ISP_MOSI);
        send_byte <<= 1;

        ispDelay();
        ISP_OUT |= (1 << ISP_SCK);
        ispDelay();
        rec_byte = (rec_byte << 1) | ((ISP_IN >> ISP_MISO) & 1);
        ISP_OUT &= ~(1 << ISP_SCK);
        ispDelay();
    }
    return rec_byte;
}

uint8_t ispTransmit_hw(uint8_t send_byte) {
    SPDR = send_byte;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

static uint8_t tryEnterProgMode(uint8_t speed_code) {
    ispSetSCKOption(speed_code);
    
    /* pulse: 2·320µs (fast) / 16·320µs (slow); 
    startup wait: 100·320µs = 32ms => 20ms datasheet */
    // Безопасные задержки: 16 тиков импульс, 100 тиков ожидание
    uint8_t pulse = (speed_code <= USBASP_ISP_SCK_32) ? 16 : 2;
    uint8_t delay = 100;  
    
    for (uint8_t tries = 5; tries > 0; tries--) {
        ISP_OUT |= (1 << ISP_RST);
        clockWait(pulse);
        ISP_OUT &= ~(1 << ISP_RST);
        clockWait(delay);

        isp_hiaddr = 0xFF;
        
        ispTransmit(0xAC);
        ispTransmit(0x53);
        uint8_t check = ispTransmit(0x00);
        ispTransmit(0x00); // 4-й байт отправляем, но не проверяем!
        
        // Доверяем эху. Если 0x53 прошел, связь установлена.
        if (check == 0x53) {
	    // --- УМНАЯ ПРОВЕРКА СТАБИЛЬНОСТИ ШИНЫ ---
            // Если эхо прошло, пробуем прочитать 1-й байт сигнатуры.
            // У всех AVR 1-й байт сигнатуры равен 0x1E.
            // Если длинный кабель вносит помехи, мы прочтем мусор (например 0x00).
            ispTransmit(0x30); // Команда Read Signature Byte
            ispTransmit(0x00);
            ispTransmit(0x00);
            uint8_t sig = ispTransmit(0x00);
            
            // Если сигнатура вернулась корректно, значит линия стабильна!
            if (sig == 0x1E) {
                return 0; // Истинный успех! Линия надежна.
            }
            // Если sig != 0x1E, значит текущая скорость слишком высока для этого кабеля.
            // Мы НЕ возвращаем успех. Цикл сделает еще попытку, а затем 
            // алгоритм авто-подбора снизит скорость до 1.5 МГц и ниже.
        }
        
        clockWait(3);
    }
    return 1;
}

uint8_t ispEnterProgrammingMode(void) {
    uint8_t start_idx = 0;
    uint8_t initial_speed = USBASP_ISP_SCK_AUTO;

    // 1. Определяем приоритетную скорость для первой попытки
    if (user_speed_requested) {
        user_speed_requested = 0; // Сбрасываем флаг
        initial_speed = prog_sck;
    } else if (last_success_speed != USBASP_ISP_SCK_AUTO) {
        initial_speed = last_success_speed;
    }

    // 2. Пробуем приоритетную скорость (если она не AUTO)
    if (initial_speed != USBASP_ISP_SCK_AUTO) {
        if (tryEnterProgMode(initial_speed) == 0) {
            prog_sck = initial_speed;      // ← синхронизировать переменную с реальностью
            last_success_speed = initial_speed;
            return 0; // Успех!
        }
        
        // Провал. Ищем индекс в таблице, чтобы начать перебор со следующей (более медленной).
        for (uint8_t i = 0; i < ISP_SPEED_CNT; i++) {
            uint8_t speed = GET_SPEED(i);
            
            if (speed == initial_speed) {
                start_idx = i + 1; // Точное совпадение. Начинаем со следующей.
                break;
            } 
            // ЗАЩИТА: Если запрошенной скорости нет в таблице,
            // находим первую доступную скорость, которая МЕДЛЕННЕЕ запрошенной.
            else if (speed < initial_speed) {
                start_idx = i;
                break;
            }
        }
        
        // Если кэш сбойнул, сбрасываем его
        if (initial_speed == last_success_speed) {
            last_success_speed = USBASP_ISP_SCK_AUTO;
        }
    }

    // 3. Полный перебор скоростей от start_idx до самой медленной
    for (uint8_t i = start_idx; i < ISP_SPEED_CNT; i++) {
        uint8_t speed = GET_SPEED(i);
        
        if (tryEnterProgMode(speed) == 0) {
            last_success_speed = speed; // Сохранили в кэш
            return 0; 
        }
    }

    return 1; // Полный провал
}
void ispUpdateExtended(uint8_t ext_addr) {

    if (!prog_address_newmode) return; 

    if (ext_addr != isp_hiaddr) {
        isp_hiaddr = ext_addr;
        ispTransmit(0x4D);
        ispTransmit(0x00);
        ispTransmit(ext_addr);
        ispTransmit(0x00);
    }
}

uint8_t ispReadFlash(uint32_t address) {
    uint8_t *p = (uint8_t*)&address;
    
    ispTransmit(0x20 | ((p[0] & 1) << 3));
    ispTransmit((p[1] >> 1) | ((p[2] & 1) << 7));
    ispTransmit((p[0] >> 1) | ((p[1] & 1) << 7));
    return ispTransmit(0);
}

uint8_t ispWriteFlash(uint32_t address, uint8_t data, uint8_t pollmode)
{
    uint8_t *p = (uint8_t*)&address;
    
    ispTransmit(0x40 | ((p[0] & 1) << 3));
    ispTransmit((p[1] >> 1) | ((p[2] & 1) << 7));
    ispTransmit((p[0] >> 1) | ((p[1] & 1) << 7));
    ispTransmit(data);

    if (!pollmode) return 0;

    // УМНЫЙ ОПРОС: Увеличен таймаут до 20 (~6.4 мс)
    if (data != 0xFF) {
        for (uint8_t t = 0; t < 20; t++) {
            if (ispReadFlash(address) == data) return 0;
            clockWait(1); 
        }
    }
    clockWait(15); // Слепая задержка 4.8 мс для 0xFF
    return 0;
}

// ИСПРАВЛЕНО: Добавлена передача 17-го бита адреса для ATmega2560
uint8_t ispFlushPage(uint32_t address) {
    uint8_t *p = (uint8_t*)&address;
    
    if (prog_pagesize > 0) {
        uint16_t mask = ~(uint16_t)(prog_pagesize - 1);
        p[0] &= (uint8_t)mask;
        p[1] &= (uint8_t)(mask >> 8);
    }
    
    ispTransmit(0x4C);
    ispTransmit((p[1] >> 1) | ((p[2] & 1) << 7));
    ispTransmit((p[0] >> 1) | ((p[1] & 1) << 7));
    ispTransmit(0);

    clockWait(15);
    return 0;
}

uint8_t ispReadEEPROM(uint16_t address) {
    
    ispTransmit(0xA0);
    ispTransmit((uint8_t)(address >> 8));
    ispTransmit((uint8_t)(address));
    return ispTransmit(0);
}

uint8_t ispWriteEEPROM(uint16_t address, uint8_t data) {
    ispTransmit(0xC0);
    ispTransmit((uint8_t)(address >> 8));
    ispTransmit((uint8_t)address);
    ispTransmit(data);

    // УВЕЛИЧЕННЫЙ ТАЙМАУТ: 25 итераций (~8 мс)
    for (uint8_t t = 25; t; --t) {
        clockWait(1); 
        if (ispReadEEPROM(address) == data) {
            return 0; // Запись успешна
        }
    }

    return 1; // Таймаут: EEPROM не ответила за 8 мс
}
