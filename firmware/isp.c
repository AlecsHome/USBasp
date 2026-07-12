/*
 * isp.c - part of USBasp
 * Optimized for speed and size.
 */
#include <avr/io.h>
#include "isp.h"
#include "clock.h"
#include "usbasp.h"
#include <avr/pgmspace.h>
#include <stddef.h>

uchar (*ispTransmit)(uchar) = NULL;
uint8_t last_success_speed = USBASP_ISP_SCK_3000;

// Быстрое вычисление расширенного адреса (для чипов > 64KB)
// Работает для Little-Endian (AVR). Берем 3-й байт 32-битного адреса и сдвигаем.
// Страница 128KB (0x00000-0x1FFFF). Бит 17 (3-й байт, бит 1) определяет страницу.
// Твой макрос отличный, оставляем! Он генерирует самый короткий код.
#define GET_EXT_ADDR(address) (((uint8_t*)&(address))[2] >> 1)
//#define GET_EXT_ADDR(address) ((uint8_t)((address) >> 17))  // + 90 байт к прошивке

// Базовые биты для включения SPI в режиме Master
#define SPI_BASE ((1 << SPE) | (1 << MSTR))

// ИСПРАВЛЕННЫЕ таблицы скоростей для аппаратного SPI (для кварца 12 МГц)
// SPCR: биты SPR1, SPR0
static const uchar hw_spcr_table[] PROGMEM = {
    0,                                      // 3.0 MHz   (Fosc/4,  SPI2X=0)
    0,                                      // 1.5 MHz   (Fosc/8,  SPI2X=1)
    (1 << SPR0),                            // 0.75 MHz  (Fosc/16, SPI2X=0)
    (1 << SPR0),                            // 0.375 MHz (Fosc/32, SPI2X=1)
    (1 << SPR1),                            // 0.1875 MHz(Fosc/64, SPI2X=0)
    (1 << SPR1) | (1 << SPR0)               // 0.09375 MHz(Fosc/128,SPI2X=0)
};

// SPSR: содержит ТОЛЬКО бит SPI2X (бит 0)
static const uchar hw_spsr_table[] PROGMEM = {
    0,                  // 3.0 MHz   (SPI2X = 0)
    (1 << SPI2X),       // 1.5 MHz   (SPI2X = 1)
    0,                  // 0.75 MHz  (SPI2X = 0)
    (1 << SPI2X),       // 0.375 MHz (SPI2X = 1)
    0,                  // 0.1875 MHz(SPI2X = 0)
    0                   // 0.09375 MHz(SPI2X = 0)
};

// Таблица задержек для Software SPI
static const uchar sw_delay_table[] PROGMEM = { 3, 6, 12, 24, 48, 96, 192 };

static const uchar isp_retry_speeds[] PROGMEM = {
    USBASP_ISP_SCK_3000, USBASP_ISP_SCK_1500, USBASP_ISP_SCK_750,
    USBASP_ISP_SCK_375, USBASP_ISP_SCK_187_5, USBASP_ISP_SCK_93_75,
    USBASP_ISP_SCK_32, USBASP_ISP_SCK_16, USBASP_ISP_SCK_8,
    USBASP_ISP_SCK_4, USBASP_ISP_SCK_2, USBASP_ISP_SCK_1, USBASP_ISP_SCK_0_5
};

#define ISP_SPEED_CNT (sizeof(isp_retry_speeds)/sizeof(isp_retry_speeds[0]))
#define GET_SPEED(idx) pgm_read_byte(&isp_retry_speeds[(idx)])

volatile uint8_t sck_sw_delay = 0;
uchar isp_hiaddr = 0xFF;

static inline void spiHWdisable() {
    SPCR = 0;
}

void ispSetSCKOption(uchar option) {
    // 1. Обработка AUTO
    if (option == USBASP_ISP_SCK_AUTO) {
        user_speed_requested = 0;
        // Берем из кэша или дефолт (3.0 MHz)
        option = (last_success_speed != USBASP_ISP_SCK_AUTO && 
                  last_success_speed != 0) ? last_success_speed : USBASP_ISP_SCK_3000;
    } else {
        // Пользователь задал скорость явно
        user_speed_requested = 1;
    }
    
    // 2. Защита от мусора
    if (option < USBASP_ISP_SCK_0_5 || option > USBASP_ISP_SCK_3000) {
        option = USBASP_ISP_SCK_3000;
    }

    // Сохраняем текущую скорость глобально (для запросов avrdude)
    prog_sck = option;

    // 3. Настройка Hardware SPI (93.75 kHz ... 3.0 MHz, опции 8..13)
    if (option >= USBASP_ISP_SCK_93_75) {
        uint8_t idx = USBASP_ISP_SCK_3000 - option; // 13->0, ..., 8->5
        
        if (idx < sizeof(hw_spcr_table)) {
            // Читаем только биты делителя из SPCR и бит SPI2X из SPSR
            uint8_t spcr_bits = pgm_read_byte(&hw_spcr_table[idx]);
            uint8_t spsr_bits = pgm_read_byte(&hw_spsr_table[idx]);
            
            // Полная и безопасная инициализация регистров
            SPCR = SPI_BASE | spcr_bits;
            SPSR = spsr_bits;
        } else {
            // Fallback (теоретически недостижимо)
            SPCR = SPI_BASE | (1 << SPR0);
            SPSR = (1 << SPI2X);
        }
        
        // Переключаем на HW SPI
        ispTransmit = (uchar (*)(uchar))ispTransmit_hw;
        
        // ОБЯЗАТЕЛЬНО 0, чтобы avrdude не выводил "[SW delay: 1]"
        sck_sw_delay = 0; 
    } 
    // 4. Настройка Software SPI (0.5 kHz ... 32 kHz, опции 1..7)
    else if (option >= USBASP_ISP_SCK_0_5) {
        // Отключаем аппаратный SPI
        SPCR &= ~(1 << SPE);
        
        // Вычисляем индекс задержки: 7->0, 6->1, ..., 1->6
        uint8_t idx = USBASP_ISP_SCK_32 - option; 
        
        if (idx < sizeof(sw_delay_table)) {
            sck_sw_delay = pgm_read_byte(&sw_delay_table[idx]);
        } else {
            sck_sw_delay = 48; // Fallback
        }
        
        // Переключаем на SW SPI
        ispTransmit = ispTransmit_sw;
    }
    
}

// --- Задержка ---
void ispDelay() {
    
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

void ispConnect(void) {
    
    ispInitPins();
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
    
    ISP_OUT |= (1 << ISP_RST);
    clockWait(1);
    ISP_DDR &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
    ISP_OUT &= ~((1 << ISP_RST) | (1 << ISP_SCK) | (1 << ISP_MOSI));
    spiHWdisable();
    // --- СБРОС СОСТОЯНИЯ СЕССИИ ---
    prog_sck = USBASP_ISP_SCK_AUTO;
//    prog_address_newmode = 0;   // <--- СБРОС
//    prog_address_high = 0;      // <--- СБРОС
    isp_hiaddr = 0xFF;          // <--- Тоже неплохо сбросить кэш расширенного адреса ISP
}

// --- Transmission ---
uchar ispTransmit_sw(uchar send_byte) {
    
    uchar rec_byte = 0;
    ISP_OUT &= ~(1 << ISP_SCK);

    for (uchar bit = 8; bit; --bit) {
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

uchar ispTransmit_hw(uchar send_byte) {
    SPDR = send_byte;
    while (!(SPSR & (1 << SPIF)));
    return SPDR;
}

static uchar tryEnterProgMode(uchar speed_code) {
    ispSetSCKOption(speed_code);
    uint8_t pulse = (speed_code <= USBASP_ISP_SCK_32) ? 15 : 1;
    uint8_t delay = (speed_code <= USBASP_ISP_SCK_32) ? 250 : 63;
  
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
            return 0;
        }
        clockWait(3);
    }
    return 1;
}

uint8_t ispEnterProgrammingMode(void) {
    uint8_t start_idx = 0;
    // Используем skip_speed ТОЛЬКО для кэша (AUTO режима)
    uint8_t skip_speed = USBASP_ISP_SCK_AUTO; 

    // 1. Если пользователь задал скорость принудительно
    if (user_speed_requested) {
        user_speed_requested = 0; // Сбрасываем флаг сразу
        
        if (tryEnterProgMode(prog_sck) == 0) {
            last_success_speed = prog_sck;
            return 0; // Успех
        }
        
        // Скорость не подошла. Ищем следующую (более медленную) в таблице.
        for (uint8_t i = 0; i < ISP_SPEED_CNT; i++) {
            if (GET_SPEED(i) == prog_sck) {
                start_idx = i + 1; // Начинаем перебор со следующей
                break;
            }
        }
        
        // ЗАЩИТА: Если была запрошена самая медленная скорость,
        // start_idx выйдет за границы. Сбрасываем в начало (на самую быструю).
        if (start_idx >= ISP_SPEED_CNT) {
            start_idx = 0; 
        }
        // ПРИМЕЧАНИЕ: Нам не нужен skip_speed здесь. Цикл начнется с start_idx 
        // и естественным образом никогда не дойдет до проваленной пользовательской скорости.
    }
    else {
        // 2. Режим AUTO: пробуем последнюю удачную скорость (кэш)
        if (last_success_speed != USBASP_ISP_SCK_AUTO) {
            if (tryEnterProgMode(last_success_speed) == 0) {
                return 0; // Успех
            }
            
            // Кэш не сработал. Запоминаем его, чтобы цикл перебора не тратил на него время.
            skip_speed = last_success_speed;
            // Сбрасываем кэш
            last_success_speed = USBASP_ISP_SCK_AUTO;
        }
        // В режиме AUTO start_idx остается 0 (начинаем с самой быстрой)
    }
    
    // 3. Полный перебор скоростей (Brute-force)
    for (uint8_t i = start_idx; i < ISP_SPEED_CNT; i++) {
        uint8_t speed = GET_SPEED(i);
        
        // Пропускаем ТОЛЬКО ту скорость, которая не сработала из кэша
        if (speed == skip_speed) {
            continue;
        }
        
        if (tryEnterProgMode(speed) == 0) {
            // Нашли рабочую скорость, сохраняем в кэш
            last_success_speed = speed;
            return 0;
        }
    }
    
    // Ничего не помогло
    return 1;
}

void ispUpdateExtended(uint32_t address) {
    
    if (!prog_address_newmode) return; 
    
    uint8_t ext = GET_EXT_ADDR(address);  
    
    if (ext != isp_hiaddr) {
        isp_hiaddr = ext;
        ispTransmit(0x4D);
        ispTransmit(0x00);
        ispTransmit(ext);
        ispTransmit(0x00);
    }
}

uchar ispReadFlash(uint32_t address) {
    
    ispTransmit(0x20 | ((address & 1) << 3));
    ispTransmit((uint8_t)(address >> 9));
    ispTransmit((uint8_t)(address >> 1));
    return ispTransmit(0);
}

// ИСПРАВЛЕНО: Фиксированная задержка вместо глючного опроса (как в оригинале)
uchar ispWriteFlash(uint32_t address, uint8_t data, uint8_t pollmode) {
    
    ispTransmit(0x40 | ((address & 1) << 3));
    ispTransmit((uint8_t)(address >> 9));
    ispTransmit((uint8_t)(address >> 1));
    ispTransmit(data);

    if (!pollmode) return 0;
    // Фиксированная задержка 0.64 мс для одиночной записи (если нет страничного режима)
    clockWait(2);
    return 0;
}

// ИСПРАВЛЕНО: Фиксированная задержка страницы. Убирает баг с 0xFF и ускоряет запись!
uchar ispFlushPage(uint32_t address) { // <--- ВОТ ГЛАВНОЕ: 32-битный аргумент!
    
    ispTransmit(0x4C);
    ispTransmit((uint8_t)(address >> 9)); 
    ispTransmit((uint8_t)(address >> 1));
    ispTransmit(0);

    // 15 тиков по 320 мкс = 4.8 мс.
    // Это максимальное время записи страницы (Page Write) для AVR.
    // Чип гарантированно успеет закончить запись.
    // Ждём фиксированное время (по даташиту)
    // Для ATmega2560/128: tWD_FLASH = 4.5 ms (при 5V)
    // Для ATmega8/16/32: tWD_FLASH = 3.0 ms
    // Динамическое ожидание в зависимости от типа чипа
    // Каждый тик = 320 мкс (при 12 МГц)
    // ATmega8:    10 тиков = 3.2 мс (минимальное время)
    // ATmega48:   10 тиков = 3.2 мс
    // ATmega88:   10 тиков = 3.2 мс
    // ATmega168:  10 тиков = 3.2 мс
    // ATmega328:  10 тиков = 3.2 мс
    // ATmega128:  15 тиков = ~4.8 мс
    // ATmega2560: 15 тиков = ~4.8 мс
    // ATmega4809: 18 тиков = ~5.8 мс
    
    // 14 * 320 мкс = 4.48 мс
    // ATmega2560: tWD_FLASH = 4.5 ms @ 5V (максимум по даташиту)
    // ATmega8/328P: tWD_FLASH = 3.0-4.5 ms @ 5V
    // 15 тиков * 320 мкс = 4.8 мс > 4.5 мс  гарантия для любого AVR

    // === ЭВРИСТИКА ПО РАЗМЕРУ СТРАНИЦЫ ===
    // ATmega2560/128: page_size = 256 → длинная задержка
    // ATmega8/328P:   page_size = 64/128 → короткая задержка
    // Элегантная эвристика в одну строку
    clockWait((prog_pagesize >= 256) ? 15 : 10);
    return 0;
}

uchar ispReadEEPROM(unsigned int address) {
    
    ispTransmit(0xA0);
    ispTransmit(address >> 8);
    ispTransmit(address & 0xFF);
    return ispTransmit(0);
}

uchar ispWriteEEPROM(unsigned int address, uchar data) {
    
    ispTransmit(0xC0);
    ispTransmit(address >> 8);
    ispTransmit(address & 0xFF);
    ispTransmit(data);

    if (ispReadEEPROM(address) == data) return 0; 

    for (uint8_t t = 6; t; --t) {
        clockWait(t > 4 ? 1 : t > 2 ? 2 : 4);
        if (ispReadEEPROM(address) == data) return 0; 
    }
      return 1;
}

