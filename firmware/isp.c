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

// --- Безопасная заглушка для предотвращения Hard Fault ---
// Если ISP команды придут до инициализации скорости, 
// контроллер не зависнет, а вернет 0xFF (имитация пустой SPI шины).
static uchar dummy_transmit(uchar b) {
    (void) b; // <--- ДОБАВИТЬ ЭТУ СТРОКУ: говорим компилятору "я знаю, что не использую"
    return 0xFF;
}

// Private data / Globals
uchar (*ispTransmit)(uchar) = dummy_transmit; // <--- ИНИЦИАЛИЗИРУЕМ ЗАГЛУШКОЙ
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

uint8_t sck_sw_delay = 0;
uchar isp_hiaddr = 0xFF;

static inline void spiHWdisable(void) {
    SPCR &= ~((1 << SPE) | (1 << MSTR)); // 1. Хирургическое отключение
    (void)SPSR;  // 2. Сброс флага прерывания
    (void)SPDR;  // 3. Сброс флага прерывания
}

void ispSetSCKOption(uchar option) {
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
        // 3. Аппаратный SPI (опции 8..13)
    if (option >= USBASP_ISP_SCK_93_75) {
        ispTransmit = (uchar (*)(uchar))ispTransmit_hw;
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
        ispTransmit = ispTransmit_sw;
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
    prog_address_newmode = 0;   // <--- СБРОС
    prog_address_high = 0;      // <--- СБРОС
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
    // Уменьшаем задержку с 63 (20 мс) до 20 (6.4 мс). 
    // Этого более чем достаточно по даташиту, и это сильно ускорит перебор скоростей!
    uint8_t delay = 250; 
    
    for (uchar tries = 3; tries > 0; tries--) {
        ISP_OUT |= (1 << ISP_RST);
        clockWait(pulse);
        ISP_OUT &= ~(1 << ISP_RST);
        clockWait(delay);
        
        ispTransmit(0xAC);
        ispTransmit(0x53);
        uint8_t check = ispTransmit(0x00);
        ispTransmit(0x00); // <-- ВАЖНО: Четвертый байт отправляем, но НЕ ПРОВЕРЯЕМ!
        
        // По спецификации AVR ISP проверяем ТОЛЬКО третий байт (должен быть 0x53)
        if (check == 0x53) {
            return 0; // Успех
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
    
    // --- O(1) ВЫЧИСЛЕНИЕ ИНДЕКСА ---
    // Таблица isp_retry_speeds убывающая: 13, 12, 11, 10...
    // Формула индекса: Индекс = Максимальный_ID (13) - Запрошенный_ID
    if (prog_sck >= USBASP_ISP_SCK_3000) {
      // Защита от мусора: если пришла цифра больше 13 (например 255), 
      // вычитание уйдет в минус. Начинаем с самой быстрой.
      start_idx = 0; 
    } else {
      start_idx = USBASP_ISP_SCK_3000 - prog_sck;
      
      // Защита от выхода за пределы массива (если скорость < 500 Гц, 
      // чего нет в таблице, индекс будет >= 13)
      if (start_idx >= ISP_SPEED_CNT) {
        start_idx = 0; 
      }
    }
    // Примечание: Здесь НЕ нужен skip_speed. Цикл начнется со start_idx, 
    // который уже указывает на скорость СЛЕДУЮЩУЮ за проваленной.
  }
  else {
    // 2. Режим AUTO: пробуем последнюю удачную скорость (из кэша)
    // Проверка != 0 защищает от ситуации, если кэш испорчен
    if (last_success_speed != USBASP_ISP_SCK_AUTO && last_success_speed != 0) {
      if (tryEnterProgMode(last_success_speed) == 0) {
        return 0; // Успех
      }
      
      // Кэш не сработал. Запоминаем его, чтобы цикл перебора не тратил на него время.
      skip_speed = last_success_speed;
      // Сбрасываем кэш
      last_success_speed = USBASP_ISP_SCK_AUTO;
    }
    // В режиме AUTO start_idx остается 0 (начинаем с самой быстрой - 3 МГц)
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

uchar ispReadFlash(uint32_t address) {
    
    ispTransmit(0x20 | ((address & 1) << 3));
    ispTransmit((uint8_t)(address >> 9));
    ispTransmit((uint8_t)(address >> 1));
    return ispTransmit(0);
}

// ИСПРАВЛЕНО: Фиксированная задержка вместо глючного опроса (как в оригинале)
uchar ispWriteFlash(uint32_t address, uint8_t data, uint8_t pollmode)
{
    ispTransmit(0x40 | ((address & 1) << 3));
    ispTransmit((uint8_t)(address >> 9));
    ispTransmit((uint8_t)(address >> 1));
    ispTransmit(data);

    if (!pollmode) return 0;

    // УМНЫЙ ОПРОС (Smart Polling)
    if (data != 0xFF) {
        // Если пишем не 0xFF, можно безопасно опрашивать чип.
        for (uint8_t t = 0; t < 15; t++) {
            if (ispReadFlash(address) == data) return 0; // Чип готов!
            clockWait(1); // Ждем 0.32 мс перед следующей попыткой
        }
    }
    
    // Если data == 0xFF (нельзя опрашивать) или если истек таймаут опроса:
    // Используем безопасную фиксированную задержку 4.8 мс.
    clockWait(15);
    return 0;
}

// ИСПРАВЛЕНО: Добавлена передача 17-го бита адреса для ATmega2560
uchar ispFlushPage(uint32_t address) {
    ispTransmit(0x4C);
    
    // Если чип большой (страница >= 256), извлекаем 17-й бит (E) быстрым макросом.
    // Если чип маленький, E всегда равен 0 (экономия тактов на ATmega8/328).
    uint8_t ext_bit = (prog_pagesize >= 256) ? GET_EXT_ADDR(address) : 0;
    
    // Достаем биты 9..15 байтового адреса напрямую из памяти (без 32-битных сдвигов).
    uint8_t high_byte = (((uint8_t*)&(address))[1]) >> 1;
    
    // Склеиваем E в 7-й бит, HHHHHHH в младшие 7 бит.
    // Для ATmega8/328p это всегда 0 | high_byte, код на 100% обратно совместим!
    ispTransmit((ext_bit << 7) | high_byte); 
    
    ispTransmit((uint8_t)(address >> 1));
    ispTransmit(0);
    
    // Точная задержка по размеру страницы (3.52 мс для мелких, 4.8 мс для больших)
    clockWait((prog_pagesize >= 256) ? 16 : 12);
    return 0;
}

uchar ispReadEEPROM(unsigned int address) {
    
    ispTransmit(0xA0);
    ispTransmit((uint8_t)(address >> 8));
    ispTransmit((uint8_t)(address));
    return ispTransmit(0);
}

uchar ispWriteEEPROM(unsigned int address, uchar data) {
    ispTransmit(0xC0);
    ispTransmit((uint8_t)(address >> 8));
    ispTransmit((uint8_t)address); // & 0xFF не нужен, каст усечет сам
    ispTransmit(data);

    // Поллинг готовности EEPROM с задержкой внутри цикла
    for (uint8_t t = 15; t; --t) {
        clockWait(1); // Даем EEPROM время на запись перед каждой попыткой чтения
        if (ispReadEEPROM(address) == data) {
            return 0; // Запись успешна
        }
    }

    return 1; // Таймаут: EEPROM не ответила за 6.3 мс (возможно, чип мертв)
}

/*
    // 15 тиков по 320 мкс = 4.8 мс.
    // Это максимальное время записи страницы (Page Write) для AVR.
    // Чип гарантированно успеет закончить запись.
    // Ждём фиксированное время (по даташиту)
    // Для ATmega2560/128: tWD_FLASH = 4.5 ms (при 5V)
    // Для ATmega8/16/32: tWD_FLASH = 3.0 ms
    // Динамическое ожидание в зависимости от типа чипа
    // Каждый тик = 320 мкс (при 12 МГц)
    // ATmega8:  10 тиков = 3.2 мс (минимальное время)
    // ATmega48: 10 тиков = 3.2 мс
    // ATmega88: 10 тиков = 3.2 мс
    // ATmega168: 10 тиков = 3.2 мс
    // ATmega328: 10 тиков = 3.2 мс
    // ATmega128: 15 тиков = 4.8 мс
    // ATmega2560: 15 тиков = 4.8 мс
    // 14 * 320 мкс = 4.48 мс
    // ATmega2560: tWD_FLASH = 4.5 ms @ 5V (максимум по даташиту)
    // ATmega8/328P: tWD_FLASH = 3.0-4.5 ms @ 5V
    // 15 тиков * 320 мкс = 4.8 мс > 4.5 мс  гарантия для любого AVR

    // === ЭВРИСТИКА ПО РАЗМЕРУ СТРАНИЦЫ ===
    // ATmega2560/128: page_size = 256 → длинная задержка
    // ATmega8/328P:   page_size = 64/128 → короткая задержка
    // Элегантная эвристика в одну строку
    clockWait((prog_pagesize >= 256) ? 15 : 10);                
*/

