#include <avr/io.h>
#include "clock.h"
#include <util/delay.h>
#include "stdio.h"
#include "stdlib.h"
#include "i2c.h"
#include "isp.h"

#define I2C_SDA_PIN           ISP_MISO                     //линия SDA
#define I2C_SCL_PIN           ISP_MOSI                     //линия SCL
                                                           //порт входа
#define I2C_SDA_PORT_READ     ISP_IN                       //порт входа
#define I2C_SCL_PORT_READ     ISP_IN

#define I2C_SDA_PORT_DIR      ISP_DDR                      //порт направления
#define I2C_SCL_PORT_DIR      ISP_DDR                      //порт направления
#define I2C_SDA_PORT          ISP_OUT                      //порт выхода
#define I2C_SCL_PORT          ISP_OUT                      //порт выхода

#define SET(reg, bit) ((reg) |= (1U << (bit)))
#define CLR(reg, bit) ((reg) &= ~(1U << (bit)))
#define GETBIT(byte, bit) ((byte >> bit) & 1)

#define I2C_SDA_LOW() do { \
    SET(I2C_SDA_PORT_DIR, I2C_SDA_PIN); \
    CLR(I2C_SDA_PORT, I2C_SDA_PIN); \
} while (0)

#define I2C_SDA_HIGH() do { \
    CLR(I2C_SDA_PORT_DIR, I2C_SDA_PIN); \
    SET(I2C_SDA_PORT, I2C_SDA_PIN); \
} while (0)

#define I2C_SCL_LOW() do { \
    SET(I2C_SCL_PORT_DIR, I2C_SCL_PIN); \
    CLR(I2C_SCL_PORT, I2C_SCL_PIN); \
} while (0)

#define I2C_SCL_HIGH() do { \
    CLR(I2C_SCL_PORT_DIR, I2C_SCL_PIN); \
    SET(I2C_SCL_PORT, I2C_SCL_PIN); \
} while (0)

#define I2C_SDA_VALUE (GETBIT(I2C_SDA_PORT_READ, I2C_SDA_PIN))
#define I2C_SCL_VALUE (GETBIT(I2C_SCL_PORT_READ, I2C_SCL_PIN))

// Оптимизированные задержки
//#define I2C_DELAY_FAST _delay_us(2)
//#define I2C_DELAY_NORMAL _delay_us(4)
static inline void i2c_delay_normal(void) { _delay_us(5); }
static inline void i2c_delay_fast(void) { _delay_us(2); }

void i2c_init() {
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
}

void i2c_start() {
    I2C_SDA_HIGH();
    I2C_SCL_HIGH();
    i2c_delay_normal();
    I2C_SDA_LOW();
    i2c_delay_normal();
    I2C_SCL_LOW();
    i2c_delay_normal();
}

void i2c_start_rep() {
    I2C_SCL_LOW();
    i2c_delay_fast();
    I2C_SDA_HIGH();
    i2c_delay_fast();
    I2C_SCL_HIGH();
    i2c_delay_fast();
    I2C_SDA_LOW();
    i2c_delay_fast();
    I2C_SCL_LOW();
    i2c_delay_fast();
}

void i2c_stop() {
    I2C_SCL_LOW();
    I2C_SDA_LOW();
    i2c_delay_fast();
    I2C_SCL_HIGH();
    i2c_delay_fast();
    I2C_SDA_HIGH();
    i2c_delay_normal();
}

// Оптимизированная отправка байта
unsigned char i2c_send_byte(unsigned char byte) {
    for(unsigned char i = 0; i < 8; i++) {
        I2C_SCL_LOW();
    	i2c_delay_fast();
        
        if(byte & 0x80) {
            I2C_SDA_HIGH();
        } else {
            I2C_SDA_LOW();
        }
        
	i2c_delay_fast();
        I2C_SCL_HIGH();
    	i2c_delay_normal();
        byte <<= 1;
    }
    
    // Чтение ACK
    I2C_SCL_LOW();
    I2C_SDA_HIGH();
    i2c_delay_fast();
    I2C_SCL_HIGH();
    i2c_delay_normal();
    
    unsigned char ack = !I2C_SDA_VALUE;
    
    I2C_SCL_LOW();
    return ack;
}

// Оптимизированное чтение байта
uint8_t i2c_read_byte(uint8_t ack) {
    uint8_t result = 0;
    I2C_SDA_HIGH();
    
    for (uint8_t i = 0; i < 8; i++) {
        I2C_SCL_LOW();
    	i2c_delay_normal();
        I2C_SCL_HIGH();
        i2c_delay_normal();
        
        result <<= 1;
        if (I2C_SDA_VALUE) result |= 1;
    }
    
    I2C_SCL_LOW();
    if (ack == I2C_ACK) {
        I2C_SDA_LOW();
    } else {
        I2C_SDA_HIGH();
    }
    i2c_delay_fast();
    I2C_SCL_HIGH();
    i2c_delay_fast();
    I2C_SCL_LOW();
    I2C_SDA_HIGH();
    
    return result;
}

unsigned char i2c_address(unsigned char address, unsigned char rw) {
    address <<= 1;
    if (rw == I2C_READ) {
        address |= 0x01;
    }
    return i2c_send_byte(address);
}
