//A minimal bare-metal GPIO and UART driver implementation for STM32F411CEU6 (no HAL or SDK):
#include "stm32f411xe.h"

/* ---------- GPIO ---------- */
void gpio_init(void) {
    // Enable GPIOA clock
    RCC->AHB1ENR |= (1 << 0);

    // PA5 as output (LED)
    GPIOA->MODER &= ~(3 << (5 * 2));
    GPIOA->MODER |=  (1 << (5 * 2));

    // PA0 as input (Button)
    GPIOA->MODER &= ~(3 << (0 * 2));
}

/* ---------- UART ---------- */
void uart2_init(void) {
    // Enable GPIOA and USART2 clocks
    RCC->AHB1ENR |= (1 << 0);
    RCC->APB1ENR |= (1 << 17);

    // Configure PA2 (TX) and PA3 (RX) as AF7
    GPIOA->MODER &= ~((3 << (2*2)) | (3 << (3*2)));
    GPIOA->MODER |=  ((2 << (2*2)) | (2 << (3*2)));
    GPIOA->AFR[0] |= (7 << (2*4)) | (7 << (3*4));

    // Configure USART2: 115200 baud, 8N1
    USART2->BRR = 0x8B; // Assuming 16 MHz clock
    USART2->CR1 = (1 << 3) | (1 << 2) | (1 << 13); // TE, RE, UE enable
}

/* ---------- UART Send ---------- */
void uart2_write(char c) {
    while (!(USART2->SR & (1 << 7))); // Wait TXE
    USART2->DR = c;
}

void uart2_write_str(const char *str) {
    while (*str) uart2_write(*str++);
}

/* ---------- Circular Buffer ---------- */
#define BUF_SIZE 128
char log_buf[BUF_SIZE];
volatile uint8_t head = 0, tail = 0;

void log_add(char c) {
    uint8_t next = (head + 1) % BUF_SIZE;
    if (next != tail) {
        log_buf[head] = c;
        head = next;
    }
}

void log_flush(void) {
    while (tail != head) {
        uart2_write(log_buf[tail]);
        tail = (tail + 1) % BUF_SIZE;
    }
}

/* ---------- Main ---------- */
int main(void) {
    gpio_init();
    uart2_init();
    uart2_write_str("System Init OK\r\n");

    while (1) {
        if (GPIOA->IDR & 1) {
            GPIOA->ODR ^= (1 << 5);
            uart2_write_str("Button Pressed\r\n");
            for (volatile int i = 0; i < 100000; i++);
        }
        log_flush();
    }
}
