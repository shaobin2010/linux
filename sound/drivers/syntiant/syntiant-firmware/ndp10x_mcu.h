/*
 * SYNTIANT CONFIDENTIAL
 *
 * _____________________
 *
 * Copyright (c) 2017-2020 Syntiant Corporation
 * All Rights Reserved.
 *
 * NOTICE:  All information contained herein is, and remains the property of
 * Syntiant Corporation and its suppliers, if any.  The intellectual and
 * technical concepts contained herein are proprietary to Syntiant Corporation
 * and its suppliers and may be covered by U.S. and Foreign Patents, patents in
 * process, and are protected by trade secret or copyright law.  Dissemination
 * of this information or reproduction of this material is strictly forbidden
 * unless prior written permission is obtained from Syntiant Corporation.
 *
 */

#ifndef NDP10X_MCU_H
#define NDP10X_MCU_H

#ifdef __cplusplus
extern "C" {
#endif

#define REG(a) *((volatile uint32_t *) (a))

/* MCU Interrupt control registers */
#define INTERRUPT_REG_BASE (0xE000E100UL)
#define INTERRUPT_REG_ENABLE (INTERRUPT_REG_BASE + 0x00)
#define INTERRUPT_REG_DISABLE (INTERRUPT_REG_BASE + 0x80)
#define INTERRUPT_REG_PENABLE (INTERRUPT_REG_BASE + 0x100)
#define INTERRUPT_REG_PDISABLE (INTERRUPT_REG_BASE + 0x180)
#define INTERRUPT_REG_PRIO (INTERRUPT_REG_BASE + 0x300)

/* MCU GPIO */
#define MCU_GPIO_REG_BASE (0x40011000UL)
#define MCU_GPIO_DATA (MCU_GPIO_REG_BASE + 0x00)
#define MCU_GPIO_FUNC_SET (MCU_GPIO_REG_BASE + 0x18)
#define MCU_GPIO_FUNC_CLEAR (MCU_GPIO_REG_BASE + 0x1C)
#define MCU_GPIO_OUT_ENABLE (MCU_GPIO_REG_BASE + 0x10)
#define MCU_GPIO_INTEN_SET (MCU_GPIO_REG_BASE + 0x20)
#define MCU_GPIO_INTEN_CLR (MCU_GPIO_REG_BASE + 0x24)
#define MCU_GPIO_INTTYPE_SET (MCU_GPIO_REG_BASE + 0x28)
#define MCU_GPIO_INTTYPE_CLR (MCU_GPIO_REG_BASE + 0x2C)
#define MCU_GPIO_INTPOL_SET (MCU_GPIO_REG_BASE + 0x30)
#define MCU_GPIO_INTPOL_CLR (MCU_GPIO_REG_BASE + 0x34)
#define MCU_GPIO_INT_STATUS (MCU_GPIO_REG_BASE + 0x38)
#define MCU_GPIO_MASK_LOW_BYTE (MCU_GPIO_REG_BASE + 0x400)

#define MCU_GPIO_MASK_LOW_BYTE_ADDRESS(m) (MCU_GPIO_MASK_LOW_BYTE + (m) * 4)
#define MCU_GPIO_WRITE_MASK_LOW_BYTE(m, v) \
    REG(MCU_GPIO_MASK_LOW_BYTE_ADDRESS(m)) = (v)

/* UART GPIO bits */
#define NDP10X_GPIO_RX_BIT 0x01
#define NDP10X_GPIO_TX_BIT 0x02


/* MCU TIMER  */
#define MCU_APB_DUALTIMER1_TIMER0_BASE      (0x40002000UL)
#define MCU_APB_DUALTIMER1_TIMER0_LOAD      (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x00)
#define MCU_APB_DUALTIMER1_TIMER0_VALUE     (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x04)
#define MCU_APB_DUALTIMER1_TIMER0_CTRL      (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x08)
#define MCU_APB_DUALTIMER1_TIMER0_INTCLR    (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x0C)
#define MCU_APB_DUALTIMER1_TIMER0_MIS       (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x14)
#define MCU_APB_DUALTIMER1_TIMER0_BGLOAD    (MCU_APB_DUALTIMER1_TIMER0_BASE + 0x18)

#define MCU_APB_DUALTIMER1_TIMER1_BASE      (0x40002020UL)
#define MCU_APB_DUALTIMER1_TIMER1_LOAD      (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x00)
#define MCU_APB_DUALTIMER1_TIMER1_VALUE     (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x04)
#define MCU_APB_DUALTIMER1_TIMER1_CTRL      (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x08)
#define MCU_APB_DUALTIMER1_TIMER1_INTCLR    (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x0C)
#define MCU_APB_DUALTIMER1_TIMER1_MIS       (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x14)
#define MCU_APB_DUALTIMER1_TIMER1_BGLOAD    (MCU_APB_DUALTIMER1_TIMER1_BASE + 0x18)

#define MCU_APB_TIMER_ENABLE        (0x00000080UL)
#define MCU_APB_TIMER_INT_CLEAR     (0x00000001UL)
#define MCU_APB_TIMER_CONTROL_SIZE  (0x00000002UL)
#define MCU_APB_TIMER_CONTROL_MODE  (0x00000040UL)
#define MCU_APB_TIMER_CONTROL_INTEN (0x00000020UL)
#define MCU_APB_TIMER0  0
#define MCU_APB_TIMER1  1
#define MCU_APB_TIMER_SIZE_16_BIT 0
#define MCU_APB_TIMER_SIZE_32_BIT 1

/* MCU UART */
#define MCU_APB_UART0_BASE                  (0x40004000UL)
#define MCU_APB_UART0_DATA            (MCU_APB_UART0_BASE + 0x00)
#define MCU_APB_UART0_STATE           (MCU_APB_UART0_BASE + 0x04)
#define MCU_APB_UART0_CTRL            (MCU_APB_UART0_BASE + 0x08)
#define MCU_APB_UART0_INTSTATCLR      (MCU_APB_UART0_BASE + 0x0C)
#define MCU_APB_UART0_BAUDDIV         (MCU_APB_UART0_BASE + 0x10)

#define MCU_APB_UART_RX_IRQ_ENABLE    (0x00000008UL)
#define MCU_APB_UART_TX_IRQ_ENABLE    (0x00000004UL)
#define MCU_APB_UART_RX_ENABLE        (0x00000002UL)
#define MCU_APB_UART_TX_ENABLE        (0x00000001UL)
#define MCU_APB_UART_TX_INTCLR        (0x00000001UL)
#define MCU_APB_UART_RX_INTCLR        (0x00000002UL)

#define MCU_INT_MBIN      (1U << 16)
#define MCU_INT_FREQ      (1U << 17)
#define MCU_INT_DNN       (1U << 18)
/* this bit is set if ANY gpio INT is asserted */
#define MCU_INT_GPIOS     (1U << 7)
#define MCU_INT_DUALTIMER (1U << 10)


/**
 * @brief Data Structure for storing UART BB state.
 *
 */
#define M301_READ_RES 0
#define M301_COMMAND  1
#define M301_REG      2
#define M301_DATA     3

struct ndp10x_uart_bb_state_s {
  uint32_t gpio_tx;                   /* integer 0-7 */
  uint32_t gpio_rx;                   /* integer 0-7 */
  uint32_t timer_used;
  uint32_t timer_int_used;
  uint32_t baud_rate;
  uint32_t freq_div;                  /* divider for 4 M clock */
  uint32_t current_state;
  uint8_t m301_message[4];
  uint32_t avd_on_off;
  uint32_t avd_low_power_div;
  uint32_t avd_full_power_div;
};

/**
 * @brief spin delay
 *
 * @param delay delay for 500ns * delay
 * 
 */

#define ndp10x_mcu_spin_delay(delay) \
{ \
        volatile char vol_read; \
        for (int cntr = 0; cntr < (delay); cntr++) { \
            vol_read; \
        } \
}

#define NDP10X_MCU_INT_ENABLE(ints) \
    REG(INTERRUPT_REG_ENABLE) = (ints)

#define NDP10X_MCU_INT_DISABLE(ints) \
    REG(INTERRUPT_REG_DISABLE) = (ints)

#define NDP10X_MCU_INT_GET_PENDING() \
    REG(INTERRUPT_REG_PDISABLE)
    
#define NDP10X_MCU_INT_CLEAR_PENDING(ints) \
    REG(INTERRUPT_REG_PDISABLE) = (ints)
    
/**
 * @brief Blocking wait for interrupt.
 *
 */
#define NDP10X_MCU_INT_WAIT() \
    __asm volatile("wfi")

void ndp10x_mcu_memmove(uint32_t *d, uint32_t *s, unsigned int len);
    
void ndp10x_mcu_memclear(uint32_t *d,unsigned int len);

/**
 * @brief sets the interrupt priority mask
 *
 * @param mask if 1 it prevents the activation of all exceptions with
 *        configurable priority
 */
#define NDP10X_MCU_INT_PRIORITY_MASK_SET(mask) \
    __asm volatile("MSR primask, %0" : : "r"(mask) : "memory");

void ndp10x_mcu_uart_rx_init(uint32_t baud_rate_div);

void ndp10x_mcu_uart_init(uint32_t baud_rate_div, uint32_t tx_enable, \
                          uint32_t rx_enable, uint32_t tx_irq_en, uint32_t rx_irq_en);

void ndp10x_mcu_uart_rx_clr(void);

uint32_t ndp10x_mcu_uart_rx_status(void);

uint32_t ndp10x_mcu_uart_rx_data(void);

void ndp10x_mcu_uart_tx_clr(void);

uint32_t ndp10x_mcu_uart_tx_status(void);

void ndp10x_mcu_uart_tx_data(uint32_t);


/**
 * @brief UART over a normal GPIO TX function
 *
 * @param *uart_bb structure for uart bit bang
 * @param byte_to_send byte to transmit
 *
 */
void ndp10x_mcu_bb_uart_tx(struct ndp10x_uart_bb_state_s *uart_bb, uint8_t byte_to_send);

void ndp10x_mcu_timer_enable(uint32_t timer_num, uint32_t counter_32bit);

void ndp10x_mcu_timer_disable(uint32_t timer_num);

void ndp10x_mcu_timer_reset(uint32_t timer_num, uint32_t value);

void ndp10x_mcu_timer_intclr(uint32_t timer_num);

uint32_t  ndp10x_mcu_timer_value(uint32_t timer_num);

uint32_t ndp10x_mcu_timer_intstat(uint32_t timer_num);

void ndp10x_mcu_lowpower_wake(uint32_t full_power_pdm_clk_divider);

void ndp10x_mcu_lowpower_enable(uint32_t low_power_pdm_clk_divider);

void ndp10x_mcu_timer_int_enable(uint32_t timer_num);

void ndp10x_mcu_timer_int_disable(uint32_t timer_num);

uint16_t ndp10x_mcu_div100(uint32_t data);

#ifdef __cplusplus
}
#endif

#endif
