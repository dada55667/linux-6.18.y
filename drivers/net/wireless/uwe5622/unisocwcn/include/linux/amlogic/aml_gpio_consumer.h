/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Stand-in for the Amlogic vendor <linux/amlogic/aml_gpio_consumer.h>,
 * which mainline kernels do not have.  The Amlogic build force-includes
 * it into every BSP source file.
 */
#ifndef _AML_GPIO_CONSUMER_STUB_H_
#define _AML_GPIO_CONSUMER_STUB_H_

#ifndef GPIO_IRQ_HIGH
#define GPIO_IRQ_HIGH 1
#endif
#ifndef GPIO_IRQ_LOW
#define GPIO_IRQ_LOW 0
#endif

/* platform/aml_wifi_stubs.c */
void extern_wifi_set_enable(int is_on);
void extern_bt_set_enable(int is_on);
void sdio_reinit(void);
void sdio_clk_always_on(int on);
void sdio_set_max_reqsz(unsigned int size);
int wifi_irq_num(void);
int wifi_irq_trigger_level(void);

#endif
