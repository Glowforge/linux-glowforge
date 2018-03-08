/**
 * Low-level i.MX6 EPIT API.
 * Copyright (C) 2015-2018 Glowforge, Inc. <opensource@glowforge.com>
 *
 * The i.MX6 has two EPIT (Enhanced Periodic Interrupt Timer) units, both of
 * which are unused by the Linux kernel. This header exposes a low-level API
 * for configuring an EPIT.
 */

#ifndef __MACH_MXC_EPIT_H__
#define __MACH_MXC_EPIT_H__

#include <linux/types.h>

struct epit;
struct device_node;

typedef void (*epit_cb)(void *);

/**
 * epit_get() - returns a pointer to the EPIT for the given device_node
 * @np:		device tree node for EPIT1 or EPIT2
 *
 * Return: pointer to epit object, or NULL
 */
struct epit *epit_get(struct device_node *np);

/**
 * epit_init_freerunning() - initializes EPIT in "set and forget" mode
 * @epit:	pointer to epit object
 * @callback:	function to execute periodically
 * @cb_arg:	user-defined object passed when the callback is invoked
 *
 * The timer is started when epit_start() is called.
 * If callback is non-NULL, it will be executed (in interrupt context) when the
 * timer overflows.
 *
 * Return: 0 on success, nonzero otherwise
 */
int epit_init_freerunning(struct epit *epit, epit_cb callback, void *cb_arg);

/**
 * epit_init_oneshot() - initializes EPIT for retriggerable one-shot timeouts
 * @epit:	pointer to epit object
 * @callback:	function to execute periodically
 * @cb_arg:	user-defined object passed when the callback is invoked
 *
 * The timer is started when epit_start() is called, and the callback is
 * executed with the given argument (in interupt context) if the timeout is
 * reached.
 */
int epit_init_oneshot(struct epit *epit, epit_cb callback, void *cb_arg);

/**
 * epit_hz_to_divisor() - converts frequency in hertz to a clock divider value
 * @epit:	pointer to epit object
 * @hz:		frequency in Hz
 *
 * Return: divisor value for the desired frequency
 */
u32 epit_hz_to_divisor(struct epit *epit, u32 hz);

/**
 * epit_start() - starts an EPIT with the given period
 * @epit:	pointer to epit object
 * @divisor:	length of timer interval in ticks
 */
void epit_start(struct epit *epit, u32 divisor);

/**
 * epit_start_hz() - starts a free-running EPIT with the given frequency
 * @epit:	pointer to epit object
 * @hz:		frequency in Hz
 */
void epit_start_hz(struct epit *epit, u32 hz);

/**
 * epit_stop() - stops an EPIT
 * @epit:	pointer to epit object
 */
void epit_stop(struct epit *epit);

/**
 * epit_set_divisor() - change an EPIT's period
 * @epit:	pointer to epit object
 * @divisor:	new length of timer interval in ticks
 *
 * Takes effect the next time epit_start() is called.
 */
void epit_set_divisor(struct epit *epit, u32 divisor);

/**
 * epit_set_hz() - change an EPIT's frequency
 * @epit:	pointer to epit object
 * @hz:		new frequency in Hz
 *
 * Takes effect the next time epit_start() is called.
 */
void epit_set_hz(struct epit *epit, u32 hz);

/**
 * epit_count() - returns the EPIT's current count
 * @epit:	pointer to epit object
 *
 * Return: current value of the EPIT's counter
 */
u32 epit_count(struct epit *epit);

/**
 * epit_is_running() - returns whether or not an EPIT is running
 * @epit:	pointer to epit object
 *
 * Return: 1 if timer is running, 0 otherwise
 */
int epit_is_running(struct epit *epit);

/**
 * epit_irq() - returns the EPIT's IRQ number
 * @epit:	pointer to epit object
 *
 * Return: the EPIT's IRQ number
 */
int epit_irq(struct epit *epit);

/**
 * epit_sdma_event() - returns the EPIT's SDMA event number
 * @epit:	pointer to epit object
 *
 * Return: the EPIT's SDMA event number
 */
int epit_sdma_event(struct epit *epit);

/**
 * epit_status_register_address() - returns physical address of status register
 * @epit:	pointer to epit object
 *
 * Return: the physical address of the EPIT's EPITSR register
 */
u32 epit_status_register_address(struct epit *epit);

#endif
