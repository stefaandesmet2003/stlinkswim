/*
 * This file is part of the Black Magic Debug project.
 *
 * Copyright (C) 2015 Gareth McMullin <gareth@blacksphere.co.nz>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include "general.h"
#include "timing_stm32.h"

#include <libopencm3/cm3/systick.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/stm32/rcc.h>

static volatile uint32_t _millis;

void platform_timing_init(void)
{
	/* Setup heartbeat timer */
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB_DIV8);
	/* Interrupt us at 10 Hz */
	systick_set_reload(rcc_ahb_frequency / (8 * SYSTICKHZ) - 1);
	systick_clear(); //sds
	/* SYSTICK_IRQ with low priority */
	nvic_set_priority(NVIC_SYSTICK_IRQ, 14 << 4);
	systick_interrupt_enable();
	systick_counter_enable();
}

void platform_timeout_set(platform_timeout *t, uint32_t ms)
{
	if (ms <= SYSTICKMS)
		ms = SYSTICKMS;
	t->time = platform_time_ms() + ms;
}

bool platform_timeout_is_expired(platform_timeout *t)
{
	return platform_time_ms() > t->time;
}

/*
//sds : da werkt nie, timing is niet precies genoeg
// dat moet hier heel juust zijn of swim geeft geen ack bij de entry
void platform_delayus (uint32_t us)
{
	uint32_t start_ticks, delta_ticks, delay_ticks, reload;
	volatile uint32_t _dummy;
	//start_ticks = systick_get_value();
	start_ticks = STK_CVR;
	_dummy = STK_CSR; // clear countflag
	//reload = systick_get_reload();
	reload = STK_RVR;
	delay_ticks = 9*us; // STK_CSR_CLKSOURCE_AHB_DIV8
	// opgelet : systick is 24-bit, telt af, met reload 8999 (voor 1000Hz SYSTICKHZ)
	do {
		delta_ticks = start_ticks - STK_CVR;
		if (STK_CSR & STK_CSR_COUNTFLAG)
			delta_ticks += (reload+1);
	}
	while (delta_ticks<delay_ticks);
}
*/

void platform_delay(uint32_t ms)
{
	platform_timeout timeout;
	platform_timeout_set(&timeout, ms);
	while (!platform_timeout_is_expired(&timeout));
}

void sys_tick_handler(void)
{
	_millis += SYSTICKMS;

}
// vgl millis
uint32_t platform_time_ms(void)
{
	return _millis;
}

