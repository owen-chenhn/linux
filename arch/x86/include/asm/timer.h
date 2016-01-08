/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _ASM_X86_TIMER_H
#define _ASM_X86_TIMER_H
#include <linux/pm.h>
#include <linux/percpu.h>
#include <linux/interrupt.h>
#include <linux/math64.h>

#define TICK_SIZE (tick_nsec / 1000)

unsigned long long native_sched_clock(void);
extern void recalibrate_cpu_khz(void);

extern int no_timer_check;

extern bool using_native_sched_clock(void);
extern struct static_key_false __use_tsc; // TODO

/*
 * We use the full linear equation: f(x) = a + b*x, in order to allow
 * a continuous function in the face of dynamic freq changes.
 *
 * Continuity means that when our frequency changes our slope (b); we want to
 * ensure that: f(t) == f'(t), which gives: a + b*t == a' + b'*t.
 *
 * Without an offset (a) the above would not be possible.
 *
 * See the comment near cycles_2_ns() for details on how we compute (b).
 */
struct cyc2ns_data {
	u32 cyc2ns_mul;
	u32 cyc2ns_shift;
	u64 cyc2ns_offset;
}; /* 16 bytes */

extern void cyc2ns_read_begin(struct cyc2ns_data *);
extern void cyc2ns_read_end(void);

/*
 * Use a ring-buffer like data structure, where a writer advances the head by
 * writing a new data entry and a reader advances the tail when it observes a
 * new entry.
 *
 * Writers are made to wait on readers until there's space to write a new
 * entry.
 *
 * This means that we can always use an {offset, mul} pair to compute a ns
 * value that is 'roughly' in the right direction, even if we're writing a new
 * {offset, mul} pair during the clock read.
 *
 * The down-side is that we can no longer guarantee strict monotonicity anymore
 * (assuming the TSC was that to begin with), because while we compute the
 * intersection point of the two clock slopes and make sure the time is
 * continuous at the point of switching; we can no longer guarantee a reader is
 * strictly before or after the switch point.
 *
 * It does mean a reader no longer needs to disable IRQs in order to avoid
 * CPU-Freq updates messing with his times, and similarly an NMI reader will
 * no longer run the risk of hitting half-written state.
 */

struct cyc2ns {
	struct cyc2ns_data data[2];	/*  0 + 2*24 = 48 */
    seqcount_t seq;             /* 32 + 4    = 36 */
}; /* exactly fits one cacheline */

DECLARE_PER_CPU_ALIGNED(struct cyc2ns, cyc2ns);

#define CYC2NS_SCALE_FACTOR 10 /* 2^10, carefully chosen */

static inline unsigned long long cycles_2_ns(unsigned long long cyc)
{
	struct cyc2ns_data data;
	unsigned long long ns;

	cyc2ns_read_begin(&data);

	ns = data.cyc2ns_offset;
	ns += mul_u64_u32_shr(cyc, data.cyc2ns_mul, data.cyc2ns_shift);

	cyc2ns_read_end();

	return ns;
}

/*
 * Scheduler clock - returns current time in nanosec units. The caller needs to
 * ensure a reschedule won't happen during the execution of this function, by
 * disabling preemption, or in some other way.
 */
static inline u64 __sched_clock(void)
{
	if (static_branch_likely(&__use_tsc)) {
		u64 tsc_now = rdtsc();

		/* return the value in ns */
		return cycles_2_ns(tsc_now);
	}

	/*
	 * Fall back to jiffies if there's no TSC available:
	 * ( But note that we still use it if the TSC is marked
	 *   unstable. We do this because unlike Time Of Day,
	 *   the scheduler clock tolerates small errors and it's
	 *   very important for it to be as fast as the platform
	 *   can achieve it. )
	 */

	/* No locking but a rare wrong value is not a big deal: */
	return (jiffies_64 - INITIAL_JIFFIES) * (1000000000 / HZ);
}

#define CONFIG_INLINE_SCHED_CLOCK

#endif /* _ASM_X86_TIMER_H */
