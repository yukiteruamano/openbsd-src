/*	$OpenBSD$ pstate.c,v 1.0 2026/06/13 11:22:00 yukiteruamano Exp $	* */
/*
 * Copyright (c) 2020-2023 joshua stein <jcs@jcs.org>
 * Copyright (c) 2026 José Maldonado <yukiteruamano@volfread.xyz>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <net/if.h>
#include <sys/param.h>
#include <sys/systm.h>
#include <sys/sysctl.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/timeout.h>

#include <machine/cpu.h>
#include <machine/cpufunc.h>
#include <machine/specialreg.h>
#include <machine/bus.h>
#include <uvm/uvm_extern.h>

#define PSTATE_PERFPOL_MANUAL	0
#define PSTATE_PERFPOL_AUTO	1
#define PSTATE_PERFPOL_HIGH	2
#define PSTATE_EPP_BALANCE_PERF	0x80
#define PSTATE_EPB_BALANCE_PERF	0x04
#define PSTATE_EPP_PERFORMANCE_PERF	0x00
#define PSTATE_EPB_PERFORMANCE_PERF	0x00

/* Explicit bit/mask definitions for HWP_REQUEST */
#define HWP_REQ_ACT_WINDOW_SHIFT	32
#define HWP_REQ_ACT_WINDOW_MASK		(0x3FFULL << HWP_REQ_ACT_WINDOW_SHIFT)
#define HWP_REQ_PKG_CTL			(1ULL << 42)
#define HWP_REQ_ACT_WINDOW_VALID	(1ULL << 59)
#define HWP_REQ_EPP_VALID		(1ULL << 60)
#define HWP_REQ_DESIRED_VALID		(1ULL << 61)
#define HWP_REQ_MAXIMUM_VALID		(1ULL << 62)
#define HWP_REQ_MINIMUM_VALID		(1ULL << 63)

enum pstate_bias_t { PSTATE_HWP_BIAS_EPP, PSTATE_HWP_BIAS_EPB };

/* IA32_HWP_CAPABILITIES */
union hwp_capabilities {
	uint64_t msr;
	struct {
		uint8_t highest_perf;
		uint8_t guaranteed_perf;
		uint8_t most_efficient_perf;
		uint8_t lowest_perf;
		uint32_t reserved;
	} __packed fields;
};

/* IA32_HWP_REQUEST / IA32_HWP_REQUEST_PKG */
union hwp_request {
	uint64_t msr;
	struct {
		uint8_t min_perf;
		uint8_t max_perf;
		uint8_t desired_perf;
		uint8_t epp;
		uint32_t reserved_hi;
	} __packed fields;
};

/*
 * Per-CPU state structure for HWP configuration and performance telemetry.
 */
struct pstate_softc {
	union hwp_capabilities	hwp_cap;
	union hwp_request	hwp_req;
	struct timeout		perf_to;
	enum pstate_bias_t	hwp_bias_style;
	uint64_t	applied_hwp_req;
	uint64_t	last_mperf;
	uint64_t	last_aperf;
	uint32_t	effective_speed;
	uint8_t	user_epp;
	uint8_t	user_epb;
	uint8_t	applied_epb;
	int	hwp_req_msr;
	int	enabled;
	int	perflevel;
};

struct pstate_perf_sample {
	uint64_t mperf;
	uint64_t aperf;
};

#ifdef PSTATE_DEBUG
#define DPRINTF(x) printf x
#else
#define DPRINTF(x)
#endif

extern int setperf_prio;
extern void est_init(struct cpu_info *);
extern uint64_t tsc_frequency;
extern int hw_power;
extern int perfpolicy_on_ac;
extern int perfpolicy_on_battery;

int pstate_hwp = 0;
struct mutex pstate_lock = MUTEX_INITIALIZER(IPL_HIGH);
struct pstate_softc *pstate_cpus[MAXCPUS];

/* Definition table modes for EPP/EPB
 *   0-3   = performance preference
 *   4-7   = balance performance (use 6 as midpoint)
 *   8-11  = balance powersave (use 9 as midpoint)
 *   12-15 = powersave preference
 */
static struct {
	int epb_min;
	int epb_max;
	int epb_recommended;
	int epp;
	char *label;
} pstate_epp_labels[] = {
	/* performance: 0 is correct for max performance */
	{ 0x00, 0x03, 0x00, 0x00, "performance" },

	/* balance_performance: 0x06 balances well in 4-7 range */
	{ 0x04, 0x07, 0x06, 0x80, "balance_performance" },

	/* balance_powersave: 0x09 is neutral in 8-11 range */
	{ 0x08, 0x0b, 0x09, 0xc0, "balance_powersave" },

	/* powersave: 0x0f is correct for max efficiency */
	{ 0x0c, 0x0f, 0x0f, 0xff, "powersave" }
};

const char *pstate_hwp_bias_label(struct pstate_softc *, int);
void pstate_apply(struct pstate_softc *);
void pstate_apply_locked(struct pstate_softc *);
void pstate_tick(void *);
static int pstate_read_counters(uint64_t *, uint64_t *);

/* Read global estate power for system */
/* XXX: Automatic change for performance mode using sense batter/AC */
static inline int
pstate_current_perfpolicy(void)
{
	return (hw_power ? perfpolicy_on_ac : perfpolicy_on_battery);
}

/*
 * Reads MSR_MPERF and MSR_APERF without delay, as recommended by Intel SDM.
 * We need this for freq/speed calculations
 * Returns 0 on success, -1 on failure.
 */
static int
pstate_read_counters(uint64_t *mperf, uint64_t *aperf)
{
	int s = splhigh();

	if (rdmsr_safe(MSR_MPERF, mperf) != 0 ||
	    rdmsr_safe(MSR_APERF, aperf) != 0) {
		splx(s);
		return (-1);
	}
	splx(s);
	return (0);
}

/*
 * Periodic timeout handler to sample performance counters (MPERF/APERF)
 * and update effective cpuspeed, ensuring accurate performance reporting.
 */
void
pstate_tick(void *arg)
{
	struct pstate_softc *sc = arg;
	uint64_t mperf, aperf;
	uint64_t mcnt, acnt;
	uint64_t hwp_status;
	uint64_t old_guar;
	uint64_t therm_status;
	const char *cause = "unknown constraint";

	if (pstate_read_counters(&mperf, &aperf) != 0) {
		printf("%s: HWP counter read failed, disabling\n",
		    curcpu()->ci_dev->dv_xname);
		sc->enabled = 0;
		return;
	}

	mcnt = mperf - sc->last_mperf;
	acnt = aperf - sc->last_aperf;

	if (mcnt > 0) {
		sc->effective_speed = (acnt * (tsc_frequency / 1000000)) / mcnt;
		if (CPU_IS_PRIMARY(curcpu()))
			cpuspeed = sc->effective_speed;
	}

	sc->last_mperf = mperf;
	sc->last_aperf = aperf;

	/* DIAGNOSTIC: compare observed effective_speed against the
	 * max_perf limit we believe is currently programmed. A sustained,
	 * large gap (effective speed far above max_perf*100 MHz) while
	 * policy=manual confirms the hardware is not honoring our limit,
	 * independent of whatever the readback check in
	 * pstate_apply_locked() shows immediately after wrmsr().
	 */
	if (pstate_current_perfpolicy() == PSTATE_PERFPOL_MANUAL) {
		uint32_t expected_max_mhz = sc->hwp_req.fields.max_perf * 100;
		if (sc->effective_speed > expected_max_mhz + 200) {
			DPRINTF(("%s: effective_speed %d MHz exceeds "
			    "programmed max_perf limit %d MHz "
			    "(min=%d max=%d desired=%d)\n",
			    curcpu()->ci_dev->dv_xname,
			    sc->effective_speed, expected_max_mhz,
			    sc->hwp_req.fields.min_perf,
			    sc->hwp_req.fields.max_perf,
			    sc->hwp_req.fields.desired_perf));
		}
	}

	/* Monitor HWP_STATUS for dynamic changes
	 * Guaranteed_Performance can change due to thermal or power constraints.
	 * We must detect and respond to these changes.
	 */
	if (rdmsr_safe(IA32_HWP_STATUS, &hwp_status) == 0) {
		
		/* Guaranteed_Performance_Change (bit 0)
		 * Indicates Guaranteed Performance value has changed,
		 * likely due to power or thermal constraints
		 */
		if (hwp_status & 0x1) {
			old_guar = sc->hwp_cap.fields.guaranteed_perf;
			
			/* Re-read HWP capabilities to get new Guaranteed_Performance */
			if (rdmsr_safe(IA32_HWP_CAPABILITIES, &sc->hwp_cap.msr) == 0) {
				if (old_guar != sc->hwp_cap.fields.guaranteed_perf) {
					DPRINTF(("%s: Guaranteed_Performance changed: "
					    "%d -> %d MHz\n",
					    curcpu()->ci_dev->dv_xname,
					    (int)old_guar * 100,
					    sc->hwp_cap.fields.guaranteed_perf * 100));
				}
			}
			
			/* Clear the status bit by writing 0 */
			wrmsr(IA32_HWP_STATUS, 0x1);
		}
		
		/* Excursion_To_Minimum (bit 2)
		 * Indicates HWP hardware was unable to meet Minimum_Performance
		 * due to constraints (thermal, power, coordination)
		 */
		if (hwp_status & 0x4) {
			/* Check MSR_THERM_STATUS (IA32_THERM_STATUS)
			 * for specific reason (SDM 15.4.5) 
			 */
			if (rdmsr_safe(MSR_THERM_STATUS, &therm_status) == 0) {
				if (therm_status & (1ULL << 12))
					cause = "current limit";
				else if (therm_status & (1ULL << 14))
					cause = "cross-domain limit";
				else if (therm_status & 0x1)
					cause = "thermal throttling";
			}
			
			DPRINTF(("%s: HWP excursion to Minimum_Performance (%s)\n",
			    curcpu()->ci_dev->dv_xname, cause));
			
			/* Clear the status bit */
			wrmsr(IA32_HWP_STATUS, 0x4);
		}
		
		/* Highest_Performance_Change (bit 3)
		 * Optional notification of highest performance capability change
		 */
		if (hwp_status & 0x8) {
			DPRINTF(("%s: Highest_Performance changed to %d MHz\n",
			    curcpu()->ci_dev->dv_xname,
			    sc->hwp_cap.fields.highest_perf * 100));
			
			wrmsr(IA32_HWP_STATUS, 0x8);
		}

		/* DIAGNOSTIC: PECI_Override_Entry/Exit (bits 4-5)
		 * If an embedded controller (EC) is overriding our Min/Max/EPP
		 * via PECI, our wrmsr() to IA32_HWP_REQUEST will succeed but
		 * be silently superseded by firmware - this is indistinguishable
		 * from software's point of view except via this status bit and
		 * IA32_HWP_PECI_REQUEST_INFO. This is common on business
		 * laptops (Dell/HP/Lenovo) running Intel DPTF.
		 */
		if (hwp_status & 0x10) {
			uint64_t peci_info;
			printf("%s: WARNING: PECI override of HWP_REQUEST "
			    "started (firmware/EC is now controlling "
			    "performance, software limits may be ignored)\n",
			    curcpu()->ci_dev->dv_xname);
			if (rdmsr_safe(IA32_HWP_PECI_REQUEST_INFO, &peci_info) == 0) {
				DPRINTF(("%s: PECI override values: min=%d "
				    "max=%d epp=%d (EPP_override=%d "
				    "Max_override=%d Min_override=%d)\n",
				    curcpu()->ci_dev->dv_xname,
				    (int)(peci_info & 0xff),
				    (int)((peci_info >> 8) & 0xff),
				    (int)((peci_info >> 24) & 0xff),
				    (int)((peci_info >> 60) & 0x1),
				    (int)((peci_info >> 62) & 0x1),
				    (int)((peci_info >> 63) & 0x1)));
			}
			wrmsr(IA32_HWP_STATUS, 0x10);
		}
		if (hwp_status & 0x20) {
			printf("%s: PECI override of HWP_REQUEST ended, "
			    "software control restored\n",
			    curcpu()->ci_dev->dv_xname);
			wrmsr(IA32_HWP_STATUS, 0x20);
		}
	}

	/*
	 * Periodically synchronize HWP state. This ensures any policy changes
	 * or sysctl updates are applied across all CPUs without relying on IPIs.
	 */
	pstate_apply(sc);

	timeout_add_sec(&sc->perf_to, 1);
}

/*
 * Initializes HWP hardware, performs capability discovery via CPUID/MSRs,
 * sets up per-CPU state, and initiates periodic telemetry.
 */
void
pstate_init(struct cpu_info *ci)
{
	struct pstate_softc *sc;
	const char *cpu_device = ci->ci_dev->dv_xname;
	uint64_t msr;
	int16_t eppepb;

	sc = malloc(sizeof(*sc), M_DEVBUF, M_WAITOK | M_ZERO);
	sc->hwp_req_msr = IA32_HWP_REQUEST;

	if (rdmsr_safe(MSR_PLATFORM_INFO, &msr) != 0) {
		free(sc, M_DEVBUF, sizeof(*sc));
		return;
	}

	/* power management must be enabled before reading capabilities */
	wrmsr(IA32_PM_ENABLE, 1);
	if (rdmsr(IA32_PM_ENABLE) != 1) {
		printf("%s: enabling HWP failed\n", cpu_device);
		free(sc, M_DEVBUF, sizeof(*sc));
		return;
	}

	/* fallback to EST if HWP capabilities not exist */
	if (rdmsr_safe(IA32_HWP_CAPABILITIES, &sc->hwp_cap.msr) != 0) {
		printf("%s: no HWP capabilities, falling back to EST\n",
		    cpu_device);
		free(sc, M_DEVBUF, sizeof(*sc));
		est_init(ci);
		return;
	}

	/* Initialize EPP mode else EPB*/
	if (ci->ci_feature_tpmflags_eax & TPM_HWP_EPP) {
		sc->hwp_bias_style = PSTATE_HWP_BIAS_EPP;
		sc->hwp_req.msr = rdmsr(sc->hwp_req_msr);

		/* EPP: default is balance_performance */
		sc->user_epp = PSTATE_EPP_BALANCE_PERF;
		eppepb = sc->user_epp;

		printf("%s: HWP EPP enabled\n", cpu_device);
	} else if (ci->ci_feature_tpmflags_ecx & TPM_EPB) {
		sc->hwp_bias_style = PSTATE_HWP_BIAS_EPB;

		/* EPB: default is balance_performance */
		sc->applied_epb = PSTATE_EPB_BALANCE_PERF;
		wrmsr(IA32_ENERGY_PERF_BIAS, PSTATE_EPB_BALANCE_PERF);
		sc->user_epb = sc->applied_epb;
		eppepb = sc->user_epb;

		printf("%s: HWP EPB enabled\n", cpu_device);
	} else {
		/* XXX: Fallback to EST - not tested at this point */
		printf("%s: no energy bias control, falling back to EST\n",
		    cpu_device);
		free(sc, M_DEVBUF, sizeof(*sc));
		est_init(ci);
		return;
	}

	sc->enabled = 1;
	sc->perflevel = 100;
	pstate_cpus[CPU_INFO_UNIT(ci)] = sc;

	/* Initialize HWP request structure with proper defaults.
	 * - Minimum_Performance = Lowest_Performance
	 * - Maximum_Performance = Highest_Performance  
	 * - Desired_Performance = 0 (hardware autonomous selection)
	 * - Activity_Window = 0 (hardware determines window)
	 * - Package_Control = 0, all Valid bits = 0 (use per-CPU MSR fully)
	 */
	sc->hwp_req.fields.min_perf = sc->hwp_cap.fields.lowest_perf;
	sc->hwp_req.fields.max_perf = sc->hwp_cap.fields.highest_perf;
	sc->hwp_req.fields.desired_perf = 0;
	sc->hwp_req.fields.reserved_hi = 0;

	pstate_read_counters(&sc->last_mperf, &sc->last_aperf);
	timeout_set(&sc->perf_to, pstate_tick, sc);
	timeout_add_sec(&sc->perf_to, 1);

	pstate_hwp = 1;
	setperf_prio = 4;
	cpu_setperf = pstate_setperf;

	/* Apply initial performance policy */
	pstate_setperf(sc->perflevel);

	printf("%s: HWP enabled (bias %s)\n", cpu_device,
	    pstate_hwp_bias_label(sc, eppepb));
	printf("%s: perf: max %d, guar %d, efficient %d, min %d (MHz)\n",
	    cpu_device,
	    sc->hwp_cap.fields.highest_perf * 100,
	    sc->hwp_cap.fields.guaranteed_perf * 100,
	    sc->hwp_cap.fields.most_efficient_perf * 100,
	    sc->hwp_cap.fields.lowest_perf * 100);
}

/*
 * Reinitializes HWP hardware from resume
 * XXX: This work (tested) but not much relieable
 */
void
pstate_resume(struct cpu_info *ci)
{
	struct pstate_softc *sc = pstate_cpus[CPU_INFO_UNIT(ci)];
	const char *cpu_device = ci->ci_dev->dv_xname;
	uint64_t msr;

	if (sc == NULL || !sc->enabled)
		return;

	if (rdmsr_safe(MSR_PLATFORM_INFO, &msr) != 0) {
		sc->enabled = 0;
		return;
	}

	/* Re-enable HWP on resume */
	wrmsr(IA32_PM_ENABLE, 1);
	if (rdmsr(IA32_PM_ENABLE) != 1) {
		printf("%s: re-enabling HWP failed\n", cpu_device);
		sc->enabled = 0;
		return;
	}

	/* Re-init in balance_performance mode */
	if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPP) {
		sc->user_epp = PSTATE_EPP_BALANCE_PERF;
	} else if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPB) {
		sc->user_epb = PSTATE_EPB_BALANCE_PERF;
	}

	/*
	 * Reset applied state to force pstate_apply() to write the
	 * configuration to hardware.
	 */
	sc->applied_hwp_req = 0;
	sc->applied_epb = 0xff;
	pstate_apply(sc);
}

/*
 * Legible values for EPP and EPB tags
*/
const char *
pstate_hwp_bias_label(struct pstate_softc *sc, int val)
{
	int i;

	for (i = 0; i < (sizeof(pstate_epp_labels) /
	    sizeof(pstate_epp_labels[0])); i++) {
		if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPP) {
			if (val == pstate_epp_labels[i].epp)
				return (pstate_epp_labels[i].label);
		} else if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPB) {
			if (val >= pstate_epp_labels[i].epb_min &&
			    val <= pstate_epp_labels[i].epb_max)
				return (pstate_epp_labels[i].label);
		}
	}

	return ("unknown");
}

/*
* Apply pstate value on HW
*/
void
pstate_apply(struct pstate_softc *sc)
{
	mtx_enter(&pstate_lock);
	pstate_apply_locked(sc);
	mtx_leave(&pstate_lock);
}

/* Apply pstate value on HW - Locked/Mutexes task */
void
pstate_apply_locked(struct pstate_softc *sc)
{
	int policy = pstate_current_perfpolicy();
	uint64_t req_msr = 0;
	uint8_t epb = 0;
	uint8_t hwp_ceiling;
	uint8_t perf_upper = 0;
	int do_req = 0, do_epb = 0;

	sc->hwp_req.fields.reserved_hi = 0;

	switch (policy) {
	case PSTATE_PERFPOL_AUTO:
		sc->hwp_req.fields.min_perf = sc->hwp_cap.fields.lowest_perf;
		sc->hwp_req.fields.max_perf = sc->hwp_cap.fields.highest_perf;
		sc->hwp_req.fields.desired_perf = 0;

		/* setup balance-performance bias mode*/
		if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPP) {
			sc->hwp_req.fields.epp = PSTATE_EPP_BALANCE_PERF;
		} else if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPB) {
			epb = PSTATE_EPB_BALANCE_PERF;
		}
		break;
	case PSTATE_PERFPOL_HIGH:
		sc->hwp_req.fields.min_perf = sc->hwp_cap.fields.lowest_perf;
		sc->hwp_req.fields.max_perf = sc->hwp_cap.fields.highest_perf;
		sc->hwp_req.fields.desired_perf = 0;

		/* setup performance bias mode*/
		if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPP) {
			sc->hwp_req.fields.epp = PSTATE_EPP_PERFORMANCE_PERF;
		} else if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPB) {
			epb = PSTATE_EPB_PERFORMANCE_PERF;
		}
		break;
	case PSTATE_PERFPOL_MANUAL:
		perf_upper = sc->hwp_cap.fields.guaranteed_perf;

		/* Calculate hwp_ceiling from guaranteed_perf, never from
		 * highest_perf, so turbo headroom is excluded from the
		 * percentage calculation itself. */
		hwp_ceiling = sc->hwp_cap.fields.lowest_perf +
		    ((sc->perflevel * (perf_upper -
		    sc->hwp_cap.fields.lowest_perf)) / 100);

		/* Clamp hwp_ceiling to Most_Efficient_Performance as a
		 * floor. Intel SDM 15.4.3 states explicitly: "The processor
		 * may not honor IA32_HWP_REQUEST.Maximum Performance settings
		 * below this value [Most_Efficient_Performance]." This is the
		 * actual root cause of turbo boost firing despite a correctly
		 * computed, correctly written, min=max=desired MSR: requesting
		 * a Maximum_Performance below most_efficient_perf is explicitly
		 * documented as a value the hardware is allowed to not honor,
		 * and on this SKU the observed fallback behavior was reverting
		 * to autonomous/turbo selection rather than clamping quietly.
		 */
		if (hwp_ceiling < sc->hwp_cap.fields.most_efficient_perf) {
			DPRINTF(("%s: hwp_ceiling %d MHz below "
			    "most_efficient_perf %d MHz, clamping to floor "
			    "(SDM 15.4.3: HW may not honor Max below this)\n",
			    curcpu()->ci_dev->dv_xname,
			    hwp_ceiling * 100,
			    sc->hwp_cap.fields.most_efficient_perf * 100));
			hwp_ceiling = sc->hwp_cap.fields.most_efficient_perf;
		}

		sc->hwp_req.fields.min_perf = hwp_ceiling;
		sc->hwp_req.fields.max_perf = hwp_ceiling;
		sc->hwp_req.fields.desired_perf = hwp_ceiling;

		/* setup bias mode on manual perf policy*/
		if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPP) {
			sc->hwp_req.fields.epp = sc->user_epp;
		} else if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPB) {
			epb = sc->user_epb;
		}
		break;
	}

	/* Validate min/max/desired range AFTER the switch, on the
	 * values that were just computed for this call - not on stale
	 * values from the previous invocation. Intel SDM 15.4.4.1 requires
	 * all three to stay within [Lowest_Performance, Highest_Performance].
	 * This mainly guards against a corrupted hwp_cap read (e.g. before
	 * pstate_init() completes) producing an out-of-range hwp_ceiling.
	 */
	if (sc->hwp_req.fields.min_perf < sc->hwp_cap.fields.lowest_perf)
		sc->hwp_req.fields.min_perf = sc->hwp_cap.fields.lowest_perf;
	if (sc->hwp_req.fields.max_perf > sc->hwp_cap.fields.highest_perf)
		sc->hwp_req.fields.max_perf = sc->hwp_cap.fields.highest_perf;
	if (sc->hwp_req.fields.desired_perf != 0) {
		if (sc->hwp_req.fields.desired_perf < sc->hwp_req.fields.min_perf)
			sc->hwp_req.fields.desired_perf = sc->hwp_req.fields.min_perf;
		if (sc->hwp_req.fields.desired_perf > sc->hwp_req.fields.max_perf)
			sc->hwp_req.fields.desired_perf = sc->hwp_req.fields.max_perf;
	}

	/* Determine if physical register writes are needed */
	if (sc->hwp_req.msr != sc->applied_hwp_req) {
		req_msr = sc->hwp_req.msr;
		do_req = 1;
	}
	if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPB && sc->applied_epb != epb) {
		do_epb = 1;
	}

	if (do_req) {
		uint64_t readback;

		wrmsr(sc->hwp_req_msr, req_msr);
		sc->applied_hwp_req = req_msr;

		DPRINTF(("%s: wrmsr(0x%x) = 0x%016llx "
		    "(min=%d max=%d desired=%d epp=%d)\n",
		    curcpu()->ci_dev->dv_xname, sc->hwp_req_msr,
		    (unsigned long long)req_msr,
		    sc->hwp_req.fields.min_perf,
		    sc->hwp_req.fields.max_perf,
		    sc->hwp_req.fields.desired_perf,
		    sc->hwp_req.fields.epp));

		/* DIAGNOSTIC: Immediately read the MSR back. If this does
		 * not match what we just wrote, something else (firmware/EC
		 * via PECI, or another kernel subsystem racing on the same
		 * MSR) is overriding it. A mismatch here is conclusive proof
		 * the discrepancy is NOT in this driver's calculation - we
		 * wrote the correct value and the hardware/firmware changed
		 * it before our very next instruction could read it back.
		 */
		if (rdmsr_safe(sc->hwp_req_msr, &readback) == 0) {
			if (readback != req_msr) {
				printf("%s: WARNING: IA32_HWP_REQUEST readback "
				    "mismatch! wrote=0x%016llx read=0x%016llx "
				    "- value was overridden immediately after "
				    "write (firmware/EC/PECI override likely)\n",
				    curcpu()->ci_dev->dv_xname,
				    (unsigned long long)req_msr,
				    (unsigned long long)readback);
			}
		}
	}
	if (do_epb) {
		wrmsr(IA32_ENERGY_PERF_BIAS, epb);
		sc->applied_epb = epb;
	}
}

/*
 * Translates OS performance level (0-100) into HWP performance requests
 * and applies them to hardware based on the active policy.
 *
 * This function performs the following logic:
 * 1. Validates the performance level based on the current policy.
 * 2. Updates the per-CPU performance level state.
 * 3. Calls pstate_apply() to calculate and write the HWP configuration.
 */
void
pstate_setperf(int level)
{
	struct pstate_softc *sc = pstate_cpus[CPU_INFO_UNIT(curcpu())];
	int policy = pstate_current_perfpolicy();
	int j;

	if (sc == NULL || !sc->enabled)
		return;

	/*
	 * Validate level only in MANUAL mode, ignore otherwise.
	 */
	if (policy == PSTATE_PERFPOL_MANUAL) {
		if (level < 1)
			level = 1;
		if (level > 100)
			level = 100;
	}

	mtx_enter(&pstate_lock);
	for (j = 0; j < MAXCPUS; j++) {
		if (pstate_cpus[j] == NULL || !pstate_cpus[j]->enabled)
			continue;
		pstate_cpus[j]->perflevel = level;
	}
	mtx_leave(&pstate_lock);

	/* Apply immediately on THIS physical core only - its own wrmsr()
	 * is safe and correct here. Remaining cores pick up the new
	 * perflevel within at most 1 second via their own local
	 * pstate_tick().
	 */
	pstate_apply(sc);

	/* Debug logging showing all relevant fields and policy name */
	DPRINTF(("%s: setperf(level=%d, policy=%s) -> "
	    "min=%d, max=%d, desired=%d, bias=%s\n",
	    curcpu()->ci_dev->dv_xname, level,
	    (policy == PSTATE_PERFPOL_AUTO ? "auto" :
	     policy == PSTATE_PERFPOL_HIGH ? "high" : "manual"),
	    sc->hwp_req.fields.min_perf,
	    sc->hwp_req.fields.max_perf,
	    sc->hwp_req.fields.desired_perf,
	    pstate_hwp_bias_label(sc,
	        (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPP) ?
	        sc->hwp_req.fields.epp : sc->applied_epb)));
}

/*
 * Sysctl interface for HWP policy tuning (Min, Max, EPP/EPB) and
 * diagnostics, providing a secure bridge between userland and HWP MSRs.
 *
 * This function enforces strict access control:
 * - Read-only enforcement for min_perf, max_perf, and desired_perf nodes.
 * - Dynamic calculation for desired_perf using aperf/mperf ratios.
 * - Policy-aware EPP bias enforcement (locked to 'performance' when
 * perfpolicy is HIGH).
 * - Requires pstate_lock to ensure atomic updates across the HWP
 * configuration structures.
 */
int
pstate_hwp_sysctl(int *name, u_int namelen, void *oldp, size_t *oldlenp,
    void *newp, size_t newlen, struct proc *p)
{
	struct pstate_softc *sc = pstate_cpus[0];
	int policy = pstate_current_perfpolicy();
	uint64_t bias_val, mperf, aperf;
	uint64_t delta_m, delta_a;
	const char *bias;
	char newbias[64];
	int newval, err, i, j;

	if (namelen != 1)
		return (ENOTDIR);

	if (sc == NULL || !sc->enabled)
		return (EOPNOTSUPP);

	if (name[0] < 1 || name[0] >= HWP_MAXID)
		return (EOPNOTSUPP);

	switch (name[0]) {
	case HWP_MIN_PERF:
		mtx_enter(&pstate_lock);
		newval = (int)sc->hwp_req.fields.min_perf;
		mtx_leave(&pstate_lock);
		return (sysctl_rdint(oldp, oldlenp, newp, newval));

	case HWP_MAX_PERF:
		mtx_enter(&pstate_lock);
		newval = (int)sc->hwp_req.fields.max_perf;
		mtx_leave(&pstate_lock);
		return (sysctl_rdint(oldp, oldlenp, newp, newval));

	case HWP_DESIRED_PERF:
		if (pstate_read_counters(&mperf, &aperf) == 0) {
			mtx_enter(&pstate_lock);
			delta_m = mperf - sc->last_mperf;
			delta_a = aperf - sc->last_aperf;

			if (delta_m > 0) {
				newval = (int)((delta_a *
				    sc->hwp_cap.fields.highest_perf) / delta_m);
				if (newval > (int)sc->hwp_req.fields.max_perf)
					newval = (int)sc->hwp_req.fields.max_perf;
				if (newval < (int)sc->hwp_req.fields.min_perf)
					newval = (int)sc->hwp_req.fields.min_perf;
			} else {
				newval = 0;
			}
			mtx_leave(&pstate_lock);
		} else {
			newval = 0;
		}
		return (sysctl_rdint(oldp, oldlenp, newp, newval));

	case HWP_EPP:
		if (policy != PSTATE_PERFPOL_MANUAL && newp != NULL)
			return (EPERM);

		if (policy == PSTATE_PERFPOL_HIGH) {
			return (sysctl_rdstring(oldp, oldlenp, newp, "performance"));
		}

		mtx_enter(&pstate_lock);
		if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPP)
			bias_val = sc->hwp_req.fields.epp;
		else if (sc->hwp_bias_style == PSTATE_HWP_BIAS_EPB)
			bias_val = sc->applied_epb;
		else {
			mtx_leave(&pstate_lock);
			return (EINVAL);
		}
		bias = pstate_hwp_bias_label(sc, bias_val);
		mtx_leave(&pstate_lock);

		if (newp == NULL)
			return (sysctl_rdstring(oldp, oldlenp, newp, bias));

		memset(newbias, 0, sizeof(newbias));
		strlcpy(newbias, bias, sizeof(newbias));
		err = sysctl_string(oldp, oldlenp, newp, newlen, newbias,
		    sizeof(newbias) - 1);
		if (err)
			return (err);

		err = EINVAL;
		for (i = 0; i < (sizeof(pstate_epp_labels) /
		    sizeof(pstate_epp_labels[0])); i++) {
			if (strcmp(pstate_epp_labels[i].label, newbias) == 0) {
				err = 0;
				break;
			}
		}

		if (err == 0) {
			mtx_enter(&pstate_lock);

			for (j = 0; j < MAXCPUS; j++) {
				if (pstate_cpus[j] == NULL || !pstate_cpus[j]->enabled)
					continue;
				if (pstate_cpus[j]->hwp_bias_style == PSTATE_HWP_BIAS_EPP) {
					pstate_cpus[j]->user_epp = pstate_epp_labels[i].epp;
					if (j == CPU_INFO_UNIT(curcpu()))
						pstate_cpus[j]->hwp_req.fields.epp = pstate_epp_labels[i].epp;
				} else {
					/* Using epb_recommended instead of epb_max
					 * for better balance within the range.
					 */
					pstate_cpus[j]->user_epb = pstate_epp_labels[i].epb_recommended;
				}
			}

			pstate_apply_locked(pstate_cpus[CPU_INFO_UNIT(curcpu())]);

			mtx_leave(&pstate_lock);
		}

		return (err);
	default:
		return (EOPNOTSUPP);
	}
}