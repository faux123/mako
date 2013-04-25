/*
 * Author: Paul Reioux aka Faux123 <reioux@gmail.com>
 *
 * Copyright 2012 Paul Reioux
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/earlysuspend.h>
#include <linux/workqueue.h>
#include <linux/cpu.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/rq_stats.h>

//#define DEBUG_INTELLI_PLUG
#undef DEBUG_INTELLI_PLUG

#define INTELLI_PLUG_MAJOR_VERSION	1
#define INTELLI_PLUG_MINOR_VERSION	6

#define DEF_SAMPLING_RATE		(50000)
#define DEF_SAMPLING_MS			(200)

#define DUAL_CORE_PERSISTENCE		15
#define TRI_CORE_PERSISTENCE		12
#define QUAD_CORE_PERSISTENCE		9

#define RUN_QUEUE_THRESHOLD		38

#define CPU_DOWN_FACTOR			3

static DEFINE_MUTEX(intelli_plug_mutex);

struct delayed_work intelli_plug_work;

static unsigned int intelli_plug_active = 0;
module_param(intelli_plug_active, uint, 0644);

static unsigned int eco_mode_active = 0;
module_param(eco_mode_active, uint, 0644);

static unsigned int persist_count = 0;
static bool suspended = false;

#define NR_FSHIFT	3
static unsigned int nr_fshift = NR_FSHIFT;
module_param(nr_fshift, uint, 0644);

static unsigned int nr_run_thresholds_full[] = {
/* 	1,  2,  3,  4 - on-line cpus target */
	5,  7,  9,  UINT_MAX /* avg run threads * 2 (e.g., 9 = 2.25 threads) */
	};

static unsigned int nr_run_thresholds_eco[] = {
/*      1,  2, - on-line cpus target */
        3,  UINT_MAX /* avg run threads * 2 (e.g., 9 = 2.25 threads) */
        };

static unsigned int nr_run_hysteresis = 4;  /* 0.5 thread */
module_param(nr_run_hysteresis, uint, 0644);

static unsigned int nr_run_last;

static unsigned int NwNs_Threshold[] = { 19, 30,  19,  11,  19,  11, 0,  11};
static unsigned int TwTs_Threshold[] = {140,  0, 140, 190, 140, 190, 0, 190};

static int mp_decision(void)
{
	static bool first_call = true;
	int new_state = 0;
	int nr_cpu_online;
	int index;
	unsigned int rq_depth;
	static cputime64_t total_time = 0;
	static cputime64_t last_time;
	cputime64_t current_time;
	cputime64_t this_time = 0;

	current_time = ktime_to_ms(ktime_get());
	if (first_call) {
		first_call = false;
	} else {
		this_time = current_time - last_time;
	}
	total_time += this_time;

	rq_depth = rq_info.rq_avg;
	//pr_info(" rq_deptch = %u", rq_depth);
	nr_cpu_online = num_online_cpus();

	if (nr_cpu_online) {
		index = (nr_cpu_online - 1) * 2;
		if ((nr_cpu_online < 4) && (rq_depth >= NwNs_Threshold[index])) {
			if (total_time >= TwTs_Threshold[index]) {
				new_state = 1;
			}
		} else if (rq_depth <= NwNs_Threshold[index+1]) {
			if (total_time >= TwTs_Threshold[index+1] ) {
				new_state = 0;
			}
		} else {
			total_time = 0;
		}
	} else {
		total_time = 0;
	}

	last_time = ktime_to_ms(ktime_get());

	return new_state;
}

static unsigned int calculate_thread_stats(void)
{
	unsigned int avg_nr_run = avg_nr_running();
	unsigned int nr_run;
	unsigned int threshold_size;

	if (!eco_mode_active) {
		threshold_size =  ARRAY_SIZE(nr_run_thresholds_full);
		nr_run_hysteresis = 8;
		nr_fshift = 3;
#ifdef DEBUG_INTELLI_PLUG
		pr_info("intelliplug: full mode active!");
#endif
	}
	else {
		threshold_size =  ARRAY_SIZE(nr_run_thresholds_eco);
		nr_run_hysteresis = 4;
		nr_fshift = 1;
#ifdef DEBUG_INTELLI_PLUG
		pr_info("intelliplug: eco mode active!");
#endif
	}

	for (nr_run = 1; nr_run < threshold_size; nr_run++) {
		unsigned int nr_threshold;
		if (!eco_mode_active)
			nr_threshold = nr_run_thresholds_full[nr_run - 1];
		else
			nr_threshold = nr_run_thresholds_eco[nr_run - 1];

		if (nr_run_last <= nr_run)
			nr_threshold += nr_run_hysteresis;
		if (avg_nr_run <= (nr_threshold << (FSHIFT - nr_fshift)))
			break;
	}
	nr_run_last = nr_run;

	return nr_run;
}

static void __cpuinit intelli_plug_work_fn(struct work_struct *work)
{
	unsigned int nr_run_stat;
	unsigned int cpu_count = 0;
	unsigned int nr_cpus = 0;

	int decision = 0;
	int i;

	if (intelli_plug_active == 1) {
		nr_run_stat = calculate_thread_stats();
#ifdef DEBUG_INTELLI_PLUG
		pr_info("nr_run_stat: %u\n", nr_run_stat);
#endif
		cpu_count = nr_run_stat;
		// detect artificial loads or constant loads
		// using msm rqstats
		nr_cpus = num_online_cpus();
		if (!eco_mode_active && (nr_cpus >= 1 && nr_cpus < 4)) {
			decision = mp_decision();
			if (decision) {
				switch (nr_cpus) {
				case 2:
					cpu_count = 3;
#ifdef DEBUG_INTELLI_PLUG
					pr_info("nr_run(2) => %u\n", nr_run_stat);
#endif
					break;
				case 3:
					cpu_count = 4;
#ifdef DEBUG_INTELLI_PLUG
					pr_info("nr_run(3) => %u\n", nr_run_stat);
#endif
					break;
				}
			}
		}

		if (!suspended) {
			switch (cpu_count) {
			case 1:
				if (persist_count > 0)
					persist_count--;
				if (persist_count == 0) {
					//take down everyone
					for (i = 3; i > 0; i--)
						cpu_down(i);
				}
#ifdef DEBUG_INTELLI_PLUG
				pr_info("case 1: %u\n", persist_count);
#endif
				break;
			case 2:
				persist_count = DUAL_CORE_PERSISTENCE;
				if (!decision)
					persist_count = DUAL_CORE_PERSISTENCE / CPU_DOWN_FACTOR;
				if (nr_cpus < 2) {
					for (i = 1; i < cpu_count; i++)
						cpu_up(i);
				} else {
					for (i = 3; i >  1; i--)
						cpu_down(i);
				}
#ifdef DEBUG_INTELLI_PLUG
				pr_info("case 2: %u\n", persist_count);
#endif
				break;
			case 3:
				persist_count = TRI_CORE_PERSISTENCE;
				if (!decision)
					persist_count = TRI_CORE_PERSISTENCE / CPU_DOWN_FACTOR;
				if (nr_cpus < 3) {
					for (i = 1; i < cpu_count; i++)
						cpu_up(i);
				} else {
					for (i = 3; i > 2; i--)
						cpu_down(i);
				}
#ifdef DEBUG_INTELLI_PLUG
				pr_info("case 3: %u\n", persist_count);
#endif
				break;
			case 4:
				persist_count = QUAD_CORE_PERSISTENCE;
				if (!decision)
					persist_count = QUAD_CORE_PERSISTENCE / CPU_DOWN_FACTOR;
				if (nr_cpus < 4)
					for (i = 1; i < cpu_count; i++)
						cpu_up(i);
#ifdef DEBUG_INTELLI_PLUG
				pr_info("case 4: %u\n", persist_count);
#endif
				break;
			default:
				pr_err("Run Stat Error: Bad value %u\n", nr_run_stat);
				break;
			}
		}
#ifdef DEBUG_INTELLI_PLUG
		else
			pr_info("intelli_plug is suspened!\n");
#endif
	}
	schedule_delayed_work_on(0, &intelli_plug_work,
		msecs_to_jiffies(DEF_SAMPLING_MS));
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void intelli_plug_early_suspend(struct early_suspend *handler)
{
	int i;
	int num_of_active_cores = 4;
	
	cancel_delayed_work_sync(&intelli_plug_work);

	mutex_lock(&intelli_plug_mutex);
	suspended = true;
	mutex_unlock(&intelli_plug_mutex);

	// put rest of the cores to sleep!
	for (i = num_of_active_cores - 1; i > 0; i--) {
		cpu_down(i);
	}
}

static void __cpuinit intelli_plug_late_resume(struct early_suspend *handler)
{
	int num_of_active_cores;
	int i;

	mutex_lock(&intelli_plug_mutex);
	/* keep cores awake long enough for faster wake up */
	persist_count = DUAL_CORE_PERSISTENCE;
	suspended = false;
	mutex_unlock(&intelli_plug_mutex);

	/* wake up everyone */
	if (eco_mode_active)
		num_of_active_cores = 2;
	else
		num_of_active_cores = 4;

	for (i = 1; i < num_of_active_cores; i++) {
		cpu_up(i);
	}

	schedule_delayed_work_on(0, &intelli_plug_work,
		msecs_to_jiffies(10));
}

static struct early_suspend intelli_plug_early_suspend_struct_driver = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB + 10,
	.suspend = intelli_plug_early_suspend,
	.resume = intelli_plug_late_resume,
};
#endif	/* CONFIG_HAS_EARLYSUSPEND */

int __init intelli_plug_init(void)
{
	/* We want all CPUs to do sampling nearly on same jiffy */
	int delay = usecs_to_jiffies(DEF_SAMPLING_RATE);

	if (num_online_cpus() > 1)
		delay -= jiffies % delay;

	//pr_info("intelli_plug: scheduler delay is: %d\n", delay);
	pr_info("intelli_plug: version %d.%d by faux123\n",
		 INTELLI_PLUG_MAJOR_VERSION,
		 INTELLI_PLUG_MINOR_VERSION);

	INIT_DELAYED_WORK(&intelli_plug_work, intelli_plug_work_fn);
	schedule_delayed_work_on(0, &intelli_plug_work, delay);

#ifdef CONFIG_HAS_EARLYSUSPEND
	register_early_suspend(&intelli_plug_early_suspend_struct_driver);
#endif
	return 0;
}

MODULE_AUTHOR("Paul Reioux <reioux@gmail.com>");
MODULE_DESCRIPTION("'intell_plug' - An intelligent cpu hotplug driver for "
	"Low Latency Frequency Transition capable processors");
MODULE_LICENSE("GPL");

late_initcall(intelli_plug_init);
