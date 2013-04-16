/*
 * arch/arm/kernel/topology.c
 *
 * Copyright (C) 2011 Linaro Limited.
 * Written by: Vincent Guittot
 *
 * based on arch/sh/kernel/topology.c
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License.  See the file "COPYING" in the main directory of this archive
 * for more details.
 */

#include <linux/cpu.h>
#include <linux/cpumask.h>
#include <linux/init.h>
#include <linux/percpu.h>
#include <linux/node.h>
#include <linux/nodemask.h>
#include <linux/of.h>
#include <linux/sched.h>
#include <linux/slab.h>

#include <asm/cputype.h>
#include <asm/topology.h>

/*
 * cpu power scale management
 */

/*
 * cpu power table
 * This per cpu data structure describes the relative capacity of each core.
 * On a heteregenous system, cores don't have the same computation capacity
 * and we reflect that difference in the cpu_power field so the scheduler can
 * take this difference into account during load balance. A per cpu structure
 * is preferred because each CPU updates its own cpu_power field during the
 * load balance except for idle cores. One idle core is selected to run the
 * rebalance_domains for all idle cores and the cpu_power can be updated
 * during this sequence.
 */

/* when CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY is in use, a new measure of
 * compute capacity is available. This is limited to a maximum of 1024 and
 * scaled between 0 and 1023 according to frequency.
 * Cores with different base CPU powers are scaled in line with this.
 * CPU capacity for each core represents a comparable ratio to maximum
 * achievable core compute capacity for a core in this system.
 *
 * e.g.1 If all cores in the system have a base CPU power of 1024 according to
 * efficiency calculations and are DVFS scalable between 500MHz and 1GHz, the
 * cores currently at 1GHz will have CPU power of 1024 whilst the cores
 * currently at 500MHz will have CPU power of 512.
 *
 * e.g.2
 * If core 0 has a base CPU power of 2048 and runs at 500MHz & 1GHz whilst
 * core 1 has a base CPU power of 1024 and runs at 100MHz and 200MHz, then
 * the following possibilities are available:
 *
 * cpu power\| 1GHz:100Mhz | 1GHz : 200MHz | 500MHz:100MHz | 500MHz:200MHz |
 * ----------|-------------|---------------|---------------|---------------|
 *    core 0 |    1024     |     1024      |     512       |     512       |
 *    core 1 |     256     |      512      |     256       |     512       |
 *
 * This information may be useful to the scheduler when load balancing,
 * so that the compute capacity of the core a task ran on can be baked into
 * task load histories.
 */
static DEFINE_PER_CPU(unsigned long, cpu_scale);
static DEFINE_PER_CPU(unsigned long, base_cpu_capacity);
static DEFINE_PER_CPU(unsigned long, invariant_cpu_capacity);
static DEFINE_PER_CPU(unsigned long, prescaled_cpu_capacity);

static int frequency_invariant_power_enabled = 1;

/* >0=1, <=0=0 */
void set_invariant_power_enabled(int val)
{
	if(val>0)
		frequency_invariant_power_enabled = 1;
	else
		frequency_invariant_power_enabled = 0;
}

unsigned long arch_scale_freq_power(struct sched_domain *sd, int cpu)
{
	return per_cpu(cpu_scale, cpu);
}

#ifdef CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY
unsigned long arch_get_cpu_capacity(int cpu)
{
	return per_cpu(invariant_cpu_capacity, cpu);
}
unsigned long arch_get_max_cpu_capacity(int cpu)
{
	return per_cpu(base_cpu_capacity, cpu);
}
#endif

static void set_power_scale(unsigned int cpu, unsigned long power)
{
	per_cpu(cpu_scale, cpu) = power;
}

#ifdef CONFIG_OF
struct cpu_efficiency {
	const char *compatible;
	unsigned long efficiency;
};

/*
 * Table of relative efficiency of each processors
 * The efficiency value must fit in 20bit and the final
 * cpu_scale value must be in the range
 *   0 < cpu_scale < 3*SCHED_POWER_SCALE/2
 * in order to return at most 1 when DIV_ROUND_CLOSEST
 * is used to compute the capacity of a CPU.
 * Processors that are not defined in the table,
 * use the default SCHED_POWER_SCALE value for cpu_scale.
 */
struct cpu_efficiency table_efficiency[] = {
	{"arm,cortex-a15", 3891},
	{"arm,cortex-a7",  2048},
	{NULL, },
};

struct cpu_capacity {
	unsigned long hwid;
	unsigned long capacity;
};

struct cpu_capacity *cpu_capacity;

unsigned long middle_capacity = 1;
/*
 * Iterate all CPUs' descriptor in DT and compute the efficiency
 * (as per table_efficiency). Also calculate a middle efficiency
 * as close as possible to  (max{eff_i} - min{eff_i}) / 2
 * This is later used to scale the cpu_power field such that an
 * 'average' CPU is of middle power. Also see the comments near
 * table_efficiency[] and update_cpu_power().
 */
static void __init parse_dt_topology(void)
{
	struct cpu_efficiency *cpu_eff;
	struct device_node *cn = NULL;
	unsigned long min_capacity = (unsigned long)(-1);
	unsigned long max_capacity = 0;
	unsigned long capacity = 0;
	int alloc_size, cpu = 0;

	alloc_size = nr_cpu_ids * sizeof(struct cpu_capacity);
	cpu_capacity = (struct cpu_capacity *)kzalloc(alloc_size, GFP_NOWAIT);

	while ((cn = of_find_node_by_type(cn, "cpu"))) {
		const u32 *rate, *reg;
		int len;

		if (cpu >= num_possible_cpus())
			break;

		for (cpu_eff = table_efficiency; cpu_eff->compatible; cpu_eff++)
			if (of_device_is_compatible(cn, cpu_eff->compatible))
				break;

		if (cpu_eff->compatible == NULL)
			continue;

		rate = of_get_property(cn, "clock-frequency", &len);
		if (!rate || len != 4) {
			pr_err("%s missing clock-frequency property\n",
				cn->full_name);
			continue;
		}

		reg = of_get_property(cn, "reg", &len);
		if (!reg || len != 4) {
			pr_err("%s missing reg property\n", cn->full_name);
			continue;
		}

		capacity = ((be32_to_cpup(rate)) >> 20) * cpu_eff->efficiency;

		/* Save min capacity of the system */
		if (capacity < min_capacity)
			min_capacity = capacity;

		/* Save max capacity of the system */
		if (capacity > max_capacity)
			max_capacity = capacity;

		cpu_capacity[cpu].capacity = capacity;
		cpu_capacity[cpu++].hwid = be32_to_cpup(reg);
	}

	if (cpu < num_possible_cpus())
		cpu_capacity[cpu].hwid = (unsigned long)(-1);

	/* If min and max capacities are equals, we bypass the update of the
	 * cpu_scale because all CPUs have the same capacity. Otherwise, we
	 * compute a middle_capacity factor that will ensure that the capacity
	 * of an 'average' CPU of the system will be as close as possible to
	 * SCHED_POWER_SCALE, which is the default value, but with the
	 * constraint explained near table_efficiency[].
	 */
	if (min_capacity == max_capacity)
		cpu_capacity[0].hwid = (unsigned long)(-1);
	else if (4*max_capacity < (3*(max_capacity + min_capacity)))
		middle_capacity = (min_capacity + max_capacity)
				>> (SCHED_POWER_SHIFT+1);
	else
		middle_capacity = ((max_capacity / 3)
				>> (SCHED_POWER_SHIFT-1)) + 1;

}

/*
 * Look for a customed capacity of a CPU in the cpu_capacity table during the
 * boot. The update of all CPUs is in O(n^2) for heteregeneous system but the
 * function returns directly for SMP system.
 */
void update_cpu_power(unsigned int cpu, unsigned long hwid)
{
	unsigned int idx = 0;

	/* look for the cpu's hwid in the cpu capacity table */
	for (idx = 0; idx < num_possible_cpus(); idx++) {
		if (cpu_capacity[idx].hwid == hwid)
			break;

		if (cpu_capacity[idx].hwid == -1)
			return;
	}

	if (idx == num_possible_cpus())
		return;

	set_power_scale(cpu, cpu_capacity[idx].capacity / middle_capacity);

	printk(KERN_INFO "CPU%u: update cpu_power %lu\n",
		cpu, arch_scale_freq_power(NULL, cpu));
}

#else
static inline void parse_dt_topology(void) {}
static inline void update_cpu_power(unsigned int cpuid, unsigned int mpidr) {}
#endif

 /*
 * cpu topology table
 */
struct cputopo_arm cpu_topology[NR_CPUS];

int arch_sd_local_flags(int level)
{
	/* Powergate at threading level doesn't make sense */
	if (level & SD_SHARE_CPUPOWER)
		return 1*SD_SHARE_POWERDOMAIN;

	return 0*SD_SHARE_POWERDOMAIN;
}

const struct cpumask *cpu_coregroup_mask(int cpu)
{
	return &cpu_topology[cpu].core_sibling;
}

void update_siblings_masks(unsigned int cpuid)
{
	struct cputopo_arm *cpu_topo, *cpuid_topo = &cpu_topology[cpuid];
	int cpu;

	/* update core and thread sibling masks */
	for_each_possible_cpu(cpu) {
		cpu_topo = &cpu_topology[cpu];

		if (cpuid_topo->socket_id != cpu_topo->socket_id)
			continue;

		cpumask_set_cpu(cpuid, &cpu_topo->core_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->core_sibling);

		if (cpuid_topo->core_id != cpu_topo->core_id)
			continue;

		cpumask_set_cpu(cpuid, &cpu_topo->thread_sibling);
		if (cpu != cpuid)
			cpumask_set_cpu(cpu, &cpuid_topo->thread_sibling);
	}
	smp_wmb();
}

/*
 * store_cpu_topology is called at boot when only one cpu is running
 * and with the mutex cpu_hotplug.lock locked, when several cpus have booted,
 * which prevents simultaneous write access to cpu_topology array
 */
void store_cpu_topology(unsigned int cpuid)
{
	struct cputopo_arm *cpuid_topo = &cpu_topology[cpuid];
	unsigned int mpidr;

	/* If the cpu topology has been already set, just return */
	if (cpuid_topo->core_id != -1)
		return;

	mpidr = read_cpuid_mpidr();

	/* create cpu topology mapping */
	if ((mpidr & MPIDR_SMP_BITMASK) == MPIDR_SMP_VALUE) {
		/*
		 * This is a multiprocessor system
		 * multiprocessor format & multiprocessor mode field are set
		 */

		if (mpidr & MPIDR_MT_BITMASK) {
			/* core performance interdependency */
			cpuid_topo->thread_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);
			cpuid_topo->core_id = MPIDR_AFFINITY_LEVEL(mpidr, 1);
			cpuid_topo->socket_id = MPIDR_AFFINITY_LEVEL(mpidr, 2);
		} else {
			/* largely independent cores */
			cpuid_topo->thread_id = -1;
			cpuid_topo->core_id = MPIDR_AFFINITY_LEVEL(mpidr, 0);
			cpuid_topo->socket_id = MPIDR_AFFINITY_LEVEL(mpidr, 1);
		}
	} else {
		/*
		 * This is an uniprocessor system
		 * we are in multiprocessor format but uniprocessor system
		 * or in the old uniprocessor format
		 */
		cpuid_topo->thread_id = -1;
		cpuid_topo->core_id = 0;
		cpuid_topo->socket_id = -1;
	}

	update_siblings_masks(cpuid);

	update_cpu_power(cpuid, mpidr & MPIDR_HWID_BITMASK);

	printk(KERN_INFO "CPU%u: thread %d, cpu %d, socket %d, mpidr %x\n",
		cpuid, cpu_topology[cpuid].thread_id,
		cpu_topology[cpuid].core_id,
		cpu_topology[cpuid].socket_id, mpidr);
}

/*
 * init_cpu_topology is called at boot when only one cpu is running
 * which prevent simultaneous write access to cpu_topology array
 */
void __init init_cpu_topology(void)
{
	unsigned int cpu;

	/* init core mask and power*/
	for_each_possible_cpu(cpu) {
		struct cputopo_arm *cpu_topo = &(cpu_topology[cpu]);

		cpu_topo->thread_id = -1;
		cpu_topo->core_id =  -1;
		cpu_topo->socket_id = -1;
		cpumask_clear(&cpu_topo->core_sibling);
		cpumask_clear(&cpu_topo->thread_sibling);

		set_power_scale(cpu, SCHED_POWER_SCALE);
	}
	smp_wmb();

	parse_dt_topology();
}


#ifdef CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY
#include <linux/cpufreq.h>

#define CPUPOWER_FREQSCALE_SHIFT 10
#define CPUPOWER_FREQSCALE_DEFAULT (1L << CPUPOWER_FREQSCALE_SHIFT)
struct cpufreq_extents {
       u32 max;
       u32 flags;
};
/* Flag set when the governor in use only allows one frequency.
 * Disables scaling.
 */
#define CPUPOWER_FREQINVAR_SINGLEFREQ 0x01
static struct cpufreq_extents freq_scale[CONFIG_NR_CPUS];

static unsigned long get_max_cpu_power(void)
{
       unsigned long max_cpu_power = 0;
       int cpu;
       for_each_online_cpu(cpu){
               if( per_cpu(cpu_scale, cpu) > max_cpu_power)
                       max_cpu_power = per_cpu(cpu_scale, cpu);
       }
       return max_cpu_power;
}


/* Called when the CPU Frequency is changed.
 * Once for each CPU.
 */
static int cpufreq_callback(struct notifier_block *nb,
                                       unsigned long val, void *data)
{
       struct cpufreq_freqs *freq = data;
       int cpu = freq->cpu;
       struct cpufreq_extents *extents;
       unsigned int curr_freq;

       if (freq->flags & CPUFREQ_CONST_LOOPS)
               return NOTIFY_OK;

       if (val != CPUFREQ_POSTCHANGE)
               return NOTIFY_OK;

       /* if dynamic load scale is disabled, set the load scale to 1.0 */
       if (!frequency_invariant_power_enabled) {
               per_cpu(invariant_cpu_capacity, cpu) = per_cpu(base_cpu_capacity, cpu);
               return NOTIFY_OK;
       }

       extents = &freq_scale[cpu];
       /* If our governor was recognised as a single-freq governor,
        * use curr = max to be sure multiplier is 1.0
        */
       if (extents->flags & CPUPOWER_FREQINVAR_SINGLEFREQ)
               curr_freq = extents->max;
       else
               curr_freq = freq->new >> CPUPOWER_FREQSCALE_SHIFT;

       per_cpu(invariant_cpu_capacity, cpu) = (curr_freq *
               per_cpu(prescaled_cpu_capacity, cpu)) >> CPUPOWER_FREQSCALE_SHIFT;
       return NOTIFY_OK;
}

/* Called when the CPUFreq governor is changed.
 * Only called for the CPUs which are actually changed by the
 * userspace.
 */
static int cpufreq_policy_callback(struct notifier_block *nb,
                                      unsigned long event, void *data)
{
       struct cpufreq_policy *policy = data;
       struct cpufreq_extents *extents;
       int cpu, singleFreq = 0, cpu_capacity;
       static const char performance_governor[] = "performance";
       static const char powersave_governor[] = "powersave";
       unsigned long max_cpu_power;

       if (event == CPUFREQ_START)
               return 0;

       if (event != CPUFREQ_INCOMPATIBLE)
               return 0;

       /* CPUFreq governors do not accurately report the range of
        * CPU Frequencies they will choose from.
        * We recognise performance and powersave governors as
        * single-frequency only.
        */
       if (!strncmp(policy->governor->name, performance_governor,
                       strlen(performance_governor)) ||
               !strncmp(policy->governor->name, powersave_governor,
                               strlen(powersave_governor)))
               singleFreq = 1;

       max_cpu_power = get_max_cpu_power();
       /* Make sure that all CPUs impacted by this policy are
        * updated since we will only get a notification when the
        * user explicitly changes the policy on a CPU.
        */
       for_each_cpu(cpu, policy->cpus) {
               /* scale cpu_power to max(1024) */
               cpu_capacity = (per_cpu(cpu_scale, cpu) << CPUPOWER_FREQSCALE_SHIFT)
                               / max_cpu_power;
               extents = &freq_scale[cpu];
               extents->max = policy->max >> CPUPOWER_FREQSCALE_SHIFT;
               if (!frequency_invariant_power_enabled) {
                       /* when disabled, invariant_cpu_scale = cpu_scale */
                       per_cpu(base_cpu_capacity, cpu) = CPUPOWER_FREQSCALE_DEFAULT;
                       per_cpu(invariant_cpu_capacity, cpu) = CPUPOWER_FREQSCALE_DEFAULT;
                       /* unused when disabled */
                       per_cpu(prescaled_cpu_capacity, cpu) = CPUPOWER_FREQSCALE_DEFAULT;
               } else {
                       if (singleFreq)
                               extents->flags |= CPUPOWER_FREQINVAR_SINGLEFREQ;
                       else
                               extents->flags &= ~CPUPOWER_FREQINVAR_SINGLEFREQ;
                       per_cpu(base_cpu_capacity, cpu) = cpu_capacity;
                       per_cpu(prescaled_cpu_capacity, cpu) = (cpu_capacity << CPUPOWER_FREQSCALE_SHIFT) / extents->max;
                       per_cpu(invariant_cpu_capacity, cpu) =
                                       ((policy->cur >> CPUPOWER_FREQSCALE_SHIFT) *
                                       per_cpu(prescaled_cpu_capacity, cpu)) >> CPUPOWER_FREQSCALE_SHIFT;
               }
       }
       return 0;
}

static struct notifier_block cpufreq_notifier = {
       .notifier_call  = cpufreq_callback,
};
static struct notifier_block cpufreq_policy_notifier = {
       .notifier_call  = cpufreq_policy_callback,
};

static int __init register_topology_cpufreq_notifier(void)
{
       int ret;

       /* init safe defaults since there are no policies at registration */
       for (ret = 0; ret < CONFIG_NR_CPUS; ret++) {
               /* safe defaults */
               freq_scale[ret].max = CPUPOWER_FREQSCALE_DEFAULT;
               per_cpu(base_cpu_capacity, ret) = CPUPOWER_FREQSCALE_DEFAULT;
               per_cpu(invariant_cpu_capacity, ret) = CPUPOWER_FREQSCALE_DEFAULT;
               per_cpu(prescaled_cpu_capacity, ret) = CPUPOWER_FREQSCALE_DEFAULT;
       }

       pr_info("topology: registering cpufreq notifiers for scale-invariant CPU Power\n");
       ret = cpufreq_register_notifier(&cpufreq_policy_notifier,
                       CPUFREQ_POLICY_NOTIFIER);

       if (ret != -EINVAL)
               ret = cpufreq_register_notifier(&cpufreq_notifier,
                       CPUFREQ_TRANSITION_NOTIFIER);

       return ret;
}

core_initcall(register_topology_cpufreq_notifier);
#endif /* CONFIG_ARCH_SCALE_INVARIANT_CPU_CAPACITY */
