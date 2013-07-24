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

#include <linux/module.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>

#include <linux/cpu.h>
#include <linux/cpufreq.h>

#include <mach/cpufreq.h>

#define MSM_CPUFREQ_LIMIT_VERSION 1

static uint32_t limited_max_freq = MSM_CPUFREQ_NO_LIMIT;

static int update_cpu_max_freq(int cpu, uint32_t max_freq)
{
	int ret = 0;

	ret = msm_cpufreq_set_freq_limits(cpu, MSM_CPUFREQ_NO_LIMIT, max_freq);
	if (ret)
		return ret;

	limited_max_freq = max_freq;
	if (max_freq != MSM_CPUFREQ_NO_LIMIT)
		pr_info("msm_cpufreq_limit: Limiting cpu%d max frequency to %d\n",
				cpu, max_freq);
	else
		pr_info("msm_cpufreq_limit: Max frequency reset for cpu%d\n", cpu);

	ret = cpufreq_update_policy(cpu);

	return ret;
}

static ssize_t msm_cpufreq_limit_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", limited_max_freq);
}

static ssize_t msm_cpufreq_limit_store(struct kobject *kobj, struct kobj_attribute *attr, const char *buf, size_t count)
{
	int cpu = 0;
	int ret = 0;
	uint32_t max_freq = limited_max_freq;

	unsigned int data;

	if(sscanf(buf, "%u\n", &data) == 1) {
		for_each_possible_cpu(cpu) {
			ret = update_cpu_max_freq(cpu, max_freq);
			if (ret)
				pr_debug("Unable to limit cpu%d max freq to %d\n",
						cpu, max_freq);
		}
		if (!ret)
			limited_max_freq = data;
	}

	return count;
}

static ssize_t msm_cpufreq_limit_version_show(struct kobject *kobj, struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf, "version: %u\n", MSM_CPUFREQ_LIMIT_VERSION);
}

static struct kobj_attribute msm_cpufreq_limit_attribute = 
	__ATTR(cpufreq_limit, 0666, msm_cpufreq_limit_show, msm_cpufreq_limit_store);

static struct kobj_attribute msm_cpufreq_limit_version_attribute = 
	__ATTR(msm_cpufreq_limit_version, 0444 , msm_cpufreq_limit_version_show, NULL);

static struct attribute *msm_cpufreq_limit_attrs[] =
	{
		&msm_cpufreq_limit_attribute.attr,
		&msm_cpufreq_limit_version_attribute.attr,
		NULL,
	};

static struct attribute_group msm_cpufreq_limit_attr_group =
	{
		.attrs = msm_cpufreq_limit_attrs,
	};

static struct kobject *msm_cpufreq_limit_kobj;

static int msm_cpufreq_limit_init(void)
{
	int sysfs_result;

	msm_cpufreq_limit_kobj = kobject_create_and_add("msm_cpufreq_limit", kernel_kobj);
	if (!msm_cpufreq_limit_kobj) {
		pr_err("%s msm_cpufreq_limit_kobj kobject create failed!\n", __FUNCTION__);
		return -ENOMEM;
        }

	sysfs_result = sysfs_create_group(msm_cpufreq_limit_kobj, &msm_cpufreq_limit_attr_group);

        if (sysfs_result) {
		pr_info("%s msm_cpufreq_limit_kobj create failed!\n", __FUNCTION__);
		kobject_put(msm_cpufreq_limit_kobj);
	}
	return sysfs_result;
}

static void msm_cpufreq_limit_exit(void)
{
	if (msm_cpufreq_limit_kobj != NULL)
		kobject_put(msm_cpufreq_limit_kobj);
}

module_init(msm_cpufreq_limit_init);
module_exit(msm_cpufreq_limit_exit);

