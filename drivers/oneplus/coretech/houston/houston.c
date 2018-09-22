#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/init.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/thermal.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/pm.h>
#include <linux/platform_device.h>
#include <linux/cpufreq.h>
#include <linux/freezer.h>
#include <linux/power_supply.h>

#define HT_POLLING_MIN_INTERVAL (100)
#define HT_REPORT_PERIOD_MAX (18000)

/* customize your monitor */
enum {
	HT_CLUS_FREQ_0_MIN,
	HT_CLUS_FREQ_0_CUR,
	HT_CLUS_FREQ_0_MAX,
	HT_CLUS_FREQ_1_MIN,
	HT_CLUS_FREQ_1_CUR,
	HT_CLUS_FREQ_1_MAX,
	HT_BAT_VOLT_NOW,
	HT_BAT_CURR_NOW,
	HT_CPU_0,
	HT_CPU_1,
	HT_CPU_2,
	HT_CPU_3,
	HT_CPU_4,
	HT_CPU_5,
	HT_CPU_6,
	HT_CPU_7,
	HT_SEN_0,
	HT_SEN_1,
	HT_SEN_2,
	HT_MONITOR_SIZE
};

const char *ht_monitor_case[HT_MONITOR_SIZE] = {
	"clus_0_min",
	"clus_0_cur",
	"clus_0_max",
	"clus_1_min",
	"clus_1_cur",
	"clus_1_max",
	"voltage_now",
	"current_now",
	"cpu0-silver-usr",
	"cpu1-silver-usr",
	"cpu2-silver-usr",
	"cpu3-silver-usr",
	"cpu0-gold-usr",
	"cpu1-gold-usr",
	"cpu2-gold-usr",
	"cpu3-gold-usr",
	"xo-therm-adc",
	"msm-therm-adc",
	"quiet-therm-adc"
};

static unsigned int ht_all_mask;

static unsigned int report_period = 600;
module_param_named(period, report_period, uint, 0644);

static unsigned int filter_mask = 0;
module_param_named(filter_mask, filter_mask, uint, 0644);

static unsigned int disable_mask = 0;
module_param_named(disable_mask, disable_mask, uint, 0644);

/* div should not be 0, check before assign */
static unsigned int report_div[HT_MONITOR_SIZE];
module_param_array_named(div, report_div, uint, NULL, 0644);

static unsigned int sample_period = 3000;

static int ht_sample_period_store(const char *buf, const struct kernel_param *kp)
{
	unsigned int val;

	if (sscanf(buf, "%u\n", &val) <= 0)
		return -EINVAL;

	if (val < HT_POLLING_MIN_INTERVAL) {
		pr_err("Not allowed to polling too fast\n");
		return -EINVAL;
	}

	sample_period = val;

	return 0;
}

static int ht_sample_period_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%u\n", sample_period);
}

static struct kernel_param_ops ht_sample_period_ops = {
	.set = ht_sample_period_store,
	.get = ht_sample_period_show,
};
module_param_cb(sample_period, &ht_sample_period_ops, NULL, 0644);

/* NOTE better not use this directly */
int sidx;
int smps[HT_REPORT_PERIOD_MAX][HT_MONITOR_SIZE]; // samples

/* TODO set ht_tzd_idx to atomic */
static int ht_tzd_idx = HT_CPU_0;
static bool __read_mostly keep_alive = false;
static bool __read_mostly ht_reporting = false;

struct track {
	struct thermal_zone_device *tzd;
	int slope;
	int offset;
	bool disabled;
};

struct ht_rocket {
	struct power_supply *psy;
	struct track trk[HT_MONITOR_SIZE];
	struct task_struct *monitor_thread;
	struct task_struct *stable_thread;
	bool running;
} rocket;

static inline int ht_next_sample_idx(void)
{
	++sidx;
	sidx %= HT_REPORT_PERIOD_MAX;
	return sidx;
}

static inline void ht_set_all_mask(void)
{
	int i;
	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		ht_all_mask |= (1 << i);
}

static inline bool ht_is_all_disabled(unsigned int mask)
{
	return ht_all_mask == mask;
}

static inline bool ht_is_all_filter(unsigned int mask)
{
	return ht_all_mask == mask;
}

static inline int ht_mapping_tags(char *name)
{
	int i;

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		if (!strcmp(name, ht_monitor_case[i]))
			return i;
	return HT_MONITOR_SIZE;
}

void ht_register_thermal_zone_device(struct thermal_zone_device *tzd)
{
	int idx;

	/* tzd is guaranteed has value */
	pr_info("tzd: %s id: %d\n", tzd->type, tzd->id);

	if (!tzd->type)
		return;

	idx = ht_mapping_tags(tzd->type);

	if (idx == HT_MONITOR_SIZE)
		return;

	if (ht_tzd_idx < HT_MONITOR_SIZE) {
		++ht_tzd_idx;
		rocket.trk[idx].tzd = tzd;
	}
}

void ht_register_power_supply(struct power_supply *psy)
{
	if (!psy)
		return;

	pr_info("ht power supply list %s\n", psy->desc->name);
	if (!strcmp(psy->desc->name, "battery")) {
		rocket.psy = psy;
		pr_info("ht power supply registed %s\n", psy->desc->name);
	}
}

#define SILVER_CLUS_IDX 0
#define GOLDEN_CLUS_IDX 4
static void ht_collect_resources(void)
{
	int temp, i, idx = ht_next_sample_idx(), ret;
	unsigned int d_mask = disable_mask;
	struct thermal_zone_device* tzd;
	struct cpufreq_policy *pol_0, *pol_1, *pol;
	union power_supply_propval prop = {0, };

	if (ht_is_all_disabled(d_mask))
		return;

	/* temperature part */
	for (i = HT_CPU_0; i < ht_tzd_idx; ++i) {
		if (d_mask & (1 << i)) {
			smps[idx][i] = 0;
			continue;
		}

		temp = -1;
		tzd = rocket.trk[i].tzd;
		if (likely(tzd)) {
			ret = thermal_zone_get_temp(tzd, &temp);
			if (ret) {
				if (ret != -EAGAIN)
					pr_debug("failed to read out thermal zone with idx %d\n", i);
				temp = -1;
			}
		}
		smps[idx][i] = temp;
	}

	/* cpufreq part */
	pol_0 = cpufreq_cpu_get(SILVER_CLUS_IDX);
	pol_1 = cpufreq_cpu_get(GOLDEN_CLUS_IDX);
	for (i = 0; i < HT_BAT_VOLT_NOW; ++i) {
		if (d_mask & (1 << i)) {
			smps[idx][i] = 0;
			continue;
		}

		pol = (i < HT_CLUS_FREQ_1_MIN)? pol_0: pol_1;
		if (likely(pol)) {
			switch(i) {
			case HT_CLUS_FREQ_0_MIN:
			case HT_CLUS_FREQ_1_MIN:
				smps[idx][i] = pol->min; break;
			case HT_CLUS_FREQ_0_CUR:
			case HT_CLUS_FREQ_1_CUR:
				smps[idx][i] = pol->cur; break;
			case HT_CLUS_FREQ_0_MAX:
			case HT_CLUS_FREQ_1_MAX:
				smps[idx][i] = pol->max; break;
			default:
				smps[idx][i] = 0;
			}
		} else {
			pr_debug("failed to read cpufreq with idx %d\n", i);
			smps[idx][i] = 0;
		}
	}
	if (pol_0) cpufreq_cpu_put(pol_0);
	if (pol_1) cpufreq_cpu_put(pol_1);

	/* battery part */
	ret = power_supply_get_property(rocket.psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
	smps[idx][HT_BAT_VOLT_NOW] = ret >= 0? prop.intval: 0;
	ret = power_supply_get_property(rocket.psy, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
	smps[idx][HT_BAT_CURR_NOW] = ret >= 0? prop.intval: 0;
}

static inline void ht_wake_up(void)
{
	wake_up_process(rocket.monitor_thread);
	wake_up_process(rocket.stable_thread);
}

/* main thread for monitor */
static int ht_monitor(void *arg)
{
	bool not_stable = false;
	while (likely(keep_alive)) {
		/* reset for every time evaluation */
		not_stable = false;

		if (!rocket.running) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule();
			continue;
		}

		if (likely(!pm_freezing))
			ht_collect_resources();

		/* check tsens data to make decision */
		if (not_stable) {
			pr_err("wakeup stable_thread\n");
			wake_up_process(rocket.stable_thread);
		}
		msleep(sample_period);
	}

	return 0;
}

/* try to stable something, maybe temperature or ...? */
static int ht_stable(void *arg)
{
	while (likely(keep_alive)) {
		if (!rocket.running) {
			set_current_state(TASK_UNINTERRUPTIBLE);
			schedule();
			continue;
		}

		/* TODO put some thoughts please */

		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule();
	}

	return 0;
}

static int ht_registed_show(char* buf, const struct kernel_param *kp)
{
	int i, offset = 0;
	struct thermal_zone_device* tzd;

	if (ht_tzd_idx == 0)
		return 0;

	for (i = 0; i < ht_tzd_idx; ++i) {
		tzd = rocket.trk[i].tzd;
		if (tzd)
			offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s, id: %d\n", tzd->type, tzd->id);
		else if (i < HT_CPU_0)
			offset += snprintf(buf + offset, PAGE_SIZE - offset, "%s\n", ht_monitor_case[i]);
	}

	return offset;
}

static struct kernel_param_ops ht_registed_ops = {
	.get = ht_registed_show,
};
module_param_cb(ht_registed, &ht_registed_ops, NULL, 0644);

static int ht_reset_show(char *buf, const struct kernel_param *kp)
{
	memset(smps, 0, sizeof(int) * HT_REPORT_PERIOD_MAX * HT_MONITOR_SIZE);
	return snprintf(buf, PAGE_SIZE, "sample data reset\n");
}

static struct kernel_param_ops ht_reset_ops = {
	.get = ht_reset_show,
};
module_param_cb(reset, &ht_reset_ops, NULL, 0644);

static int ht_report_proc_show(struct seq_file *m, void *v)
{
	unsigned int i, cnt = 0, j;
	unsigned int f_mask = filter_mask, d_mask = disable_mask;
	unsigned int report_cnt = 0;
	struct thermal_zone_device* tzd;

	report_cnt = report_period;

	/* cap max */
	if (unlikely(report_cnt > HT_REPORT_PERIOD_MAX))
		report_cnt = HT_REPORT_PERIOD_MAX;

	if (!report_cnt)
		return 0;

	ht_reporting = true;

	if (ht_is_all_filter(f_mask) || ht_is_all_disabled(d_mask))
		goto out;

	/* mark */
	for (i = 0; i < ht_tzd_idx; ++i) {
		if (f_mask & (1 << i) || d_mask & (1 << i))
			continue;
		tzd = rocket.trk[i].tzd;
		if (tzd)
			seq_printf(m, "%s\t", tzd->type);
		else if (i < HT_CPU_0)
			seq_printf(m, "%s\t", ht_monitor_case[i]);
	}
	seq_printf(m, "\n");

	/* fill gap */
	i = sidx + HT_REPORT_PERIOD_MAX - report_cnt + 1;

	/* value */
	for (; cnt < report_cnt; ++cnt, ++i) {
		i %= HT_REPORT_PERIOD_MAX;

		for (j = 0; j < HT_MONITOR_SIZE; ++j) {
			if (f_mask & (1 << j) || d_mask & (1 << j))
				continue;

			seq_printf(m, "%d\t", smps[i][j]/ (report_div[j]? report_div[j]: 1));
		}
		seq_printf(m, "\n");
	}

out:
	ht_reporting = false;
	return 0;
}

static int ht_report_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ht_report_proc_show, NULL);
}

static const struct file_operations ht_report_proc_fops = {
	.open= ht_report_proc_open,
	.read= seq_read,
	.llseek= seq_lseek,
	.release= single_release,
};

static int ht_init(void)
{
	int i;

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		report_div[i] = (i >= HT_CPU_0)? 100: 1;

	ht_set_all_mask();

	/* default disabled, enable while you need it */
	disable_mask = ht_all_mask;

	rocket.monitor_thread = kthread_create(ht_monitor, NULL, "ht_rocket");
	if (IS_ERR(rocket.monitor_thread)) {
		pr_err("Can't create ht monitor thread\n");
		goto out;
	}

	rocket.stable_thread = kthread_create(ht_stable, NULL, "ht_stable");
	if (IS_ERR(rocket.stable_thread)) {
		pr_err("Can't create ht stable thread\n");
		wake_up_process(rocket.monitor_thread);
		goto out;
	}

	keep_alive = true;
	rocket.running = true;
	ht_wake_up();
	proc_create("ht_report", S_IFREG | 0400, NULL, &ht_report_proc_fops);

	return 0;
out:
	return -EINVAL;
}

pure_initcall(ht_init);
MODULE_AUTHOR("tedlin <ted.lin@oneplus.com>");
MODULE_DESCRIPTION("Oneplus Houston");
MODULE_LICENSE("GPL v2");
