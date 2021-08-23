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
#include <linux/poll.h>
#include <linux/ioctl.h>
#include "houston.h"
#include "../drivers/gpu/msm/kgsl.h"
#include "../drivers/gpu/msm/kgsl_pwrctrl.h"
#include <oneplus/houston/houston_helper.h>
#include <linux/jiffies.h>

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
	HT_FPS_PROCESS,
	HT_FPS_LAYER,
	HT_FPS_PID,
	HT_FPS_ALIGN,
	HT_FPS_1,
	HT_FPS_2,
	HT_FPS_3,
	HT_FPS_4,
	HT_FPS_5,
	HT_FPS_6,
	HT_FPS_7,
	HT_FPS_8,
	HT_FPS_MISS_LAYER,
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
	"quiet-therm-adc",
	"process name",
	"layer name",
	"pid",
	"fps_align",
	"actualFps",
	"predictFps",
	"Vsync",
	"gameFps",
	"NULL",
	"NULL",
	"NULL",
	"NULL",
	"missedLayer"
};

/*
 * houston monitor
 * @data: sample data
 * @layer: sample data for frame info
 * @process: sample data for frame process info
 */
struct sample_data {
	u64 data[MAX_REPORT_PERIOD][HT_MONITOR_SIZE];
	char layer[MAX_REPORT_PERIOD][FPS_LAYER_LEN];
	char process[MAX_REPORT_PERIOD][FPS_PROCESS_NAME_LEN];
};

struct ht_monitor {
	struct power_supply *psy;
	struct thermal_zone_device *tzd[HT_MONITOR_SIZE];
	struct task_struct *thread;
	struct sample_data *buf;
} monitor = {
	.psy = NULL,
	.thread = NULL,
	.buf = NULL,
};

struct ht_util_pol {
	unsigned long *utils[HT_CPUS_PER_CLUS];
	unsigned long *hi_util;
};

static struct game_fps_data_struct
{
	u64 fps;
	u64 enqueue_time;
} game_fps_data;

static DECLARE_WAIT_QUEUE_HEAD(ht_poll_waitq);

static DEFINE_SPINLOCK(egl_buf_lock);
#define EGL_BUF_MAX (128)
static char egl_buf[EGL_BUF_MAX] = {0};

static atomic_t cached_fps[2];

static atomic64_t fps_align_ns;

static struct kgsl_pwrctrl *gpwr;

static unsigned int ht_all_mask;

static unsigned int report_period = 600;
module_param_named(period, report_period, uint, 0644);

static unsigned int filter_mask = 0;
module_param_named(filter_mask, filter_mask, uint, 0644);

static unsigned int disable_mask = 0;
module_param_named(disable_mask, disable_mask, uint, 0644);

static bool bat_sample_high_resolution = false;
module_param_named(bat_sample_high_resolution, bat_sample_high_resolution, bool, 0664);

/* force update battery current */
static unsigned long bat_update_period_us = 1000000; // 1 sec
module_param_named(bat_update_period_us, bat_update_period_us, ulong, 0664);

static unsigned int thermal_update_period_hz = 100;
module_param_named(thermal_update_period_hz, thermal_update_period_hz, uint, 0664);

/* div should not be 0, check before assign */
static unsigned int report_div[HT_MONITOR_SIZE];
module_param_array_named(div, report_div, uint, NULL, 0644);

static unsigned int sample_period = 3000;

static dev_t ht_ctl_dev;
static struct class *driver_class;
static struct cdev cdev;

static struct ht_util_pol ht_utils[HT_CLUSTERS];

static inline const char* ht_ioctl_str(unsigned int cmd)
{
	switch (cmd) {
	case HT_IOC_COLLECT: return "HT_IOC_COLLECT";
	case HT_IOC_SCHEDSTAT: return "HT_IOC_SCHEDSTAT";
	case HT_IOC_FPS_STABILIZER_UPDATE: return "HT_IOC_FPS_STABILIZER_UPDATE";
	case HT_IOC_FPS_PARTIAL_SYS_INFO: return "HT_IOC_FPS_PARTIAL_SYS_INFO";
	}
	return "UNKNOWN";
}

static inline void ht_update_battery(void)
{
	static u64 prev = 0;
	u64 cur = ktime_to_us(ktime_get());

	if (cur - prev >= bat_update_period_us) {
		prev = cur;
	} else if (prev > cur) {
		prev = cur;
	}
}

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

/* fps */
static int game_fps_pid = -1;
module_param_named(game_fps_pid, game_fps_pid, int, 0664);

static struct ai_parcel parcel;

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

static inline int ht_get_temp(int monitor_idx)
{
	int temp = 0;
	struct thermal_zone_device* tzd;
        tzd=rocket.trk[monitor_idx].tzd;
	if (!tzd)
		return 0;
	if (disable_mask & (1 << monitor_idx))
		return 0;
	if (thermal_zone_get_temp(tzd, &temp))
		return 0;
	return temp;
}

static unsigned int ht_get_temp_delay(int idx)
{
	static unsigned long next[HT_MONITOR_SIZE] = {0};
	static unsigned int temps[HT_MONITOR_SIZE] = {0};
	/* only allow for reading sensor data */
	if (unlikely(idx < HT_CPU_0 || idx > HT_SEN_2))
		return 0;
	/* update */
	if (jiffies > next[idx] && jiffies - next[idx] > thermal_update_period_hz) {
		next[idx] = jiffies;
		temps[idx] = ht_get_temp(idx);
	}
	if (jiffies < next[idx]) {
		next[idx] = jiffies;
		temps[idx] = ht_get_temp(idx);
	}
	return temps[idx];
}

static int egl_buf_store(const char *buf, const struct kernel_param *kp)
{
	char _buf[EGL_BUF_MAX] = {0};

	if (strlen(buf) >= EGL_BUF_MAX)
		return 0;

	if (sscanf(buf, "%s\n", _buf) <= 0)
		return 0;

	spin_lock(&egl_buf_lock);
	memcpy(egl_buf, _buf, EGL_BUF_MAX);
	egl_buf[EGL_BUF_MAX - 1] = '\0';
	spin_unlock(&egl_buf_lock);

	return 0;
}

static int egl_buf_show(char *buf, const struct kernel_param *kp)
{
	char _buf[EGL_BUF_MAX] = {0};

	spin_lock(&egl_buf_lock);
	memcpy(_buf, egl_buf, EGL_BUF_MAX);
	_buf[EGL_BUF_MAX - 1] = '\0';
	spin_unlock(&egl_buf_lock);

	return snprintf(buf, PAGE_SIZE, "%s\n", _buf);
}

static struct kernel_param_ops egl_buf_ops = {
	.set = egl_buf_store,
	.get = egl_buf_show,
};
module_param_cb(egl_buf, &egl_buf_ops, NULL, 0664);

/* fps stable parameter, default -1 max */
static int efps_max = -1;
static int efps_pid = 0;
static int expectfps = -1;
static int efps_max_store(const char *buf, const struct kernel_param *kp)
{
	int val;
	int pid;
	int eval;
	int ret;

	ret = sscanf(buf, "%d,%d,%d\n", &pid, &val, &eval);
	if (ret < 2)
		return 0;

	efps_pid = pid;
	efps_max = val;
	expectfps = (ret == 3) ? eval : -1;
	return 0;
}

static int efps_max_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%d,%d,%d\n", efps_pid, efps_max, expectfps);
}

static struct kernel_param_ops efps_max_ops = {
	.set = efps_max_store,
	.get = efps_max_show,
};
module_param_cb(efps_max, &efps_max_ops, NULL, 0664);

void ht_register_thermal_zone_device(struct thermal_zone_device *tzd)
{
	int idx;

	/* tzd is guaranteed has value */
	pr_info("tzd: %s id: %d\n", tzd->type, tzd->id);

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

void ht_register_kgsl_pwrctrl(void *pwr)
{
	gpwr = (struct kgsl_pwrctrl *) pwr;
	pr_info("Registered lgsl pwrctrl\n");
}

void ht_register_cpu_util(unsigned int cpu, unsigned int first_cpu,
		unsigned long *util, unsigned long *hi_util)
{
	struct ht_util_pol *hus;

	switch (first_cpu) {
	case 0: case 1: case 2: case 3: hus = &ht_utils[0]; break;
	case 4: case 5: case 6: case 7: hus = &ht_utils[1]; break;
	default:
		/* should not happen */
		pr_info("wrong first cpu utils idx %d\n", first_cpu);
		return;
	}

	if (cpu == CLUS_1_IDX)
		cpu = 0;
	else
		cpu %= HT_CPUS_PER_CLUS;
	hus->utils[cpu] = util;
	hus->hi_util = hi_util;
	pr_debug("ht register cpu %d util, first cpu %d\n", cpu, first_cpu);
}

/* fps stabilizer update & online config update */
static DECLARE_WAIT_QUEUE_HEAD(ht_fps_stabilizer_waitq);
static char ht_online_config_buf[PAGE_SIZE];
static bool ht_disable_fps_stabilizer_bat = true;
module_param_named(disable_fps_stabilizer_bat, ht_disable_fps_stabilizer_bat, bool, 0664);

static int ht_online_config_update_store(const char *buf, const struct kernel_param *kp)
{
	int ret;

	ret = sscanf(buf, "%s\n", ht_online_config_buf);
	pr_debug("fpsst: %d, %s\n", ret, ht_online_config_buf);
	wake_up(&ht_fps_stabilizer_waitq);
	return 0;
}

static int ht_online_config_update_show(char *buf, const struct kernel_param *kp)
{
	return snprintf(buf, PAGE_SIZE, "%s\n", ht_online_config_buf);
}

static struct kernel_param_ops ht_online_config_update_ops = {
	.set = ht_online_config_update_store,
	.get = ht_online_config_update_show,
};
module_param_cb(ht_online_config_update, &ht_online_config_update_ops, NULL, 0664);

static int ht_fps_data_sync_store(const char *buf, const struct kernel_param *kp)
{
	u64 fps_data[FPS_COLS] = {0};
	static long long fps_align;
	static int cur_idx;
	int ret;

	if (strlen(buf) >= FPS_DATA_BUF_SIZE)
		return 0;

	ret = sscanf(buf, "%llu,%llu,%llu,%llu,%llu,%lld\n",
			&fps_data[0], &fps_data[1], &fps_data[2], &fps_data[3],
			&fps_data[4], &fps_align);
	if (ret != 6) {
		pr_err("fps data params invalid. got %d inputs instead of %d,%s has been ignored\n",
				(FPS_COLS+1), ret, buf);
		return 0;
	}

	game_fps_data.enqueue_time = fps_align;
	game_fps_data.fps = fps_data[3];

	pr_info("fps data params: %llu %llu %llu %llu %llu %lld\n",
			fps_data[0], fps_data[1], fps_data[2],
			fps_data[3], fps_data[4], fps_align);

	atomic_set(&cached_fps[0], (fps_data[7]));
	atomic_set(&cached_fps[1], ((fps_data[1] + 5)/10));
	atomic64_set(&fps_align_ns, fps_align);

	if (!monitor.buf)
		return 0;

	if (cur_idx != sidx) {
		cur_idx = sidx;
		monitor.buf->layer[cur_idx][0] = '\0';
		monitor.buf->process[cur_idx][0] = '\0';
		monitor.buf->data[cur_idx][HT_FPS_1] = 0;
		monitor.buf->data[cur_idx][HT_FPS_2] = 0;
		monitor.buf->data[cur_idx][HT_FPS_3] = 0;
		monitor.buf->data[cur_idx][HT_FPS_4] = 0;
		monitor.buf->data[cur_idx][HT_FPS_5] = 0;
		monitor.buf->data[cur_idx][HT_FPS_6] = 0;
		monitor.buf->data[cur_idx][HT_FPS_7] = 0;
		monitor.buf->data[cur_idx][HT_FPS_8] = 0;
		monitor.buf->data[cur_idx][HT_FPS_MISS_LAYER] = 0;
	} else if (monitor.buf->layer[cur_idx]) {
		monitor.buf->data[cur_idx][HT_FPS_MISS_LAYER] += 1;
		return 0;
	}

	monitor.buf->data[cur_idx][HT_FPS_1] = fps_data[0];
	monitor.buf->data[cur_idx][HT_FPS_2] = fps_data[1];
	monitor.buf->data[cur_idx][HT_FPS_3] = fps_data[2];
	monitor.buf->data[cur_idx][HT_FPS_4] = fps_data[3];
	monitor.buf->data[cur_idx][HT_FPS_PID] = fps_data[4];
	monitor.buf->data[cur_idx][HT_FPS_ALIGN] = fps_align;

	return 0;
}

static struct kernel_param_ops ht_fps_data_sync_ops = {
	.set = ht_fps_data_sync_store,
};
module_param_cb(fps_data_sync, &ht_fps_data_sync_ops, NULL, 0220);

static int get_gpu_percentage(void)
{
	struct kgsl_clk_stats *stats = NULL;

	if (gpwr) {
		stats = &gpwr->clk_stats;
		return stats->total_old != 0 ? (stats->busy_old * 100 / stats->total_old) : 0;
	}
	return 0;
}

static int game_info_show(char *buf, const struct kernel_param *kp)
{
	int gpu, cpu, fps;
	static u64 last_enqueue_time;

	if (game_fps_pid == -1) {
		gpu = -1;
		cpu = -1;
		fps = -1;
	} else {
		cpu = ohm_get_cur_cpuload(true);
		gpu = get_gpu_percentage();
		fps = game_fps_data.fps / 10;

		if (fps < 0 || fps > 150) {
			pr_err("fps: %d is out of range", fps);
			fps = 0;
		}
		if (game_fps_data.enqueue_time == last_enqueue_time) {
			pr_err("Did not update the fps data");
			fps = 0;
		}
		last_enqueue_time = game_fps_data.enqueue_time;
	}
	return snprintf(buf, PAGE_SIZE, "%d %d %d\n", gpu, cpu, fps);
}

static struct kernel_param_ops game_info_ops = {
	.get = game_info_show,
};
module_param_cb(game_info, &game_info_ops, NULL, 0664);

static long ht_ctl_ioctl(struct file *file, unsigned int cmd, unsigned long __user arg)
{
	if (_IOC_TYPE(cmd) != HT_IOC_MAGIC) return 0;
	if (_IOC_NR(cmd) > HT_IOC_MAX) return 0;
	pr_debug("%s: cmd: %s %x, arg: %lu\n", __func__, ht_ioctl_str(cmd), cmd, arg);

	switch (cmd) {
	case HT_IOC_COLLECT:
	{
		memset(&parcel, 0, sizeof(parcel));
		parcel.pid = current->tgid;
		parcel.fps_align_ns = atomic64_read(&fps_align_ns);
		parcel.fps = atomic_read(&cached_fps[0]);
		parcel.efps = atomic_read(&cached_fps[1]);

		if(copy_to_user((struct ai_parcel __user*)arg, &parcel, sizeof(parcel)))
			return 0;
		break;
	}
	case HT_IOC_SCHEDSTAT:
	{
		struct task_struct *task;
		u64 exec_ns = 0;
		u64 pid;
		if (copy_from_user(&pid, (u64 *) arg, sizeof(u64)))
			return 0;

		rcu_read_lock();
		task = find_task_by_vpid(pid);
		if (task) {
			get_task_struct(task);
			rcu_read_unlock();
			exec_ns = task_sched_runtime(task);
			put_task_struct(task);
			if (exec_ns == 0)
				pr_info("fpsst: schedstat is 0\n");
		} else {
			rcu_read_unlock();
			pr_info("fpsst: can't find task for schedstat\n");
		}
		if (copy_to_user((u64 *) arg, &exec_ns, sizeof(u64)))
			return 0;
		break;
	}
	case HT_IOC_FPS_STABILIZER_UPDATE:
	{
		DEFINE_WAIT(wait);
		prepare_to_wait(&ht_fps_stabilizer_waitq, &wait, TASK_INTERRUPTIBLE);
		freezable_schedule();
		finish_wait(&ht_fps_stabilizer_waitq, &wait);

		pr_debug("fpsst: %s\n", ht_online_config_buf);

		if (ht_online_config_buf[0] == '\0') {
			// force invalid config
			char empty[] = "{}";
			copy_to_user((struct ht_fps_stabilizer_buf __user *) arg, &empty, strlen(empty));
		} else {
			copy_to_user((struct ht_fps_stabilizer_buf __user *) arg, &ht_online_config_buf, PAGE_SIZE);
		}
		break;
	}
	case HT_IOC_FPS_PARTIAL_SYS_INFO:
	{
		struct ht_partial_sys_info data;
		union power_supply_propval prop = {0, };
		int ret;

		if (ht_disable_fps_stabilizer_bat || rocket.psy==NULL) {
			data.volt = data.curr = 0;
		} else {
			ht_update_battery();
			ret = power_supply_get_property(rocket.psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
			data.volt = ret >= 0? prop.intval: 0;
			ret = power_supply_get_property(rocket.psy, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
			data.curr = ret >= 0? prop.intval: 0;
		}

		// related to cpu cluster configuration
		// clus 0
		data.utils[0] = ht_utils[0].utils[0]? (u64) *(ht_utils[0].utils[0]): 0;
		data.utils[1] = ht_utils[0].utils[1]? (u64) *(ht_utils[0].utils[1]): 0;
		data.utils[2] = ht_utils[0].utils[2]? (u64) *(ht_utils[0].utils[2]): 0;
		data.utils[3] = ht_utils[0].utils[3]? (u64) *(ht_utils[0].utils[3]): 0;
		// clus 1
		data.utils[4] = ht_utils[1].utils[0]? (u64) *(ht_utils[1].utils[0]): 0;
		data.utils[5] = ht_utils[1].utils[1]? (u64) *(ht_utils[1].utils[1]): 0;
		data.utils[6] = ht_utils[1].utils[2]? (u64) *(ht_utils[1].utils[2]): 0;
		data.utils[7] = ht_utils[1].utils[3]? (u64) *(ht_utils[1].utils[3]): 0;

		// pick skip-therm since not support shell-therm.
		data.skin_temp = ht_get_temp_delay(HT_SEN_2);
		if (copy_to_user((struct ht_partial_sys_info __user *) arg, &data, sizeof(struct ht_partial_sys_info)))
			return 0;
		break;
	}
	default:
	{
		// handle unsupported ioctl cmd
		return -1;
	}
	}
	return 0;
}

/* poll for fps data synchronization */
static unsigned int ht_ctl_poll(struct file *fp, poll_table *wait)
{
	poll_wait(fp, &ht_poll_waitq, wait);
	return POLLIN;
}

static const struct file_operations ht_ctl_fops = {
	.owner = THIS_MODULE,
	.poll = ht_ctl_poll,
	.unlocked_ioctl = ht_ctl_ioctl,
	.compat_ioctl = ht_ctl_ioctl,
};

static int fps_sync_init(void)
{
	int rc;
	struct device *class_dev;

	rc = alloc_chrdev_region(&ht_ctl_dev, 0, 1, HT_CTL_NODE);
	if (rc < 0) {
		pr_err("alloc_chrdev_region failed %d\n", rc);
		return 0;
	}

	driver_class = class_create(THIS_MODULE, HT_CTL_NODE);
	if (IS_ERR(driver_class)) {
		rc = -ENOMEM;
		pr_err("class_create failed %d\n", rc);
		goto exit_unreg_chrdev_region;
	}
	class_dev = device_create(driver_class, NULL, ht_ctl_dev, NULL, HT_CTL_NODE);
	if (IS_ERR(class_dev)) {
		pr_err("class_device_create failed %d\n", rc);
		rc = -ENOMEM;
		goto exit_destroy_class;
	}
	cdev_init(&cdev, &ht_ctl_fops);
	cdev.owner = THIS_MODULE;
	rc = cdev_add(&cdev, MKDEV(MAJOR(ht_ctl_dev), 0), 1);
	if (rc < 0) {
		pr_err("cdev_add failed %d\n", rc);
		goto exit_destroy_device;
	}
	pr_info("fps data sync ready\n");
	return 0;
exit_destroy_device:
	device_destroy(driver_class, ht_ctl_dev);
exit_destroy_class:
	class_destroy(driver_class);
exit_unreg_chrdev_region:
	unregister_chrdev_region(ht_ctl_dev, 1);
	return 0;
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
	if(rocket.psy){
		ret = power_supply_get_property(rocket.psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &prop);
		smps[idx][HT_BAT_VOLT_NOW] = ret >= 0? prop.intval: 0;
		ret = power_supply_get_property(rocket.psy, POWER_SUPPLY_PROP_CURRENT_NOW, &prop);
		smps[idx][HT_BAT_CURR_NOW] = ret >= 0? prop.intval: 0;
	}
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

	/* Init cached fps info */
	atomic_set(&cached_fps[0], 60);
	atomic_set(&cached_fps[1], 60);
	atomic64_set(&fps_align_ns, 0);

	for (i = 0; i < HT_MONITOR_SIZE; ++i)
		report_div[i] = (i >= HT_CPU_0)? 100: 1;

	ht_set_all_mask();

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
	fps_sync_init();

	return 0;
out:
	return -EINVAL;
}

pure_initcall(ht_init);
MODULE_AUTHOR("tedlin <ted.lin@oneplus.com>");
MODULE_DESCRIPTION("Oneplus Houston");
MODULE_LICENSE("GPL v2");
