#ifndef __INCLUDE_HOUSTON__
#define __INCLUDE_HOUSTON__

#ifndef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#endif

#define HT_CLUSTERS 2
#define HT_CPUS_PER_CLUS 4
#define CLUS_0_IDX 0
#define CLUS_1_IDX 4
#define HT_CTL_NODE "ht_ctl"

#define MAX_REPORT_PERIOD 18000

#define FPS_COLS 5
#define FPS_LAYER_LEN 128
#define FPS_PROCESS_NAME_LEN 64
#define FPS_DATA_BUF_SIZE 256

/* fps stabilizer related */
struct ht_fps_stabilizer_buf {
	char buf[PAGE_SIZE];
};

struct ht_partial_sys_info {
	u32 volt;
	u32 curr;
	u64 utils[8];
	u32 skin_temp;
};

/* pick one unique magic number */
#define HT_IOC_MAGIC 'k'
#define HT_IOC_COLLECT _IOR(HT_IOC_MAGIC, 0, struct ai_parcel)
#define HT_IOC_SCHEDSTAT _IOWR(HT_IOC_MAGIC, 1, u64)
#define HT_IOC_FPS_STABILIZER_UPDATE _IOR(HT_IOC_MAGIC, 3, struct ht_fps_stabilizer_buf)
#define HT_IOC_FPS_PARTIAL_SYS_INFO _IOR(HT_IOC_MAGIC, 4, struct ht_partial_sys_info)
#define HT_IOC_MAX 4

#define AI_THREAD_PARCEL_MAX (10)

struct ai_thread_parcel {
	u32 tid;
	u64 exec_time_ns; // schedstat
	u64 inst; // pmu related
	u64 cycle;
	u64 cache_miss_l1;
	u64 cache_miss_l2;
	u64 cache_miss_l3;
};

struct ai_parcel {
	u32 pid;
	u32 fps;
	u32 efps;
	long long fps_align_ns;
	u32 cpu;
	u32 clus;
	u32 cpu_cur_freq_0;
	u32 cpu_cur_freq_1;
	u32 cpu_cur_freq_2;
	u32 cpu_orig_freq_0;
	u32 cpu_orig_freq_1;
	u32 cpu_orig_freq_2;
	u32 cpu_cc_min_freq_1;
	u32 cpu_cc_max_freq_1;
	u32 cpu_orig_min_freq_1;
	u32 cpu_orig_max_freq_1;
	u32 gpu_freq;
	u64 ddr_freq;
	u32 ddr_voting;
	u32 volt_now; // battery part
	u32 curr_now;
	u64 queued_ts_us;
	u64 prev_queued_ts_us;
	u32 thread_amount; // default maximum 10 threads u32 boost_cnt;
	u64 notify_start_ts_us;
	u64 notify_end_ts_us;
	u64 utils[8];
	u32 skin_temp;
#ifdef CONFIG_CONTROL_CENTER
	struct cc_boost_ts cbt[CC_BOOST_TS_SIZE];
#endif
	struct ai_thread_parcel t[AI_THREAD_PARCEL_MAX];
};

extern int ohm_get_cur_cpuload(bool ctrl);

#endif
