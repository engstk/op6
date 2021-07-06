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
#define HT_IOC_SCHEDSTAT _IOWR(HT_IOC_MAGIC, 1, u64)
#define HT_IOC_FPS_STABILIZER_UPDATE _IOR(HT_IOC_MAGIC, 3, struct ht_fps_stabilizer_buf)
#define HT_IOC_FPS_PARTIAL_SYS_INFO _IOR(HT_IOC_MAGIC, 4, struct ht_partial_sys_info)
#define HT_IOC_MAX 4

extern int ohm_get_cur_cpuload(bool ctrl);

#endif
