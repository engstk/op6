#ifndef __INCLUDE_HOUSTON__
#define __INCLUDE_HOUSTON__

#ifndef pr_fmt
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
#endif

#define MAX_REPORT_PERIOD 18000

#define FPS_COLS 5
#define FPS_LAYER_LEN 128
#define FPS_PROCESS_NAME_LEN 64
#define FPS_DATA_BUF_SIZE 256

extern int ohm_get_cur_cpuload(bool ctrl);

#endif
