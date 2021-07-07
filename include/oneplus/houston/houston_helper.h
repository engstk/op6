#ifndef __INCLUDE__HOUSTON_HELPER__
#define __INCLUDE__HOUSTON_HELPER__

#ifdef CONFIG_HOUSTON
extern void ht_register_kgsl_pwrctrl(void *pwr);
extern void ht_register_cpu_util(unsigned int cpu, unsigned int first_cpu, unsigned long *util, unsigned long *hi_util);
#else
static void ht_register_kgsl_pwrctrl(void *pwr) {};
static void ht_register_cpu_util(unsigned int cpu, unsigned int first_cpu, unsigned long *util, unsigned long *hi_util) {};
#endif

#endif
