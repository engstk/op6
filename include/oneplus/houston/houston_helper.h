#ifndef __INCLUDE__HOUSTON_HELPER__
#define __INCLUDE__HOUSTON_HELPER__

#ifdef CONFIG_HOUSTON

extern void ht_register_kgsl_pwrctrl(void *pwr);

#else

static void ht_register_kgsl_pwrctrl(void *pwr) {};

#endif

#endif
