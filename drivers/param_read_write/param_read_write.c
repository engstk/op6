/*
 * drivers/param_read_write/param_read_write.c 
 * Fake param_read_write Avengers theme
 * Made by pappschlumpf (Erik Müller)
 */

#include <linux/input.h>
#include <linux/moduleparam.h>

static unsigned int cust_flag = 0;

module_param(cust_flag, uint, 0644);
