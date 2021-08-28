// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2019 op, Inc.
 * Author: Siba Prasad
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 */

#include <linux/kernel.h>
#include <linux/kdev_t.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/init.h>
#include <linux/module.h>
#include<linux/uaccess.h>
#include<linux/sysfs.h>
#include<linux/kobject.h>
#include <linux/device.h>
#include<linux/slab.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/wait.h>

#define MAXLEN		255

static int sys_value;
struct mutex sys_mutex1;
spinlock_t sys_spinlock;
unsigned long global_mutex;
unsigned long global_spinlock;

dev_t dev;
static struct class *dev_class;
static struct cdev sys_cdev;
struct kobject *kobj_ref;
uint8_t *kernel_buffer;

static int __init sysdriver_init(void);
static void __exit sysdriver_exit(void);

static struct task_struct *sys_thread1;
static struct task_struct *sys_thread2;
static struct task_struct *sys_thread3;
static struct task_struct *sys_thread4;

/*************** Driver Functions **********************/
static int sysdebug_open(struct inode *inode, struct file *file);
static int sysdebug_release(struct inode *inode, struct file *file);
static ssize_t sysdebug_read(struct file *filp, char __user *buf,
		size_t len, loff_t *off);
static ssize_t sysdebug_write(struct file *filp, const char *buf,
		size_t len, loff_t *off);

/*************** Sysfs Functions **********************/
static ssize_t sysfs_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf);
static ssize_t sysfs_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count);

static int sys_null(void);
static int sys_stackoverflow(void);
static int sys_bufoverflow(void);
static int sys_free(void);
static int sys_doublefree(void);
static int sys_mutexlock(void);
static int sys_spin(void);

static int thread_function1(void *data);
static int thread_function2(void *data);
static int thread_function3(void *data);
static int thread_function4(void *data);

static struct kobj_attribute sysvalue_attr = __ATTR(sys_value, 0660,
		sysfs_show, sysfs_store);

static int thread_function1(void *data)
{
	while (!kthread_should_stop()) {
		mutex_lock(&sys_mutex1);
		global_mutex++;
		pr_info("In Thread Function1 %lu\n", global_mutex);
		mutex_unlock(&sys_mutex1);
		msleep(1000);
	}
	do_exit(0);
}

static int thread_function2(void *data)
{
	while (!kthread_should_stop()) {
		mutex_lock(&sys_mutex1);
		global_mutex++;
		pr_info("In Thread Function2 %lu\n", global_mutex);
		mutex_lock(&sys_mutex1);
		mutex_unlock(&sys_mutex1);
		msleep(1000);
	}
	do_exit(0);
}

static int thread_function3(void *data)
{
	while (!kthread_should_stop()) {
		spin_lock(&sys_spinlock);
		global_spinlock++;
		pr_info("In Thread Function3 %lu\n", global_spinlock);
		spin_lock(&sys_spinlock);
		spin_unlock(&sys_spinlock);
	}
	do_exit(0);
}

static int thread_function4(void *data)
{
	while (!kthread_should_stop()) {
		spin_lock(&sys_spinlock);
		global_spinlock++;
		pr_info("In Thread Function4 %lu\n", global_spinlock);
		spin_unlock(&sys_spinlock);
	}
	do_exit(0);
}

static const struct file_operations fops = {
	.owner          = THIS_MODULE,
	.read           = sysdebug_read,
	.write          = sysdebug_write,
	.open           = sysdebug_open,
	.release        = sysdebug_release,
};

/* Creata a group of attributes helps to create and destroy them all at once */
static struct attribute *attrs[] = {
	&sysvalue_attr.attr,
	NULL,	/* need to NULL terminate the list of attributes */
};

/*
 * By specifying a name, a subdirectory will be created for the
 * attributes with the directory being the name of the attribute group.
 */
static struct attribute_group attr_group = {
	.attrs = attrs,
};

static ssize_t sysfs_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	pr_info("Sysfs- Read!!!\n");
	return snprintf(buf, MAXLEN, "%d\nChoose BUG:\n"
			"1 = Null pointer dereference\n2 = Stack overflow\n"
			"3 = Use-after-free\n4 = Double free\n"
			"5 = Buffer overflow\n6 = Mutex lock\n"
			"7 = Spinlock\n", sys_value);
}

static ssize_t sysfs_store(struct kobject *kobj,
		struct kobj_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;

	pr_info("Sysfs- Write!!!\n");
	ret = sscanf(buf, "%d\n", &sys_value);

	if (ret != 1)
		return -EINVAL;

	if (sys_value < 0) {
		pr_err("Value is not valid...\n");
		return -EINVAL;
	} else if (sys_value == 1) {
		pr_info("Calling Null pointer error...\n");
		sys_null();
	} else if (sys_value == 2) {
		pr_info("Calling Stack Overflow error...\n");
		sys_stackoverflow();
	} else if (sys_value == 3) {
		pr_info("Calling Use-after-free error...\n");
		sys_free();
	} else if (sys_value == 4) {
		pr_info("Calling Double free error...\n");
		sys_doublefree();
	} else if (sys_value == 5) {
		pr_info("Calling Buffer Overflow error...\n");
		sys_bufoverflow();
	} else if (sys_value == 6) {
		pr_info("Calling Mutex lock error...\n");
		sys_mutexlock();
	} else if (sys_value == 7) {
		pr_info("Calling Spinlock error...\n");
		sys_spin();
	} else {
		return count;
	}
	return count;
}

static int sys_null(void)
{
	struct inode {
		int i_ino;
	};

	struct inode *p = NULL;

	pr_info("Inside NULL function\n");
	p->i_ino = 1;
	return 0;
}

static void fun(int x)
{
	pr_info("Inside Fun_Overflow function\n");

	if (x == 1)
		return;
	x = 6;
	fun(x);
}

static int sys_stackoverflow(void)
{
	int x = 5;

	pr_info("Inside Stack Overflow function\n");
	fun(x);
	return 0;
}

static int sys_bufoverflow(void)
{
	char *ptr = kmalloc(100, GFP_KERNEL);

	pr_info("Inside buffer overflow function\n");
	pr_info("slub_debug catches buffer overflow\n");
	memset(ptr, 'x', 300);
	kfree(ptr);
	return 0;
}

static int sys_free(void)
{
	char *ptr = kmalloc(100, GFP_KERNEL);

	pr_info("Inside use-after-free function\n");
	kfree(ptr);
	pr_info("slub_debug catches use after free\n");
	memset(ptr, 'c', 100);
	return 0;
}

static int sys_doublefree(void)
{
	char *ptr = kmalloc(100, GFP_KERNEL);

	pr_info("Inside double free function\n");
	kfree(ptr);
	pr_info("slub_debug catches double free\n");
	kfree(ptr);
	return 0;
}

static int sys_mutexlock(void)
{
	/* Creating Thread 1 */
	sys_thread1 = kthread_run(thread_function1, NULL, "sys Thread1");

	if (sys_thread1) {
		pr_info("Kthread1 Created Successfully...\n");
	} else {
		pr_err("Cannot create kthread1\n");
		return 0;
	}

	/* Creating Thread 2 */
	sys_thread2 = kthread_run(thread_function2, NULL, "sys Thread2");

	if (sys_thread2) {
		pr_info("Kthread2 Created Successfully...\n");
	} else {
		pr_err("Cannot create kthread2\n");
		return 0;
	}
	return 0;
}

static int sys_spin(void)
{
	/* Creating Thread 3 */
	sys_thread3 = kthread_run(thread_function3, NULL, "sys Thread3");

	if (sys_thread3) {
		pr_info("Kthread3 Created Successfully...\n");
	} else {
		pr_err("Cannot create kthread3\n");
		return 0;
	}

	/* Creating Thread 4 */
	sys_thread4 = kthread_run(thread_function4, NULL, "sys Thread4");

	if (sys_thread4) {
		pr_info("Kthread4 Created Successfully...\n");
	} else {
		pr_err("Cannot create kthread4\n");
		return 0;
	}
	return 0;
}

static int sysdebug_open(struct inode *inode, struct file *file)
{
	pr_info("Device File Opened...!!!\n");
	return 0;
}

static int sysdebug_release(struct inode *inode, struct file *file)
{
	pr_info("Device File Closed...!!!\n");
	return 0;
}

static ssize_t sysdebug_read(struct file *filp, char __user *buf,
		size_t len, loff_t *off)
{
	pr_info("Read function\n");
	return 0;
}
static ssize_t sysdebug_write(struct file *filp, const char __user *buf,
		size_t len, loff_t *off)
{
	pr_info("Write Function\n");
	return len;
}

static int __init sysdriver_init(void)
{
	/*Allocating Major number*/
	int err = alloc_chrdev_region(&dev, 0, 1, "debugmaj_Dev");

	if (err < 0) {
		pr_err("Cannot allocate major number\n");
		return err;
	}
	pr_info("Major = %d Minor = %d\n", MAJOR(dev), MINOR(dev));

	/*Creating cdev structure*/
	cdev_init(&sys_cdev, &fops);

	/*Adding character device to the system*/
	if ((cdev_add(&sys_cdev, dev, 1)) < 0) {
		pr_err("Cannot add the device to the system\n");
		goto remove_class;
	}

	/*Creating struct class*/
	dev_class = class_create(THIS_MODULE, "debug_class");
	if (dev_class == NULL) {
		pr_err("Cannot create the struct class\n");
		goto remove_class;
	}

	/*Creating device*/
	if ((device_create(dev_class, NULL, dev, NULL, "debug_device"))
			== NULL) {
		pr_err("Cannot create the Device 1\n");
		goto remove_device;
	}

	/*Creating a directory in /sys/kernel/ */
	kobj_ref = kobject_create_and_add("debug_sysfs", kernel_kobj);
	if (!kobj_ref)
		return -ENOMEM;

	/*Creating sysfs file for my_value*/
	if (sysfs_create_group(kobj_ref, &attr_group)) {
		pr_err("Cannot create sysfs file......\n");
		goto remove_sysfs;
	}

	spin_lock_init(&sys_spinlock);
	mutex_init(&sys_mutex1);
	pr_info("Debug Driver Insert...Done!!!\n");
	return 0;

remove_sysfs:
	kobject_put(kobj_ref);
	sysfs_remove_group(kernel_kobj, &attr_group);
remove_device:
	class_destroy(dev_class);
remove_class:
	unregister_chrdev_region(dev, 1);
	cdev_del(&sys_cdev);
	return -EINVAL;
}

void __exit sysdriver_exit(void)
{
	kthread_stop(sys_thread1);
	kthread_stop(sys_thread2);
	kthread_stop(sys_thread3);
	kthread_stop(sys_thread4);
	kobject_put(kobj_ref);
	sysfs_remove_group(kernel_kobj, &attr_group);
	device_destroy(dev_class, dev);
	class_destroy(dev_class);
	cdev_del(&sys_cdev);
	unregister_chrdev_region(dev, 1);
	pr_info("Debug Driver Remove...Done!!!\n");
}

module_init(sysdriver_init);
module_exit(sysdriver_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Siba Prasad");
MODULE_DESCRIPTION("Debug driver using sysfs entry");
