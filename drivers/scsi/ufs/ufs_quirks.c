/*
 * Copyright (c) 2013-2019, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include "ufshcd.h"
#include "ufs_quirks.h"

/* liochen@BSP, 2016/11/30, Add ufs info into *##*37847# */
#include <linux/project_info.h>

static struct ufs_card_fix ufs_fixups[] = {
	/* UFS cards deviations table */
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL, UFS_DEVICE_NO_VCCQ),
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL,
		UFS_DEVICE_NO_FASTAUTO),
	UFS_FIX(UFS_VENDOR_SAMSUNG, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_HOST_PA_TACTIVATE),
	UFS_FIX(UFS_VENDOR_TOSHIBA, "THGLF2G9C8KBADG",
		UFS_DEVICE_QUIRK_PA_TACTIVATE),
	UFS_FIX(UFS_VENDOR_TOSHIBA, "THGLF2G9D8KBADG",
		UFS_DEVICE_QUIRK_PA_TACTIVATE),
	UFS_FIX(UFS_VENDOR_SKHYNIX, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_HOST_PA_SAVECONFIGTIME),
	UFS_FIX(UFS_VENDOR_SKHYNIX, UFS_ANY_MODEL, UFS_DEVICE_NO_VCCQ),
	UFS_FIX(UFS_VENDOR_SKHYNIX, UFS_ANY_MODEL,
		UFS_DEVICE_QUIRK_WAIT_AFTER_REF_CLK_UNGATE),
	UFS_FIX(UFS_VENDOR_SKHYNIX, "hB8aL1",
		UFS_DEVICE_QUIRK_HS_G1_TO_HS_G3_SWITCH),
	UFS_FIX(UFS_VENDOR_SKHYNIX, "hC8aL1",
		UFS_DEVICE_QUIRK_HS_G1_TO_HS_G3_SWITCH),
	UFS_FIX(UFS_VENDOR_SKHYNIX, "hD8aL1",
		UFS_DEVICE_QUIRK_HS_G1_TO_HS_G3_SWITCH),
	UFS_FIX(UFS_VENDOR_SKHYNIX, "hC8aM1",
		UFS_DEVICE_QUIRK_HS_G1_TO_HS_G3_SWITCH),
	UFS_FIX(UFS_VENDOR_SKHYNIX, "h08aM1",
		UFS_DEVICE_QUIRK_HS_G1_TO_HS_G3_SWITCH),
	UFS_FIX(UFS_VENDOR_SKHYNIX, "hC8GL1",
		UFS_DEVICE_QUIRK_HS_G1_TO_HS_G3_SWITCH),
	UFS_FIX(UFS_VENDOR_SKHYNIX, "hC8HL1",
		UFS_DEVICE_QUIRK_HS_G1_TO_HS_G3_SWITCH),

	END_FIX
};

/* liochen@BSP, 2016/11/30, Add ufs info into *##*37847# */
static int ufs_get_capacity_info(struct ufs_hba *hba,  u64 *pcapacity)
{
       int err;
       u8 geometry_buf[QUERY_DESC_GEOMETRY_DEF_SIZE];

       err = ufshcd_read_geometry_desc(hba, geometry_buf,
                                       QUERY_DESC_GEOMETRY_DEF_SIZE);
       if (err)
               goto out;

       *pcapacity = (u64)geometry_buf[0x04] << 56 |
                                (u64)geometry_buf[0x04 + 1] << 48 |
                                (u64)geometry_buf[0x04 + 2] << 40 |
                                (u64)geometry_buf[0x04 + 3] << 32 |
                                (u64)geometry_buf[0x04 + 4] << 24 |
                                (u64)geometry_buf[0x04 + 5] << 16 |
                                (u64)geometry_buf[0x04 + 6] << 8 |
                                (u64)geometry_buf[0x04 + 7];

       printk("ufs_get_capacity_info size = 0x%llx", *pcapacity);

out:
       return err;
}

static char* ufs_get_capacity_size(u64 capacity)
{
       if (capacity == 0x1D62000){ //16G
               return "16G";
       } else if (capacity == 0x3B9E000){ //32G
               return "32G";
       } else if (capacity == 0x7734000){ //64G
               return "64G";
       } else if (capacity == 0xEE60000){ //128G
               return "128G";
       } else if (capacity == 0xEE64000){ //128G V4
               return "128G";
       } else if (capacity == 0x1DCBC000) {
               return "256G";
       } else {
               return "0G";
       }
}

/* liochen@BSP, 2016/11/30, Add ufs info into *##*37847# */
char ufs_vendor_and_rev[32] = {'\0'};
char ufs_product_id[32] = {'\0'};
int ufs_fill_info(struct ufs_hba *hba)
{
	int err=0;
	u64 ufs_capacity = 0;
	char ufs_vendor[9]={'\0'};
	char ufs_rev[6]={'\0'};

	/* Error Handle: Before filling ufs info, we must confirm sdev_ufs_device structure is not NULL*/
	if(!hba->sdev_ufs_device) {
		dev_err(hba->dev, "%s:hba->sdev_ufs_device is NULL!\n", __func__);
		goto out;
	}

	/* Copy UFS info from host controller structure (ex:vendor name, firmware revision) */
	if(!hba->sdev_ufs_device->vendor) {
		dev_err(hba->dev, "%s: UFS vendor info is NULL\n", __func__);
		strlcpy(ufs_vendor, "UNKNOWN", 7);
	} else {
		strlcpy(ufs_vendor, hba->sdev_ufs_device->vendor,
			sizeof(ufs_vendor)-1);
	}

	if(!hba->sdev_ufs_device->rev) {
		dev_err(hba->dev, "%s: UFS firmware info is NULL\n", __func__);
		strlcpy(ufs_rev, "NONE", 4);
	} else {
		strlcpy(ufs_rev, hba->sdev_ufs_device->rev, sizeof(ufs_rev)-1);
	}

	if(!hba->sdev_ufs_device->model) {
		dev_err(hba->dev, "%s: UFS product id info is NULL\n", __func__);
		strlcpy(ufs_product_id, "UNKNOWN", 7);
	} else {
		strlcpy(ufs_product_id, hba->sdev_ufs_device->model, 16);
	}

	/* Get UFS storage size*/
	err = ufs_get_capacity_info(hba, &ufs_capacity);
	if (err) {
		dev_err(hba->dev, "%s: Failed getting capacity info\n", __func__);
		goto out;
	}

	/* Combine vendor name with firmware revision */
	strcat(ufs_vendor_and_rev, ufs_vendor);
	if (strncmp(ufs_vendor, "MICRON", 6) != 0) {
		strcat(ufs_vendor_and_rev, " ");
		strcat(ufs_vendor_and_rev, ufs_get_capacity_size(ufs_capacity));
	}
	strcat(ufs_vendor_and_rev, " ");
	strcat(ufs_vendor_and_rev, ufs_rev);

	push_component_info(UFS, ufs_product_id, ufs_vendor_and_rev);
out:
	return err;

}

void ufs_advertise_fixup_device(struct ufs_hba *hba)
{
	int err;
	u8 str_desc_buf[QUERY_DESC_MAX_SIZE + 1];
	char *model;
	struct ufs_card_fix *f;

	model = kmalloc(MAX_MODEL_LEN + 1, GFP_KERNEL);
	if (!model)
		goto out;

	memset(str_desc_buf, 0, QUERY_DESC_MAX_SIZE);
	err = ufshcd_read_string_desc(hba, hba->dev_info.i_product_name,
			str_desc_buf, QUERY_DESC_MAX_SIZE, ASCII_STD);
	if (err)
		goto out;

	str_desc_buf[QUERY_DESC_MAX_SIZE] = '\0';
	strlcpy(model, (str_desc_buf + QUERY_DESC_HDR_SIZE),
		min_t(u8, str_desc_buf[QUERY_DESC_LENGTH_OFFSET],
		      MAX_MODEL_LEN));
	/* Null terminate the model string */
	model[MAX_MODEL_LEN] = '\0';

	for (f = ufs_fixups; f->quirk; f++) {
		/* if same wmanufacturerid */
		if (((f->w_manufacturer_id ==
			hba->dev_info.w_manufacturer_id) ||
		     (f->w_manufacturer_id == UFS_ANY_VENDOR)) &&
		    /* and same model */
		    (STR_PRFX_EQUAL(f->model, model) ||
		     !strcmp(f->model, UFS_ANY_MODEL)))
			/* update quirks */
			hba->dev_info.quirks |= f->quirk;
	}
out:
	kfree(model);
}
