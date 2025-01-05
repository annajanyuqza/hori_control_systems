/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __HID_HORI_H
#define __HID_HORI_H

struct hori_drv_data {
	unsigned long quirks;
	void *device_props;	/* Device specific properties */
};

#ifdef CONFIG_HORI_FF
int horiff_init(struct hid_device *hdev);
#else
static inline int horiff_init(struct hid_device *hdev) { return -1; }
#endif

#endif
