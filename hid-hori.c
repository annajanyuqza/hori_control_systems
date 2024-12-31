// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for some logitech "special" devices
 *
 *  Copyright (c) 1999 Andreas Gal
 *  Copyright (c) 2000-2005 Vojtech Pavlik <vojtech@suse.cz>
 *  Copyright (c) 2005 Michael Haboustak <mike-@cinci.rr.com> for Concept2, Inc
 *  Copyright (c) 2006-2007 Jiri Kosina
 *  Copyright (c) 2008 Jiri Slaby
 *  Copyright (c) 2010 Hendrik Iben
 */

#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/wait.h>

#include "usbhid/usbhid.h"
#include "hid-ids.h"

#define 	HID_SIMUL_AUTOMOBILE   0x02
#define 	HID_SIMUL_ACCELERATOR   0xC4
#define 	HID_SIMUL_BRAKE   0xC5
#define 	HID_SIMUL_CLUTCH   0xC6
#define 	HID_SIMUL_SHIFTER   0xC7
#define 	HID_SIMUL_STEERING   0xC8

#define HR_RDESC		0x001
#define HR_RDESC_REL_ABS	0x002

#define TSC_WHEEL_RDESC_ORIG_SIZE	130
#define TSC_SHIFTER_RDESC_ORIG_SIZE	97

#define BTN_BASE7		0x12c
#define BTN_BASE8		0x12d
#define BTN_BASE9		0x12e

#define ABS_CLUTCH		0x22

/* Size of the original descriptors of the Driving Force (and Pro) wheels */
#define TCS_WHEEL_RDESC_ORIG_SIZE	87

#define HORI_TCS_POLL		0x00

struct hori_priv {
	u8 slider;
	bool alternate;
};

/* Fixed report descriptors for HORI FFB Truck Control Systems
 * wheel controllers
 *
 * The original descriptors hide the separate throttle and brake axes in
 * a custom vendor usage page, providing only a combined value as
 * GenericDesktop.Y.
 * These descriptors remove the combined Y axis and instead report
 * separate throttle (Y) and brake (RZ).
 */
static const __u8 tcs_wheel_rdesc_fixed[] = {
0x05, 0x01,                    		// Usage Page (Generic Desktop)        			0
0x09, 0x04,                    		// Usage (Joystick)                    					2
0xa1, 0x01,                    		// Collection (Application)            				4
0x85, 0x01,                    		//  Report ID (1)                      					6
0x15, 0x00,                    		//  Logical Minimum (0)                				8
0x25, 0x07,                    		//  Logical Maximum (7)                				10
0x35, 0x00,                    		//  Physical Minimum (0)               				12
0x46, 0x3b, 0x07, 0x08,             //  Physical Maximum (1800)             			14
0x65, 0x14,                    		//  Unit (EnglishRotation: deg)        				17
0x09, 0x39,                    		//  Usage (Hat switch)                 					19
0x75, 0x04,                    		//  Report Size (4)                    					21
0x95, 0x01,                    		//  Report Count (1)                   					23
0x81, 0x42,                    		//  Input (Data,Var,Abs,Null)          				25
0x65, 0x00,                    		//  Unit (None)                        					27
0x25, 0x01,                    		//  Logical Maximum (1)                				29
0x45, 0x01,                    		//  Physical Maximum (1)               				31
0x05, 0x09,                    		//  Usage Page (Button)                				33
0x19, 0x01,                    		//  Usage Minimum (1)                  				35
0x29, 0x36,                    		//  Usage Maximum (54)                 				37
0x75, 0x01,                    		//  Report Size (1)                    					39
0x95, 0x36,                    		//  Report Count (54)                  					41
0x81, 0x02,                    		//  Input (Data,Var,Abs)               					43
0x95, 0x06,                    		//  Report Count (6)                   					45
0x81, 0x01,                    		//  Input (Cnst,Arr,Abs)               					47
0x27, 0xff, 0xff, 0x00, 0x00,  	//  Logical Maximum (65535)            				49
0x47, 0xff, 0xff, 0x00, 0x00,  	//  Physical Maximum (65535)           			54
0x75, 0x10,                    		//  Report Size (16)                   					59
0x95, 0x08,                    		//  Report Count (8)                   					61
0x05, 0x01,                    		//  Usage Page (Generic Desktop)       			63
0x09, 0x30,                    		//  Usage (X)                          					65
0x09, 0x31,                    		//  Usage (Y)                          					67
0x09, 0x32,                    		//  Usage (Z)                          					69
0x09, 0x33,                    		//  Usage (Rx)                         					71
0x09, 0x34,                    		//  Usage (Ry)                         					73
0x09, 0x35,                    		//  Usage (Rz)                         					75
0x09, 0x36,                    		//  Usage (Slider)                    					77
0x09, 0x36,                    		//  Usage (Slider)                     					79
0x81, 0x02,                    		//  Input (Data,Var,Abs)             					81
0x26, 0xff, 0x00,              		//  Logical Maximum (255)              				83
0x46, 0xff, 0x00,              		//  Physical Maximum (255)             				86
0x06, 0x00, 0xff,              		//  Usage Page (Vendor Defined Page 1) 		89
0x09, 0x01,                    		//  Usage (Vendor Usage 1)             				92
0x75, 0x08,                   		//  Report Size (8)                    					94
0x95, 0x04,                    		//  Report Count (4)                   					96
0x81, 0x02,                    		//  Input (Data,Var,Abs)               					98
0xc0,                          			// End Collection                      					100
0x06, 0x20, 0xff,              		// Usage Page (Vendor Usage Page 0xff20) 		101
0x09, 0x01,                    		// Usage (Vendor Usage 0x01)           			104
0xa1, 0x01,                    		// Collection (Application)            				106
0x85, 0x20,                    		//  Report ID (32)                     					108
0x15, 0x00,                    		//  Logical Minimum (0)                				110
0x26, 0xff, 0x00,             		//  Logical Maximum (255)              				112
0x75, 0x08,                    		//  Report Size (8)                    					115
0x95, 0x1f,                    			//  Report Count (31)                  					117
0x09, 0x02,                    		//  Usage (Vendor Usage 0x02)          			119
0x81, 0x00,                    		//  Input (Data,Arr,Abs)               					121
0x09, 0x02,                    		//  Usage (Vendor Usage 0x02)          			123
0x91, 0x00,                    		//  Output (Data,Arr,Abs)              				125
0xc0,                          			// End Collection                      					127
0x06, 0x21, 0xff,              		// Usage Page (Vendor Usage Page 0xff21) 		128
0x09, 0x03,                    		// Usage (Vendor Usage 0x03)           			131
0xa1, 0x01,                    		// Collection (Application)            				133
0x85, 0x21,                    		//  Report ID (33)                     					135
0x15, 0x00,                    		//  Logical Minimum (0)                				137
0x26, 0xff, 0x00,              		//  Logical Maximum (255)              				139
0x75, 0x08,                    		//  Report Size (8)                    					142
0x95, 0x3f,                        		//  Report Count (63)                  					144
0x09, 0x04,                    		//  Usage (Vendor Usage 0x04)          			146
0x81, 0x00,                    		//  Input (Data,Arr,Abs)               					148
0x09, 0x04,                    		//  Usage (Vendor Usage 0x04)          			150
0x91, 0x00,                    		//  Output (Data,Arr,Abs)              				152
0xc0,                          			// End Collection                      					154

};

static const __u8 *hori_report_fixup(struct hid_device *hdev, __u8 *rdesc,
				     unsigned int *rsize)
{
			hid_info(hdev,
				"fixing up HORI Truck Control System Wheel report descriptor\n");
			*rsize = sizeof(tcs_wheel_rdesc_fixed);
			return tcs_wheel_rdesc_fixed;
}

static int hori_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{

	if (usage->type == EV_ABS && ( usage->code == ABS_THROTTLE || usage->code == ABS_MISC )) {
		switch (hdev->product) {
		case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_WHEEL:
			field->application = HID_GD_MULTIAXIS;
			break;
		default:
			break;
		}
	}

	return 0;
}

static int hori_raw_event(struct hid_device *hdev, struct hid_report *report,
	 u8 *data, int size)
{
	struct hori_priv *priv = hid_get_drvdata(hdev);

	if (priv->alternate)
		priv->slider = data[7];

	data[1] = priv->slider;

	priv->alternate = !priv->alternate;
	return 0;
}

static int hori_probe(struct hid_device *hdev, const struct hid_device_id *id)
{
	int ret;
	struct hori_priv *priv;

	priv = devm_kzalloc(&hdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	hid_set_drvdata(hdev, priv);

	ret = hid_parse(hdev);
	if (ret) {
		hid_err(hdev, "parse failed\n");
		return ret;
	}

	ret = hid_hw_start(hdev, HID_CONNECT_DEFAULT);
	if (ret) {
		hid_err(hdev, "hw start failed\n");
		return ret;
	}

	return 0;
}

static const struct hid_device_id hori_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_HORI, USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_WHEEL) },
	{ }
};
MODULE_DEVICE_TABLE(hid, hori_devices);


static struct hid_driver hori_driver = {
	.name = "hori",
	.id_table = hori_devices,
	.report_fixup = hori_report_fixup,
        .input_mapped =hori_input_mapped,
	.probe = hori_probe,
	.raw_event = hori_raw_event,
};

module_hid_driver(hori_driver);

MODULE_AUTHOR("LinuxGamesTV <linuxgamestv@gmail.com>");
MODULE_DESCRIPTION("HID driver for HORI Wheel Systems");
MODULE_LICENSE("GPL");
