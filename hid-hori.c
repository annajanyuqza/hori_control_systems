// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for some Hori "special" devices
 *
 *  Copyright (c) 2024/2025 Peter Stein <linuxgamestv@gmail.com>
 */

#include <linux/bits.h>
#include <linux/crc32.h>
#include <linux/device.h>
#include <linux/hid.h>
#include <linux/module.h>
#include <linux/random.h>
#include <linux/sched.h>
#include <linux/usb.h>
#include <linux/wait.h>

#include <linux/unaligned.h>

#include "usbhid/usbhid.h"
#include "hid-ids.h"

#define HORI_TCS_WHEEL_RDESC_ORIG_SIZE		155
#define HORI_TCS_SHIFTER_RDESC_ORIG_SIZE	83

#define ABS_CLUTCH      ABS_RZ
#define JOY_RANGE	(BTN_DEAD - BTN_JOYSTICK + 1)

/* Fixed report descriptors for HORI FFB Truck Control Systems
 * wheel controllers
 *
 * The original descriptors hide the separate throttle and brake axes in
 * a custom vendor usage page, providing only a combined value as
 * GenericDesktop.THROTTLE.
 * These descriptors remove the combined THROTTLE axis and instead report
 * separate throttle (GAS) and brake (BRAKE).
 */
static const __u8 tcs_wheel_rdesc_fixed[] = {
	0x05, 0x01,                    // Usage Page (Generic Desktop)           0
	0x09, 0x04,                    // Usage (Joystick)                       2
	0xa1, 0x01,                    // Collection (Application)               4
	0x85, 0x01,                    //  Report ID (1)                         6
	0x15, 0x00,                    //  Logical Minimum (0)                   8
	0x25, 0x07,                    //  Logical Maximum (7)                   10
	0x35, 0x00,                    //  Physical Minimum (0)                  12
	0x46, 0x3b, 0x07, 0x08,        //  Physical Maximum (315)                14
	0x65, 0x14,                    //  Unit (EnglishRotation: deg)           17
	0x09, 0x39,                    //  Usage (Hat switch)                    19
	0x75, 0x04,                    //  Report Size (4)                       21
	0x95, 0x01,                    //  Report Count (1)                      23
	0x81, 0x42,                    //  Input (Data,Var,Abs,Null)             25
	0x65, 0x00,                    //  Unit (None)                           27
	0x25, 0x01,                    //  Logical Maximum (1)                   29
	0x45, 0x01,                    //  Physical Maximum (1)                  31
	0x05, 0x09,                    //  Usage Page (Button)                   33
	0x19, 0x01,                    //  Usage Minimum (1)                     35
	0x29, 0x36,                    //  Usage Maximum (54)                    37
	0x75, 0x01,                    //  Report Size (1)                       39
	0x95, 0x36,                    //  Report Count (54)                     41
	0x81, 0x02,                    //  Input (Data,Var,Abs)                  43
	0x95, 0x06,                    //  Report Count (6)                      45
	0x81, 0x01,                    //  Input (Cnst,Arr,Abs)                  47
	0x27, 0xff, 0xff, 0x00, 0x00,  //  Logical Maximum (65535)               49
	0x47, 0xff, 0xff, 0x00, 0x00,  //  Physical Maximum (65535)              54
	0x75, 0x10,                    //  Report Size (16)                      59
	0x95, 0x08,                    //  Report Count (8)                      61
	0x05, 0x01,                    //  Usage Page (Generic Desktop)          63
	0x09, 0x30,                    //  Usage (X)                             65
	0x09, 0x31,                    //  Usage (Y)                             67
	0x09, 0x32,                    //  Usage (Z)                             69
	0x09, 0x33,                    //  Usage (Rx)                            71
	0x09, 0x34,                    //  Usage (Ry)                            73
	0x09, 0x35,                    //  Usage (Rz)                            75
	0x09, 0x36,                    //  Usage (Slider)                        77
	0x09, 0x36,                    //  Usage (Slider)                        79
	0x81, 0x02,                    //  Input (Data,Var,Abs)                  81
	0x26, 0xff, 0x00,              //  Logical Maximum (255)                 83
	0x46, 0xff, 0x00,              //  Physical Maximum (255)                86
	0x06, 0x00, 0xff,              //  Usage Page (Vendor Defined Page 1)    89
	0x09, 0x01,                    //  Usage (Vendor Usage 1)                92
	0x75, 0x08,                    //  Report Size (8)                       94
	0x95, 0x04,                    //  Report Count (4)                      96
	0x81, 0x02,                    //  Input (Data,Var,Abs)                  98
	0xc0,                          // End Collection                         100
	0x06, 0x20, 0xff,              // Usage Page (Vendor Usage Page 0xff20)  101
	0x09, 0x01,                    // Usage (Vendor Usage 0x01)              104
	0xa1, 0x01,                    // Collection (Application)               106
	0x85, 0x20,                    //  Report ID (32)                        108
	0x15, 0x00,                    //  Logical Minimum (0)                   110
	0x26, 0xff, 0x00,              //  Logical Maximum (255)                 112
	0x75, 0x08,                    //  Report Size (8)                       115
	0x95, 0x1f,                    //  Report Count (31)                     117
	0x09, 0x02,                    //  Usage (Vendor Usage 0x02)             119
	0x81, 0x00,                    //  Input (Data,Arr,Abs)                  121
	0x09, 0x02,                    //  Usage (Vendor Usage 0x02)             123
	0x91, 0x00,                    //  Output (Data,Arr,Abs)                 125
	0xc0,                          // End Collection                         127
	0x06, 0x21, 0xff,              // Usage Page (Vendor Usage Page 0xff21)  128
	0x09, 0x03,                    // Usage (Vendor Usage 0x03)              131
	0xa1, 0x01,                    // Collection (Application)               133
	0x85, 0x21,                    //  Report ID (33)                        135
	0x15, 0x00,                    //  Logical Minimum (0)                   137
	0x26, 0xff, 0x00,              //  Logical Maximum (255)                 139
	0x75, 0x08,                    //  Report Size (8)                       142
	0x95, 0x3f,                    //  Report Count (63)                     144
	0x09, 0x04,                    //  Usage (Vendor Usage 0x04)             146
	0x81, 0x00,                    //  Input (Data,Arr,Abs)                  148
	0x09, 0x04,                    //  Usage (Vendor Usage 0x04)             150
	0x91, 0x00,                    //  Output (Data,Arr,Abs)                 152
	0xc0,                          // End Collection                         154
};

static const __u8 tcs_shifter_rdesc_fixed[] = {
	0x05, 0x01,        // Usage Page (Generic Desktop)           0
	0x09, 0x04,        // Usage (Joystick)                       2
	0xa1, 0x01,        // Collection (Application)               4
	0x85, 0x01,        //  Report ID (1)                         6
	0x15, 0x00,        //  Logical Minimum (0)                   8
	0x25, 0x01,        //  Logical Maximum (1)                   10
	0x35, 0x00,        //  Physical Minimum (0)                  12
	0x45, 0x01,        //  Physical Maximum (1)                  14
	0x05, 0x09,        //  Usage Page (Button)                   16
	0x19, 0x01,        //  Usage Minimum (1)                     18
	0x29, 0x2b,        //  Usage Maximum (43)                    20
	0x75, 0x01,        //  Report Size (1)                       22
	0x95, 0x2b,        //  Report Count (43)                     24
	0x81, 0x02,        //  Input (Data,Var,Abs)                  26
	0x95, 0x05,        //  Report Count (5)                      28
	0x81, 0x01,        //  Input (Cnst,Arr,Abs)                  30
	0x26, 0xff, 0x00,  //  Logical Maximum (255)                 32
	0x46, 0xff, 0x00,  //  Physical Maximum (255)                35
	0x06, 0x00, 0xff,  //  Usage Page (Vendor Defined Page 1)    38
	0x09, 0x01,        //  Usage (Vendor Usage 1)                41
	0x75, 0x08,        //  Report Size (8)                       43
	0x95, 0x04,        //  Report Count (4)                      45
	0x81, 0x02,        //  Input (Data,Var,Abs)                  47
	0x09, 0x02,        //  Usage (Vendor Usage 2)                49
	0x95, 0x1f,        //  Report Count (31)                     51
	0x91, 0x02,        //  Output (Data,Var,Abs)                 53
	0xc0,              // End Collection                         55
	0x06, 0x21, 0xff,  // Usage Page (Vendor Usage Page 0xff21)  56
	0x09, 0x03,        // Usage (Vendor Usage 0x03)              59
	0xa1, 0x01,        // Collection (Application)               61
	0x85, 0x22,        //  Report ID (34)                        63
	0x15, 0x00,        //  Logical Minimum (0)                   65
	0x26, 0xff, 0x00,  //  Logical Maximum (255)                 67
	0x75, 0x08,        //  Report Size (8)                       70
	0x95, 0x3f,        //  Report Count (63)                     72
	0x09, 0x04,        //  Usage (Vendor Usage 0x04)             74
	0x81, 0x00,        //  Input (Data,Arr,Abs)                  76
	0x09, 0x04,        //  Usage (Vendor Usage 0x04)             78
	0x91, 0x00,        //  Output (Data,Arr,Abs)                 80
	0xc0,              // End Collection                         82
};

static const s32 hat_to_axis[9][2] = {
	{0, 0},
	{0, -1},
	{1, -1},
	{1, 0},
	{1, 1},
	{0, 1},
	{-1, 1},
	{-1, 0},
	{-1, -1},
};

static const __u8 *hori_report_fixup(struct hid_device *hdev, __u8 *rdesc,
				     unsigned int *rsize)
{
	switch (hdev->product) {
	case USB_DEVICE_ID_HORI_TRUCK_WHEEL:
		if (*rsize != HORI_TCS_WHEEL_RDESC_ORIG_SIZE)
			break;

		hid_info(hdev, "fixing up HORI TCS Wheel report descriptor\n");
		*rsize = sizeof(tcs_wheel_rdesc_fixed);
		return tcs_wheel_rdesc_fixed;

	case USB_DEVICE_ID_HORI_TRUCK_SHIFTER:
		if (*rsize != HORI_TCS_SHIFTER_RDESC_ORIG_SIZE)
			break;

		hid_info(hdev, "fixing up HORI TCS Shifter report descriptor\n");
		*rsize = sizeof(tcs_shifter_rdesc_fixed);
		return tcs_shifter_rdesc_fixed;
	}

	return rdesc;
}

static void parse_tcs_axis_report(struct input_dev *input_dev, u8 *data)
{
	u8 hat = data[1] & 0x0f;

	input_report_abs(input_dev, ABS_X, data[14] << 8);
	input_report_abs(input_dev, ABS_Y, data[12] << 8);
	input_report_abs(input_dev, ABS_WHEEL, data[10]  << 8);
	input_report_abs(input_dev, ABS_RX, data[16] << 8);
	input_report_abs(input_dev, ABS_RY, data[18] << 8);
	input_report_abs(input_dev, ABS_CLUTCH, data[20] << 8);
	input_report_abs(input_dev, ABS_GAS, data[22] << 8);
	input_report_abs(input_dev, ABS_BRAKE, data[24]  << 8);

	/* Hat is always the upper nibble of the penultimate byte of the report. This hatswitch is very tricky */
	input_report_abs(input_dev, ABS_HAT0X, hat_to_axis[hat][2]);
	input_report_abs(input_dev, ABS_HAT0Y, -hat_to_axis[hat][6]);

	input_report_abs(input_dev, ABS_MISC, data[26] << 8);
}

static int hori_raw_event(struct hid_device *hdev,
			  struct hid_report *report, u8 *data, int len)
{
	if (hdev->product != USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		return 0;

	/* Several wheels report as this id when operating in emulation mode. */
	struct input_dev *input_dev = hid_get_drvdata(hdev);

	parse_tcs_axis_report(input_dev, data);
	input_sync(input_dev);

	return 0;
}

static int hori_input_configured(struct hid_device *hdev,
				 struct hid_input *input)
{
	if (hdev->product != USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		return 0;

	/* Several wheels report as this id when operating in emulation mode. */
	struct input_dev *input_dev = input->input;
	hid_set_drvdata(hdev, input_dev);

	set_bit(EV_ABS, input_dev->evbit);

	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_WHEEL, input_dev->absbit);
	set_bit(ABS_RX, input_dev->absbit);
	set_bit(ABS_RY, input_dev->absbit);
	set_bit(ABS_CLUTCH, input_dev->absbit);
	set_bit(ABS_GAS, input_dev->absbit);
	set_bit(ABS_BRAKE, input_dev->absbit);
	set_bit(ABS_HAT0X, input_dev->absbit);
	set_bit(ABS_HAT0Y, input_dev->absbit);
	set_bit(ABS_MISC, input_dev->absbit);

	input_set_abs_params(input_dev, ABS_X, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_WHEEL, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_RX, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_RY, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_CLUTCH, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_GAS, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_BRAKE, 0, 65535, 255, 0);
	input_set_abs_params(input_dev, ABS_HAT0X, -1, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_HAT0Y, -1, 1, 0, 0);
	input_set_abs_params(input_dev, ABS_MISC, 0, 255, 255, 0);

	return 0;
}

static int hori_input_mapping(struct hid_device *dev, struct hid_input *input,
			      struct hid_field *field, struct hid_usage *usage,
			      unsigned long **bit, int *max)
{
	if (dev->product != USB_DEVICE_ID_HORI_TRUCK_WHEEL)
		return 0;

	/* Skip mapping of device axes */
	switch (usage->hid) {
	case HID_GD_X: case HID_GD_Y: case HID_GD_Z:
	case HID_GD_RX: case HID_GD_RY: case HID_GD_RZ:
	case HID_GD_SLIDER: case HID_GD_HATSWITCH:
		return -1;
	}

	return 0;
}

static const struct hid_device_id hori_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_HORI, USB_DEVICE_ID_HORI_TRUCK_WHEEL) },
	{ HID_USB_DEVICE(USB_VENDOR_ID_HORI, USB_DEVICE_ID_HORI_TRUCK_SHIFTER) },
	{ }
};
MODULE_DEVICE_TABLE(hid, hori_devices);

static struct hid_driver hori_driver = {
	.name = "hid-hori-control-systems",
	.id_table = hori_devices,
	.report_fixup = hori_report_fixup,
	.input_mapping = hori_input_mapping,
	.input_configured = hori_input_configured,
	.raw_event = hori_raw_event,
};
module_hid_driver(hori_driver);

MODULE_AUTHOR("Peter Stein <linuxgamestv@gmail.com>");
MODULE_DESCRIPTION("HID driver for HORI Control Systems Wheels");
MODULE_LICENSE("GPL");
