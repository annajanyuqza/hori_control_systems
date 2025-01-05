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
#include "hid-hori.h"
#include "hid-ids.h"

#define HORI_BAD_RELATIVE_KEYS	0x002
#define HORI_DUPLICATE_USAGES	0x004
#define HORI_TSC_WHEEL_RDESC_ORIG_SIZE	155
#define HORI_TSC_SHIFTER_RDESC_ORIG_SIZE	83
#define HORI_BUTTONS0_HAT_SWITCH	GENMASK(3, 0)
#define HORI_INPUT_REPORT_USB_SIZE		64
#define HID_IN_PACKET 16

#define BTN_BASE7		0x12c
#define BTN_BASE8		0x12d
#define BTN_BASE9          0x12e
#define ABS_CLUTCH      0x0b
#define ABS_CLUTCH      ABS_RZ

#define HORI_TCS_POLL		0x00

#define button1     BTN_TRIGGER
#define button2     BTN_THUMB
#define button3     BTN_THUMB2
#define button4     BTN_TOP
#define button5     BTN_TOP2
#define button6     BTN_PINKIE
#define button7     BTN_BASE
#define button8     BTN_BASE2
#define button9    BTN_BASE3
#define button10     BTN_BASE4
#define button11     BTN_BASE5
#define button12     BTN_BASE6
#define button13     BTN_BASE7
#define button14     BTN_BASE8
#define button15     BTN_BASE9
#define button16     BTN_DEAD
#define button17     BTN_TRIGGER_HAPPY1
#define button18     BTN_TRIGGER_HAPPY2
#define button19     BTN_TRIGGER_HAPPY3
#define button20     BTN_TRIGGER_HAPPY4
#define button21     BTN_TRIGGER_HAPPY5
#define button22     BTN_TRIGGER_HAPPY6
#define button23     BTN_TRIGGER_HAPPY7
#define button24     BTN_TRIGGER_HAPPY8
#define button25     BTN_TRIGGER_HAPPY9
#define button26     BTN_TRIGGER_HAPPY10
#define button27     BTN_TRIGGER_HAPPY11
#define button28     BTN_TRIGGER_HAPPY12
#define button29     BTN_TRIGGER_HAPPY13
#define button30     BTN_TRIGGER_HAPPY14
#define button31     BTN_TRIGGER_HAPPY15
#define button32     BTN_TRIGGER_HAPPY16
#define button33     BTN_TRIGGER_HAPPY17
#define button34     BTN_TRIGGER_HAPPY18
#define button35     BTN_TRIGGER_HAPPY19
#define button36     BTN_TRIGGER_HAPPY20
#define button37     BTN_TRIGGER_HAPPY21
#define button38     BTN_TRIGGER_HAPPY22
#define button39     BTN_TRIGGER_HAPPY23
#define button40     BTN_TRIGGER_HAPPY24
#define button41     BTN_TRIGGER_HAPPY25
#define button42     BTN_TRIGGER_HAPPY26
#define button43     BTN_TRIGGER_HAPPY27
#define button44     BTN_TRIGGER_HAPPY28
#define button45     BTN_TRIGGER_HAPPY29
#define button46     BTN_TRIGGER_HAPPY30
#define button47     BTN_TRIGGER_HAPPY31
#define button48     BTN_TRIGGER_HAPPY32
#define button49     BTN_TRIGGER_HAPPY33
#define button50     BTN_TRIGGER_HAPPY34
#define button51     BTN_TRIGGER_HAPPY35
#define button52     BTN_TRIGGER_HAPPY36
#define button53     BTN_TRIGGER_HAPPY37
#define button54     BTN_TRIGGER_HAPPY38

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
0x46, 0x3b, 0x07, 0x08,             //  Physical Maximum (315)                 		        14
0x65, 0x14,                    		//  Unit (EnglishRotation: deg)        				17
0x09, 0x39,                    		//  Usage (Hat switch)                  				19
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
0x81, 0x02,                    		//  Input (Data,Var,Abs)                   				43
0x95, 0x06,                    		//  Report Count (6)                   					45
0x81, 0x01,                    		//  Input (Cnst,Arr,Abs)                  				47
0x27, 0xff, 0xff, 0x00, 0x00,  	//  Logical Maximum (65535)            				49
0x47, 0xff, 0xff, 0x00, 0x00,  	//  Physical Maximum (65535)          		           	54
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
0x81, 0x02,                    	       	//  Input (Data,Var,Abs)             					81
0x26, 0xff, 0x00,                           //  Logical Maximum (255)              				83
0x46, 0xff, 0x00,                           //  Physical Maximum (255)             				86
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
0x95, 0x3f,                    			//  Report Count (63)                  					144
0x09, 0x04,                    		//  Usage (Vendor Usage 0x04)          			146
0x81, 0x00,                    		//  Input (Data,Arr,Abs)               					148
0x09, 0x04,                    		//  Usage (Vendor Usage 0x04)          			150
0x91, 0x00,                    		//  Output (Data,Arr,Abs)              				152
0xc0,                          			// End Collection                      					154
};

static const __u8 tcs_shifter_rdesc_fixed[] = {
0x05, 0x01,					 // Usage Page (Generic Desktop)        			0
0x09, 0x04,                    		// Usage (Joystick)                    					2
0xa1, 0x01,                    		// Collection (Application)            				4
0x85, 0x01,                    		//  Report ID (1)                      					6
0x15, 0x00,                    		//  Logical Minimum (0)                				8
0x25, 0x01,                    		//  Logical Maximum (1)          	  				10
0x35, 0x00,                    		//  Physical Minimum (0)               				12
0x45, 0x01,                    		//  Physical Maximum (1)               				14
0x05, 0x09,                    		//  Usage Page (Button)                				16
0x19, 0x01,                    		//  Usage Minimum (1)                  				18
0x29, 0x2b,                    		//  Usage Maximum (43)                 				20
0x75, 0x01,                    		//  Report Size (1)                    					22
0x95, 0x2b,                    		//  Report Count (43)                  					24
0x81, 0x02,                    		//  Input (Data,Var,Abs)               					26
0x95, 0x05,                    		//  Report Count (5)                   					28
0x81, 0x01,                    		//  Input (Cnst,Arr,Abs)               					30
0x26, 0xff, 0x00,              		//  Logical Maximum (255)              				32
0x46, 0xff, 0x00,              		//  Physical Maximum (255)             				35
0x06, 0x00, 0xff,              		//  Usage Page (Vendor Defined Page 1) 		38
0x09, 0x01,                    		//  Usage (Vendor Usage 1)             				41
0x75, 0x08,                    		//  Report Size (8)                    					43
0x95, 0x04,                    		//  Report Count (4)                   					45
0x81, 0x02,                    		//  Input (Data,Var,Abs)               					47
0x09, 0x02,                    		//  Usage (Vendor Usage 2)             				49
0x95, 0x1f,                    			//  Report Count (31)                  					51
0x91, 0x02,                    		//  Output (Data,Var,Abs)              				53
0xc0,                          			// End Collection                      					55
0x06, 0x21, 0xff,              		// Usage Page (Vendor Usage Page 0xff21) 		56
0x09, 0x03,                    		// Usage (Vendor Usage 0x03)           			59
0xa1, 0x01,                    		// Collection (Application)            				61
0x85, 0x22,                    		//  Report ID (34)                     					63
0x15, 0x00,                    		//  Logical Minimum (0)                				65
0x26, 0xff, 0x00,              		//  Logical Maximum (255)              				67
0x75, 0x08,                    		//  Report Size (8)                    					70
0x95, 0x3f,                    			//  Report Count (63)                  					72
0x09, 0x04,                    		//  Usage (Vendor Usage 0x04)          			74
0x81, 0x00,                    		//  Input (Data,Arr,Abs)               					76
0x09, 0x04,                    		//  Usage (Vendor Usage 0x04)          			78
0x91, 0x00,                   		//  Output (Data,Arr,Abs)              				80
0xc0,                          			// End Collection                      					82
};

static const __u8 *hori_report_fixup(struct hid_device *hdev, __u8 *rdesc,
				     unsigned int *rsize)
{
	struct hori_drv_data *drv_data = hid_get_drvdata(hdev);

	switch (hdev->product) {

	/* Several wheels report as this id when operating in emulation mode. */
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_WHEEL:
		if (*rsize == HORI_TSC_WHEEL_RDESC_ORIG_SIZE) {
			hid_info(hdev,
				"fixing up HORI TCS  Wheel report descriptor\n");
			rdesc = tcs_wheel_rdesc_fixed;
			*rsize = sizeof(tcs_wheel_rdesc_fixed);
		}
		break;;
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_SHIFTER:
		if (*rsize == HORI_TSC_SHIFTER_RDESC_ORIG_SIZE) {
			hid_info(hdev,
				"fixing up HORI TCS  Shifter report descriptor\n");
			rdesc = tcs_shifter_rdesc_fixed;
			*rsize = sizeof(tcs_shifter_rdesc_fixed);
		}
		break;
        }

	return rdesc;

}

/*
struct input_keymap_entry {
#define INPUT_KEYMAP_BY_INDEX	(1 << 0)
	__u8  flags;
	__u8  len;
	__u16 index;
	__u32 keycode;
	__u8  scancode[32];
};
*/
/* Here we mpp the xis new */ 
static int hori_input_mapped(struct hid_device *hdev, struct hid_input *hi,
		struct hid_field *field, struct hid_usage *usage,
		unsigned long **bit, int *max)
{

	struct hori_drv_data *drv_data = hid_get_drvdata(hdev);

	if ((drv_data->quirks & HORI_BAD_RELATIVE_KEYS) && usage->type == EV_KEY &&
			(field->flags & HID_MAIN_ITEM_RELATIVE))
		field->flags &= ~HID_MAIN_ITEM_RELATIVE;

	if ((drv_data->quirks & HORI_DUPLICATE_USAGES) && (usage->type == EV_KEY ||
			 usage->type == EV_REL || usage->type == EV_ABS))
		clear_bit(usage->code, *bit);

	/* Ensure that Logitech wheels are not given a default fuzz/flat value */
	if (usage->type == EV_ABS && (usage->code == ABS_X ||
			usage->code == ABS_Y || usage->code == ABS_Z ||
			usage->code == ABS_RZ)) {
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

static int hori_input_mapping(struct hid_device *dev,
                             struct hid_input *input,
                             struct hid_field *field,
                             struct hid_usage *usage,
                             unsigned long **bit,
                             int *max)
{
    /*
     * We are reporting the events in x52_raw_event.
     * Skip the hid-input processing.
     */
    return -1;
}

#define asus_map_key_clear(c)	hid_map_usage_clear(hi, usage, bit, \
						    max, EV_KEY, (c))
static int hori_input_configured(struct hid_device *hdev,
                                struct hid_input *input)
{
    struct input_dev * input_dev = input->input;

    hid_set_drvdata(hdev, input_dev);
    
    int max_stick;
    
	switch (hdev->product) {

	/* Several wheels report as this id when operating in emulation mode. */
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_WHEEL:
                set_bit(EV_ABS, input_dev->evbit);
		set_bit(EV_MSC, input_dev->evbit);
		set_bit(MSC_SCAN, input_dev->mscbit);
                set_bit(EV_KEY, input_dev->evbit);
                    
                set_bit(BTN_TRIGGER, input_dev->keybit);
                set_bit(BTN_THUMB, input_dev->keybit);
                set_bit(BTN_THUMB2, input_dev->keybit);
                set_bit(BTN_TOP, input_dev->keybit);
                set_bit(BTN_TOP2, input_dev->keybit);
                set_bit(BTN_PINKIE, input_dev->keybit);
                set_bit(BTN_BASE, input_dev->keybit);
                set_bit(BTN_BASE2, input_dev->keybit);
                set_bit(BTN_BASE3, input_dev->keybit);
                set_bit(BTN_BASE4, input_dev->keybit);
                set_bit(BTN_BASE5, input_dev->keybit);
                set_bit(BTN_BASE6, input_dev->keybit);
                set_bit(BTN_BASE7, input_dev->keybit);
                set_bit(BTN_BASE8, input_dev->keybit);
                set_bit(BTN_BASE9, input_dev->keybit);
                set_bit(BTN_DEAD, input_dev->keybit);

                set_bit(BTN_TRIGGER_HAPPY1, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY2, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY3, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY4, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY5, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY6, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY7, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY8, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY9, input_dev->keybit);;
                set_bit(BTN_TRIGGER_HAPPY10, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY11, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY12, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY13, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY14, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY15, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY16, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY17, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY18, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY19, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY20, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY21, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY22, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY23, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY24, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY25, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY26, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY27, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY28, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY29, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY30, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY31, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY32, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY33, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY34, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY35, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY36, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY37, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY38, input_dev->keybit);
                
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
                /*set_bit(ABS_MISC, input_dev->absbit);*/
                
                input_set_abs_params(input_dev, ABS_X, 0, 65535, 255, 4095);
                input_set_abs_params(input_dev, ABS_Y, 0, 65535, 255, 4095);
                input_set_abs_params(input_dev, ABS_WHEEL, 0, 65535, 255, 4095);
                input_set_abs_params(input_dev, ABS_RX, 0, 65535, 255, 4095);
                input_set_abs_params(input_dev, ABS_RY, 0, 65535, 255, 4095);
                input_set_abs_params(input_dev, ABS_CLUTCH, 0, 65535, 255, 4095);
                input_set_abs_params(input_dev, ABS_GAS, 0, 65535, 255, 4095);
                input_set_abs_params(input_dev, ABS_BRAKE, 0, 65535, 255, 4095);

                input_set_abs_params(input_dev, ABS_HAT0X, -1, 1, 0, 0);
                input_set_abs_params(input_dev, ABS_HAT0Y, -1, 1, 0, 0);

               /* input_set_abs_params(input_dev, ABS_MISC, 0, 65535, 255, 4095);*/

                input_set_capability(input_dev, EV_MSC, MSC_SCAN);
                
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER);
                input_set_capability(input_dev, EV_KEY, BTN_THUMB);
                input_set_capability(input_dev, EV_KEY, BTN_THUMB2);
                input_set_capability(input_dev, EV_KEY, BTN_TOP);
                input_set_capability(input_dev, EV_KEY, BTN_TOP2);
                input_set_capability(input_dev, EV_KEY, BTN_PINKIE);
                input_set_capability(input_dev, EV_KEY, BTN_BASE);
                input_set_capability(input_dev, EV_KEY, BTN_BASE2);
                input_set_capability(input_dev, EV_KEY, BTN_BASE3);
                input_set_capability(input_dev, EV_KEY, BTN_BASE4);
                input_set_capability(input_dev, EV_KEY, BTN_BASE5);
                input_set_capability(input_dev, EV_KEY, BTN_BASE6);
                input_set_capability(input_dev, EV_KEY, BTN_BASE7);
                input_set_capability(input_dev, EV_KEY, BTN_BASE8);
                input_set_capability(input_dev, EV_KEY, BTN_BASE9);
                input_set_capability(input_dev, EV_KEY, BTN_DEAD);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY1);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY2);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY3);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY4);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY5);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY6);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY7);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY8);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY9);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY10);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY11);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY12);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY13);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY14);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY15);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY16);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY17);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY18);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY19);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY20);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY21);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY22);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY23);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY24);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY25);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY26);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY27);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY28);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY29);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY30);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY31);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY32);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY33);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY34);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY35);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY36);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY37);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY38);
		break;;
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_SHIFTER:
		set_bit(EV_MSC, input_dev->evbit);
		set_bit(MSC_SCAN, input_dev->mscbit);
                set_bit(EV_KEY, input_dev->evbit);
                
                set_bit(BTN_TRIGGER, input_dev->keybit);
                set_bit(BTN_THUMB, input_dev->keybit);
                set_bit(BTN_THUMB2, input_dev->keybit);
                set_bit(BTN_TOP, input_dev->keybit);
                set_bit(BTN_TOP2, input_dev->keybit);
                set_bit(BTN_PINKIE, input_dev->keybit);
                set_bit(BTN_BASE, input_dev->keybit);
                set_bit(BTN_BASE2, input_dev->keybit);
                set_bit(BTN_BASE3, input_dev->keybit);
                set_bit(BTN_BASE4, input_dev->keybit);
                set_bit(BTN_BASE5, input_dev->keybit);
                set_bit(BTN_BASE6, input_dev->keybit);
                set_bit(BTN_BASE7, input_dev->keybit);
                set_bit(BTN_BASE8, input_dev->keybit);
                set_bit(BTN_BASE9, input_dev->keybit);
                set_bit(BTN_DEAD, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY1, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY2, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY3, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY4, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY5, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY6, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY7, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY8, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY9, input_dev->keybit);;
                set_bit(BTN_TRIGGER_HAPPY10, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY11, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY12, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY13, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY14, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY15, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY16, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY17, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY18, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY19, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY20, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY21, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY22, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY23, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY24, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY25, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY26, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY27, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY28, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY29, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY30, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY31, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY32, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY33, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY34, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY35, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY36, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY37, input_dev->keybit);
                set_bit(BTN_TRIGGER_HAPPY38, input_dev->keybit);
    
                
                input_set_capability(input_dev, EV_MSC, MSC_SCAN);
                
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER);
                input_set_capability(input_dev, EV_KEY, BTN_THUMB);
                input_set_capability(input_dev, EV_KEY, BTN_THUMB2);
                input_set_capability(input_dev, EV_KEY, BTN_TOP);
                input_set_capability(input_dev, EV_KEY, BTN_TOP2);
                input_set_capability(input_dev, EV_KEY, BTN_PINKIE);
                input_set_capability(input_dev, EV_KEY, BTN_BASE);
                input_set_capability(input_dev, EV_KEY, BTN_BASE2);
                input_set_capability(input_dev, EV_KEY, BTN_BASE3);
                input_set_capability(input_dev, EV_KEY, BTN_BASE4);
                input_set_capability(input_dev, EV_KEY, BTN_BASE5);
                input_set_capability(input_dev, EV_KEY, BTN_BASE6);
                input_set_capability(input_dev, EV_KEY, BTN_BASE7);
                input_set_capability(input_dev, EV_KEY, BTN_BASE8);
                input_set_capability(input_dev, EV_KEY, BTN_BASE9);
                input_set_capability(input_dev, EV_KEY, BTN_DEAD);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY1);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY2);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY3);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY4);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY5);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY6);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY7);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY8);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY9);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY10);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY11);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY12);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY13);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY14);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY15);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY16);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY17);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY18);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY19);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY20);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY21);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY22);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY23);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY24);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY25);
                input_set_capability(input_dev, EV_KEY, BTN_TRIGGER_HAPPY26);
		break;;
        }
    return 0;
}


/* Main DualSense input report excluding any BT/USB specific headers. */
struct hori_input_report {
	uint8_t x, y;
	uint8_t seq_number;
        uint8_t hat_buttons[24];
	uint8_t wheel_buttons[54];
	uint8_t shifter_buttons[42];

} __packed;
/* Common input report size shared equals the size of the USB report minus 1 byte for ReportID. 
static_assert(sizeof(struct hori_input_report) == HORI_INPUT_REPORT_USB_SIZE - 1);*/

static const struct {int x; int y; } hori_tcs_wheel_hat_mapping[] = {
	{0, -1}, {1, -1}, {1, 0}, {1, 1}, {0, 1}, {-1, 1}, {-1, 0}, {-1, -1},
	{0, 0},
};

static int hori_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int len)
{
    struct input_dev *input_dev = hid_get_drvdata(hdev);
    uint16_t range;
	int dev_max_range, ret, count;

	struct hori_tcs *hori_tcs = hid_get_drvdata(hdev);

	if(range < 270)
		range = 270;
	else if (range > dev_max_range)
		range = dev_max_range;

	range = DIV_ROUND_CLOSEST((range * 0xffff), 1800);

	switch (hdev->product) {

	/* Several wheels report as this id when operating in emulation mode. */
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_WHEEL:
            
                input_report_abs(input_dev, ABS_X, data[14] << 8);
                input_report_abs(input_dev, ABS_Y, data[12] << 8);
                input_report_abs(input_dev, ABS_WHEEL, data[10]  << 8);                
                input_report_abs(input_dev, ABS_RX, data[16] << 8);
                input_report_abs(input_dev, ABS_RY, data[18] << 8);
                input_report_abs(input_dev, ABS_CLUTCH, data[20] << 8);
                input_report_abs(input_dev, ABS_GAS, data[22] << 8);                
                input_report_abs(input_dev, ABS_BRAKE, data[24]  << 8);
                
                input_report_abs(input_dev, ABS_HAT0X, data[0]  );
                input_report_abs(input_dev, ABS_HAT0Y, data[0]  >> 2  ); 

 /*	input_report_abs(input_dev, ABS_HAT0X, hori_tcs_wheel_hat_mapping[value].x);
	input_report_abs(input_dev, ABS_HAT0Y, hori_tcs_wheel_hat_mapping[value].y);
*/
                input_report_key(input_dev, EV_MSC, MSC_SCAN);
        
                input_event(input_dev, EV_KEY, BTN_TRIGGER, data[1] );
                input_event(input_dev, EV_KEY, BTN_THUMB, data[1] >> 4 );
                input_event(input_dev, EV_KEY, BTN_THUMB2, data[1] << 2);
                input_event(input_dev, EV_KEY, BTN_TOP, data[1] >> 2);
                input_event(input_dev, EV_KEY, BTN_TOP2, data[2]);                                                  /* 5 */
                input_event(input_dev, EV_KEY, BTN_PINKIE, data[2] << 4);
                input_event(input_dev, EV_KEY, BTN_BASE, data[2] << 2);
                input_event(input_dev, EV_KEY, BTN_BASE2, data[2] >> 4);
                input_event(input_dev, EV_KEY, BTN_BASE3, data[2] >> 8);
                input_event(input_dev, EV_KEY, BTN_BASE4, data[2] << 8);
                input_event(input_dev, EV_KEY, BTN_BASE5, data[2] >> 2);
                input_event(input_dev, EV_KEY, BTN_BASE6, data[3] >> 8);
                input_event(input_dev, EV_KEY, BTN_BASE7, data[3] >> 4);
                input_event(input_dev, EV_KEY, BTN_BASE8, data[3] << 8);
                input_event(input_dev, EV_KEY, BTN_BASE9, data[15]);
                input_event(input_dev, EV_KEY, BTN_DEAD, data[16]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY1, data[3] << 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY2, data[3] << 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY3, data[3] >> 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY4, data[4] >> 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY5, data[4] >> 2);                         /* 21 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY6, data[4] << 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY7, data[4] << 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY8, data[4] << 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY9, data[4] >> 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY10, data[4] );           /* 26 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY11, data[4]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY12, data[5] << 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY13, data[5]);                    /* 29 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY14, data[5] << 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY15, data[5] << 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY16, data[5] >> 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY17, data[5] >> 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY18, data[5] >> 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY19, data[6] << 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY20, data[6] << 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY21, data[6] << 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY22, data[6] >> 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY23, data[6] >> 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY24, data[6] >> 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY25, data[6]);                     /* 37 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY26, data[7]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY27, data[7] << 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY28, data[7] >> 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY29, data[7]);                  /* 45 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY30, data[8] >> 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY31, data[7] >> 2);         /* 47 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY32, data[0] << 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY33, data[7] >> 4);          /* 49 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY34, data[8] << 8);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY35, data[8] >> 4);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY36, data[8] >> 2);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY37, data[8]);                   /* 53 */
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY38, data[8] << 2);

                break;;
                
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_SHIFTER:
                input_report_key(input_dev, EV_MSC, MSC_SCAN);
                
                input_event(input_dev, EV_KEY, BTN_TRIGGER, data[1]);
                input_event(input_dev, EV_KEY, BTN_THUMB, data[2] );
                input_event(input_dev, EV_KEY, BTN_THUMB2, data[17]);
                input_event(input_dev, EV_KEY, BTN_TOP, data[25]);
                input_event(input_dev, EV_KEY, BTN_TOP2, data[33]);
                input_event(input_dev, EV_KEY, BTN_PINKIE, data[6]);
                input_event(input_dev, EV_KEY, BTN_BASE, data[7]);
                input_event(input_dev, EV_KEY, BTN_BASE2, data[8]);
                input_event(input_dev, EV_KEY, BTN_BASE3, data[9]);
                input_event(input_dev, EV_KEY, BTN_BASE4, data[10]);
                input_event(input_dev, EV_KEY, BTN_BASE5, data[2] >> 4);
                input_event(input_dev, EV_KEY, BTN_BASE6, data[12] >> 16);
                input_event(input_dev, EV_KEY, BTN_BASE7, data[13] << 2);
                input_event(input_dev, EV_KEY, BTN_BASE8, data[14]);
                input_event(input_dev, EV_KEY, BTN_BASE9, data[15]);
                input_event(input_dev, EV_KEY, BTN_DEAD, data[16]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY1, data[3]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY2, data[18]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY3, data[19]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY4, data[20]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY5, data[21]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY6, data[22]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY7, data[23]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY8, data[24]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY9, data[4]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY10, data[26]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY11, data[27]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY12, data[28]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY13, data[29]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY14, data[30]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY15, data[31]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY16, data[32]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY17, data[5]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY18, data[34]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY19, data[34]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY20, data[36]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY21, data[37]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY22, data[38]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY23, data[39]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY24, data[40]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY25, data[6]);
                input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY26, data[42]);
                break;;
        }
        
   /*     input_sync(input_dev); */
    return ret;
}
/*
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
*/
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
	.name = "hid-hori-wheels",
	.id_table = hori_devices,
        .report_fixup = hori_report_fixup,
        .input_mapped = hori_input_mapped,
	.probe = hori_probe,
	.input_mapping = hori_input_mapping,
	.input_configured = hori_input_configured,
	.raw_event = hori_raw_event,
};
module_hid_driver(hori_driver);

MODULE_AUTHOR("LinuxGamesTV <linuxgamestv@gmail.com>");
MODULE_DESCRIPTION("HID driver for HORI Control Systems Wheels");
MODULE_LICENSE("GPL");
