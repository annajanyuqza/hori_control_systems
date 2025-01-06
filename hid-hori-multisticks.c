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
#include "hid-ids.h"

#define HORI_TSC_SHIFTER_RDESC_ORIG_SIZE	83
#define HORI_INPUT_REPORT_USB_SIZE		64
#define HID_IN_PACKET 16

#define BTN_BASE7		0x12c
#define BTN_BASE8		0x12d
#define BTN_BASE9          0x12e

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

static const __u8 *hori_multisticks_report_fixup(struct hid_device *hdev, __u8 *rdesc,
				     unsigned int *rsize)
{
	struct hori_multisticks_drv_data *drv_data = hid_get_drvdata(hdev);

	switch (hdev->product) {

	/* Several wheels report as this id when operating in emulation mode. */
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

static int hori_multisticks_input_mapping(struct hid_device *dev,
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
static int hori_multisticks_input_configured(struct hid_device *hdev,
                                struct hid_input *input)
{
    struct input_dev * input_dev = input->input;

    hid_set_drvdata(hdev, input_dev);
    
    int max_stick;
    
	switch (hdev->product) {
;
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

static int hori_multisticks_raw_event(struct hid_device *hdev, struct hid_report *report, u8 *data, int len)
{
    struct input_dev *input_dev = hid_get_drvdata(hdev);
    uint16_t range;
	int dev_max_range, ret, count;

	struct hori_multisticks_tcs *hori_multisticks_tcs = hid_get_drvdata(hdev);

	if(range < 270)
		range = 270;
	else if (range > dev_max_range)
		range = dev_max_range;

	range = DIV_ROUND_CLOSEST((range * 0xffff), 1800);

	switch (hdev->product) {

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

static int hori_multisticks_probe(struct hid_device *hdev,
			     const struct hid_device_id *id)
{
	int ret;

	hdev->quirks |= HID_QUIRK_INPUT_PER_APP;

	ret = hid_parse(hdev);
	if (ret)
		return ret;

	return hid_hw_start(hdev, HID_CONNECT_DEFAULT);
}

static const struct hid_device_id hori_multisticks_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_HORI, USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_SHIFTER) },
	{ }
};
MODULE_DEVICE_TABLE(hid, hori_multisticks_devices);

static struct hid_driver hori_multisticks_driver = {
	.name = "hid-hori-multisticks",
	.id_table = hori_multisticks_devices,
        .report_fixup = hori_multisticks_report_fixup,
	.probe = hori_multisticks_probe,
        .input_mapping = hori_multisticks_input_mapping,
        .multisticks_input_configured = hori_multisticks_input_configured,
        .multisticks_raw_event = hori_multisticks_raw_event,
};
module_hid_driver(hori_multisticks_driver);

MODULE_AUTHOR("LinuxGamesTV <linuxgamestv@gmail.com>");
MODULE_DESCRIPTION("HID driver for HORI Control Systems Multifunktions Sticks/Shifter");
MODULE_LICENSE("GPL");
