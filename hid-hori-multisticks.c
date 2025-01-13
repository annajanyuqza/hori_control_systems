// SPDX-License-Identifier: GPL-2.0-or-later
/*
 *  HID driver for some Hori "special" devices
 *
 *  Copyright (c) 2024/2025 Peter Stein, LinuxGamesTV aka BdMdesigN <linuxgamestv@gmail.com> 
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

#define BTN_BASE7		0x12c
#define BTN_BASE8		0x12d
#define BTN_BASE9          0x12e

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

static void _parse_tsc_shifter_button_report(struct input_dev *input_dev, u8 *data)
{
    input_report_key(input_dev, EV_MSC, MSC_SCAN);

    input_event(input_dev, EV_KEY, BTN_TRIGGER, data[1]  &1 );                                             // 1
    input_event(input_dev, EV_KEY, BTN_THUMB, data[1] >> 1 &1);                                         // 2
    input_event(input_dev, EV_KEY, BTN_THUMB2, data[1] >> 2 &1);                                       // 3
    input_event(input_dev, EV_KEY, BTN_TOP, data[1] >> 3 &1);                                                // 4
    input_event(input_dev, EV_KEY, BTN_TOP2, data[1] >> 4&1);                                               // 5
    input_event(input_dev, EV_KEY, BTN_PINKIE, data[1] >> 5 &1);                                            // 6
    input_event(input_dev, EV_KEY, BTN_BASE, data[1] >> 6 &1);                                              // 7
    input_event(input_dev, EV_KEY, BTN_BASE2, data[1] >> 7 &1);                                            // 8
    input_event(input_dev, EV_KEY, BTN_BASE3, data[2]  &1);                                                    // 9
    input_event(input_dev, EV_KEY, BTN_BASE4, data[2] >> 5 &1);                                            // 10
    input_event(input_dev, EV_KEY, BTN_BASE5, data[2] >> 6 &1);                                            // 11
    input_event(input_dev, EV_KEY, BTN_BASE6, data[2] >> 7 &1);                                            // 12
    input_event(input_dev, EV_KEY, BTN_BASE7, data[2] &1);                                                     // 13
    input_event(input_dev, EV_KEY, BTN_BASE8, data[2] >> 1 &1);                                            // 14
    input_event(input_dev, EV_KEY, BTN_BASE9, data[2] >> 2 &1);                                            // 15
    input_event(input_dev, EV_KEY, BTN_DEAD, data[2] >> 3 &1);                                              // 16
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY1, data[3] &1);                              // 17
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY2, data[3] >> 1 &1);                     // 18
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY3, data[3] >> 2 &1);                     // 19
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY4, data[3] >> 3 &1);                     // 20
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY5, data[3] >> 4 &1);                     // 21
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY6, data[3] >> 5 &1);                     // 22
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY7, data[3] >> 6 &1);                     // 23
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY8, data[3] >> 7 &1);                     // 24
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY9, data[4] &1);                              // 25
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY10, data[4] >> 1 &1);                   // 26
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY11, data[4] >>  2 &1);                  // 27
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY12, data[4] >> 3 &1);                   // 28
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY13, data[4] >> 4 &1);                   // 29
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY14, data[4] >> 5  &1);                  // 30
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY15, data[4] >> 6 &1);                   // 31
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY16, data[4] >> 7  &1);                  // 32
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY17, data[5] &1);                            // 33
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY18, data[5] >> 1  &1);                  // 34
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY19, data[5] >> 2  &1);                  // 35
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY20, data[5] >> 3  &1);                  // 36
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY21, data[5] >> 4 &1);                   // 37
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY22, data[5] >> 5 &1);                   // 38
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY23, data[5] >> 6 &1);                   // 39
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY24, data[5] >> 7 &1);                   // 40
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY25, data[6] &1);                            // 41
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY26, data[6] >> 1  &1);                  // 42
    input_event(input_dev, EV_KEY, BTN_TRIGGER_HAPPY27, data[6] >> 2  &1);                  // 43
}

static int  hori_multisticks_raw_event(struct hid_device *hdev,
                         struct hid_report *report, u8 *data, int len)
{
    struct input_dev *input_dev = hid_get_drvdata(hdev);

    switch (hdev->product) {

	/* Several wheels report as this id when operating in emulation mode. */
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_SHIFTER:
                _parse_tsc_shifter_button_report(input_dev, data);
                break;
        }
    input_sync(input_dev);
    return 0;
}

static int hori_multisticks_input_configured(struct hid_device *hdev,
                                struct hid_input *input)
{
    struct input_dev *input_dev = input->input;
    int i;
    
    hid_set_drvdata(hdev, input_dev);

    set_bit(MSC_SCAN, input_dev->mscbit);
    set_bit(EV_KEY, input_dev->evbit);
    set_bit(EV_ABS, input_dev->evbit);
    
	switch (hdev->product) {

	/* Several wheels report as this id when operating in emulation mode. */
	case USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_WHEEL:

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

                for (i = 0x10; i < 43; i++) {
                        set_bit(BTN_TRIGGER_HAPPY + i - 0x10, input_dev->keybit);
                }
                
                break;
        }
    return 0;
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
static const struct hid_device_id hori_multisticks_devices[] = {
	{ HID_USB_DEVICE(USB_VENDOR_ID_HORI, USB_DEVICE_ID_HORI_TRUCK_CONTROL_SYSTEM_SHIFTER) },
	{ }
};
MODULE_DEVICE_TABLE(hid, hori_multisticks_devices);

static struct hid_driver hori_multisticks_driver = {
	.name = "hid-hori-multisticks",
	.id_table = hori_multisticks_devices,
        .report_fixup = hori_multisticks_report_fixup,
        .input_mapping = hori_multisticks_input_mapping,
        .input_configured = hori_multisticks_input_configured,
        .raw_event = hori_multisticks_raw_event,
};
module_hid_driver(hori_multisticks_driver);

MODULE_AUTHOR("LinuxGamesTV <linuxgamestv@gmail.com>");
MODULE_DESCRIPTION("HID driver for HORI Control Systems Multifunktions Sticks/Shifter");
MODULE_LICENSE("GPL");
