#Makefile
MODULE_LOADED := $(shell lsmod | grep hid_universal_pidff)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

udev-rules:
	install -m 0644 udev/99-hori-wheels.rules /etc/udev/rules.d/

install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean

unload:
	@if [ "$(MODULE_LOADED)" != "" ]; then\
		rmmod hid_hori;\
	fi

load: unload
	insmod hid-hori.ko
