#Makefile
obj-m := hid-hori.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

udev-rules:
	install -m 0644 udev/99-hori-wheels.rules /etc/udev/rules.d/

install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
