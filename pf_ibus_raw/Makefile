obj-m += af_ibus_raw.o
KPATH=/lib/modules/`uname -r`/build

all:
	make -C ${KPATH} M=$(PWD) modules

clean:
	make -C ${KPATH} M=$(PWD) clean
