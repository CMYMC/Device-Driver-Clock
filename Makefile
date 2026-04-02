obj-m += i2c_sim.o 
obj-m += ds1302_rotary.o 
obj-m += unit.o 
KDIR := ~/Codes/bsp/rpi_kernel/linux

all:
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C $(KDIR) M=$(PWD) modules
	scp i2c_sim.ko pi@10.10.14.80:~/
	scp ds1302_rotary.ko pi@10.10.14.80:~/
	scp unit.ko pi@10.10.14.80:~/
clean:
	make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -C $(KDIR) M=$(PWD) clean

