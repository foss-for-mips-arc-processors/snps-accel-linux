override LINUXINCLUDE := -I$(src)/include $(LINUXINCLUDE)
subdir-ccflags-y += -I$(src)/include

ifeq ($(DEBUG),1)
subdir-ccflags-y += -DDEBUG
endif

obj-m += drivers/misc/snps_accel/
obj-m += drivers/remoteproc/snps_accel/
