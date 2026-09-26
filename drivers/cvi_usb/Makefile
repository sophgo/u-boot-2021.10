ifeq ($(CONFIG_SYS_BOARD),"cv84x6")
utask-dir := utask/cv84x6
else
utask-dir := utask/cv186x
endif

obj-y := $(utask-dir)/cvi_usb.o
obj-y += $(utask-dir)/cps_cvi.o
obj-y += $(utask-dir)/core.o
obj-y += $(utask-dir)/ep0.o
obj-y += $(utask-dir)/udc-core.o
obj-y += $(utask-dir)/gadget.o
obj-y += $(utask-dir)/usb_tty.o