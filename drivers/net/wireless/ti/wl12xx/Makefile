wl12xx-objs	= main.o cmd.o acx.o debugfs.o scan.o event.o

define filechk_version.h
	(echo 'static const char *wl12xx_git_head = \
			 "$(shell git describe --dirty)";')
endef

obj-$(CONFIG_WL12XX)		+= wl12xx.o
