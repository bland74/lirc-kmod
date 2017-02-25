.PATH:  ${.CURDIR}/..

KMOD    = lirc
SRCS    = lirc_dev.c
SRCS	+= device_if.h bus_if.h

KMODDIR = /boot/modules

EXPORT_SYMS=    yes

.include <bsd.kmod.mk>
