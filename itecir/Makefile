.PATH:  ${.CURDIR}/..

KMOD    = itecir
SRCS    = itecir.c
SRCS	+= opt_acpi.h
SRCS    += device_if.h bus_if.h acpi_if.h

KMODDIR = /boot/modules

EXPORT_SYMS=    yes

.include <bsd.kmod.mk>
