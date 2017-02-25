/*-
 * Copyright (c) 2017 Alexander Nedotsukov
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/bus.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/rman.h>


#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include "itecirreg.h"
#include "lirc_dev.h"

#ifndef LIRC_ACPI_HACK
#define LIRC_ACPI_HACK 1
#endif

#define INB(scp,port) bus_space_read_1((scp)->bt,(scp)->bh,(port))
#define OUTB(scp,port,val) \
	bus_space_write_1((scp)->bt,(scp)->bh,(port),(val))

#define DEVICE2SOFTC(dev) ((struct itecir_softc *) device_get_softc(dev))

#define DIV_ROUND_CLOSEST(x, y) (((x)+((y) / 2)) / (y))

struct itecir_softc {
	device_t	dev;
	struct lircdev	*lirc;
	bus_space_tag_t	bt;
	bus_space_handle_t bh;
	int		rid_ioport;
	struct resource	*res_ioport;
	int		rid_irq;
	struct resource	*res_irq;
	void		*intr_cookie;

	u_int8_t	lastbit;
	unsigned long	acc;
};

static int		itecir_probe (device_t);
static int		itecir_attach (device_t);
static int		itecir_detach (device_t);

static int		itecir_allocate_resources(struct itecir_softc *scp);
static void		itecir_deallocate_resources(struct itecir_softc *scp);
static int		itecir_resource_int_value(struct itecir_softc* scp, const char* resname, int defval);
static void		itecir_init_hw(struct itecir_softc * scp);
static void		itecir_shutdown_hw(struct itecir_softc * scp);
static void		itecir_set_carrier(struct itecir_softc * scp);
static void		itecir_enable_rx(struct itecir_softc * scp);
static void		itecir_disable_rx(struct itecir_softc * scp);
static void		itecir_idle_rx(struct itecir_softc * scp);

static lircdev_open_t	itecir_open;
static lircdev_close_t	itecir_close;
static void		itecir_intr(void*);

static bool		is_high_carrier_freq(unsigned int freq);
static u_int8_t		carrier_freq_bits(unsigned int freq);
static u_int8_t		pulse_width_bits(unsigned int freq, int duty_cycle);
static unsigned int	pulse_duration(unsigned long val);

static struct lircdevsw itecir_lircdevsw = {
	.d_open =	itecir_open,
	.d_close =	itecir_close
};

static devclass_t itecir_devclass;

/* Tunables */
static int itecir_default_freq = ITE_DEFAULT_CARRIER_FREQ;
TUNABLE_INT("hw.itecir.default_freq", &itecir_default_freq);
static int itecir_default_baudrate = ITE_DEFAULT_BAUDRATE;
TUNABLE_INT("hw.itecir.default_baudrate", &itecir_default_baudrate);

static device_method_t itecir_methods[] = {
	DEVMETHOD(device_probe,		itecir_probe),
	DEVMETHOD(device_attach,	itecir_attach),
	DEVMETHOD(device_detach,	itecir_detach),
	{ 0, 0 }
};

static driver_t itecir_driver = {
	"itecir",
	itecir_methods,
	sizeof (struct itecir_softc)
};

DRIVER_MODULE(itecir, acpi, itecir_driver, itecir_devclass, 0, 0);
MODULE_DEPEND(itecir, acpi, 1, 1, 1);
MODULE_DEPEND(itecir, lirc, 1, 1, 1);
MODULE_VERSION(itecir, 1);

#if LIRC_ACPI_HACK
static ACPI_STATUS
acpi_make_srs_from_bus_resource(device_t dev, ACPI_BUFFER* buf)
{
	ACPI_HANDLE handle;
	ACPI_STATUS status;
	ACPI_RESOURCE *end, *res;
	unsigned long count, ioport, irq;

	handle = acpi_get_handle(dev);

	buf->Pointer = NULL;
	buf->Length = ACPI_ALLOCATE_BUFFER;
	status = AcpiGetCurrentResources(handle, buf);
	if (ACPI_SUCCESS(status) && buf->Pointer == NULL)
		status = AE_NO_MEMORY;

	if (ACPI_FAILURE(status))
		return (status);

	bus_get_resource(dev, SYS_RES_IOPORT, 0, &ioport, &count);
	bus_get_resource(dev, SYS_RES_IRQ, 0, &irq, &count);

	res = (ACPI_RESOURCE *)((char *)buf->Pointer);
	end = (ACPI_RESOURCE *)((char *)buf->Pointer + buf->Length);
	for (;;) {
		switch (res->Type) {
		case ACPI_RESOURCE_TYPE_IRQ:
			res->Data.Irq.Interrupts[0] = irq;
			break;
		case ACPI_RESOURCE_TYPE_IO:
			res->Data.Io.Minimum = ioport;
			res->Data.Io.Maximum = ioport;
			break;
		}
		if (res->Type == ACPI_RESOURCE_TYPE_END_TAG)
			break;
		res = ACPI_NEXT_RESOURCE(res);
		if (res >= end)
			break;
	}

	return (AE_OK);
}

static ACPI_STATUS
acpi_set_resource(device_t dev, ACPI_BUFFER* buf)
{
	ACPI_HANDLE handle;

	handle = acpi_get_handle(dev);

	return (AcpiSetCurrentResources(handle, buf));
}
#endif

static int
itecir_probe (device_t dev)
{
	static char *ids[] = { "ITE8704", "ITE8713", NULL };
	struct itecir_softc *scp = DEVICE2SOFTC(dev);

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, ids) == NULL)
		return (ENXIO); 
#if LIRC_ACPI_HACK
	/* XXX This must come from _PRS somehow */
	if (bus_set_resource(dev, SYS_RES_IOPORT, 0, 0x2F8, IT87_CIR_IOREG_LEN) != 0) {
		device_printf(dev, "cannot assign io port resource");
		return (ENXIO);
	}
	if (bus_set_resource(dev, SYS_RES_IRQ, 0, 5, 1) != 0) {
		device_printf(dev, "cannot assign irq resource");
		return (ENXIO);
	}
#endif
	device_set_desc(dev, "ITE CIR transceiver");
	bzero(scp, sizeof(*scp));
	scp->dev = dev;
	return (0);
}


static int
itecir_attach(device_t dev)
{
	struct itecir_softc *scp = DEVICE2SOFTC(dev);

	if (itecir_allocate_resources(scp))
		goto errexit;

	scp->bt = rman_get_bustag(scp->res_ioport);
	scp->bh = rman_get_bushandle(scp->res_ioport);

	if (bus_setup_intr(dev, scp->res_irq, INTR_TYPE_TTY | INTR_MPSAFE,
		NULL, &itecir_intr, scp, &scp->intr_cookie) == 0)
		; /* Success */
	else
		goto errexit;
#if LIRC_ACPI_HACK
	ACPI_BUFFER srsbuf;
	ACPI_STATUS status;

	device_printf(dev, "enabling device via ACPI\n");

	status = acpi_make_srs_from_bus_resource(dev, &srsbuf);

	if (ACPI_FAILURE(status))
		device_printf(dev, "unable to make SRS: %s\n", AcpiFormatException(status));
	else {
		status = acpi_set_resource(dev, &srsbuf);
		if (ACPI_FAILURE(status))
			device_printf(dev, "unable to execute SRS: %s\n", AcpiFormatException(status));
		AcpiOsFree(srsbuf.Pointer);
	}
#endif
	scp->lirc = lircdev_create(&itecir_lircdevsw, dev);
	if (scp->lirc == NULL)
		goto errexit;

	itecir_init_hw(scp);

	return (0);

errexit:
	itecir_deallocate_resources(scp);
	return (ENXIO);
}

static int
itecir_detach(device_t dev)
{
	struct itecir_softc *scp = DEVICE2SOFTC(dev);

	itecir_shutdown_hw(scp);

	if (scp->lirc != NULL) {
		lircdev_destroy(scp->lirc);
		scp->lirc = NULL;
	}

	if (scp->intr_cookie)
		bus_teardown_intr(dev, scp->res_irq, scp->intr_cookie);

	itecir_deallocate_resources(scp);

	return (0);
}

static int
itecir_allocate_resources(struct itecir_softc *scp)
{
	scp->rid_ioport = 0;
	scp->res_ioport = bus_alloc_resource_any(scp->dev, SYS_RES_IOPORT,
			&scp->rid_ioport, RF_ACTIVE);
	if (scp->res_ioport == NULL) {
		device_printf(scp->dev, "cannot allocate io ports\n");
		goto errexit;
	}
	scp->rid_irq = 0;
	scp->res_irq = bus_alloc_resource_any(scp->dev, SYS_RES_IRQ,
			&scp->rid_irq, RF_SHAREABLE | RF_ACTIVE);
	if (scp->res_irq == NULL) {
		device_printf(scp->dev, "cannot allocate irq\n");
		goto errexit;
	}
	return (0);

errexit:
	itecir_deallocate_resources(scp);
	return (ENXIO);
}

static void
itecir_deallocate_resources(struct itecir_softc *scp)
{
	if (scp->res_irq != 0) {
		bus_deactivate_resource(scp->dev, SYS_RES_IRQ,
			scp->rid_irq, scp->res_irq);
		bus_release_resource(scp->dev, SYS_RES_IRQ,
			scp->rid_irq, scp->res_irq);
		scp->res_irq = 0;
	}
	if (scp->res_ioport != 0) {
		bus_deactivate_resource(scp->dev, SYS_RES_IOPORT,
			scp->rid_ioport, scp->res_ioport);
		bus_release_resource(scp->dev, SYS_RES_IOPORT,
			scp->rid_ioport, scp->res_ioport);
		scp->res_ioport = 0;
	}
}

static int
itecir_resource_int_value(struct itecir_softc* scp, const char* resname, int defval)
{
	int result;
	if (resource_int_value(device_get_name(scp->dev), device_get_unit(scp->dev), resname, &result) != 0)
		result = defval;
	return (result);
}

static void
itecir_intr(void *arg)
{
	struct itecir_softc *scp = (struct itecir_softc *)arg;
	int iir;
	int fifo;
	u_int8_t data;
	u_int8_t newbit;
	int bit;

	iir = INB(scp, IT87_CIR_IIR);
	switch (iir & IT87_CIR_IIR_II) {
	case IT87_CIR_IIR_RXDS:
	case IT87_CIR_IIR_RXFO:
		fifo = INB(scp, IT87_CIR_RSR) & IT87_CIR_RSR_RXFBC;
		while (fifo--) {
			data = INB(scp, IT87_CIR_DR);
			/* Loop through */
			for (bit = 0; bit < 8; ++bit) {
				newbit = (data >> bit) & 1;
				if (newbit != scp->lastbit) {
					lircdev_putev(scp->lirc, !scp->lastbit, pulse_duration(scp->acc));
					scp->acc = 0;
					scp->lastbit = newbit;
				}
				++scp->acc;
			}
		}
		if (scp->lastbit == 1 && pulse_duration(scp->acc) > ITE_IDLE_TIMEOUT) {
			scp->acc = 0;
			scp->lastbit = 0;
			itecir_idle_rx(scp);
		}
		lircdev_wakeup(scp->lirc);
		break;
	default:
		;
	}
}

static void
itecir_init_hw(struct itecir_softc * scp)
{
	OUTB(scp, IT87_CIR_IER, (INB(scp, IT87_CIR_IER) &
		~(IT87_CIR_IER_IEC | IT87_CIR_IER_RFOIE | IT87_CIR_IER_RDAIE | IT87_CIR_IER_TLDLIE)) | IT87_CIR_IER_BR);

	OUTB(scp, IT87_CIR_BDLR, ITE_BAUDRATE_DIVISOR & 0xff);
	OUTB(scp, IT87_CIR_BDHR, ITE_BAUDRATE_DIVISOR >> 8);

	OUTB(scp, IT87_CIR_IER, INB(scp, IT87_CIR_IER) & ~IT87_CIR_IER_BR);

	OUTB(scp, IT87_CIR_RCR, IT87_CIR_RCR_RXDCR_DEFAULT);

	OUTB(scp, IT87_CIR_TCR1, IT87_CIR_TCR1_TXMPM_DEFAULT | IT87_CIR_TCR1_TXENDF | IT87_CIR_TCR1_TXRLE | IT87_CIR_TCR1_FIFOTL_DEFAULT | IT87_CIR_TCR1_FIFOCLR);

	itecir_set_carrier(scp);
}

static void
itecir_shutdown_hw(struct itecir_softc * scp)
{
	OUTB(scp, IT87_CIR_IER, INB(scp, IT87_CIR_IER) & ~(IT87_CIR_IER_IEC | IT87_CIR_IER_RFOIE | IT87_CIR_IER_RDAIE | IT87_CIR_IER_TLDLIE));

	itecir_disable_rx(scp);

	OUTB(scp, IT87_CIR_TCR1, INB(scp, IT87_CIR_TCR1) | IT87_CIR_TCR1_FIFOCLR);
}

static void
itecir_set_carrier(struct itecir_softc * scp)
{
	unsigned int freq, low_freq, high_freq;
	u_int8_t allowance;
	bool use_demodulator;
	u_int8_t val;

	low_freq = itecir_resource_int_value(scp, "lowcarrierfreq", 0);
	high_freq = itecir_resource_int_value(scp, "highcarrierfreq", 0);

	if (low_freq == 0) {
		freq = itecir_default_freq;
		allowance = IT87_CIR_RCR_RXDCR_DEFAULT;
		use_demodulator = false;
	} else {
		freq = (low_freq + high_freq) / 2;
		allowance =
			DIV_ROUND_CLOSEST(10000 * (high_freq - low_freq),
				          ITE_RXDCR_PER_10000_STEP * (high_freq + low_freq));
		if (allowance < 1)
			allowance = 1;
		if (allowance > IT87_CIR_RCR_RXDCR_MAX)
			allowance = IT87_CIR_RCR_RXDCR_MAX;
		use_demodulator = false;
	}

	val = INB(scp, IT87_CIR_RCR) & ~(IT87_CIR_RCR_HCFS | IT87_CIR_RCR_RXEND | IT87_CIR_RCR_RXDCR);
	if (is_high_carrier_freq(freq))
		val |= IT87_CIR_RCR_HCFS;
	if (use_demodulator)
		val |= IT87_CIR_RCR_RXEND;
	val |= allowance;

	OUTB(scp, IT87_CIR_RCR, val);

	OUTB(scp, IT87_CIR_TCR2, (carrier_freq_bits(freq) << IT87_CIR_TCR2_CFQ_SHIFT) | pulse_width_bits(freq, 33));
}

static void
itecir_enable_rx(struct itecir_softc * scp)
{
	OUTB(scp, IT87_CIR_RCR, INB(scp, IT87_CIR_RCR) | IT87_CIR_RCR_RXEN);

	itecir_idle_rx(scp);

	OUTB(scp, IT87_CIR_IER, INB(scp, IT87_CIR_IER) | IT87_CIR_IER_RDAIE | IT87_CIR_IER_RFOIE | IT87_CIR_IER_IEC);
}

static void
itecir_disable_rx(struct itecir_softc * scp)
{
	OUTB(scp, IT87_CIR_IER, INB(scp, IT87_CIR_IER) & ~(IT87_CIR_IER_RDAIE | IT87_CIR_IER_RFOIE));

	OUTB(scp, IT87_CIR_RCR, INB(scp, IT87_CIR_RCR) & ~IT87_CIR_RCR_RXEN);

	itecir_idle_rx(scp);
}

static void
itecir_idle_rx(struct itecir_softc * scp)
{
	OUTB(scp, IT87_CIR_RCR, INB(scp, IT87_CIR_RCR) | IT87_CIR_RCR_RXACT);

	OUTB(scp, IT87_CIR_TCR1, INB(scp, IT87_CIR_TCR1) | IT87_CIR_TCR1_FIFOCLR);
}

static int
itecir_open(device_t dev)
{
	struct itecir_softc *scp = DEVICE2SOFTC(dev);

	itecir_enable_rx(scp);
	return (0);
}

static int
itecir_close(device_t dev)
{
	struct itecir_softc *scp = DEVICE2SOFTC(dev);

	itecir_shutdown_hw(scp);
	return (0);
}

static bool is_high_carrier_freq(unsigned int freq)
{
	return (freq >= ITE_HCF_MIN_CARRIER_FREQ);
}

static u_int8_t carrier_freq_bits(unsigned int freq)
{
	if (is_high_carrier_freq(freq)) {
		if (freq < 425000)
			return (ITE_CFQ_400);
		else if (freq < 465000)
			return (ITE_CFQ_450);
		else if (freq < 490000)
			return (ITE_CFQ_480);
		else
			return (ITE_CFQ_500);
	} else {
		if (freq < ITE_LCF_MIN_CARRIER_FREQ)
			freq = ITE_LCF_MIN_CARRIER_FREQ;
		if (freq > ITE_LCF_MAX_CARRIER_FREQ)
			freq = ITE_LCF_MAX_CARRIER_FREQ;

		freq = DIV_ROUND_CLOSEST(freq - ITE_LCF_MIN_CARRIER_FREQ, 1000);

		return (freq);
	}
}

static u_int8_t pulse_width_bits(unsigned int freq, int duty_cycle)
{
	unsigned long period_ns, on_ns;

	/* sanitize freq into range */
	if (freq < ITE_LCF_MIN_CARRIER_FREQ)
		freq = ITE_LCF_MIN_CARRIER_FREQ;
	if (freq > ITE_HCF_MAX_CARRIER_FREQ)
		freq = ITE_HCF_MAX_CARRIER_FREQ;

	period_ns = 1000000000UL / freq;
	on_ns = period_ns * duty_cycle / 100;

	if (is_high_carrier_freq(freq)) {
		if (on_ns < 750)
			return (ITE_TXMPW_A);
		else if (on_ns < 850)
			return (ITE_TXMPW_B);
		else if (on_ns < 950)
			return (ITE_TXMPW_C);
		else if (on_ns < 1080)
			return (ITE_TXMPW_D);
		else
			return (ITE_TXMPW_E);
	} else {
		if (on_ns < 6500)
			return (ITE_TXMPW_A);
		else if (on_ns < 7850)
			return (ITE_TXMPW_B);
		else if (on_ns < 9650)
			return (ITE_TXMPW_C);
		else if (on_ns < 11950)
			return (ITE_TXMPW_D);
		else
			return (ITE_TXMPW_E);
	}
}

static unsigned int
pulse_duration(unsigned long val)
{
	return ITE_BITS_TO_NS(val, 1000000000ull / itecir_default_baudrate);
}
