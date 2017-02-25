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
#include <sys/conf.h>
#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/mutex.h>
#include <sys/condvar.h>
#include <sys/selinfo.h>
#include <sys/proc.h>
#include <sys/fcntl.h>
#include <sys/filio.h>
#include <sys/uio.h>
#include <sys/poll.h>
#include <sys/bus.h>
#include <machine/stdarg.h>

#include "lirc_dev.h"
#include "lirc.h"

#define DEV2SOFTC(dev)	((struct lircdev *) (dev)->si_drv1)
#define BUFFERSIZE	(512)

struct lircdev {
	struct lircdevsw	*sw;
	device_t		dev;
	struct cdev		*cdev;
	struct selinfo		rsel;
	struct sigio		*sigio;
	bool			async;

	struct mtx		rx_mtx;
	struct cv		rx_cv;
	lirc_t			*rx_buffer;
	unsigned int		rx_tail;
	unsigned int		rx_head;
	bool			overrun;
};

static int		lirc_loader(struct module *m, int what, void *arg);
static int		lircdev_printf(struct lircdev* scp, const char * fmt, ...);
static d_open_t		lirc_open;
static d_close_t	lirc_close;
static d_read_t		lirc_read;
static d_write_t	lirc_write;
static d_ioctl_t	lirc_ioctl;
static d_poll_t		lirc_poll;

static MALLOC_DEFINE(M_LIRC, "lirc", "lirc device");
static int itecir_unit = 0;

/* Tunables */
static int itecir_rx_buffersize = BUFFERSIZE;
TUNABLE_INT("hw.lirc.rx_buffersize", &itecir_rx_buffersize);

static struct cdevsw lirc_cdevsw = {
	.d_version =	D_VERSION,
	.d_open =	lirc_open,
	.d_close =	lirc_close,
	.d_read =	lirc_read,
	.d_write =	lirc_write,
	.d_ioctl =	lirc_ioctl,
	.d_poll =	lirc_poll,
        .d_name =	"lirc"
};

static moduledata_t lirc_mod = {
	"lirc",
	lirc_loader,
	NULL
};

DECLARE_MODULE(lirc, lirc_mod, SI_SUB_KLD, SI_ORDER_ANY);
MODULE_VERSION(lirc, 1);

static int
lirc_loader(struct module *m, int what, void *arg)
{
	switch (what) {
	case MOD_LOAD:
		printf("LIRC device support loaded\n");
		break;
	case MOD_UNLOAD:
		break;
	default:
		return (EOPNOTSUPP);
	}
	return (0);
}

static int
lircdev_printf(struct lircdev* scp, const char * fmt, ...)
{
	va_list ap;
	int retval;

	retval = printf("%s: ", devtoname(scp->cdev));
	va_start(ap, fmt);
	retval += vprintf(fmt, ap);
	va_end(ap);
	return (retval);
}

struct lircdev*
lircdev_create(struct lircdevsw *sw, device_t dev)
{
	struct lircdev *scp;

	scp = malloc(sizeof(struct lircdev), M_LIRC, M_WAITOK | M_ZERO);
	scp->sw = sw;
	scp->dev = dev;
	scp->rx_buffer = malloc(sizeof (scp->rx_buffer[0]) * itecir_rx_buffersize, M_LIRC, M_WAITOK);
	mtx_init(&scp->rx_mtx, "lirc_lk", NULL, MTX_DEF);
	cv_init(&scp->rx_cv, "lirc_cv");
	scp->cdev = make_dev(&lirc_cdevsw, 0,
		UID_ROOT, GID_OPERATOR, 0664, "lirc%d", itecir_unit++);
	scp->cdev->si_drv1 = scp;

	lircdev_printf(scp, "<Infrared Remote Control> on %s\n", device_get_nameunit(dev));

	return (scp);
}

void
lircdev_destroy(struct lircdev* scp)
{
	if (scp->cdev != NULL) {
		destroy_dev(scp->cdev);
		scp->cdev = NULL;
	}
	cv_destroy(&scp->rx_cv);
	mtx_destroy(&scp->rx_mtx);
	free(scp->rx_buffer, M_LIRC);
	free(scp, M_LIRC);
}

void
lircdev_putev(struct lircdev* scp, bool flag, unsigned long val)
{
	unsigned int new_rx_tail;
	lirc_t newval;

	newval = (val / 1000) & PULSE_MASK;
	if (flag)
		newval |= PULSE_BIT;

	mtx_lock(&scp->rx_mtx);
	new_rx_tail = (scp->rx_tail + 1) % itecir_rx_buffersize;
	if (new_rx_tail == scp->rx_head && !scp->overrun) {
		scp->overrun = true;
		mtx_unlock(&scp->rx_mtx);
		lircdev_printf(scp, "buffer overflow\n");
		return;
	}
	scp->rx_buffer[scp->rx_tail] = newval;
	scp->rx_tail = new_rx_tail;
	mtx_unlock(&scp->rx_mtx);
}

void
lircdev_wakeup(struct lircdev* scp)
{
	if (scp->async && scp->sigio != NULL)
		pgsigio(&scp->sigio, SIGIO, 0);
	cv_broadcast(&scp->rx_cv);
	selwakeup(&scp->rsel);
}

static int
lirc_open(struct cdev* dev, int oflags, int devtype, struct thread *td)
{
	struct lircdev *scp = DEV2SOFTC(dev);

	scp->async = false;
	scp->sw->d_open(scp->dev);
	return (0);
}

static int
lirc_close(struct cdev* dev, int fflag, int devtype, struct thread *td)
{
	struct lircdev *scp = DEV2SOFTC(dev);

	scp->sw->d_close(scp->dev);
	funsetown(&scp->sigio);
	return (0);
}

static int
lirc_ioctl(struct cdev* dev, u_long cmd, caddr_t data, int flag, struct thread *td)
{
	struct lircdev *scp = DEV2SOFTC(dev);
	unsigned long value = 0;

	switch (cmd) {
	case FIOSETOWN:
		return (fsetown(*(int *)data, &scp->sigio));
	case FIOGETOWN:
		*(int *)data = fgetown(&scp->sigio);
		return (0);
	case FIOASYNC:
		scp->async = !!*(int *)data;
		/* FALLTHROUGH */
	case FIONBIO:
		return (0);
	case LIRC_GET_FEATURES:
		*(unsigned long*)data = LIRC_CAN_REC_MODE2;
		break;
	case LIRC_GET_REC_MODE:
		*(unsigned long*)data = LIRC_MODE_MODE2;
		break;
	case LIRC_SET_REC_MODE:
		value = *(unsigned long*)data;
		if (value != LIRC_MODE_MODE2)
			return (ENOSYS);
		break;
	default:
		return (ENXIO);
	}
	return (0);
}

static int
lirc_read(struct cdev* dev, struct uio *uio, int ioflag)
{
	struct lircdev *scp = DEV2SOFTC(dev);
	void* p;
	int read = 0;
	int toread;
	int error = 0;

	mtx_lock(&scp->rx_mtx);
	while (uio->uio_resid > 0) {
		toread = min(uio->uio_resid / sizeof (scp->rx_buffer[0]),
			(scp->rx_head <= scp->rx_tail ? scp->rx_tail : itecir_rx_buffersize) - scp->rx_head);
		if (toread > 0) {
			p = scp->rx_buffer + scp->rx_head;
			mtx_unlock(&scp->rx_mtx);
			error = uiomove(p, toread*sizeof (scp->rx_buffer[0]), uio);
			mtx_lock(&scp->rx_mtx);
			if (error)
				break;
			scp->rx_head = (scp->rx_head + toread) % itecir_rx_buffersize;
			read += toread;
		} else {
			if (read > 0)
				break;
			if (ioflag & O_NONBLOCK) {
				error = EWOULDBLOCK;
				break;
			}
			error = cv_wait_sig(&scp->rx_cv, &scp->rx_mtx);
			if (error)
				break;
		}
	}
	mtx_unlock(&scp->rx_mtx);
	return (error);
}

static int
lirc_write(struct cdev* dev, struct uio *uio, int ioflag)
{
	return (EOPNOTSUPP);
}

static int
lirc_poll(struct cdev* dev, int which, struct thread *td)
{
	struct lircdev *scp = DEV2SOFTC(dev);
	int revents = 0;

	mtx_lock(&scp->rx_mtx);
	if (which & (POLLIN | POLLRDNORM)) {
		if (scp->rx_head != scp->rx_tail)
			revents |= which & (POLLIN | POLLRDNORM);
		else
			selrecord(td, &scp->rsel);
	}
	mtx_unlock(&scp->rx_mtx);
	return (revents);
}
