#ifndef _LIRC_DEV_H_
#define _LIRC_DEV_H_

struct lircdev;

typedef int lircdev_open_t(device_t device);
typedef int lircdev_close_t(device_t device);

struct lircdevsw {
	lircdev_open_t	*d_open;
	lircdev_close_t	*d_close;
};

struct lircdev	*lircdev_create(struct lircdevsw *sw, device_t device);
void		 lircdev_destroy(struct lircdev *dev);
void		 lircdev_putev(struct lircdev *dev, bool flag, unsigned long val);
void		 lircdev_wakeup(struct lircdev *dev);

#endif // _LIRC_DEV_H_
