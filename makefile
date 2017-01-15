SRCS=device_if.h bus_if.h gpio_if.h gpiobus_if.h gpio_sw.c
KMOD=gpio_sw

.include <bsd.kmod.mk>

