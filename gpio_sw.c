// based on gpioled.c

#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bio.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <sys/kthread.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/uio.h>
#include <sys/systm.h>
#include <sys/fcntl.h> // FREAD FWRITE

//#include <dev/led/led.h>
#include <sys/gpio.h>
#include "gpio_if.h"
//#include "gpiobus_if.h"

// default pin, if not defined by hint
#define PIN_FOR_SW       24

#undef  DEBUG
//#define DEBUG

#ifdef DEBUG
#define duprintf(fmt, args...) do { uprintf("%s(): ", __func__);   \
    uprintf(fmt,##args); } while (0)
#else
#define duprintf(fmt, args...)
#endif

#define GPIO_SW_LOCK(_sc)               mtx_lock_spin(&(_sc)->sc_mtx)
#define GPIO_SW_UNLOCK(_sc)             mtx_unlock_spin(&(_sc)->sc_mtx)
#define GPIO_SW_LOCK_INIT(_sc) \
  mtx_init(&_sc->sc_mtx, device_get_nameunit(_sc->sc_dev), "gpio_sw", MTX_SPIN)
#define GPIO_SW_LOCK_DESTROY(_sc)       mtx_destroy(&_sc->sc_mtx)

struct gpio_sw_softc 
{
  device_t         sc_dev;
  device_t         sc_busdev;
  struct mtx       sc_mtx;
  struct cdev      *sc_ctl_dev;
  int              sc_unit;
  char             Buf[ 128] ;
  int              Len ;
  int              BufSize ; // sizeof( Buf)
  int              GpioStatus ; // 1 - success, 0 - fail
  int              Pin ;
};

static d_ioctl_t   sw_ioctl;
static d_open_t    sw_open;
static d_close_t   sw_close;
static d_read_t    sw_read;
static d_write_t   sw_write;

static struct cdevsw gpioc_cdevsw =
{
  .d_version = D_VERSION,
  .d_open    = sw_open,
  .d_close   = sw_close,
  .d_read    = sw_read,
  .d_write   = sw_write,
  .d_ioctl   = sw_ioctl,
  .d_name    = "sw",
} ;

//=============================================
// single wire methods
// return 1 - ok, 0 - fail
//=============================================

#define DHT11_OK       0
#define DHT11_NO_CONN  1
#define DHT11_CS_ERROR 2

// return 1 - success, 0 - time out.
static int WaitLevel( struct gpio_sw_softc *sc, unsigned short int *Count, unsigned int WaitingState)
{
  unsigned int Val ;
  *Count = 0 ;
  while ( *Count < 1000)
  {
    GPIO_PIN_GET( sc->sc_busdev, sc->Pin, &Val);
    if ( Val == WaitingState) return 1 ;
    DELAY( 1) ;
    *Count = *Count + 1 ;
  } ;
  return 0 ;
}

static int read_DHT11(struct gpio_sw_softc *sc, unsigned char *buf)
{
  //reset DHT11
  DELAY( 1) ;
  GPIO_PIN_SETFLAGS( sc->sc_busdev, sc->Pin, GPIO_PIN_OUTPUT) ;
  DELAY( 1) ;
  GPIO_PIN_SET( sc->sc_busdev, sc->Pin, 0);
  DELAY( 18000) ; // reset pulse
  GPIO_SW_LOCK( sc);
  GPIO_PIN_SET( sc->sc_busdev, sc->Pin, 1);
  DELAY( 1) ;
  GPIO_PIN_SETFLAGS( sc->sc_busdev, sc->Pin, GPIO_PIN_INPUT) ;
  DELAY( 1) ;

  unsigned short int Buf1[ 42];
  unsigned short int Buf0[ 42] ;
  unsigned short int Threshold ;
  unsigned char i;

  i = 0 ;
  while ( i < 42)
  {
    if ( ! WaitLevel(sc, &Buf1[ i], 0) ) break ;
    if ( ! WaitLevel(sc, &Buf0[ i], 1) ) break ;
    i = i + 1 ;
  } ;
  GPIO_SW_UNLOCK( sc);
  if ( i != 42) return DHT11_NO_CONN ;

  unsigned int Sum = 0 ;
  i = 0 ;
  while ( i < 42)
  {
    duprintf("i=%i 1=%i 0=%i\n", i, Buf1[ i], Buf0[ i]);
    Sum = Sum + Buf0[ i] ;
    i = i + 1 ;
  } ;
  Threshold = (unsigned short)(Sum/42) ;
  duprintf("Threshold=%i\n", Threshold);

  i = 2 ;
  int j = 0 ;
  unsigned char BitPos = 0x80 ;
  while ( i < 42)
  {
    if ( Buf1[ i] > Threshold) buf[ j] = buf[ j] | BitPos ;
    BitPos = BitPos >> 1 ;
    if ( BitPos == 0) { BitPos = 0x80 ; j = j + 1 ; } ;
    i = i + 1 ;
  } ;

  unsigned char CheckSum = buf[ 0] + buf[ 1] + buf[ 2] + buf[ 3] ;
  duprintf("sum=%x\n", CheckSum);

  if ( CheckSum != buf[ 4]) return DHT11_CS_ERROR;
  return DHT11_OK;      
}

//----------- End of single wire methods ----------------

//---------------------------------------------
static void gpio_sw_identify (driver_t *driver, device_t parent)
{
  devclass_t dc = devclass_find("gpio_sw");
  if ( dc == NULL) printf("devclass_find(gpio_sw) - fail.\n") ;
  else             printf("devclass_find(gpio_sw) - success.\n") ;
  if ( devclass_get_device(dc, 0) == NULL)
  {
    printf("devclass_get_device() - fail.\n") ;
    device_t child = device_add_child( parent, "gpio_sw", -1);
    if ( child == NULL) printf("device_add_child() - fail.\n") ;
    else                printf("device_add_child() - success.\n") ;
  }
  else printf("devclass_get_device(0) - success.\n") ;

  int Pin ;
  if ( resource_int_value( "gpio_sw", 1, "pin", &Pin) == 0)
  {
    if ( devclass_get_device(dc, 1) == NULL)
    {
      printf("devclass_get_device() - fail.\n") ;
      device_t child = device_add_child( parent, "gpio_sw", -1);
      if ( child == NULL) printf("device_add_child() - fail.\n") ;
      else                printf("device_add_child() - success.\n") ;
    }
    else printf("devclass_get_device(1) - success.\n") ;
  } ;
} ;

//---------------------------------------------
static int gpio_sw_probe(device_t dev)
{
  device_set_desc(dev, "SingleWire over GPIO");
  return 0 ;
}

//---------------------------------------------
static int gpio_sw_attach( device_t dev)
{
  struct gpio_sw_softc *sc;

  sc = device_get_softc( dev);
  sc->sc_dev = dev;
  sc->sc_busdev = device_get_parent( dev);
  sc->sc_unit = device_get_unit( dev);
  GPIO_SW_LOCK_INIT(sc);
  sc->BufSize = 128 ;
  // get a value from the hints mechanism
  if ( resource_int_value( device_get_name(dev), sc->sc_unit, "pin", &sc->Pin) != 0 ) sc->Pin = PIN_FOR_SW ;
  printf("SingleWire over GPIO pin=%i\n", sc->Pin) ;

  sc->sc_ctl_dev = make_dev(&gpioc_cdevsw, sc->sc_unit,
            UID_ROOT, GID_WHEEL, 0600, "sw%d", sc->sc_unit);
  if ( sc->sc_ctl_dev != NULL)
  {
    printf("Created sw%d\n", sc->sc_unit) ;
    sc->sc_ctl_dev->si_drv1 = sc ; // сохраняем sc в sc_ctl_dev
  }
  else printf("Failed to create sw%d\n", sc->sc_unit) ;

  // GPIO_PIN_INPUT  GPIO_PIN_OUTPUT  GPIO_PIN_PULLDOWN  GPIO_PIN_PULLUP
  sc->GpioStatus = 1 ;
  if ( GPIO_PIN_SETFLAGS( sc->sc_busdev, sc->Pin, GPIO_PIN_INPUT) != 0)
  {
    printf("GPIO_PIN_SETFLAGS() error, pin=%i\n", sc->Pin) ;
    sc->GpioStatus = 0 ;
  } ;

  return 0 ;
}

//---------------------------------------------
static int gpio_sw_detach( device_t dev)
{
  struct gpio_sw_softc *sc = device_get_softc(dev);
  if ( sc->sc_ctl_dev != NULL)
  {
     printf("destroy sw%d\n", sc->sc_unit) ;
     destroy_dev(sc->sc_ctl_dev);
     sc->sc_ctl_dev = NULL;
  } ;
  GPIO_SW_LOCK_DESTROY(sc);
  return 0 ;
}


//---------------------------------------------
static int sw_ioctl(struct cdev *cdev, u_long cmd, caddr_t arg, int fflag, 
    struct thread *td)
{
//  struct gpio_sw_softc *sc = cdev->si_drv1;
  return (0);
}

//---------------------------------------------
static int sw_open(struct cdev *cdev __unused, int oflags, int devtype __unused,
    struct thread *td __unused)
{
  return 0 ;
} ;

//---------------------------------------------
static int sw_close(struct cdev *cdev __unused, int fflag __unused, int devtype __unused,
    struct thread *td __unused)
{
  return 0 ;
}

//---------------------------------------------
// The read function just takes the buf that was saved via
// echo_write() and returns it to userland for accessing.
// uio(9)
static int sw_read( struct cdev *cdev, struct uio *uio, int ioflag __unused)
{
  duprintf("read - start, uio_resid=%i\n", uio->uio_resid);
  struct gpio_sw_softc *sc = cdev->si_drv1 ;
  if ( uio->uio_resid >= sc->BufSize) sc->Len=sc->BufSize-1 ; // to work with buffered IO
  else sc->Len=uio->uio_resid;

  if ( ! sc->GpioStatus) return ENXIO ;
  int i = 0 ;
  while ( i < sc->Len)
  {
    sc->Buf[ i] = 0 ;
    i = i + 1 ;
  } ;
  int RetVal = read_DHT11( sc, sc->Buf) ;
  duprintf(" read_DHT11=%i  %x %x %x %x %x\n", RetVal, sc->Buf[ 0], sc->Buf[ 1], sc->Buf[ 2], sc->Buf[ 3], sc->Buf[ 4] ) ;
  if ( RetVal == DHT11_NO_CONN) duprintf("Device not present.\n") ;
  if ( RetVal == DHT11_CS_ERROR) duprintf("Check sum error.\n") ;
  if ( RetVal != 0) return EIO ;
  int error = uiomove( sc->Buf, sc->Len, uio) ;
  if ( error != 0) { duprintf("uiomove failed!\n") ; return error ; } ;
  return 0 ;
}

//---------------------------------------------
static int sw_write(struct cdev *cdev, struct uio *uio, int ioflag __unused)
{
  return 0 ;
} ;


//---------------------------------------------
static devclass_t gpio_sw_devclass;

static device_method_t gpio_sw_methods[] = {
        /* Device interface */
        DEVMETHOD(device_identify,      gpio_sw_identify),
        DEVMETHOD(device_probe,         gpio_sw_probe),
        DEVMETHOD(device_attach,        gpio_sw_attach),
        DEVMETHOD(device_detach,        gpio_sw_detach),

        { 0, 0 }
};

static driver_t gpio_sw_driver = 
{
  "gpio_sw",
  gpio_sw_methods,
  sizeof(struct gpio_sw_softc),
};

DRIVER_MODULE(gpio_sw, gpio, gpio_sw_driver, gpio_sw_devclass, 0, 0);

