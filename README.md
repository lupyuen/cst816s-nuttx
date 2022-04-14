# Hynitron CST816S Touch Controller Driver for Apache NuttX RTOS

[(Used by PineDio Stack BL604)](https://lupyuen.github.io/articles/pinedio2)

See https://lupyuen.github.io/articles/pinedio2#touch-panel

[__Follow the updates on Twitter__](https://twitter.com/MisterTechBlog/status/1514049092388745219)

# Install Driver

To add this repo to your NuttX project...

```bash
pushd nuttx/nuttx/drivers/input
git submodule add https://github.com/lupyuen/cst816s-nuttx cst816s
ln -s cst816s/cst816s.c .
popd

pushd nuttx/nuttx/include/nuttx/input
ln -s ../../../drivers/input/cst816s/cst816s.h .
popd
```

Next update the Makefile and Kconfig...

-   [See the modified Makefile and Kconfig](https://github.com/lupyuen/incubator-nuttx/commit/5dbf67df8f36cdba2eb0034dac0ff8ed0f8e73e1)

Then update the NuttX Build Config...

```bash
## TODO: Change this to the path of our "incubator-nuttx" folder
cd nuttx/nuttx

## Preserve the Build Config
cp .config ../config

## Erase the Build Config and Kconfig files
make distclean

## For BL602: Configure the build for BL602
./tools/configure.sh bl602evb:nsh

## For PineDio Stack BL604: Configure the build for BL604
./tools/configure.sh bl602evb:pinedio

## For ESP32: Configure the build for ESP32.
## TODO: Change "esp32-devkitc" to our ESP32 board.
./tools/configure.sh esp32-devkitc:nsh

## Restore the Build Config
cp ../config .config

## Edit the Build Config
make menuconfig 
```

In menuconfig, enable the Hynitron CST816S Driver under "Device Drivers → Input Device Support".

Edit the function `bl602_bringup` or `esp32_bringup` in this file...

```text
## For BL602:
nuttx/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c

## For ESP32: Change "esp32-devkitc" to our ESP32 board 
nuttx/boards/xtensa/esp32/esp32-devkitc/src/esp32_bringup.c
```

And call `cst816s_register` to load our driver:

https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_bringup.c#L826-L843

```c
#ifdef CONFIG_INPUT_CST816S
/* I2C Address of CST816S Touch Controller */

#define CST816S_DEVICE_ADDRESS 0x15
#include <nuttx/input/cst816s.h>
#endif /* CONFIG_INPUT_CST816S */
...
#ifdef CONFIG_INPUT_CST816S
int bl602_bringup(void)
{
  ...
  /* Init I2C bus for CST816S */

  struct i2c_master_s *cst816s_i2c_bus = bl602_i2cbus_initialize(0);
  if (!cst816s_i2c_bus)
    {
      _err("ERROR: Failed to get I2C%d interface\n", 0);
    }

  /* Register the CST816S driver */

  ret = cst816s_register("/dev/input0", cst816s_i2c_bus, CST816S_DEVICE_ADDRESS);
  if (ret < 0)
    {
      _err("ERROR: Failed to register CST816S\n");
    }
#endif /* CONFIG_INPUT_CST816S */
```

Here's how we created the CST816S Driver for NuttX on PineDio Stack BL604...

# Cypress MBR3108

NuttX Driver for Cypress MBR3108 Touch Controller looks structurally similar to PineDio Stack's CST816S ... So we copy-n-paste into our CST816S Driver

-   [NuttX Driver for Cypress MBR3108](https://github.com/lupyuen/incubator-nuttx/blob/master/drivers/input/cypress_mbr3108.c)

# I2C Scan

PineDio Stack's Touch Panel is a peculiar I2C Device ... It won't respond to I2C Scan unless we tap the screen and wake it up!

-   ["Building a Rust Driver for PineTime’s Touch Controller"](https://lupyuen.github.io/articles/building-a-rust-driver-for-pinetimes-touch-controller)

# GPIO Interrupt

PineDio Stack's Touch Panel triggers a GPIO Interrupt when we tap the screen ... Here's how we handle the GPIO Interrupt

```c
int cst816s_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c_dev,
                     uint8_t i2c_devaddr)
{
  ...
  /* Prepare interrupt line and handler. */

  ret = bl602_irq_attach(BOARD_TOUCH_INT, cst816s_isr_handler, priv);
  if (ret < 0)
    {
      kmm_free(priv);
      ierr("Attach interrupt failed\n");
      return ret;
    }

  ret = bl602_irq_enable(false);
  if (ret < 0)
    {
      kmm_free(priv);
      ierr("Disable interrupt failed\n");
      return ret;
    }
```

[(Source)](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L593-L661)

`bl602_irq_attach` is defined below...

```c
//  Attach Interrupt Handler to GPIO Interrupt for Touch Controller
//  Based on https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/src/bl602_gpio.c#L477-L505
static int bl602_irq_attach(gpio_pinset_t pinset, FAR isr_handler *callback, FAR void *arg)
{
  int ret = 0;
  uint8_t gpio_pin = (pinset & GPIO_PIN_MASK) >> GPIO_PIN_SHIFT;
  FAR struct bl602_gpint_dev_s *dev = NULL;  //  TODO

  DEBUGASSERT(callback != NULL);

  /* Configure the pin that will be used as interrupt input */

  #warning Check GLB_GPIO_INT_TRIG_NEG_PULSE  ////  TODO
  bl602_expander_set_intmod(gpio_pin, 1, GLB_GPIO_INT_TRIG_NEG_PULSE);
  ret = bl602_configgpio(pinset);
  if (ret < 0)
    {
      gpioerr("Failed to configure GPIO pin %d\n", gpio_pin);
      return ret;
    }

  /* Make sure the interrupt is disabled */

  bl602_expander_pinset = pinset;
  bl602_expander_callback = callback;
  bl602_expander_arg = arg;
  bl602_expander_intmask(gpio_pin, 1);

  irq_attach(BL602_IRQ_GPIO_INT0, bl602_expander_interrupt, dev);
  bl602_expander_intmask(gpio_pin, 0);

  gpioinfo("Attach %p\n", callback);

  return 0;
}
```

[(Source)](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L686-L727)

Note that we're calling `bl602_expander` to handle interrupts. There doesn't seem to be a way to do this with the current BL602 GPIO Driver (`bl602evb/bl602_gpio.c`).

We are building `bl602_expander` here...

-   [lupyuen/bl602_expander](https://github.com/lupyuen/bl602_expander)

To test interrupts we uncomment `#define TEST_CST816S_INTERRUPT`...

```c
int cst816s_register(FAR const char *devpath,
                     FAR struct i2c_master_s *i2c_dev,
                     uint8_t i2c_devaddr)
{
...
//  Uncomment this to test interrupts (tap the screen)
#define TEST_CST816S_INTERRUPT
#ifdef TEST_CST816S_INTERRUPT
#warning Testing CST816S interrupt
  bl602_irq_enable(true);
#endif /* TEST_CST816S_INTERRUPT */
```

[(Source)](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L593-L661)

There's bug with BL602 GPIO Interrupts that we have fixed for our driver...

https://github.com/apache/incubator-nuttx/issues/5810#issuecomment-1098633538

# Test GPIO Interrupt

Tapping the screen on PineDio Stack ... Correctly triggers a GPIO Interrupt 🎉

```text
gpio_pin_register: Registering /dev/gpio0
gpio_pin_register: Registering /dev/gpio1
gpint_enable: Disable the interrupt
gpio_pin_register: Registering /dev/gpio2
bl602_gpio_set_intmod: ****gpio_pin=115, int_ctlmod=1, int_trgmod=0
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
cst816s_register:
bl602_expander_set_intmod: ****gpio_pin=9, int_ctlmod=1, int_trgmod=0
bl602_irq_attach: Attach 0x2305e9de
bl602_irq_enable: Disable interrupt
cst816s_register: Driver registered
bl602_irq_enable: Enable interrupt

NuttShell (NSH) NuttX-10.2.0-RC0
nsh> bl602_expander_interrupt: Interrupt! callback=0x2305e9de, arg=0x42020a60
bl602_expander_interrupt: Call callback=0x2305e9de, arg=0x42020a60
cst816s_poll_notify:
bl602_expander_interrupt: Interrupt! callback=0x2305e9de, arg=0x42020a60
bl602_expander_interrupt: Call callback=0x2305e9de, arg=0x42020a60
cst816s_poll_notify:
bl602_expander_interrupt: Interrupt! callback=0x2305e9de, arg=0x42020a60
bl602_expander_interrupt: Call callback=0x2305e9de, arg=0x42020a60
cst816s_poll_notify:
bl602_expander_interrupt: Interrupt! callback=0x2305e9de, arg=0x42020a60
bl602_expander_interrupt: Call callback=0x2305e9de, arg=0x42020a60
cst816s_poll_notify:
```

LVGL Test App `lvgltest` fails to open `/dev/input0`, but that's OK because we haven't implemented the I2C part.

```text
nsh> ls /dev
/dev:
 console
 gpio0
 gpio1
 gpio2
 i2c0
 input0
 lcd0
 null
 spi0
 spitest0
 timer0
 urandom
 zero

nsh> lvgltest
tp_init: Opening /dev/input0
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_probe_device: family_id: 0x34, device_id: 0x00aa, device_rev: 35
cst816s_probe_device: Probe failed, dev-id mismatch!
cst816s_probe_device:   Expected: family_id: 0x9a, device_id: 0x0a03, device_rev: 1
tp_init: open /dev/input0 failed: 6
Terminating!
bl602_expander_interrupt: Interrupt! callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Call callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Interrupt! callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Call callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Interrupt! callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Call callback=0x2305e9e8, arg=0
```

# Read Touch Data

Here's how we read the Touched Coordinates in our driver...

```c
static int cst816s_get_touch_data(FAR struct cst816s_dev_s *dev, FAR void *buf)
{
  iinfo("\n"); ////
  struct touch_sample_s data;
  uint8_t readbuf[7];
  int ret;

  /* Read the raw touch data. */

  ret = cst816s_i2c_read(dev, CST816S_REG_TOUCHDATA, readbuf, sizeof(readbuf));
  if (ret < 0)
    {
      iinfo("Read touch data failed\n");
      return ret;
    }

  /* Interpret the raw touch data. */

  uint8_t idhigh = readbuf[0] & 0x0f;
  uint8_t idlow  = readbuf[1];
  uint8_t touchpoints = readbuf[2] & 0x0f;  /* Touch Points can only be 0 or 1. */
  uint8_t xhigh = readbuf[3] & 0x0f;
  uint8_t xlow  = readbuf[4];
  uint8_t yhigh = readbuf[5] & 0x0f;
  uint8_t ylow  = readbuf[6];
  uint16_t id = (idhigh << 8) | idlow;
  uint16_t x  = (xhigh  << 8) | xlow;
  uint16_t y  = (yhigh  << 8) | ylow;

  /* Validate the touch coordinates. */

  bool valid = true;
  if (x >= 240 || y >= 240) {
    valid = false;
    return -EINVAL;  /* Must not return invalid coordinates, because lvgldemo can't handle. */
    //  iwarn("Invalid touch data: x=%d, y=%d\n", x, y);
  }

  /* Set the touch data fields. */

  memset(&data, 0, sizeof(data));
  data.npoints     = 1;
  data.point[0].id = id;
  data.point[0].x  = x;
  data.point[0].y  = y;

  /* Set the touch flags. */

  if (touchpoints > 0)  /* Panel was touched. */
    {
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID;
        }
    }
  else  /* Panel was just untouched. */
    {
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }

  /* Return the touch data. */

  memcpy(buf, &data, sizeof(data));

  iinfo("  id:      %d\n",   data.point[0].id);
  iinfo("  flags:   %02x\n", data.point[0].flags);
  iinfo("  x:       %d\n",   data.point[0].x);
  iinfo("  y:       %d\n",   data.point[0].y);

  return sizeof(data);
}
```

[(Source)](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L213-L302)

# Test Touch Data

Touch Data from `lvgltest` looks OK!

```text
gpio_pin_register: Registering /dev/gpio0
gpio_pin_register: Registering /dev/gpio1
gpint_enable: Disable the interrupt
gpio_pin_register: Registering /dev/gpio2
bl602_gpio_set_intmod: ****gpio_pin=115, int_ctlmod=1, int_trgmod=0
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
cst816s_register: 
bl602_expander_set_intmod: ****gpio_pin=9, int_ctlmod=1, int_trgmod=0
bl602_irq_attach: Attach 0x2305e9de
bl602_irq_enable: Disable interrupt
cst816s_register: Driver registered

NuttShell (NSH) NuttX-10.2.0-RC0
nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open: 
...
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x0
bl602_i2c_recvdata: count=3, temp=0x0
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       0
cst816s_get_touch_data:   y:       0
...
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x0
bl602_i2c_recvdata: count=3, temp=0x0
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       0
cst816s_get_touch_data:   y:       0
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x800c00b5
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       181
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       180
cst816s_get_touch_data:   y:       12
...
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0xd400e6
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       230
cst816s_get_touch_data:   y:       212
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd400e5
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       229
cst816s_get_touch_data:   y:      212
...
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd400e5
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       229
cst816s_get_touch_data:   y:       212
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ca0013
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       202
...
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x800a0009
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       9
cst816s_get_touch_data:   y:       10
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xa0009
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       9
cst816s_get_touch_data:   y:       10
...
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xa0009
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       9
cst816s_get_touch_data:   y:       10
...
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x100
bl602_i2c_recvdata: count=3, temp=0x6d0001
transfer success
cst816s_get_touch_data:   id:      1
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       109
```

# Screen Is Sideways

According to the touch data from `lvgltest`, our screen is rotated sideways...

-   Top Left: x=181, y=12

-   Top Right: x=230, y=212

-   Bottom Left: x=9, y=10

-   Bottom Right: x=19, y=202

So be careful when mapping the touch coordinates.

We can rotate the display in the ST7789 Driver. But first we need to agree which way is "up"...

https://twitter.com/MisterTechBlog/status/1514438646568415232

# I2C Logging

The driver won't return any valid touch data unless we enable I2C Logging. Sounds like a I2C timing issue.

`lvgltest` reads the driver very often, which generates highly frequent I2C Reads for the touch data.

Maybe the driver should do an I2C Read of the touch data only upon GPIO Interrupt. And cache the touch data until the next GPIO Interrupt.

TODO
