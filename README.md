![Touch Panel Calibration for Pine64 PineDio Stack BL604 RISC-V Board](https://lupyuen.github.io/images/touch-title.jpg)

# Hynitron CST816S Touch Controller Driver for Apache NuttX RTOS

[(Used by PineDio Stack BL604)](https://lupyuen.github.io/articles/pinedio2)

Read the article...

-   ["NuttX Touch Panel Driver for PineDio Stack BL604"](https://lupyuen.github.io/articles/touch)

Watch the demo...

-   [PineDio Stack Demo on YouTube](https://www.youtube.com/shorts/2Nzjrlp5lcE)

# Install Driver

If you're using NuttX on PineDio Stack, there's no need to install the driver...

-   [lupyuen/incubator-nuttx (pinedio branch)](https://github.com/lupyuen/incubator-nuttx/tree/pinedio)

-   [lupyuen/incubator-nuttx-apps (pinedio branch)](https://github.com/lupyuen/incubator-nuttx-apps/tree/pinedio)

Otherwise to add this repo to your NuttX project...

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

Enable I2C Warnings because of the [I2C Workaround for CST816S](https://github.com/lupyuen/cst816s-nuttx#i2c-logging)...

-   Click "Build Setup" → "Debug Options"

-   Check the boxes for the following...
    -   Enable Warnings Output
    -   I2C Warnings Output

-   (Optional) To enable logging for the CST816S Driver, check the boxes for...
    -   Enable Error Output
    -   Enable Informational Debug Output
    -   Enable Debug Assertions
    -   Input Device Error Output
    -   Input Device Warnings Output
    -   Input Device Informational Output

-   Note that "Enable Informational Debug Output" must be unchecked for the LoRaWAN Test App `lorawan_test` to work (because the LoRaWAN Timers are time-sensitive)

Edit the function [`bl602_i2c_transfer`](https://github.com/lupyuen/incubator-nuttx/blob/touch/arch/risc-v/src/bl602/bl602_i2c.c#L671-L773) and apply this workaround patch...

-   ["I2C Logging"](https://github.com/lupyuen/cst816s-nuttx#i2c-logging)

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

# Touch Data

Apache NuttX RTOS has a standard data format for Touch Panels ... Let's implement this for PineDio Stack

```c
/* This structure contains information about a single touch point.
 * Positional units are device specific.
 */

struct touch_point_s
{
  uint8_t  id;        /* Unique identifies contact; Same in all reports for the contact */
  uint8_t  flags;     /* See TOUCH_* definitions above */
  int16_t  x;         /* X coordinate of the touch point (uncalibrated) */
  int16_t  y;         /* Y coordinate of the touch point (uncalibrated) */
  int16_t  h;         /* Height of touch point (uncalibrated) */
  int16_t  w;         /* Width of touch point (uncalibrated) */
  uint16_t gesture;   /* Gesture of touchscreen contact */
  uint16_t pressure;  /* Touch pressure */
  uint64_t timestamp; /* Touch event time stamp, in microseconds */
};

/* The typical touchscreen driver is a read-only, input character device
 * driver.the driver write() method is not supported and any attempt to
 * open the driver in any mode other than read-only will fail.
 *
 * Data read from the touchscreen device consists only of touch events and
 * touch sample data.  This is reflected by struct touch_sample_s.  This
 * structure is returned by either the driver read method.
 *
 * On some devices, multiple touchpoints may be supported. So this top level
 * data structure is a struct touch_sample_s that "contains" a set of touch
 * points.  Each touch point is managed individually using an ID that
 * identifies a touch from first contact until the end of the contact.
 */

struct touch_sample_s
{
  int npoints;                   /* The number of touch points in point[] */
  struct touch_point_s point[1]; /* Actual dimension is npoints */
};
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/touch/include/nuttx/input/touchscreen.h#L113-L148)

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

  uint8_t id = readbuf[5] >> 4;
  uint8_t touchpoints = readbuf[2] & 0x0f;
  uint8_t xhigh = readbuf[3] & 0x0f;
  uint8_t xlow  = readbuf[4];
  uint8_t yhigh = readbuf[5] & 0x0f;
  uint8_t ylow  = readbuf[6];
  uint8_t event = readbuf[3] >> 6;  /* 0 = Touch Down, 1 = Touch Up, 2 = Contact */
  uint16_t x  = (xhigh  << 8) | xlow;
  uint16_t y  = (yhigh  << 8) | ylow;

  /* If touch coordinates are invalid, return the last valid coordinates. */

  bool valid = true;
  if (x >= 240 || y >= 240)
    {
      iwarn("Invalid touch data: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      if (last_event == 0xff)  /* Quit if we have no last valid coordinates. */
        {
          ierr("Can't return touch data: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
          return -EINVAL;
        }
      valid = false;
      id = last_id;
      x  = last_x;
      y  = last_y;
    }

  /* Remember the last valid touch data. */

  last_event = event;
  last_id    = id;
  last_x     = x;
  last_y     = y;

  /* Set the touch data fields. */

  memset(&data, 0, sizeof(data));
  data.npoints     = 1;
  data.point[0].id = id;
  data.point[0].x  = x;
  data.point[0].y  = y;

  /* Set the touch flags. */

  if (event == 0)  /* Touch Down */
    {
      iinfo("DOWN: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_DOWN | TOUCH_ID_VALID;
        }
    }
  else if (event == 1)  /* Touch Up */
    {
      iinfo("UP: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      if (valid)  /* Touch coordinates were valid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID | TOUCH_POS_VALID;
        }
      else  /* Touch coordinates were invalid. */
        {
          data.point[0].flags  = TOUCH_UP | TOUCH_ID_VALID;
        }
    }
  else  /* Reject Contact */
    {
      iinfo("CONTACT: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      return -EINVAL;
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

Note that our NuttX Driver for PineDio Stack's Touch Panel returns 4 possible states: Touch Down vs Touch Up, Valid vs Invalid.

We got this code thanks to JF's CST816S driver for the Self-Test Firmware...

-   [pinedio-stack-selftest/drivers/cst816s.c](https://codeberg.org/JF002/pinedio-stack-selftest/src/branch/master/drivers/cst816s.c)

And from our previous work on PineTime, which also uses CST816S...

-   ["Building a Rust Driver for PineTime’s Touch Controller"](https://lupyuen.github.io/articles/building-a-rust-driver-for-pinetimes-touch-controller)

-   [CST816S Driver in Rust](https://github.com/lupyuen/stm32bluepill-mynewt-sensor/blob/pinetime/rust/app/src/touch_sensor.rs)

-   [Hynitron Reference Driver](https://github.com/lupyuen/hynitron_i2c_cst0xxse/blob/master/cst0xx_core.c#L407-L466)

# Test Touch Data

NuttX Driver for PineDio Stack Touch Panel responds correctly to touch! 🎉

PineDio Stack Touch Screen feels laggy on Apache #NuttX RTOS right now ... 2 things we can fix: 1️⃣ Increase SPI Frequency 2️⃣ Switch to SPI DMA eventually

-   [Watch the demo on YouTube](https://www.youtube.com/shorts/2Nzjrlp5lcE)

[(UPDATE: We have bumped up the SPI Frequency to max 40 MHz, still feels laggy)](https://github.com/lupyuen/incubator-nuttx/blob/touch/boards/risc-v/bl602/bl602evb/configs/pinedio/defconfig#L580)

Here's the detailed log...

```text
gpio_pin_register: Registering /dev/gpio0
gpio_pin_register: Registering /dev/gpio1
gpint_enable: Disable the interrupt
gpio_pin_register: Registering /dev/gpio2
bl602_gpio_set_intmod: ****gpio_pin=115, int_ctlmod=1, int_trgmod=0
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
cst816s_register: path=/dev/input0, addr=21
bl602_expander_set_intmod: gpio_pin=9, int_ctlmod=1, int_trgmod=0
bl602_irq_attach: Attach 0x2305e596
bl602_irq_enable: Disable interrupt
cst816s_register: Driver registered
bl602_irq_enable: Enable interrupt

NuttShell (NSH) NuttX-10.2.0-RC0
nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open:

bl602_expander_interrupt: Interrupt! callback=0x2305e596, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e596, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1700de
ransfer success
cst816s_get_touch_data: DOWN: id=0,touch=0, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1700de
ransfer success
cst816s_get_touch_data: DOWN: id=0, ouch=0, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1700de
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
cst816s_get_touch_data: UP: id=0, touch=2, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23
bl602_expander_interrupt: Interrupt! callback=0x2305e596, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e596, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd900db
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=219, y=217
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       219
cst816s_get_touch_data:   y:       217

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd900db
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=219, y=217
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:  19
cst816s_get_touch_data:   x:       219
cst816s_get_touch_data:   y:       217

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=636, y=3330
cst816s_get_touch_data: UP: id=0, touch=2, x=219, y=217
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       219
cst816s_get_touch_data:   y:       217
bl602_expander_interrupt: Interrupt! callback=0x2305e596, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e596, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xdb0022
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=34, y=219
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       34
cst816s_get_touch_data:   y:       219

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xdb0022
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=34, y=219
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       34
cst816s_get_touch_data:   y:       219

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=636, y=3330
cst816s_get_touch_data: UP: id=0, touch=2, x=34, y=219
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       34
cst816s_get_touch_data:   y:       219
bl602_expander_interrupt: Interrupt! callback=0x2305e596, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e596, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x180018
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=24, y=24
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       24
cst816s_get_touch_data:   y:       24

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x180018
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=24, y=24
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       24
cst816s_get_touch_data:   y:       24

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=636, y=3330
cst816s_get_touch_data: UP: id=0, touch=2, x=24, y=24
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       24
cst816s_get_touch_data:   y:       24
bl602_expander_interrupt: Interrupt! callback=0x2305e596, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e596, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x8d0076
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=118, y=141
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       118
cst816s_get_touch_data:   y:       141

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x8d0076
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=118, y=141
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       118
cst816s_get_touch_data:   y:       141

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=4, touch=2, x=636, y=3330
cst816s_get_touch_data: UP: id=0, touch=2, x=118, y=141
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       118
cst816s_get_touch_data:   y:       141

tp_cal result
offset x:23, y:24
range x:194, y:198
invert x/y:1, x:0, y:1
```

Let's break down the log...

## Enable GPIO Interrupt

At NuttX Startup, we register the CST816S Driver as `/dev/input0` and enable the GPIO interrupt...

```text
gpio_pin_register: Registering /dev/gpio0
gpio_pin_register: Registering /dev/gpio1
gpint_enable: Disable the interrupt
gpio_pin_register: Registering /dev/gpio2
bl602_gpio_set_intmod: ****gpio_pin=115, int_ctlmod=1, int_trgmod=0
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
cst816s_register: path=/dev/input0, addr=21
bl602_expander_set_intmod: gpio_pin=9, int_ctlmod=1, int_trgmod=0
bl602_irq_attach: Attach 0x2305e596
bl602_irq_enable: Disable interrupt
cst816s_register: Driver registered
bl602_irq_enable: Enable interrupt

NuttShell (NSH) NuttX-10.2.0-RC0
nsh>
```

## Start LVGL App

We run the LVGL Test App `lvgltest`...

```text
nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open:
```

Which calls [`cst816s_open()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L384-L420) to open our CST816S Driver.

The app begins the Touchscreen Calibration process.

## Read Touch Data

The LVGL Test App calls [`cst816s_read()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L328-L382) repeatedly on the CST816S Driver to get Touch Data...

```c
bool tp_read(struct _lv_indev_drv_t *indev_drv, lv_indev_data_t *data)
{
  ...
  /* Read one sample */

  nbytes = read(fd, &sample, sizeof(struct touch_sample_s));
```

[(Source)](https://github.com/lupyuen/lvgltest-nuttx/blob/main/tp.c#L115-L132)

Since the screen hasn't been touched and we have no Touch Data yet, our driver returns an error `-EINVAL`...

```c
static ssize_t cst816s_read(FAR struct file *filep, FAR char *buffer,
                            size_t buflen)
{
  ...
  int ret = -EINVAL;

  /* Read the touch data, only if screen has been touched or if we're waiting for touch up */
  if ((priv->int_pending || last_event == 0) && buflen >= outlen)
    {
      ret = cst816s_get_touch_data(priv, buffer);
    }
```

[(Source)](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L336-L370)

`int_pending` becomes true when a GPIO Interrupt gets triggered later.

`last_event` becomes 0 when we get a Touch Down event later.

_Why do we check `int_pending`?_

To reduce contention on the I2C Bus, we only read the Touch Data over I2C when the screen has been touched. We'll see this in a while.

(But the LVGL Test App really shouldn't call `read()` repeatedly. It ought to call `poll()` and block until Touch Data is available)

_Why do we we check `last_event`?_

The Touch Controller triggers a GPIO Interrupt only upon Touch Down, not on Touch Up.

So after Touch Down, we allow  [`cst816s_read()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L328-L382) to call `cst816s_get_touch_data()` to fetch the Touch Data repeatedly, until we see the Touch Up Event. We'll see this in a while.

## Trigger GPIO Interrupt

We touch the screen and trigger a GPIO Interrupt...

```text
bl602_expander_interrupt: Interrupt! callback=0x2305e596, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e596, arg=0x42020a70
cst816s_poll_notify:
```

The Interrupt Handler in our driver sets `int_pending` to true...

```c
static int cst816s_isr_handler(int _irq, FAR void *_context, FAR void *arg)
{
  FAR struct cst816s_dev_s *priv = (FAR struct cst816s_dev_s *)arg;
  irqstate_t flags;

  DEBUGASSERT(priv != NULL);

  flags = enter_critical_section();
  priv->int_pending = true;
  leave_critical_section(flags);

  cst816s_poll_notify(priv);
  return 0;
}
```

[(Source)](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L598-L611)

And calls [`cst816s_poll_notify()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L472-L498) to unblock all `poll()` callers and notify them that Touch Data is available.

(But LVGL Test App doesn't `poll()` our driver, so this doesn't effect anything)

## Touch Down Event

Remember that the LVGL Test App keeps calling [`cst816s_read()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L328-L382) repeatedly to get Touch Data.

Now that `int_pending` is true, our driver proceeds to call [`cst816s_get_touch_data()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L222-L326) and fetch the Touch Data over I2C...

```text
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1700de
ransfer success
cst816s_get_touch_data: DOWN: id=0,touch=0, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23
```

The Touch Data that was read from CST816S over I2C...

```text
cst816s_get_touch_data: DOWN: id=0,touch=0, x=222, y=23
```

Gets returned directly to the LVGL Test App as a Touch Down Event...

```text
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23
```

[`cst816s_get_touch_data()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L222-L326) sets `last_event` to 0 because it's a Touch Down Event.

[`cst816s_read()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L372-L382) sets `int_pending` to false.

## Touch Down Event Again

LVGL Test App is still calling [`cst816s_read()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L328-L382) repeatedly to get Touch Data.

Now that `last_event` is 0 (Touch Down), our driver proceeds to call [`cst816s_get_touch_data()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L222-L326) and fetch the Touch Data over I2C...

```text
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1700de
ransfer success
cst816s_get_touch_data: DOWN: id=0, ouch=0, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1700de
ransfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23
```

This happens twice because we haven't received a Touch Up Event.

## Touch Up Event

When our finger is no longer touching the screen, [`cst816s_get_touch_data()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L222-L326) receives a Touch Up Event...

```text
cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
cst816s_get_touch_data: UP: id=0, touch=2, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23
```

For Touch Up Events the Touch Coordinates are invalid...

```text
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
```

The driver patches the Touch Coordinates with the data from the last Touch Down Event...

```text
cst816s_get_touch_data: UP: id=0, touch=2, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23
```

And returns the valid coordinates to the LVGL Test App. The patching is done here...

```c
static int cst816s_get_touch_data(FAR struct cst816s_dev_s *dev, FAR void *buf) {
...
  /* If touch coordinates are invalid, return the last valid coordinates. */

  bool valid = true;
  if (x >= 240 || y >= 240)
    {
      iwarn("Invalid touch data: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
      if (last_event == 0xff)  /* Quit if we have no last valid coordinates. */
        {
          ierr("Can't return touch data: id=%d, touch=%d, x=%d, y=%d\n", id, touchpoints, x, y);
          return -EINVAL;
        }
      valid = false;
      id = last_id;
      x  = last_x;
      y  = last_y;
    }

  /* Remember the last valid touch data. */

  last_event = event;
  last_id    = id;
  last_x     = x;
  last_y     = y;

  /* Set the touch data fields. */

  memset(&data, 0, sizeof(data));
  data.npoints     = 1;
  data.point[0].id = id;
  data.point[0].x  = x;
  data.point[0].y  = y;
```

[(Source)](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L258-L282)

`last_event` is now set to 1 (Touch Up). 

[`cst816s_read()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L328-L382) will no longer call [`cst816s_get_touch_data()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L222-L326) to fetch the Touch Data, until the screen is touched again.

## Screen Calibration Result

When we have touched the 4 screen corners, the LVGL Test App displays the Screen Calibration result...

```text
tp_cal result
offset x:23, y:24
range x:194, y:198
invert x/y:1, x:0, y:1
```

Which will be used to tweak the Touch Coordinates in the apps.

# Screen Is Sideways

According to the Touch Data from the LVGL Test App, our screen is rotated sideways...

-   Top Left: x=181, y=12

-   Top Right: x=230, y=212

-   Bottom Left: x=9, y=10

-   Bottom Right: x=19, y=202

So be careful when mapping the touch coordinates.

We can rotate the display in the ST7789 Driver. But first we need to agree which way is "up"...

https://twitter.com/MisterTechBlog/status/1514438646568415232

# I2C Logging

[`cst816s_get_touch_data()`](https://github.com/lupyuen/cst816s-nuttx/blob/main/cst816s.c#L222-L326) won't return any valid Touch Data unless we enable I2C Logging. Could be an I2C Timing Issue or Race Condition.

With I2C Logging Enabled: We get the Touch Down Event (with valid Touch Data)...

```text
nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open:

bl602_expander_interrupt: Interrupt! callback=0x2305e596, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e596, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c tbl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1700de
Transfer success
cst816s_get_touch_data: DOWN: id=0,touch=0, x=222, y=23
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       222
cst816s_get_touch_data:   y:       23
```

With I2C Logging Disabled: We only get the Touch Up Event (with invalid Touch Data)...

```text
nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open:

bl602_expander_interrupt: Interrupt! callback=0x2305e55e, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e55e, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
cst816s_get_touch_data: Can't return touch data: id=9, touch=2, x=639, y=1688

bl602_expander_interrupt: Interrupt! callback=0x2305e55e, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e55e, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
cst816s_get_touch_data: Invalid touch data: id=9, touch=2, x=639, y=1688
cst816s_get_touch_data: Can't return touch data: id=9, touch=2, x=639, y=1688
```

This happens before and after we have reduced the number of I2C Transfers (by checking GPIO Interrupts via `int_pending`).

The workaround is to call `i2cwarn()` in the [BL602 I2C Driver](https://github.com/lupyuen/incubator-nuttx/blob/touch/arch/risc-v/src/bl602/bl602_i2c.c) to force this specific log to be printed...

```c
static int bl602_i2c_transfer(struct i2c_master_s *dev,
                              struct i2c_msg_s *   msgs,
                              int                      count) {
      ...
      if (priv->i2cstate == EV_I2C_END_INT)
        {
          i2cinfo("i2c transfer success\n");
#ifdef CONFIG_INPUT_CST816S
          /* Workaround for CST816S. See https://github.com/lupyuen/cst816s-nuttx#i2c-logging */

          i2cwarn("i2c transfer success\n");
#endif /* CONFIG_INPUT_CST816S */
        }
```

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/touch/arch/risc-v/src/bl602/bl602_i2c.c#L753-L761)

After patching the workaround, we get the Touch Down Event (with valid Touch Data)...

```text
nsh> lvgltest
tp_init: Opening /dev/input0
cst816s_open:

bl602_expander_interrupt: Interrupt! callback=0x2305e55e, arg=0x42020a70
bl602_expander_interrupt: Call callback=0x2305e55e, arg=0x42020a70
cst816s_poll_notify:

cst816s_get_touch_data:
cst816s_i2c_read:
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: DOWN: id=0, touch=0, x=200, y=26
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       200
cst816s_get_touch_data:   y:       26
```

LoRaWAN Test App `lorawan_test` also tested OK with the patch.

__TODO:__ Investigate the internals of the [BL602 I2C Driver](https://github.com/lupyuen/incubator-nuttx/blob/touch/arch/risc-v/src/bl602/bl602_i2c.c). Look for I2C Timing Issues or Race Conditions.

__TODO:__ Probe the I2C Bus with a Logic Analyser. Watch for I2C Hardware issues.

__TODO:__ Why must we disable logging? Eventually we must disable `CONFIG_DEBUG_INFO` (Informational Debug Output) because the LoRaWAN Test App `lorawan_test` fails when `CONFIG_DEBUG_INFO` is enabled (due to LoRaWAN Timers)

__TODO:__ LoRaWAN Test App, LoRaWAN Library, SX1262 Library, NimBLE Porting Layer, SPI Test Driver should have their own flags for logging

__TODO:__ Move CST816S Interrupt Handler to [BL602 GPIO Expander](https://github.com/lupyuen/bl602_expander)

__TODO:__ Implement SPI DMA on NuttX so that the touchscreen feels less laggy

__TODO:__ [Add a button](https://docs.lvgl.io/7.11/get-started/quick-overview.html#button-with-label) and a message box to the [LVGL Test App `lvgltest`](https://github.com/lupyuen/lvgltest-nuttx/blob/main/lvgltest.c#L110-L198) to demo the touchscreen
