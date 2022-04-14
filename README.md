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

TODO

# Test GPIO Interrupt

GPIO Interrupt works when we tap the screen!

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

LVGL Test App fails to open `/dev/input0`, but that's OK because we haven't implemented the I2C part.

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

# Test Touch Data

Touch Data looks erratic, most are invalid, with some valid points.

LVGL Test App seems to be polling the Touch Controller very frequently.

```text
gpio_pin_register: Registering /dev/gpio0
gpio_pin_register: Registering /dev/gpio1
gpint_enable: Disable the interrupt
gpio_pin_register: Registering /dev/gpio2
bl602_gpio_set_intmod: ****gpio_pin=115, int_ctlmod=1, int_trgmod=0
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
cst816s_register: 
bl602_expander_set_intmod: ****gpio_pin=105, int_ctlmod=1, int_trgmod=0
bl602_irq_attach: Attach 0x2305e9d2
bl602_irq_enable: Disable interrupt
cst816s_register: Driver registered

NuttShell (NSH) NuttX-10.2.0-RC0
nsh> [Klvgltest
tp_init: Opening /dev/input0
cst816s_open: 
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=732, y=3
cst816s_get_touch_data:   id:      90
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       732
cst816s_get_touch_data:   y:       3
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=236, y=2601
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   0c
cst816s_get_touch_data:   x:       236
cst816s_get_touch_data:   y:       2601
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=732, y=41
cst816s_get_touch_data:   id:      90
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       732
cst816s_get_touch_data:   y:       41
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=620, y=3369
cst816s_get_touch_data:   id:      75
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       620
cst816s_get_touch_data:   y:       3369
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=620, y=3330
cst816s_get_touch_data:   id:      75
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       620
cst816s_get_touch_data:   y:       3330
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touc_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst81s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:      639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_ouch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: 2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:      639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:  y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=00, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
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
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=732, y=41
cst816s_get_touch_data:   id:      90
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       732
cst816s_get_touch_data:   y:       41
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
cst816sget_touch_data:   y:       0
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80da001b
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       218
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xda001b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       218
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=620, y=3330
cst816s_get_touch_data:   id:      75
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       620
cst816s_get_touch_data:   y:       3330
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=620, y=3330
cst816s_get_touch_data:   id:      75
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       620
cst816s_get_touch_data:   y:       3330
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=00, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data:
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:  y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data:
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_toch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816si2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=00, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst81s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x200011
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       17
cst816s_get_touch_data:   y:       32
tp_cal result
offset x:3, y:27
range x:1685, y:705
invert x/y:1, x:0, y:1

cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfr success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfr success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_tuch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:      102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
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
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x660067
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       103
cst816s_get_touch_data:   y:       102
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_daa:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f008c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       140
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x805800a3
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       163
cst816s_get_touch_data:   y:       88
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5800a3
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       163
cst816s_get_touch_data:   y:       88
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x8061008b
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       139
cst816s_get_touch_data:   y:       97
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x61008b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       139
cst816s_get_touch_data:   y:       97
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
i2c transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_ouch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data:
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5e008e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       142
cst816s_get_touch_data:   y:       94
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x805c0088
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_dat: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfe: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c0088
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       136
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x805600a8
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       168
cst816s_get_touch_data:   y:       86
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5600a8
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       168
cst816s_get_touch_data:   y:       86
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816sget_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdat: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_ata:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_ata:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5c00a4
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       164
cst816s_get_touch_data:   y:       92
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x805f009f
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:      95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x5f009f
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       159
cst816s_get_touch_data:   y:       95
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x801200d3
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       211
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1200d2
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       210
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1200d2
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       210
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1200d2
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       210
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1200d2
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       210
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1200d2
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       210
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1200d2
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       210
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1200d2
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       210
cst816s_get_touch_data:   y:       18
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x801800c9
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       24
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1800c9
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       24
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1800c9
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       24
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1800c9
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       24
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1800c9
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       24
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x1800c9
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       201
cst816s_get_touch_data:   y:       24
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x800c00b7
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:      183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00b7
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       183
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x800a00aa
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       170
cst816s_get_touch_data:   y:       10
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xb00a5
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       165
cst816s_get_touch_data:   y:       11
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x800c00af
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:  x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc00af
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       175
cst816s_get_touch_data:   y:       12
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c60014
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c60014
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_toch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_dat: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_daa:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc60014
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       20
cst816s_get_touch_data:   y:       198
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cc000e
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cc000e
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfe: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cc000e
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcc000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       204
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c50029
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       41
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc50029
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       41
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c5001d
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c5001d
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x40000000
bl602_i2c_recvdata: count=3, temp=0x40c5001d
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfr success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc5001d
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       29
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c1001a
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c1001a
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1001a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       26
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c2002a
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       42
cst816s_get_touch_data:   y:       194
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc2002a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       42
cst816s_get_touch_data:   y:       194
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer:subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1002c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       44
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1002c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       44
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1002c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       44
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1002c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       44
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc1002c
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       44
cst816s_get_touch_data:   y:       193
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80c20023
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:      35
cst816s_get_touch_data:   y:       194
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc20023
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       35
cst816s_get_touch_data:   y:       194
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=00, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc20023
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       35
cst816s_get_touch_data:   y:       194
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc20023
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       35
cst816s_get_touch_data:   y:       194
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0xce001b
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ce001b
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ce001b
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce001b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce001b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_tuch_data:   flags:   1c
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce001b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_ouch_data:   flags:   1c
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce001b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce001b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce001b
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       27
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cd000d
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       13
cst816s_get_touch_data:   y:       205
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cd000d
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       13
cst816s_get_touch_data:   y:       205
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcd000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       205
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcd000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       205
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcd000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       205
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer:i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcd000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       205
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcd000e
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       14
cst816s_get_touch_data:   y:       205
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0xce0002
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ce0002
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ce0002
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce0002
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:    206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce0002
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce0002
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce0002
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce0002
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xce0002
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       2
cst816s_get_touch_data:   y:       206
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cb0008
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_ic_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cb0008
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cb0008
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_dta:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcb0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       203
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ca000a
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ca000a
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80ca000a
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca000a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca000a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca000a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca000a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca000a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca000a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xca000a
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       10
cst816s_get_touch_data:   y:       202
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cf0008
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80cf0008
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x40000000
bl602_i2c_recvdata: count=3, temp=0x40cf0008
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xcf0008
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       8
cst816s_get_touch_data:   y:       207
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x800d0013
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xd0013
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       19
cst816s_get_touch_data:   y:       13
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80100001
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7 temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       16
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
bl602_i2c_recvdata: count=3, temp=0x80110001
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       17
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x110001
transfer success
cst816s_get_touch_ata:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       17
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x110001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       17
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x110001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       17
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x110001
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       1
cst816s_get_touch_data:   y:       17
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80100003
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x80010000
bl602_i2c_recvdata: count=3, temp=0x80100003
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_daa:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0x100003
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       3
cst816s_get_touch_data:   y:       16
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x10000
bl602_i2c_recvdata: count=3, temp=0xd500e1
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   19
cst816s_get_touch_data:   x:       225
cst816s_get_touch_data:   y:       213
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x40000000
bl602_i2c_recvdata: count=3, temp=0x40c500da
transfer success
cst816s_get_touch_data:   id:      0
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_send_data: count=1, temp=0x0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_daa:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_toch_data:   flags:   1c
cst816_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_daa:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:      197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst81s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       21
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7 temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7 temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c bl602_i2c_recvdata: count=7, temp=0x500
bl602_i2c_recvdata: count=3, temp=0xc500da
transfer success
cst816s_get_touch_data:   id:      5
cst816s_get_touch_data:   flags:   1c
cst816s_get_touch_data:   x:       218
cst816s_get_touch_data:   y:       197
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data:
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data:
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:      639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_et_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_dat: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_tuch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_tr success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_datacst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, suben=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_ic_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_tuch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_ic_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_gettouch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_ic_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:      639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch dat: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_tuch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_ouch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_dta: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer:i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_toch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=00, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_ic_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
b602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfe: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch dta: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch dta: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2ctransfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_gt_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:  y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=00, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
st816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:  y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfe: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
cst816s_read: 
cst816s_get_touch_data: 
cst816s_i2c_read: 
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
bl602_i2c_transfer: subflag=0, subaddr=0x0, sublen=0
bl602_i2c_transfer: i2c transfer success
cst816s_get_touch_data: Invalid touch data: x=639, y=1688
cst816s_get_touch_data:   id:      76
cst816s_get_touch_data:   flags:   09
cst816s_get_touch_data:   x:       639
cst816s_get_touch_data:   y:       1688
```
