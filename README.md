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

GOP Interrupt works when we tap the screen!

```text
gpio_pin_register: Registering /dev/gpio0
gpio_pin_register: Registering /dev/gpio1
gpint_enable: Disable the interrupt
gpio_pin_register: Registering /dev/gpio2
bl602_gpio_set_intmod: ****gpio_pin=115, int_ctlmod=1, int_trgmod=0
spi_test_driver_register: devpath=/dev/spitest0, spidev=0
bl602_expander_set_intmod: ****gpio_pin=105, int_ctlmod=1, int_trgmod=0
bl602_irq_attach: Attach 0x2305e9e8
bl602_irq_enable: Disable interrupt
bl602_irq_enable: Enable interrupt
cst816s_register: Driver registered

NuttShell (NSH) NuttX-10.2.0-RC0
nsh> bl602_expander_interrupt: Interrupt! callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Call callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Interrupt! callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Call callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Interrupt! callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Call callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Interrupt! callback=0x2305e9e8, arg=0
bl602_expander_interrupt: Call callback=0x2305e9e8, arg=0
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
