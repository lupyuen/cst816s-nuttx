# Hynitron CST816S Touch Controller Driver for Apache NuttX RTOS

(Used by PineDio Stack BL604)

See https://lupyuen.github.io/articles/pinedio2#touch-panel

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

  ret = cst816s_register("/dev/input0", cst816s_i2c_bus, 0x15);
  if (ret < 0)
    {
      _err("ERROR: Failed to register CST816S\n");
    }
#endif /* CONFIG_INPUT_CST816S */
```

Here's how we created the CST816S Driver for NuttX on PineDio Stack BL604...

# Cypress MBR3108

NuttX Driver for Cypress MBR3108 Touch Controller looks structurally similar to PineDio Stack's CST816S ... So we copy-n-paste into our CST816S Driver

[(Source)](https://github.com/lupyuen/incubator-nuttx/blob/master/drivers/input/cypress_mbr3108.c)
