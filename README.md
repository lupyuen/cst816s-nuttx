# Hynitron CST816S Touch Panel Driver for Apache NuttX RTOS

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

-   TODO: [See the modified Makefile and Kconfig](https://github.com/lupyuen/incubator-nuttx/commit/96f26693070eb9d6bbbf016ac6375ef041e3b24a?diff=unified)

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

TODO" And call `???` to load our driver:

