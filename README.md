# mints-scada-hardware
Minimal Test Stand Supervisory Control and Data Acquisition system hardware and firmware. See [mints-scada-software](https://github.com/psas/mints-scada-software) for the computer software to control the whole system. 

# Assembly
### stm32devboards
* Populate all SMD components on the board
    * Only populate the 60.4 ohm CAN termination resistors on the nodes on the ends of the chain.
    * Populate the USB port and solder its mounting lugs
    * Do not populate the headers on the sides as they will make it more difficult to work on the boards if problems are discovered
* Complete the board testing
* Populate THT components
### interface-module
* Select the desired configuration of the board. This can be one of
    * 8x Push and/or Pull outputs
    * 4x I2C
    * 4x ADC
    * See notes in KiCad for configuration information
* Assemble only the required components for that version of the board. Leave the rest unpopulated

# Firmware flashing
Firmware files are located in the `firmware/bin` directory.

### Command line (recommended)
Command line firmware flashing instructions are provided for Ubuntu 24.04. Other debian-based Linux distros should follow a similar procedure. Flashing from Mac or Windows should be possible, but is not covered here.
#### Setup
1. Install `dfu-util` with `sudo apt install dfu-util`
2. Set up udev rules
    1. Put the following into `/etc/udev/rules.d/60-stm32`
    ```
    # STM32
    ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", TAG+="uaccess"
    ```
    2. Either reboot or run `sudo udevadm control --reload-rules && udevadm trigger` to reload the udev rules

#### Flashing:
1. Remove the devboard from the module if it was installed.
2. Put the device in bootloader mode
    * If the device is unplugged, hold the `BOOT` button while plugging it in
    * If the device is already plugged in, press `RESET` and `BOOT`, then release `RESET` then `BOOT`
3. Run `dfu-util --list` and confirm that there are two devices with the vendor:product ID `0483:df11`, one with the name `@Option Bytes` and one with the name `@Internal Flash`. If any other devices with that ID are present, remove them before continuing.
4. Flash the device with `dfu-util --device 0483:df11 --alt 0 --dfuse-address 0x08000000:leave --download [path/to/file/]<filename>.bin`. It should erase the flash, then download the new file and show `File downloaded successfully` when it is done.

### Visual
[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) can be used to flash device firmware. Setup will be platform-dependent, and thusly not covered here.

# Firmware modifications (depricated)
Firmware modification instructions are provided for Ubuntu 24.04. Other debian-based Linux distros should follow a similar procedure. Flashing from Mac or Windows should be possible, but is not covered here.

You will need several tools to flash the firmware

1. Install the required software
    * Python3 virtual environments `sudo apt install python3-venv`
    * dfu-util `sudo apt install dfu-util`
    * [Visual Studio Code](https://code.visualstudio.com/)
    * [PlatformIO Extension for VSCode](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)
2. Set up udev rules as described in the `Firmware Flashing` section
3. Open the firmware source folder in Visual Studio Code
4. Open PlatformIO Home
5. Install the `ststm32` platform

To build and upload:
1. Open the PlatformIO tab in the left toolbar
2. Under `PROJECT TASKS` > `Build` > `General` execute the `Build` procedure by clicking on it
3. Execute `Upload` to upload to the microcontroller. Ensure that only a single board is plugged in to ensure you are writing to the correct device.

# Building Firmware
Building firmware from source shouldn't be needed unless you are making changes to it. Firmware build instructions are provided for Ubuntu 24.04. Other debian-based Linux distros should follow a similar procedure. Flashing from Mac or Windows should be possible, but is not covered here.

You will need:
* GCC for arm `sudo apt install gcc-arm-none-eabi`
* make `sudo apt install make`
* Everything as described in the command line flashing setup section above.

To build the code, simply run `make` in the firmware folder. This will build the firmware and copy the .bin into the firmware folder. Running `make flash` will build the firmware then upload it to a connected DFU mode microcontroller.

Initialization code was generated using [https://www.st.com/en/development-tools/stm32cubemx.html](STM32Cube initialization code generator). You should not need to mofidy this, but if you do, the `module_io.ioc` is included. If you regenerate the code, you may have to add
```
#######################################
# Flash
#######################################
flash: all
	dfu-util --device 0483:df11 --alt 0 --dfuse-address 0x08000000:leave --download $(BUILD_DIR)/$(TARGET).bin
```
to the end of `Makefile`. Optionally, add `cp $(BUILD_DIR)/$(TARGET).bin $(TARGET).bin` to target `all` in the makefile (add line 171) to copy the bin out of the build direcotry. Generally, CubeMX does not seem to update the Makefile if it exists.

# Testing
### stm32devboards
1. Populate all SMD components as described in the `assembly` sections. Do not populate THT components yet
2. Visually inspect the board. Fix any issues and inspect again until no issues remain
    * Verify that no pins are shorted
    * Verify the correct orientation and alignment of all components. Pay special attention to the STM32 since its square shape makes it more difficult to notice errors
    * Bridges can hide under the USB port where you can't see them unless you tilt the board to look deep under the port. They can often be fixed by applying flux then heating the two bridged pins.
    * Fixing any of these issues is easier before soldering THT parts since everything here is reflow safe, so it is less likely to be damaged during repairs.
3. Use a DMM to test between `GND` `3v3`, and `GND` and either pad of the polyfuse (the component in the corner of the board by the USB port). Verify there are no shorts.
4. On your computer run `sudo dmesg -Hw` to monitor system messages in real time (`-m`) with human-readable timestamps (`-h`). Keep this window open and visible for the next step
5. Plug the board in via USB. Watch `dmesg`. The following should appear. If it does not, then there is a problem with your board.
```
[<starttime>] usb 3-2: new full-speed USB device number <id> using xhci_hcd
[  +0.148877] usb 3-2: New USB device found, idVendor=0483, idProduct=df11, bcdDevice=22.00
[  +0.000014] usb 3-2: New USB device strings: Mfr=1, Product=2, SerialNumber=3
[  +0.000006] usb 3-2: Product: STM32  BOOTLOADER
[  +0.000006] usb 3-2: Manufacturer: STMicroelectronics
[  +0.000005] usb 3-2: SerialNumber: FFFFFFFEFFFF
```
6. Use the above procedure to flash the `blink-fast.bin` firmware. Confirm that the LED blinks at about 10Hz.
7. Use the above procedure to flash the `blink-slow.bin` firmware. Confirm that the LED blinks at about 1Hz. This confirms that you are able to flash new firmware to the device.
8. Install the THT components as described in the assembly section.