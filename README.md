# mints-scada-hardware
Minimal Test Stand Supervisory Control and Data Acquisition system hardware and firmware. See [mints-scada-software](https://github.com/psas/mints-scada-software) for the computer software to control the whole system. 

# Assembly
## stm32devboards
* Populate all SMD components on the board
    * Only populate the 60.4 ohm CAN termination resistors on the nodes on the ends of the chain.
    * Populate the USB port and solder its mounting lugs
    * Do not populate the headers on the sides as they will make it more difficult to work on the boards if problems are discovered
* Complete the board testing
* Populate THT components
## interface-module
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
1. Install `dfu-util` with `sudo apt install dfu-util`
2. Set up udev rules
    1. Put the following into `/etc/udev/rules.d/60-stm32`
    ```
    # STM32
    ACTION=="add", SUBSYSTEM=="usb", ATTRS{idVendor}=="0483", ATTRS{idProduct}=="df11", TAG+="uaccess"
    ```
    2. Either reboot or run `sudo udevadm control --reload-rules && udevadm trigger` to reload the udev rules
3. Remove the devboard from the module if it was installed.
4. Put the device in bootloader mode
    * If the device is unplugged, hold the `BOOT` button while plugging it in
    * If the device is already plugged in, press `RESET` and `BOOT`, then release `RESET` then `BOOT`
5. Run `dfu-util --list` and confirm that there are two devices with the vendor:product ID `0483:df11`, one with the name `@Option Bytes` and one with the name `@Internal Flash`. If any other devices with that ID are present, remove them before continuing.
6. Flash the device with `dfu-util --device 0483:df11 --alt 0 --dfuse-address 0x08000000:leave --download [path/to/file/]<filename>.bin`

### Visual
[STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html) can be used to flash device firmware. Setup will be platform-dependent, and thusly not covered here.

# Firmware modifications
Firmware modification instructions are provided for Ubuntu 24.04. Other debian-based Linux distros should follow a similar procedure. Flashing from Mac or Windows should be possible, but is not covered here.

You will need several tools to flash the firmware
* Python3 virtual environments `sudo apt install python3-venv`
* dfu-util `sudo apt install dfu-util`
* [Visual Studio Code](https://code.visualstudio.com/)
* [PlatformIO Extension for VSCode](https://marketplace.visualstudio.com/items?itemName=platformio.platformio-ide)

1. Install the required software
2. Set up udev rules as described in the `Firmware Flashing` section
3. 

# Testing


