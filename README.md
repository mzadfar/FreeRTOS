# RTOS Programming

The purpose of this repository is to start from practicing RTOS programming on FreeRTOS, and proide a base for embedded code development using Hardware Abstraction Layer (HAL) of STM32F439ZI developed by ST.

## Description

The following topics will be practised here on a NUCLEO-F439ZI board:
* Empty project
* SemiHosting
* Virtual Com Port
* Task Yielding
* Task Call uing Polling
* Task Call uing Interrupt
* Task Wait and Notify
* Task Wait, Notify, and Action
* Delete task
* Prioritize tasks

## Getting Started

### Dependencies

* Host can be any OS like Fedora 40 Linux in this case or Ubuntu, Windows etc.  
* arm-none-eabi-gcc should be installed.
* gcc should be installed.
* cmake should be installed.
* Download and install [ST-LINK](https://www.st.com/en/development-tools/stsw-link007.html) latest firmware.
* In this Fedora 40 OS, Download [ST-LINK](https://koji.fedoraproject.org/koji/buildinfo?buildID=2389744) v1/v2 JTAG/SWD debugging/flashing tool for STM32 microcontrollers
* After above installation, Cmake still has issue with LIBUSB, and in this Fedora 40 OS using "yum provides */libusb.h"
* LIBUSB should be installed.
* STM32F4 Standard Peripheral Library Expansion [STSW-STM32120](https://www.st.com/en/embedded-software/stm32-standard-peripheral-library-expansion/products.html) should be downloaded. Find the one that has "system" under "MCU Peripheral" column.

### Installing

* Check arm-none-eabi-gcc --version on a terminal. If it has not been installed, Fedora will suggest to install, and go with it.
* Check cmake --version on a terminal. If it has not been installed, Fedora will suggest to install, and go with it. Typically, it asks for gcc installation during installing cmake if you have not installed gcc yet.
* Install [ST-LINK](https://www.st.com/en/development-tools/stsw-link007.html) latest firmware using: sudo rpm -Uhv st-stlink-udev-rules-1.0.3-2-linux-noarch.rpm
* In this Fedora 40 OS, install [ST-LINK](https://koji.fedoraproject.org/koji/buildinfo?buildID=2389744) v1/v2 JTAG/SWD debugging/flashing tool for STM32 microcontrollers using: sudo rpm -Uhv stlink-1.7.0-7.fc40.x86_64.rpm
* cmake has issue with LIBUSB, and in this Fedora 40 OS using "yum provides */libusb.h" command find a version of LIBUSB that suit you, and install it using (in this case): sudo dnf install libusb1-devel-1.0.27-2.fc40.x86_64
* Download stlink-testing.zip to open a terminal inside the extracted folder and run cmake.
* Run make

### Executing program

#### Empty project

* This is a FreeRTOS empty example proect to initiate practicing FreeRTOS. First copy "[Libraries](https://github.com/ahasanzadeh/BareMetal/tree/main/008_VirtualComTxCharacter)" folder from project into this prject folder. Open a terminal inside the folder, and run "make clean" to remove all previousely generated *.hex, *.elf, *.bin, *.srec etc files, and then run "make -j all". 

#### SemiHosting

* This is a FreeRTOS semihosting example to debug project using OPENOCD and a console like telnet etc. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Virtual Com Port

* This is a FreeRTOS example equiped with virtual COM port to communicate with PC. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Task Yielding

* This is a FreeRTOS proect to implement task yielding. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Task Call uing Polling

* This is a FreeRTOS proect to call a task using polling. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Task Call uing Interrupt

* This is a FreeRTOS proect to call a task using interrupt. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Task Wait and Notify

* This is a FreeRTOS proect to put a task on wait and notify states. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Task Wait, Notify, and Action

* This is a FreeRTOS proect to put a task on wait, notify, and action states. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Delete task

* This is a FreeRTOS proect to delete a task. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

#### Prioritize tasks

* This is a FreeRTOS proect to swith priority of tasks. Follow corresponding oder for "FreeRTOS Empty project" above. No ModelSim simulation for this example.

## Help

TBD

## Authors

Contributors names and contact info

TBD

## Version History

* 0.1
    * Initial Release

## License

This project is licensed under the FREE License - see the LICENSE.md file for details

## Acknowledgments

Inspiration, code snippets, etc.
* [ah](https://github.com/ahasanzadeh/)