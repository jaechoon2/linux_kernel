linux_kernel
============

logiCVC, logiWIN, logiI2S drivers for Linux kernel version:

VERSION = 3

PATCHLEVEL = 10

SUBLEVEL = 0

EXTRAVERSION =

Copy all driver source, header files from git folder tree to the Linux kernel tree at same places.
Kconfig and Makefile contain snippets which must be added to original Linux kernel Kconfig and Makefile.

Device drivers support OpenFirmware functionality.
Devicetree examples can be found in Documentation folder.

Device drivers are developed and used under ARM architecture (Xilinx Zynq)
and x86 architecture (i7, Atom).
