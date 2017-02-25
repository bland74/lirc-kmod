# LIRC Kernel Module for FreeBSD
lirc-kmod is a kernel module for [LIRC](http://www.lirc.org "LIRC – Linux Infrared Remote Control") compatible device driver (framework) and hardware specific backends.
Current implementation supports ```LIRC_MODE_MODE2``` receivers only.
## Build
You need to have FreeBSD kernel source installed before you start. If you use a binary distribution please make sure that kernel source version matches.
When ready, simply kick start the build from the directory where you put the lirc-kmod source:
```console
# make
# make install
```
All files will be installed under the ```/boot/modules``` directory.
## Configuration
Add a line specific for your IR hardware to the ```/boot/loader.conf``` file to enable the driver. For example:
```
itecir_load="YES"
```
A ```hw.lirc.rx_buffersize``` tunable can be used to control LIRC receive buffer size if required. Use ```/boot/loader.conf``` file to change the default:
```
hw.lirc.rx_buffersize=128
```
### ITE CIR Backend
Currently supported chips are ITE8704 and ITE8713.
There are global tunables provided to control the default IR communication frequency and baudrate. For example:
```
hw.itecir.default_freq=38000
hw.itecir.default_baudrate=115200
```
A device specific hints can be given in the ```/boot/device.hints``` file to enable precious signal demodulation. For example:
```
hints.itecir.0.lowcarrierfreq=27000
hints.itecir.0.highcarrierfreq=400000
```
## Known Issues
FreeBSD ACPI support does not strictly follow the original specification. It is always assumed that _CRS verb provides a correct value for the subsequent _SRS call. This is not true for at least [ASUS EeeBox PC EB1505](https://www.asus.com/Mini-PCs/EeeBox_PC_EB1505/). A workaround put in place in order to build a working _SRS buffer. You may need to adjust values inside the code guarded by the ```LIRC_ACPI_HACK``` knob.
