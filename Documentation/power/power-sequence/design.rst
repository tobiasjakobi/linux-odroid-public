====================================
Power Sequence Library
====================================

:Date: Feb, 2017
:Author: Peter Chen <peter.chen@nxp.com>


Introduction
============

We have an well-known problem that the device needs to do a power
sequence before it can be recognized by related host, the typical
examples are hard-wired mmc devices and usb devices. The host controller
can't know what kinds of this device is in its bus if the power
sequence has not done, since the related devices driver's probe calling
is determined by runtime according to eunumeration results. Besides,
the devices may have custom power sequence, so the power sequence library
which is independent with the devices is needed.

Design
============

The power sequence library includes the core file and customer power
sequence library. The core file exports interfaces are called by
host controller driver for power sequence and customer power sequence
library files to register its power sequence instance to global
power sequence list. The custom power sequence library creates power
sequence instance and implement custom power sequence.

Since the power sequence describes hardware design, the description is
located at board description file, eg, device tree dts file. And
a specific power sequence belongs to device, so its description
is under the device node, please refer to:
Documentation/devicetree/bindings/power/pwrseq/pwrseq-generic.txt

Custom power sequence library allocates one power sequence instance at
bootup periods using postcore_initcall, this static allocated instance is
used to compare with device-tree (DT) node to see if this library can be
used for the node or not. When the result is matched, the core API will
try to get resourses (->get, implemented at each library) for power
sequence, if all resources are got, it will try to allocate another
instance for next possible request from host driver.

Then, the host controller driver can carry out power sequence on for this
DT node, the library will do corresponding operations, like open clocks,
toggle gpio, etc. The power sequence off routine will close and free the
resources, and is called when the parent is removed. And the power
sequence suspend and resume routine can be called at host driver's
suspend and resume routine if needed.

The exported interfaces
.. kernel-doc:: drivers/power/pwrseq/core.c
   :export:
