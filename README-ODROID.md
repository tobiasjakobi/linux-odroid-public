# Vanilla Linux 5.1.y for the Odroid-X2/U2/U3

This is a [linux-5.1.y](https://git.kernel.org/cgit/linux/kernel/git/stable/linux-stable.git/log/?h=linux-5.1.y) tree with some modifications to make it work on a Hardkernel Odroid-X2 developer board. The U2/U3 boards should work as well, but are neither owned nor tested by me.


TODOs:

   - remove more of the always-on properties of the various regulators
   - modify refclk code in usb3503 (make it more generic)
   - interaction between DMC and G2D core clocks needs testing
   - extcon patches need to be applied again

EXTERNAL TODOs:

   - IOMMU buffer exchange between MFC/FIMC and DRM needs testing
   - CONFIG_TRIM_UNUSED_KSYMS not working (undefined symbols during modpost phase)
