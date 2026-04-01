# usb_sources.py — PlatformIO pre-build script
#
# Adds the STM32_USB_Device_Library Core and CDC source files to the build.
# These middleware sources are part of framework-stm32cubef4 but are NOT
# compiled automatically — they must be added explicitly because they require
# application-supplied usbd_conf.h / usbd_cdc_if.c glue files.

Import("env")

import os

packages_dir = env.PioPlatform().get_package_dir("framework-stm32cubef4")

usb_middleware_base = os.path.join(
    packages_dir, "Middlewares", "ST", "STM32_USB_Device_Library"
)

usb_sources = [
    os.path.join(usb_middleware_base, "Core", "Src", "usbd_core.c"),
    os.path.join(usb_middleware_base, "Core", "Src", "usbd_ctlreq.c"),
    os.path.join(usb_middleware_base, "Core", "Src", "usbd_ioreq.c"),
    os.path.join(usb_middleware_base, "Class", "CDC", "Src", "usbd_cdc.c"),
]

usb_include_dirs = [
    os.path.join(usb_middleware_base, "Core", "Inc"),
    os.path.join(usb_middleware_base, "Class", "CDC", "Inc"),
]

# Add USB middleware source files to compilation
env.Append(CPPPATH=usb_include_dirs)
env.Append(EXTRA_SCRIPTS=[])  # no-op, just ensure env is ready

for src in usb_sources:
    if os.path.isfile(src):
        env.AppendUnique(EXTRAOBJS=[
            env.Object(src)
        ])
    else:
        print(f"[usb_sources.py] WARNING: not found: {src}")

print("[usb_sources.py] USB Device Library sources added.")
