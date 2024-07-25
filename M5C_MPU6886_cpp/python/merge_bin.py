#not yet complteted 07/25

import os
from SCons.Script import Import

Import("env")

def merge_bin(source, target, env):
    # Define paths to the binaries
    bootloader_bin = os.path.join(env.subst("$BUILD_DIR"), "bootloader.bin")
    partitions_bin = os.path.join(env.subst("$BUILD_DIR"), "partitions.bin")
    boot_bin = os.path.join(env.subst("$BUILD_DIR"), "boot_app0.bin")
    app_bin = os.path.join(env.subst("$BUILD_DIR"), "firmware.bin")
    output_bin = os.path.join(env.subst("$BUILD_DIR"), "merged-firmware.bin")

    # Run esptool.py command
    env.Execute(
        "esptool.py --chip esp32 merge_bin -o {} --flash_mode dio --flash_freq 40m --flash_size 4MB 0x1000 {} 0x8000 {} 0xe000 {} 0x10000 {}".format(
            output_bin, bootloader_bin, partitions_bin, boot_bin, app_bin
        )
    )

env.AddPreAction("upload", merge_bin)