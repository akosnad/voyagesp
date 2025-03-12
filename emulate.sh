#!/usr/bin/env bash

espflash partition-table -o partitions.bin --to-binary partitions.csv   

espflash save-image --chip esp32 -s 4mb -T partitions.csv --target-app-partition ota_0 \
  target/xtensa-esp32-none-elf/release/voyagesp app.bin

esptool.py --chip esp32 merge_bin --output result.bin --fill-flash-size 4MB \
  0x1000 ../esp-bootloader/build/bootloader/bootloader.bin \
  0x8000 partitions.bin \
  0x10000 app.bin \
  --flash_freq 40m

qemu-system-xtensa -nographic -machine esp32 -drive file=result.bin,if=mtd,format=raw
