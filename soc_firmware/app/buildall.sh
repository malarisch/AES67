#!/bin/bash
west build . -b litex_vexriscv_cyc1000 -d build-cyc1000 &&\
west build . -b litex_vexriscv_cyclone10 -d build-cyc10 &&\
west build . -b esp32s3_devkitc/esp32s3/procpu -d build-esp32