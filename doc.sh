#!/bin/sh

cargo doc && \
python3 -m http.server -d target/thumbv6m-none-eabi/doc
