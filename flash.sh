#!/bin/bash
pushd base && \
    cargo objcopy --bin sandbox --release -- -O ihex  program.hex && \
    st-flash --format ihex write program.hex
rm program.hex
popd


