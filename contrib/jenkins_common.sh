#!/bin/sh

set -ex

base="$PWD"
deps="$base/deps"
inst="$deps/install"

export deps inst

mkdir -p "$deps"
rm -rf "$inst"

cd "$deps"

# Get libosmocore for verify_value_string_arrays_are_terminated.py
osmo-deps.sh libosmocore

# Get OpenBSC for gsm_data_shared.*
osmo-deps.sh openbsc

cd "$base"

"$deps"/libosmocore/contrib/verify_value_string_arrays_are_terminated.py $(find . -name "*.[hc]")
