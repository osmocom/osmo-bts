#!/bin/sh

# this is a common helper script that is shared among all BTS model
# specific helper scripts like jenkins_sysmobts.sh.  You shouldn't call
# this directly, but rather indirectly via the bts-specific scripts

if ! [ -x "$(command -v osmo-deps.sh)" ]; then
	echo "Error: We need to have scripts/osmo-deps.sh from http://git.osmocom.org/osmo-ci/ in PATH !"
	exit 2
fi

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
