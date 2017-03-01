#!/usr/bin/env bash

set -ex

base="$PWD"
deps="$base/deps"
inst="$deps/install"
export deps inst

mkdir "$deps" || true
rm -rf "$inst"

export PKG_CONFIG_PATH="$inst/lib/pkgconfig:$PKG_CONFIG_PATH"
export LD_LIBRARY_PATH="$inst/lib"

osmo-build-dep.sh libosmocore

"$deps"/libosmocore/contrib/verify_value_string_arrays_are_terminated.py $(find . -name "*.[hc]")

osmo-build-dep.sh libosmo-abis

cd "$deps"

# Get osmo-pcu for pcuif_proto.h
osmo-deps.sh osmo-pcu

# Get openbsc for gsm_data_shared.*
osmo-deps.sh openbsc

cd "$deps"
if ! test -d layer1-api;
then
  git clone git://git.osmocom.org/octphy-2g-headers layer1-api
fi
cd layer1-api
git fetch origin
if [ $FIRMWARE_VERSION = "master" ];
then
git reset --hard origin/master
else
git reset --hard $FIRMWARE_VERSION
fi

cd "$base"

set +x
echo
echo
echo
echo " =============================== osmo-bts-octphy+trx ==============================="
echo
set -x

autoreconf --install --force
configure_flags="\
  --with-openbsc=$deps/openbsc/openbsc/include \
  --with-osmo-pcu=$deps/osmo-pcu/include \
  --with-octsdr-2g=$deps/layer1-api/ \
  --enable-octphy \
  --enable-trx \
  "
./configure $configure_flags
$MAKE $PARALLEL_MAKE
$MAKE check \
  || cat-testlogs.sh
DISTCHECK_CONFIGURE_FLAGS="$configure_flags" \
  $MAKE distcheck \
  || cat-testlogs.sh
